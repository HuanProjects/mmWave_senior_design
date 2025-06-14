import sys, struct, time, serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from struct import Struct
import re
import colorsys
from collections import deque

# ---------------------------
# Configuration
# ---------------------------
CONFIG_FILE = 'vital_signs_ISK.cfg'
CLI_PORT = 'COM3'
DATA_PORT = 'COM4'
ARDUINO_PORT = 'COM5'
ARDUINO_BAUD = 115200
FRAME_HEADER_LENGTH = 40
TLV_HEADER_LENGTH   = 8
MAGIC_WORD = bytes([2,1,4,3,6,5,8,7])
WAVE_BUFFER = 60
HISTORY_LENGTH = 10
HEART_Y_MIN, HEART_Y_MAX = 40, 100
BREATH_Y_MIN, BREATH_Y_MAX = 0, 25
GREY_RGBA = [0.9, 0.9, 0.9, 1.0]
X = np.arange(HISTORY_LENGTH, dtype=float)

# — make everything white background, black foreground by default —
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

# Create Struct objects for efficient unpacking
_U32 = Struct('<I')
_F   = Struct('<f')
_VITALS = Struct('<HHfff15f15f')  # Vital signs TLV format

# ---------------------------
# Helper unpack functions
# ---------------------------
def uint32_unpack(mv, offset):
    return _U32.unpack_from(mv, offset)[0]

def float_unpack(mv, offset):
    return _F.unpack_from(mv, offset)[0]

def int16_converter(data):
    val = data[0] + (data[1] << 8)
    return val if val < 32768 else val - 65536

def int8_converter(b):
    return b - 256 if b > 127 else b

# ---------------------------
# Sensor setup
# ---------------------------
def load_config(port_str, cfg):
    cli = serial.Serial(port_str, 115200, timeout=1)
    txt = open(cfg).read()
    for line in txt.splitlines():
        cmd = line.strip()
        if cmd:
            cli.write((cmd + "\n").encode())
            time.sleep(0.05)
    return cli, txt

# ---------------------------
# Frame reader
# ---------------------------
def record_frames(data_port):
    buffer = bytearray()
    while True:
        new_data = data_port.read(data_port.in_waiting or 1)
        if new_data:
            buffer.extend(new_data)
        else:
            time.sleep(0.01)
        while True:
            idx = buffer.find(MAGIC_WORD)
            if idx < 0:
                buffer = buffer[-len(MAGIC_WORD):]
                break
            buffer = buffer[idx:]
            if len(buffer) < FRAME_HEADER_LENGTH:
                break
            if buffer[:8] != MAGIC_WORD:
                buffer = buffer[8:]
                continue
            header = struct.unpack('<Q8I', buffer[:FRAME_HEADER_LENGTH])
            plen = header[2]
            ntlvs = header[7]
            if len(buffer) < plen:
                break
            frame = buffer[:plen]
            yield frame, ntlvs
            buffer = buffer[plen:]

# ---------------------------
# Helper function 
# ---------------------------
def pointCloud(mv, start, length):
    hdr = np.frombuffer(mv[start:start+20], dtype='<f4')
    scale = hdr[[3,0,1,2,4]]
    raw = np.frombuffer(
        mv[start+20:start+length],
        dtype=[('el','i1'), ('az','i1'), ('dv','<i2'),('rv','<i2'),('sr','<i2')]
    )
    el = raw['el'].astype(np.float32) * scale[1]
    az = raw['az'].astype(np.float32) * scale[2]
    rv = raw['rv'].astype(np.float32) * scale[0]
    dv = raw['dv'].astype(np.float32) * scale[3]
    sr = raw['sr'].astype(np.float32) * scale[4]
    x = rv * np.cos(el) * np.sin(az)
    y = rv * np.cos(el) * np.cos(az)
    z = rv * np.sin(el)
    return np.column_stack((x, y, z, dv, sr))

def targetList(mv, start, length):
    dtype = np.dtype([
        ('tid','<u4'),
        ('posX','<f4'),('posY','<f4'),('posZ','<f4'),
        ('velX','<f4'),('velY','<f4'),('velZ','<f4'),
        ('accX','<f4'),('accY','<f4'),('accZ','<f4'),
        ('_cov','<f4',16),
        ('g','<f4'),('confidenceLevel','<f4')
    ])
    raw = np.frombuffer(mv[start:start+length], dtype=dtype)
    return raw[['tid','posX','posY','posZ','velX','velY','velZ','accX','accY','accZ','g','confidenceLevel']]

def vitalSigns(mv, start, length):
    up = _VITALS.unpack_from(mv, start)
    return {
        'id': up[0], 'rangebin': up[1], 'breathingDeviation': up[2],
        'heart_rate': up[3], 'breathing_rate': up[4],
        'heart_waveform': up[5:20], 'breath_waveform': up[20:35]
    }

# ---------------------------
# Process frame
# ---------------------------
def process_frame(frame: bytes, numTlvs: int):
    mv = memoryview(frame)
    off = FRAME_HEADER_LENGTH
    types, targets, cloud, tids, pres, vit = [], None, None, None, None, None

    for idx in range(numTlvs):
        # 1) Make sure we can read a full TLV header
        if off + TLV_HEADER_LENGTH > len(mv):
            print(f"[TLV {idx}] incomplete header at offset {off}, stopping.")
            break

        # 2) Unpack type and payload‐length (payload only)
        t       = _U32.unpack_from(mv, off)[0]
        length  = _U32.unpack_from(mv, off + 4)[0]   # payload length
        types.append(t)

        # 3) Compute payload bounds
        start = off + TLV_HEADER_LENGTH
        end   = start + length

        # 4) Guard against running past the frame
        if end > len(mv):
            print(f"[TLV {idx}] payload (len {length}) at offset {start} overruns frame (size {len(mv)}).")
            break

        # 5) Slice out the payload and dispatch
        payload = mv[start:end]
        if t == 1010:
            targets = targetList(payload, 0, length)
        elif t == 1020:
            cloud   = pointCloud(payload, 0, length)
        elif t == 1011:
            tids    = np.frombuffer(payload, dtype=np.uint8)
        elif t == 1021:
            pres    = (uint32_unpack(mv, start) == 1)
        elif t == 1040:
            vit     = vitalSigns(payload, 0, length)

        # 6) Advance by header + payload
        off += TLV_HEADER_LENGTH + length

    return {
        'tlv_types':   types,
        'targets':     targets,
        'cloud':       cloud,
        'target_ids':  tids,
        'presence':    pres,
        'vital_signs': vit
    }

# ---------------------------
# Parse boundaryBox from cfg
# ---------------------------
def parse_xyz_limits(config_text):
    pattern = r"boundaryBox\s+(-?\d+(?:\.\d+)?(?:\s+-?\d+(?:\.\d+)?){5})"
    match = re.search(pattern, config_text)
    if not match:
        return None
    values = list(map(float, match.group(1).split()))
    return {"x_limits":(values[0],values[1]),"y_limits":(values[2],values[3]),"z_limits":(values[4],values[5])}

class SerialReader(QtCore.QThread):
    frameReady = QtCore.pyqtSignal(object, object, object)  
    # will emit (res, prev_cloud, prev_ids)

    def __init__(self, data_port_str, baud, parent=None):
         super().__init__(parent)
         self.dp = serial.Serial(data_port_str, baud, timeout=1)
         self._frames = record_frames(self.dp)
         self._last_res = None

    def run(self):
        for frame, ntlvs in self._frames:
            res = process_frame(frame, ntlvs)
            prev_cloud = self._last_res.get('cloud') if self._last_res else None
            prev_ids   = self._last_res.get('target_ids') if self._last_res else None
            self.frameReady.emit(res, prev_cloud, prev_ids)
            self._last_res = res
            if self.isInterruptionRequested():
                break

def map_to_grid(value, src_min, src_max, grid_size=8):
    # clamp
    v = max(min(value, src_max), src_min)
    # normalize to [0,1] then scale to [0, grid_size−1], then +1 to get 1…grid_size
    return int(round((v - src_min) / (src_max - src_min) * (grid_size - 1))) + 1

class PlotWindow:
    def __init__(self, limits):
        self.app = QtWidgets.QApplication(sys.argv)
        # Create a 2x2 grid layout
        self.win = pg.GraphicsLayoutWidget(title="Vital Signs Viewer")
        self.win.resize(1000, 800)

        # keep the last two frames so everything is delayed by one frame
        self._frames = deque(maxlen=2)
        
        # Pre-generate a brush for each possible ID (0–10)
        # We’ll use an HSV→RGB mapping so they’re nicely spaced.
        self.id_brushes = []
        for i in range(11):
            # hue from 0.0 to just under 1.0
            h = i / 11.0
            r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
            # alpha at 200/255 for a little transparency
            self.id_brushes.append(pg.mkBrush(int(r*255), int(g*255), int(b*255), 200))

        # A fallback grey brush for “unknown” IDs
        grey_val = int(0.9*255)
        self.grey_brush = pg.mkBrush(grey_val, grey_val, grey_val, 200)

        # ── build matching pens from those brushes ───────────────
        self.id_pens = [pg.mkPen(brush.color(), width=2)
                        for brush in self.id_brushes]

         # Heart‐waveform plot (top‐left)
        self.hr_plot = self.win.addPlot(row=0, col=0, title="Heart Waveform", background='w')
        self.hr_plot.setYRange(-1, 1)                    # lock Y between -1 and +1
        self.hr_plot.setXRange(0, WAVE_BUFFER - 1)        # X from 0…59
        self.hr_wave_hist = {}    # tid -> deque of last 60 samples
        self.hr_curves    = {}    # tid -> PlotDataItem
        self.hr_plot.addLegend()

        # Breath‐waveform plot (top‐right)
        self.br_plot = self.win.addPlot(row=0, col=1, title="Breath Waveform", background='w')
        self.br_plot.setYRange(-1, 1)
        self.br_plot.setXRange(0, WAVE_BUFFER - 1)
        self.br_wave_hist = {}
        self.br_curves    = {}
        self.br_plot.addLegend()

        # Point cloud (bottom-left)
        self.cloud_plot = self.win.addPlot(row=1, col=0, title="Point Cloud (Top-Down)", background='w')
        self.cloud_scatter = pg.ScatterPlotItem(
            pen=None,
            brush=pg.mkBrush(150,150,150,200),
            size=4,
            pxMode=True
        )
        self.cloud_plot.addItem(self.cloud_scatter)
        self.cloud_plot.setXRange(*limits['x_limits'])
        self.cloud_plot.setYRange(*limits['y_limits'])

        # ——————————————————————————————————————————————
        # Info panel (bottom-right)
        # ——————————————————————————————————————————————
        # Create a LabelItem (rich-text widget) with center justification
        self.info_label = pg.LabelItem(justify='center')
        # Add it to the grid at row=1, col=1
        self.win.addItem(self.info_label, row=1, col=1)

        self.limits = limits
        # open Arduino…
        self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
        # Show the window
        self.win.show()

    def start(self):
        self.thread = SerialReader(DATA_PORT, 921600)
        self.thread.frameReady.connect(self.update)
        self.thread.start()
        sys.exit(self.app.exec())

    def _brush_list_for_ids(self, ids):
        """Map a 1D array of integer IDs into a list of QBrushes."""
        brushes = []
        for tid in ids:
            if 0 <= tid < len(self.id_brushes):
                brushes.append(self.id_brushes[tid])
            else:
                brushes.append(self.grey_brush)
        return brushes
    
    def update(self, res, *_):
        # 1) push the new result
        self._frames.append(res)
        if len(self._frames) < 2:
            return

        older, newer = self._frames[0], self._frames[1]

        # pull the delayed vital_signs TLV
        if vs := (self._frames[0].get('vital_signs') if len(self._frames) >= 2 else None):
            pres = "Present" if self._frames[0].get('presence', False) else "Absent"
            if (pres == "Present"):
                hr   = vs['heart_rate']
                br   = vs['breathing_rate']
            else: 
                hr   = 0
                br   = 0
            # new:
            detected = "Detected" if pres == "Present" else "Not Detected"
            html = (
                "<div style='text-align:center;'>"
                f"<span style='font-size:48pt; font-weight:bold;'>HR: {hr:.1f} bpm</span><br>"
                f"<span style='font-size:48pt; font-weight:bold;'>BR: {br:.1f} bpm</span><br>"
                f"<span style='font-size:36pt;'>Presence: {detected}</span>"
                "</div>"
            )
            self.info_label.setText(html)


        # — compute the set of currently tracked IDs from the newest 'targets' TLV —
        tgt = newer.get('targets')
        if tgt is not None and tgt.size:
            xmin, xmax = self.limits['x_limits']
            ymin, ymax = self.limits['y_limits']

            grid_pairs = []
            for t in tgt:
                gx = map_to_grid(t['posX'], xmin, xmax, 8)
                gy = map_to_grid(t['posY'], ymin, ymax, 8)
                grid_pairs.append(f"{gx} {gy}")

            out = " ".join(grid_pairs) + "\n"
            self.arduino.write(out.encode('utf-8'))

            current_ids = set(int(x) for x in tgt['tid'])
        else:
            current_ids = set()

        # — Vital signs (delayed one frame) —
        if vs := older.get('vital_signs'):
            tid = vs['id']

            # If this ID is *not* in the current target list, reset its history to zeros
            if vs := older.get('vital_signs'):
                tid = vs['id']

                # if ID dropped out, clear its curves
            if tid not in current_ids:
                if tid in self.hr_curves:
                    self.hr_curves[tid].setData([], [])
                if tid in self.br_curves:
                    self.br_curves[tid].setData([], [])
            else:
                # — heartbeat waveform —
                if tid not in self.hr_wave_hist:
                    self.hr_wave_hist[tid] = deque(maxlen=WAVE_BUFFER)
                    red_pen = pg.mkPen('r', width=2)
                    c = self.hr_plot.plot(pen=red_pen, name=f"ID {tid}")
                    self.hr_curves[tid] = c

                # append the new 15‐point chunk
                self.hr_wave_hist[tid].extend(vs['heart_waveform'])
                yh = list(self.hr_wave_hist[tid])
                xh = list(range(len(yh)))
                self.hr_curves[tid].setData(xh, yh)

                # — breathing waveform —
                if tid not in self.br_wave_hist:
                    self.br_wave_hist[tid] = deque(maxlen=WAVE_BUFFER)
                    blue_pen = pg.mkPen('b', width=2)
                    c = self.br_plot.plot(pen=blue_pen, name=f"ID {tid}")
                    self.br_curves[tid] = c

                self.br_wave_hist[tid].extend(vs['breath_waveform'])
                yb = list(self.br_wave_hist[tid])
                xb = list(range(len(yb)))
                self.br_curves[tid].setData(xb, yb)

        # — Point cloud from older frame, colored by IDs from newer frame — (unchanged)
        cloud = older.get('cloud')
        ids   = newer.get('target_ids')
        if cloud is not None:
            x = -cloud[:,0]; y = cloud[:,1]
            if ids is not None and len(ids) == len(x):
                brushes = self._brush_list_for_ids(ids)
                self.cloud_scatter.setData(x=x, y=y, brush=brushes, size=4, pxMode=True)
            else:
                self.cloud_scatter.setData(x=x, y=y, size=4, pxMode=True)

def main():
    # 1) Configure the radar once on COM7
    cli, cfg_txt = load_config(CLI_PORT, CONFIG_FILE)
    limits = parse_xyz_limits(cfg_txt) or {"x_limits":(-2,2),"y_limits":(-2,2),"z_limits":(-2,2)}
    cli.close()                         # ← close COM7 immediately

    # 2) Now start the GUI & reader thread (only opens COM6)
    plotter = PlotWindow(limits)
    plotter.start()

if __name__ == '__main__':
    main()