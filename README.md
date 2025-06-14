# Vital‑Signs Radar GUI

A **Python + PyQtGraph** application that drives a TI IWR6843ISK *Vital‑Signs* radar, decodes its binary frames in real time, and visualises cardio‑respiratory metrics live.  
It also maps each tracked target to an 8 × 8 LED grid via an attached Arduino.

---

## 1. High‑level purpose

```text
    PC (Python GUI)
     │
     │ 115 200 baud  ──►  Radar (CLI port)     ← one‑time config file
     │
     │ 921 600 baud  ◄──  Radar (DATA port)    ← continuous frames
     │
     │ 115 200 baud  ──►  Arduino              ← 8×8 grid indices
```

1. **Config phase:** `load_config()` streams `vital_signs_ISK.cfg` to the radar’s CLI port.  
2. **Acquisition phase:** a QThread reads frames at 921 600 baud, the GUI plots the previous frame (one‑frame latency) and sends the quantised (x,y) grid coordinates to an Arduino.

---

## 2. Runtime architecture

| Layer | Role |
|-------|------|
| `load_config()` | Replays CLI configuration file |
| `SerialReader`  | Background thread; yields decoded frames |
| `PlotWindow`    | PyQtGraph GUI; plots HR/BR waveforms and point‑cloud |
| Arduino link    | Receives “gx gy …” grid indices @115 200 baud |

---

## 3. Key constants

| Name | Purpose |
|------|---------|
| `MAGIC_WORD` | 8‑byte sync pattern at start of every radar frame |
| `FRAME_HEADER_LENGTH` / `TLV_HEADER_LENGTH` | Byte lengths for frame / TLV headers |
| `WAVE_BUFFER = 60` | GUI retains 60 samples per waveform (≈ 4 s) |

---

## 4. TLV parsers

| Function | Decodes | Returns |
|----------|---------|---------|
| `pointCloud()` | TLV 1020 | **N × 5** `[[x,y,z,doppler,snr]…]` |
| `targetList()` | TLV 1010 | Object‑list with pos/vel/acc |
| `vitalSigns()` | TLV 1040 | Dict: HR, BR, 15‑sample waveforms |

`_VITALS` struct layout:

```text
uint16 id, uint16 rangeBin,
float  breathingDeviation,
float  heartRate, float breathingRate,
float  heartWave[15], float breathWave[15]
```

---

## 5. Frame‑decoding pipeline

```python
for frame, ntlvs in record_frames(port):
    res = process_frame(frame, ntlvs)
```

1. **`record_frames()`** buffers serial bytes, locates `MAGIC_WORD`, yields entire frames.  
2. **`process_frame()`** walks the TLVs, dispatches by type, returns a dict containing only the TLVs present.  
3. **GUI update** uses `(older, newer)` frames so point colours (which arrive one TLV later) match the cloud they belong to.

---

## 6. GUI widgets

| Widget | Role |
|--------|------|
| **Heart Waveform** | Rolling 60‑sample deque per target (red) |
| **Breath Waveform** | Rolling 60‑sample deque per target (blue) |
| **Point Cloud** | Top‑down scatter, coloured by target‑ID |
| **Info Panel** | Large HR / BR readout + presence |
| **Arduino out** | Sends “gx gy …” grid line each refresh |

Brush palette: 11 evenly spaced HSV hues (alpha ≈0.78); fallback grey for unknown IDs.

---

## 7. Coordinate → grid mapping

```python
gx = map_to_grid(posX, xmin, xmax, 8)   # 1…8
gy = map_to_grid(posY, ymin, ymax, 8)
```

Linear clamp‑&‑scale to inclusive 1‑to‑8 integer per axis; uses `boundaryBox` from the radar config.

---

## 8. Extending / maintaining

| Task | Where |
|------|-------|
| Change COM ports / baud | Edit constants at top of file |
| Support new TLV | Add parser + `elif` in `process_frame()` |
| Change grid resolution | `map_to_grid(grid_size=…)` (and Arduino firmware) |
| Alter point‑cloud limits | Change `boundaryBox` in cfg or override in code |

For more information about this, refer to TI's instruction for IWR6843 Vital Sign with people tracking: https://dev.ti.com/tirex/explore/node?node=A__AEDHIkNCuNhTPfENyjdNtw__radar_toolbox__1AslXXD__LATEST