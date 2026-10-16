# RH56DFTP Dexterous Hand — Quick API & Modbus Reference

AI- extracted API- Lookup/Cheatsheet.
Compact register and function lookup extracted from *Inspire-Robots RH56DFTP User Manual (V1.0.0)*.

---

## 1. Connection & Protocol Specifications

- **Protocol**: Modbus TCP / Modbus RTU
- **Default IP**: `192.168.123.211` (or `192.168.123.210`)
- **Default Port**: `6000`
- **Device ID / Slave Address**: `1`
- **Supported Function Codes**:
  - `0x03` (`Read Holding Registers`): Max 120–125 registers per query.
  - `0x06` (`Write Single Register`).
  - `0x10` / `16` (`Write Multiple Registers`): Max 120 registers.

### Degree of Freedom (DOF) Index Mapping
All 6-DOF register groups follow this indexing order:
| Index `m` | Finger / Joint | Description |
| :---: | :--- | :--- |
| `0` | Little Finger | Flexion / Bending |
| `1` | Ring Finger | Flexion / Bending |
| `2` | Middle Finger | Flexion / Bending |
| `3` | Index Finger | Flexion / Bending |
| `4` | Thumb Bend | Flexion / Bending |
| `5` | Thumb Rotate | Rotation / Opposition |

---

## 2. Motion Control Registers (Write / Read)

| Address | Register Name | Type | Unit / Range | Access | Description |
| :---: | :--- | :---: | :---: | :---: | :--- |
| `1486–1491` | `ANGLE_SET[0..5]` | 6 × `short` (int16) | `0 – 1000` (`-1` = hold) | W/R | **Joint Target Angle**.<br>`1000` = fully open, `0` = fully closed. |
| `1498–1503` | `FORCE_SET[0..5]` | 6 × `short` (int16) | `0 – 3000` (g) | W/R | **Force Control Threshold**.<br>Finger stops closing when contact force reaches this value. |
| `1522–1527` | `SPEED_SET[0..5]` | 6 × `short` (int16) | `0 – 1000` | W/R | **Joint Speed**.<br>`1000` = ~600 ms full stroke. |
| `1474–1479` | `POS_SET[0..5]` | 6 × `short` (int16) | `0 – 2000` (`-1` = hold) | W/R | Raw actuator stroke position setting. *(Use `ANGLE_SET` instead).* |

---

## 3. Real-Time Feedback Registers (Read-Only)

| Address | Register Name | Type | Unit / Range | Access | Description |
| :---: | :--- | :---: | :---: | :---: | :--- |
| `1546–1551` | `ANGLE_ACT[0..5]` | 6 × `short` (int16) | `0 – 1000` | R | **Current actual joint angle**. |
| `1582–1587` | `FORCE_ACT[0..5]` | 6 × `short` (int16) | `-4000 – 4000` (g) | R | **Current fingertip force**. |
| `1534–1539` | `POS_ACT[0..5]` | 6 × `short` (int16) | `0 – 2000` | R | Current linear actuator stroke position. |
| `1594–1599` | `CURRENT[0..5]` | 6 × `short` (int16) | `0 – 2000` (mA) | R | Motor current for each DOF. |
| `1606–1611` | `ERROR[0..5]` | 6 × `byte` | Bitfield | R | Actuator error flags (see bitfield table below). |
| `1612–1617` | `STATUS[0..5]` | 6 × `byte` | Status code | R | Actuator operational status. |
| `1618–1623` | `TEMP[0..5]` | 6 × `byte` | `0 – 100` (°C) | R | Actuator temperature. |

### Actuator Error Code Bitfield (`ERROR[m]`)
| Bit | Error Flag | Meaning | Clearable via `CLEAR_ERROR`? |
| :---: | :--- | :--- | :---: |
| `Bit 0` | Locked-rotor | Motor stall / physical obstruction | Yes |
| `Bit 1` | Over-temperature | Temperature exceeded thermal limit | **No** (clears when cooled) |
| `Bit 2` | Over-current | Current exceeded safe threshold | Yes |
| `Bit 3` | Abnormal motor | Drive circuit / motor malfunction | Yes |
| `Bit 4` | Communication | Internal bus communication fault | Yes |

---

## 4. Tactile Sensor Matrix Mapping (Read-Only)

All tactile points are **16-bit unsigned integers** (`0 – 4095`), Little-Endian.
Because individual read requests cannot exceed ~120 registers, read sensor groups in chunks of $\le 120$.

| Region | Start Addr | End Addr | Dimensions ($H \times W$) | Total Points | Register Count | Order |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **Little Tip** | `3000` | `3008` | $3 \times 3$ | 9 | 9 | Row-major |
| **Little Nail** | `3018` | `3113` | $12 \times 8$ | 96 | 96 | Row-major |
| **Little Pad** | `3210` | `3289` | $10 \times 8$ | 80 | 80 | Row-major |
| **Ring Tip** | `3370` | `3378` | $3 \times 3$ | 9 | 9 | Row-major |
| **Ring Nail** | `3388` | `3483` | $12 \times 8$ | 96 | 96 | Row-major |
| **Ring Pad** | `3580` | `3659` | $10 \times 8$ | 80 | 80 | Row-major |
| **Middle Tip** | `3740` | `3748` | $3 \times 3$ | 9 | 9 | Row-major |
| **Middle Nail** | `3758` | `3853` | $12 \times 8$ | 96 | 96 | Row-major |
| **Middle Pad** | `3950` | `4029` | $10 \times 8$ | 80 | 80 | Row-major |
| **Index Tip** | `4110` | `4118` | $3 \times 3$ | 9 | 9 | Row-major |
| **Index Nail** | `4128` | `4223` | $12 \times 8$ | 96 | 96 | Row-major |
| **Index Pad** | `4320` | `4399` | $10 \times 8$ | 80 | 80 | Row-major |
| **Thumb Tip** | `4480` | `4488` | $3 \times 3$ | 9 | 9 | Row-major |
| **Thumb Nail** | `4498` | `4593` | $12 \times 8$ | 96 | 96 | Row-major |
| **Thumb Mid** | `4690` | `4698` | $3 \times 3$ | 9 | 9 | Row-major |
| **Thumb Pad** | `4708` | `4803` | $12 \times 8$ | 96 | 96 | Row-major |
| **Palm** | `4900` | `5011` | $8 \times 14$ | 112 | 112 | Col-major (inverted: $R_8 \to R_1$) |

> **Note on Palm Indexing**: Unlike finger matrices which map row-major ($R_1C_1, R_1C_2, \dots$), the palm array stores column-first bottom-up ($R_8C_1, R_7C_1, \dots, R_1C_1, R_8C_2, \dots$).

---

## 5. System & Calibration Commands

| Address | Parameter | Value | Action |
| :---: | :--- | :---: | :--- |
| `1004` | `CLEAR_ERROR` | `1` | Clears active clearable motor errors. |
| `1005` | `SAVE` | `1` | Saves current settings to internal Flash memory. |
| `1006` | `RESET_PARA` | `1` | Restores factory default settings. |
| `1009` | `GESTURE_FORCE_CLB` | `1` | **Calibrates force sensors**. Hand must be fully open without contacting objects. |
| `1000` | `HAND_ID` | `1 – 254` | Modbus Device ID (default `1`). |

---

## 6. Python `pymodbus` Quick Code Snippets

### Read Actual Joint Angles
```python
from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient("192.168.123.211", port=6000)
client.connect()

res = client.read_holding_registers(address=1546, count=6, device_id=1)
if not res.isError():
    angles = res.registers  # [Little, Ring, Middle, Index, ThumbBend, ThumbRot]
```

### Command Finger Angles & Force Threshold
```python
# Set force threshold to 600g for all fingers
client.write_registers(address=1498, values=[600] * 6, device_id=1)

# Close fingers (0 = closed, 1000 = open)
target_angles = [0, 0, 0, 0, 300, 500]
client.write_registers(address=1486, values=target_angles, device_id=1)
```

### Safe Tactile Matrix Read Helper
```python
def read_tactile_patch(client: ModbusTcpClient, addr: int, count: int) -> list[int]:
    values = []
    for offset in range(0, count, 120):
        chunk = min(120, count - offset)
        res = client.read_holding_registers(address=addr + offset, count=chunk, device_id=1)
        if not res.isError():
            values.extend(res.registers)
        else:
            return []
    return values
```