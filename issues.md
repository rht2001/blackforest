# Technical Backlog & Major Architecture Issues (Post-Hackathon / Phase 4)

Issues and architectural enhancements deferred to preserve velocity during the 24h hackathon sprint:

---

## 1. Threaded / Asynchronous Modbus Network Engine
- **Current State**: Modbus TCP read/write queries run synchronously on the main Python thread.
- **Problem**: Reading 17 tactile arrays takes ~30–50ms. Any temporary network latency or packet retransmission blocks the Tkinter GUI thread, causing occasional UI stutter.
- **Proposed Solution**: 
  - Offload Modbus telemetry polling to a dedicated daemon background worker thread (`threading.Thread`).
  - Worker continuously polls angles, forces, and tactile matrices into a thread-safe `PhysicalState` queue.
  - UI simply reads the latest cached `PhysicalState` snapshot at 60 FPS without ever touching the network socket directly.

---

## 2. Formal Grasp State-Machine
- **Current State**: State transitions currently rely on loose boolean flags (`self.auto_mode_active`, `self.auto_contact_trigerred`, `self.state = "initialised"`).
- **Problem**: Edge cases (e.g. partial contact, slip events, re-grasping) become difficult to reason about and trigger race conditions.
- **Proposed Solution**:
  - Implement a formal finite state machine (FSM):
    - `IDLE` $\to$ `APPROACH` $\to$ `CLOSING_UNTIL_CONTACT` $\to$ `STEADY_GRASP` $\to$ `SLIP_ALERT` $\to$ `RELEASE`.
  - Transitions explicitly documented with guard conditions (e.g. force threshold exceeded, CoP displacement $> \Delta d$).

---

## 3. 3D Spatial Forward Kinematics & Pose Transformation
- **Current State**: Contact points are tracked on 2D local tactile sensor arrays.
- **Problem**: Cannot compute 6-DoF Cartesian object pose $(x, y, z, \text{roll}, \text{pitch}, \text{yaw})$ for downstream robotic arm placement without link transformations.
- **Proposed Solution**:
  - Implement forward kinematic (FK) transformation matrices for the 5 fingers using joint angles from `ANGLE_ACT`.
  - Project local sensor array CoPs into hand palm base coordinates $\mathbf{P}_{\text{base}} = \mathbf{T}_{\text{finger}}(\theta) \cdot \mathbf{P}_{\text{local\_cop}}$.

---

## 4. Full Pytest & Mock CI Suite
- **Current State**: Lightweight verification scripts executed manually via CLI.
- **Proposed Solution**:
  - Add standard `tests/` directory with automated `pytest` test suite.
  - Test protocol codecs, synthetic slip generators, and position file persistence in CI.
