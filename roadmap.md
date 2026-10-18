# Roadmap
- started discussion and planned roadmap for next steps
- Atomic planning, only have checkable items
- Keep **OOP** best practices and stick to **KIS**
- Stick to english code and annotations

## 1. Grab the Object
- [x] tactile input as trigger to grab the object/sensor
- [x] close the hand until full force threshold (chapter 2.6.12 in doc, "FORCE_SET")
- [x] confirm grip by polling forces (did they reach threshold? - Not sure, wether this would belong to 2.?)

## 2. Feedback acquisition and analysis
- [x] ensure "steady state"
- [x] fetch all available sensor data from robotic hand and store in ```@dataclass PhysicalState```
- [ ] confirm grip of cylindrical object: ```def confirm_grip(PhysicalState) == True```
  - Planned logic: DOF angle homogeneity across fingers 0-3 (Little, Ring, Middle, Index) with tolerance check, plus active contact pad threshold.
- [ ] fetch ```PhysicalState``` again (e.g. ```PhysicalState.update()```, as we need this function anyways for polling loop of movement detection). -> set new steady state.

## 2.5 Detect sensor type and orientation by probing thumb
- [x] Detect features with thumb to decide on type of object/sensor (Touch-and-retract low-force raster routine with contact coordinate recording)

## 3. Activate movement/Slip detection
- [x] continuously fetch ```PhysicalState.update()``` to detect movements/slip
- [x] Time, or Force Threshold (so e.g. vibrations are not detected)

---

## Branching Task: Ultra-Lean Dummy Hand API (For UI Testing)
- [x] Mock class adhering to same interface signatures (`read_angles`, `write_angles`, `read_tactile_matrix`) # though reading not returning realistic values
- [ ] Return synthetic contact patches when fingers close past arbitrary threshold
- [ ] Toggleable slip simulation trigger for testing UI alerts

---

## 4. Perspective & Pitch: 3D Pose Estimation & Placement Repeatability (Proposal)
- [ ] **Forward Kinematics**: Project 2D sensor Centers of Pressure (CoP) into 3D hand Cartesian coordinates using finger joint angles.
- [ ] **Object Pose Fitting**: Fit a cylindrical axis / centroid between opposing contact pads (fingers vs. thumb).
- [ ] **Repeatable Placement Delta**: Compute $\Delta \mathbf{P} = (\Delta x, \Delta y, \Delta z)$ and $\Delta \mathbf{R}$ relative to a reference target grasp to guide robotic arm placement.

---

## Mockup & Design Specs (For Discussion)

