# Roadmap
- started discussion and planned roadmap for next steps
- Atomic planning, only have checkable items
- Keep **OOP** best practices and stick to **KIS**
- Stick to english code and annotations

## 1. Grab the Object
- [ ] tactile input as trigger to grab the object/sensor
- [ ] close the hand until full force threshold (chapter 2.6.12 in doc, "FORCE_SET")
- [ ] confirm grip by polling forces (did they reach threshold? - Not sure, wether this would belong to 2.?)

## 2. Feedback acquisition and analysis
- [ ] ensure "steady state"
- [ ] fetch all available sensor data from robotic hand and store in ```@dataclass PhysicalState```
- [ ] confirm grip of cylindrical object: ```def confirm_grip(PhysicalState) == True```
  - Planned logic: DOF angle homogeneity across fingers 0-3 (Little, Ring, Middle, Index) with tolerance check, plus active contact pad threshold.
- [ ] Detect features with thumb to decide on type of object/sensor
- [ ] fetch ```PhysicalState``` again (e.g. ```PhysicalState.update()```, as we need this function anyways for polling loop of movement detection). -> set new steady state.

## 3. Activate movement/Slip detection
- [ ] continuously fetch ```PhysicalState.update()``` to detect movements/slip
- [ ] Time, or Force Threshold (so e.g. vibrations are not detected)

---

## Branching Task: Ultra-Lean Dummy Hand API (For UI Testing)
- [ ] Mock class adhering to same interface signatures (`read_angles`, `write_angles`, `read_tactile_matrix`)
- [ ] Interpolate angles toward targets without physics simulation
- [ ] Return synthetic contact patches when fingers close past arbitrary threshold
- [ ] Toggleable slip simulation trigger for testing UI alerts

---

## 4. Perspective & Pitch: 3D Pose Estimation & Placement Repeatability (Proposal)
- [ ] **Forward Kinematics**: Project 2D sensor Centers of Pressure (CoP) into 3D hand Cartesian coordinates using finger joint angles.
- [ ] **Object Pose Fitting**: Fit a cylindrical axis / centroid between opposing contact pads (fingers vs. thumb).
- [ ] **Repeatable Placement Delta**: Compute $\Delta \mathbf{P} = (\Delta x, \Delta y, \Delta z)$ and $\Delta \mathbf{R}$ relative to a reference target grasp to guide robotic arm placement.

---

## Mockup & Design Specs (For Discussion)

### `confirm_grip(PhysicalState)` Specification Mockup
```python
def confirm_grip(state: PhysicalState, angle_tolerance: int = 100, min_contact_fingers: int = 3) -> tuple[bool, str]:
    """
    Evaluates whether a stable cylindrical grasp has occurred.
    Returns (is_confirmed, diagnostic_reason).
    """
    finger_angles = state.joint_angles[0:4]  # Little, Ring, Middle, Index
    
    # 1. Check if fingers are not fully closed (empty hand) and not fully open
    if any(a < 50 for a in finger_angles):
        return False, "Grasp missed: One or more fingers closed completely (empty grasp)."
    if all(a > 950 for a in finger_angles):
        return False, "Hand still open: No contact detected."

    # 2. Homogeneity check across the 4 primary fingers
    spread = max(finger_angles) - min(finger_angles)
    if spread > angle_tolerance:
        return False, f"Non-homogeneous grasp: Angle spread {spread} exceeds tolerance {angle_tolerance}."

    # 3. Tactile contact check (sanity check on pad pressures)
    active_pads = sum(
        1 for name in ["Little Pad", "Ring Pad", "Middle Pad", "Index Pad"]
        if state.sensors.get(name) and state.sensors[name].total_force > 100
    )
    if active_pads < min_contact_fingers:
        return False, f"Insufficient tactile contact: Only {active_pads}/{min_contact_fingers} pads engaged."

    return True, "Stable cylindrical grasp confirmed."
```

# Open Questions & Important Thoughts
- Generally ensure that movement detection is turned off before e.g. feature detection!
