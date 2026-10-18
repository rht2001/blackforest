# RH56DFTP digital twin and tactile contacts

The read-only 3D viewer uses the left Tactile Toolkit hand, its own URDF hierarchy,
and **1,062 individually colored sensing points across 17 sensor fields**.
It follows actual angle feedback and the tactile matrices already acquired by
Blackforest. The physical controller, register polling, pressure offsets, grasp
logic, publisher and localhost bridge are unchanged.

## Run

From the repository environment:

```powershell
uv run python digital_twin/bridge_server.py
```

Open [the 3D viewer](http://127.0.0.1:18765/). Keep the existing Blackforest hand
application running (`uv run python src/main.py`) for actual telemetry. Publishing
is enabled by default on this branch. The viewer starts in live mode. Manual
sliders and Motion demo affect the virtual hand only; the demo clearly labels
its artificial motion and localized pressure patterns.

The HTML is self-contained and can also be opened offline in a WebGL2 browser.
Drag to rotate, scroll to zoom, and use Reset view to restore the camera.
The existing [2D tactile view](http://127.0.0.1:18765/tactile) remains available.

The bridge accepts `POST /api/state` from the existing publisher and exposes
`GET /api/state` to the browser. It listens on localhost only, retains the newest
state in memory, and never opens a Modbus session or commands the hand. The
browser retains its single 300 ms polling loop.

## Sensor fields and wiring

| Region | Cells | Toolkit sensor link suffix |
| --- | ---: | --- |
| Each of four fingers: tip | 3 × 3 = 9 | `force_sensor_3` |
| Each of four fingers: nail/contact face | 12 × 8 = 96 | `force_sensor_2` |
| Each of four fingers: proximal pad | 10 × 8 = 80 | `force_sensor_1` |
| Thumb tip | 3 × 3 = 9 | `force_sensor_4` |
| Thumb nail/contact face | 12 × 8 = 96 | `force_sensor_3` |
| Thumb intermediate | 3 × 3 = 9 | `force_sensor_2` |
| Thumb pad | 12 × 8 = 96 | `force_sensor_1` |
| Palm | 8 × 14 = 112 | `palm_force_sensor` |

Four fingers contribute 740 points; the thumb 210; the palm 112. Every cell has
a stable `region:wire_index` ID. The transport matrices preserve acquisition order.
Fingers/thumb use ordinary row-major order. Palm acquisition runs bottom-to-top
within successive columns: wire 0 → row 7/column 0, wire 7 → row 0/column 0,
wire 8 → row 7/column 1, wire 111 → row 0/column 13 (all zero-based). The viewer
reads values by **wire index**, independently of their physical row/column.

Each point has its own GUI pressure color: green below 800, yellow from 800,
orange from 2000, red from 3200. Values are calibrated counts from
`max(0, raw - sensor_offset)`, not physical pressure units. Zero is green;
missing, invalid, disconnected or stale readings are gray. Force readouts in
grams remain separate. All cells remain visible even when no contact is active.

## Contacts and export

Activation threshold defaults to 200 counts. A cell is active when its value is
**strictly above** the threshold. Blue markers show the pressure-weighted centre
of the active points in each field. Changing the threshold changes selection
and centres without altering source readings or motor behavior.

**Export activated points** downloads `hand_contact_cloud.json`. The same
snapshot is available through `window.getContactCloud()`;
`window.getContactPoints()` returns its activated-point array.

The export includes:

* Point ID, sensor region/link, wire index, physical row/column, calibrated count,
  3D `position_m`, and unit `normal`.
* Common frame **`left_base_link`**, units **metres**, joint/actuator pose,
  bridge packet sequence and field reception timestamps, and simulation flag.
* Reference/geometry SHA-256 hashes, applied kinematic settings, and the explicit
  `APPROXIMATE_UNCALIBRATED` geometry status.
* Pressure-weighted field centres and their normals.

Contacts require fresh actual-angle and tactile fields, at most four seconds old,
whose bridge reception timestamps differ by no more than 500 ms. Stale, missing,
disconnected or skewed streams produce an invalid snapshot with no contact points.
These timestamps measure **bridge reception**, not hardware capture synchronization.
Missing individual regions omit only their contacts. Target angles never substitute
for actual feedback. A held pose can still show fresh cell pressure, but its
contacts are withheld when timing requirements fail.

This is a foundation for saving contact clouds and fitting object-shape hypotheses.
It does not infer an object or perform collision detection. Sparse contacts do not
describe uncontacted surfaces; mesh/point calibration and uncertainty estimates
are required before trusting shape or collision estimates.

## Geometry, frames and remaining physical validation

Each sensor field uses a regular grid within a minimum-area oriented rectangle
fitted to its own sensor STL, inset by 12%. The grid follows the sensor rectangle
edges, including rotations inside the local mesh frame. Positive-local-Z ray
intersections put all 1,062 centres on the mesh
surface, with no projection misses. Surface normals come from those triangles.
Row/column orientation follows the existing reference-view convention. This estimates
electrode locations from sensor-body geometry; it is not measured electrode calibration.

Full Toolkit FK composes parent-relative URDF origin translations, RPY rotations
(`Rz Ry Rx`) and local-axis joint rotations, in metres/radians. Cascaded thumb
mimics retain their actual drivers. Sensor points follow their own sensor-link
frames through the same FK. DEX and MuJoCo numeric frames are not applied.
The immutable original input reference remains for provenance; only Toolkit is active.

Display uses a separate `Ry(-pi/2)` presentation rotation, open-pose camera centering,
and conversion to renderer millimetres. Exports exclude camera transformations and
the 0.35 mm rendering lift used to avoid depth overlap. They transform the actual
approximate mesh-surface points into `left_base_link`. World/base alignment does not
change the exported base-frame coordinates. Previously saved valid Toolkit settings
can still load from browser storage; there is no geometry editor in the user flow.

Validate on the real hand: activate one taxel at a time to confirm field identity,
wire ordering and mirroring; measure electrode centres/active-area bounds; check
joint origins, angle endpoints, thumb coupling, and dynamic timing. Source angle
mapping remains an unmeasured candidate until checked experimentally.

Source assets come from AlexArutiunian/tactile-rh56dftp at the commit and blob hashes
recorded in `digital_twin/joint_asset_manifest.json`. The pinned Toolkit URDF is
retained under `digital_twin/profile_sources/`. The manifest retains the unresolved
upstream mesh redistribution-permission note; verify permission before publicly
redistributing the source meshes or their embedded geometry.

## Build and tests

Runtime needs only the self-contained HTML and a WebGL2 browser. Editable sources
are `toolkit_controls.js`, `joint_kinematics.js`, `joint_pressure.js`,
`tactile_points.js`, `joint_profiles_ui.js`, CSS and panel HTML. The canonical
reference JSON and asset manifest retain source provenance.

```powershell
uv run python digital_twin/rebuild_joint_viewer.py
uv run python -m unittest discover -s tests -v
node tests/test_joint_kinematics.js
node tests/test_joint_pressure.js
node tests/test_tactile_points.js
node tests/test_live_pressure_browser.js
```

The rebuild derives `tactile_geometry.json` deterministically from the embedded
source meshes and refuses changed reference hashes. NumPy is an existing project
dependency. Browser tests additionally require Playwright and an installed browser;
set `PLAYWRIGHT_PACKAGE` to its package directory if it is outside Node's normal
resolution path, and `TEST_BROWSER_CHANNEL` to select another installed channel.
Screenshots go to the OS temporary directory or `HAND_VIEWER_ARTIFACT_DIR`, so test
outputs do not pollute the repository.

Tests cover the HTTP publisher path, XML-vs-JavaScript FK, all sensor surfaces,
deterministic geometry hashes, exact point counts/wiring, individual-cell colors,
moving frames/normals, base-frame invariance, weighted centres, exports, thresholds,
stale/skewed data, disconnects and demos. No test connects to physical hardware.
# Shared taxel positions

The viewer shares the positions of its existing 1,062 model points. There is
no second kinematics implementation in Python. Each record contains only
`{id, x, y, z, pressure}`, in **left_base_link metres**, before display transforms
and marker lift. Pressure uses the twin's calibrated sensor counts (0..4095),
or `null` for missing/stale readings. Zero is a valid reading. IDs use
`region:wire_index`, matching existing sensor data.

Inside the twin use `window.getTaxelPositions()`. Other programs can request
GET `http://127.0.0.1:18765/api/taxels`, or use this helper from `src`:

```python
from taxel_coordinates import get_taxel_positions
points = get_taxel_positions()
```

From the repository root use `from src.taxel_coordinates import get_taxel_positions`.
The viewer sends the latest cached positions at most five times per second
when positions or pressure change, and once per second otherwise. The bridge only stores
and relays that list; `/api/state` and hardware control remain unchanged.
The viewer must be open. If no viewer publishes for four seconds, GET returns
`[]`. Reload the viewer and restart the bridge after updating.

These are the displayed model positions, including manual/demo and held poses;
they do not certify fresh hardware feedback. Existing contact-cloud functions
retain their pressure and feedback validity checks. Electrode placement and
joint mappings still need real-hand validation.

# Movable sensor objects

The **Sensor objects** controls display the repository's `documents/VEGABAR_29_STL.stl`
and `documents/VEGABAR_38.stl` as separate, colored objects. Select a sensor, change
X/Y/Z in millimetres or RX/RY/RZ in degrees, or hold **Shift and drag** in the scene
to translate the selected sensor in the camera plane. Ordinary dragging still
rotates the camera. Each sensor has its own visibility, reset, and focus controls;
placements survive a reload in this browser.

Position refers to the center of the STL bounding box, in the hand's
`left_base_link` frame. Rotation uses Rz(RZ) × Ry(RY) × Rx(RX), about that center.
`window.getSensorObjects()` returns the two placements in metres/radians.
STL coordinates are interpreted as millimetres (STL itself is unitless): source
extents are approximately 36 × 119 × 36 mm for VEGABAR 29 and 68 × 162 × 63 mm
for VEGABAR 38. Check these dimensions against the actual parts before using
them for metric contact estimation.

`build_sensor_objects.py` embeds indexed meshes with the original triangle
topology and source hashes when `rebuild_joint_viewer.py` runs. It explicitly
converts STL-local millimetres to centered mesh-local metres, then each object's
pose places it in the hand base before the common camera/display transform.
The objects are visual geometry for placement; physical collision response and
automatic contact/pressure simulation are future work.

Tests: `tests/test_sensor_objects.py` checks mesh provenance, dimensions, units,
triangles, and embedded data. `tests/test_sensor_objects_browser.js` checks
independent placement/rotation, visibility, persistence, shift-drag, reset, and
the existing hand/taxel behavior.

### Automatic thumb closure and grasp CSV

The automatic contact grasp closes the four fingers first. Once their motion
stops, it commands the standard thumb bend/rotation pose (234, 670), keeping
the finger targets closed. After the thumb has responded and remained stable
for 0.3 seconds (with the existing minimum closing delay), it queues one capture
and continues the existing grip verification. A failed thumb command or motion
timeout resets the hand without saving a completed closure.

Each completed automatic closure writes `grasp_captures/grasp_<UTC timestamp>_<id>.csv`
relative to the repository. Columns are exactly `id,x,y,z,pressure`, one row per
taxel (1062 rows). Coordinates remain in `left_base_link` metres; pressure is
calibrated sensor counts. Unavailable pressure is an empty CSV field, not zero.
Capture is independent of cylinder classification, so a completed closure can
be recorded even if subsequent grip verification fails. Manual slider and Close
button commands keep their existing behavior.

Keep the bridge and 3D viewer open. Restart the bridge and controller after this
update. CSV capture runs in a background worker and requires a viewer update
received after thumb settling (`GET /api/taxels?after=<Unix seconds>`). This
prevents reuse of an earlier cached pose; it is a viewer snapshot, not a
hardware-synchronized measurement. If no fresh update arrives within three
seconds, capture reports `CSV not saved` and leaves no empty CSV. Robot polling
continues even when the viewer, bridge, or output directory is unavailable.
CSV files are already excluded by the repository's `*.csv` ignore rule.

### View saved JSON captures

In the 3D viewer, open **Saved captures → Open contact JSON files** and select
one or more `hand_contact_cloud*.json` exports. Choose a capture in the list:

- **Hand + contacts** reconstructs its saved angles and calibration and shows
  the stored pressure points. Contacts are visible through the hand surface.
- **Point cloud only** hides the hand and objects and includes all 1,062 taxel
  positions when the saved pose is compatible. Stored active points keep their
  pressure colors; the other positions are reconstructed from the saved angles,
  calibration and matching geometry, and appear gray. Their pressure values
  were not stored and remain unknown. **Show non-active taxels** toggles them.
  Rotate and zoom normally; **Reset view** fits the visible points and axes.

Historical values do not expire. The saved filename, capture time and contact
count identify the active file. These exports contain activated points above
the export threshold, not all 1062 sensor readings. Missing pressure values
are not inferred. The threshold is fixed while replaying; the contact-centre checkbox
still works. Sensor objects are hidden because the export does not store their
poses. Import multiple files to compare captures, and use **Return to live**
(or the normal Live/Manual buttons) to exit replay. Reopen files after reloading
the viewer; files are held in memory for the current session.

Both saved views share an optional **Show coordinate frame** control. It shows
positive X (red), Y (green) and Z (blue) arrows at the JSON's `left_base_link`
origin `(0,0,0)`, with a length of 40 mm each. The axes use the same transform
as the points and hand, so switching views does not move or rotate the frame.
Reconstructed gray points require matching geometry and a valid saved pose;
incompatible files still show their stored contacts and coordinate frame.

Replay works offline, reads files locally, preserves their point coordinates,
and never publishes historical values to `/api/taxels`. Saved calibration is
used only for replay and never overwrites browser calibration. Files with a
matching profile, geometry and calibration support the hand view; otherwise
only their stored point cloud is available, with an explanation.

If the controller reports `CSV not saved: HTTP Error 404`, the bridge at the
configured address lacks the updated taxel endpoint (for example an older
bridge process still running). Stop that bridge, start this checkout's
`digital_twin/bridge_server.py` again and reload the 3D viewer. The controller
now reports this recovery instruction explicitly. A manually downloaded JSON
is separate from automatic CSV capture. The connection-reset traceback in a
controller log concerns the robot TCP connection, not JSON replay.
