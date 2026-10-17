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
