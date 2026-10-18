# One-page tactile identification

Start **Start Sensor Identification.cmd** from this Desktop repository. The UI
opens at http://127.0.0.1:18765/. Everything happens on that page.

1. Choose **Manual**, **Live**, or **History**.
2. Inspect the displayed hand and its data.
3. Press **Estimate**. A frozen copy of that source's points is sent to the
   geometric matcher. The point cloud and fitted geometries appear in the same
   page. **Hand view** returns to the hand without changing the selected data.

**Manual:** set the six angles under **Set finger angles**. These define taxel
positions, not physical contacts. Manual cells have unknown pressure and appear
gray in the estimate. With no contact readings, AMBIGUOUS is the correct result.
No artificial pressure is created from angles.

**Live:** displays actual hand angles and pressure through the existing bridge
address, normally port 18765. Stale or unsynchronized readings cannot be
estimated. No new hardware reads or motor commands are introduced.

**History:** load native `hand_contact_cloud.json` files or telemetry CSV files.
CSV supports semicolon/comma delimiters, quoted multiline fields, JSON arrays
and complete NumPy array strings. It needs `angles` (six actual readings) and,
for pressure evidence, `tactile_matrix` with exactly 112 rows and 17 columns.
The supplied `index;angles;forces;position;tactile_matrix;...` format is supported.
Choose **Whole probing run**, select a run, and press **Build run point cloud**.
Each row's actual six angles drive forward kinematics for that row's taxels;
contacts are collected across `thumb_probing` and `probing_complete` samples.
The default is thumb-only contact above the Estimation settings threshold (200).
Rows reporting slip are skipped. No means or actuator forces substitute for taxels.
Choose All hand regions to include other fingers and palm as supporting contacts.
The displayed cloud is the exact frozen cloud that **Estimate** receives.
**Save run point cloud JSON** exports the estimator frame for later reuse.

Repeated observations in each 1 mm spatial voxel contribute one observed point
(the maximum-pressure contact), avoiding extra fit weight from a long hold.
Point spacing is adjustable from 0.1 to 10 mm. Counts before and after reduction
are retained. Fewer than eight distinct contacts or insufficient spatial coverage
still produce an ambiguous estimate; accumulation does not invent evidence.

Run IDs, idle states, and discontinuities in source indices keep grasps separate.
The flat `thumb_frames.csv` is supported with six `angle_*_raw` columns and the
210 thumb taxel columns `thumb_tip_000` through `thumb_pad_095` for their respective
regions. Original 112 × 17 matrices retain all hand regions. Combined run files
show a run selector. Use **Single row** for Previous/Next pose inspection.
Files remain local; the limit is 256 MB. Larger recordings should be split into
individual run files. A cloud may contain up to 20,000 spatial contacts.
Truncated arrays (`...`) are rejected. Means cannot replace per-cell pressure.
Missing/invalid pressure stays unknown. CSV row time is not invented, and CSV
counts are shown as recorded; pressure offsets and failed-read validity are not
supplied by that format. A CSV pose uses the current viewer's explicitly
uncalibrated kinematic model. JSON uses its recorded pose/calibration.

The saved-file hand/point-cloud controls, axes, sensor objects, pressure display
and connection options remain accessible. The old source controls are grouped
under **Advanced display & connection** rather than mixed with the main modes.

Estimate captures all available cell positions. Recognition uses only valid
pressure strictly above its threshold; known inactive cells with reconstructed outward normals provide local penetration limits.
They do not demand a positive clearance or prove empty space. Gray points are positions with unknown pressure.
For native JSON containing active points only, other positions are reconstructed
from the recorded pose only when its matching hand geometry is available;
their pressures remain unknown. Clouds without a reproducible hand pose retain
only their recorded points.

Both CAD surfaces receive the same multistart rigid alignment budget, followed
by exact unsigned triangle refinement. No scaling is allowed. A and B fit well,
neither fits within the search, insufficient coverage, calibration uncertainty
and an unreliable alignment have different ambiguity reasons. Scores are costs,
not probabilities; local search does not prove global infeasibility.

Hand geometry is still uncalibrated, so measured clouds can produce scores but
not a confident sensor label through this UI. Shared shaft regions may remain
indistinguishable even after calibration. A's STEP confirms millimetres; B's
physical scale, taxel locations, joint mapping and timing still need validation.

Accumulated coordinates assume the object stays stationary relative to the hand
base. CSV does not establish this assumption or provide object tracking. Separate
grasps need object-frame registration before their clouds can be merged.
Run accumulation keeps oriented taxel and hand-surface limits from every valid
probing/hold pose, including samples without active pressure. Purple points show
hand surfaces, blue points show recorded inactive taxels, and gray points show
surfaces whose pressure is unknown. Unknown values never become zero readings.
The last saved pose is used only for displaying the hand; it does not replace
limits collected at earlier poses. Single-row/live behavior remains available.

Contact points and space limits are separate arrays. `boundary_points` is passed
to the same local oriented-surface penetration checks used for individual poses.
Without active contacts the run still loads, displays and exports its limits;
Estimate returns `space_constraints_only`, with an ambiguous/unanchored object
pose rather than inventing a surface match. Limits alone do not identify the
object or define a bounded global object location.

Spatial reduction retains evidence type and normal direction, so opposing
surfaces are not merged. Limits start at 2 mm spacing (or the contact spacing if
larger), coarsening when necessary to remain within 4096 representative limits.
`accumulation` records the final spacing and observation counts. This is a reduced
oriented surface sweep, not a watertight swept-volume collision certificate.

The offline command uses the same reconstruction as the UI:

```powershell
node digital_twin/sensor_identification/build_run_cloud.cjs run_002_47589.csv --output thumb_cloud.json
node digital_twin/sensor_identification/build_run_cloud.cjs thumb_frames.csv --list
node digital_twin/sensor_identification/build_run_cloud.cjs thumb_frames.csv --run run_002 --calibration saved_contact_reference.json --output thumb_cloud.json
```

It writes an estimator frame (`units: m`, `frame: left_base_link`), a contact
XYZ/pressure CSV, and a `.limits.csv` with normals, evidence roles and source indices. Without `--calibration`, it explicitly uses the current uncalibrated
profile defaults. A native contact JSON can supply `kinematics_calibration`; geometry
fingerprints must match. Historical timing and read validity remain unverified.

The report is embedded as the page's estimate view; it does not open another tab.
Its header records which source was frozen. Changing modes while fitting does
not alter that snapshot. The independent CLI and CAD benchmark tests remain.

```powershell
uv sync --extra sensor-identification
uv run --extra sensor-identification python digital_twin/start_sensor_simulation.py
node tests/test_identification_history.js
node tests/test_identification_runs.js
node tests/test_identification_measurements.js
uv run --extra sensor-identification python -m unittest discover -s tests -p test_identification_service.py -v
```


Purple hand boundary points fill palmar gaps between sensing points. The 379
mesh samples have at least 5 mm separation in the open sampling pose and a
3 mm halo around taxels. Positions/normals live in each source link frame and
follow the same FK and saved calibration as the hand. Articulation changes
world spacing. Show hand boundary points toggles their display only.

Frozen estimate frames include `boundary_points` separately from pressure
measurements, in the same base-link metre frame. Known inactive taxels with
normals and these samples penalize inward nearest CAD surfaces in coarse
ranking and exact triangle refinement. A 4 mm local tangent weighting avoids
extending a finger plane across the grasp; the selected tolerance permits
small penetration. Scores report hand-limit RMS and maximum residuals; a
maximum above tolerance makes a fit incompatible. This is an approximate
oriented surface constraint, not watertight solid collision detection: the
supplied CAD assemblies are open meshes and sparse points can miss crossings.
Unknown taxel pressures impose no additional limit. Saved clouds without a
reproducible hand pose receive no invented hand limits. Boundary points never
become pressure evidence and cannot identify an object without active contacts.


Strict fitting and hand overlay
-------------------------------
Every valid above-threshold contact must lie within the selected position tolerance
of an exact STL triangle. Average error cannot excuse an outlier. Certified
clearance is also checked by its maximum violation. Feasible refinements are
preferred to lower-scoring infeasible poses.

All supplied taxel positions with outward normals impose geometric hand limits,
even when pressure is unknown. Unknown pressure is never invented as a contact
or non-contact reading. After refinement, every candidate triangle is audited
against 8 by 8 mm tangent patches extending inward from the taxels and purple
hand points. This catches crossings and enclosing shapes hidden by unsigned
nearest-surface distance. The maximum allowed inward depth is independently
0.25 mm; increasing contact tolerance never increases this allowance. These
local patches are a conservative surface approximation, not a watertight hand
collision certificate; unsampled gaps and inaccurate calibration remain limitations.

Compatible poses appear in the hand scene using the returned STL-mm to base-mm
transform and the captured hand pose/calibration. The UI pauses live replay
while displaying that frozen snapshot. Both compatible candidates can be
selected and remain provisional when identification is ambiguous. Rejected
poses appear only on request in Fit report. Clouds without a reproducible hand
pose remain report-only. CSV runs store the last contact-bearing hand pose;
the overlay and hand-limit checks use that pose, while contact checks use all
accumulated run contacts. This does not check collision at every earlier pose.
