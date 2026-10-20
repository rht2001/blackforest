# One-page tactile identification

## Recorded grasp starting poses

For the original two CAD exports in `left_base_link`, Estimate starts near the
common poses derived from 53 recordings. Bounding-box centers (mm), followed by
XYZ Euler angles (degrees, Rz Ã— Ry Ã— Rx):

- VEGABAR 29: (20.01, 23.57, 129.25), (-3.18, -2.77, 91.19).
- VEGABAR 38: (13.35, 35.86, 128.96), (-179.58, -3.30, 90.44).

Quick placement adjusts about these centers within Â±(30, 12, 15) mm and permits
Â±30 degrees of local X/Z tilt and Â±180 degrees of shaft roll (local Y).
These are search windows, not measured confidence intervals. Every optimization
stage keeps the center within that window. Detailed validation also refines the
empirical seed, while retaining the wider multistart search for unusual grasps.
Only the matching source SHA256 and explicit base frame enable this prior;
arbitrary-frame inputs and other CAD retain their existing initialization.
Contact/clearance and full hand triangle checks still decide compatibility.
The prior never establishes identity or overrides the calibration gate. Empty
or insufficient-contact clouds retain their existing ambiguity result.


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
regions. Original 112 Ã— 17 matrices retain all hand regions. Combined run files
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
by sampled local-surface refinement and optional full-mesh validation. No scaling is allowed. A and B fit well,
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
3 mm (adjustable hand penetration allowance); increasing contact tolerance never increases this allowance. These
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


### Lightweight run clouds: deepest bend at every rotation

For each distinct recorded thumb rotation reading, reconstruction keeps the
sample with maximum calibrated thumb bend. Equal bend maxima at the same
rotation use the last sample. It combines these selected poses into an area
of contact points and oriented hand-space limits, instead of collapsing the
entire run to one deepest pose. No rotation bins or interpolated poses are
introduced. Source order is preserved.

Inactive taxels and hand surfaces are retained even if no contact activates.
`accumulation.selected_poses` records rotation, bend and source index for every
selected pose; `frames_skipped_pose` counts discarded intermediate bend frames.
Reload the original CSV to rebuild a cloud with this selection. Existing JSON
clouds are unchanged. The API's `pose_selection: 'all'` retains the full sweep
when explicitly needed; the UI and CLI default to one bend peak per rotation.


### Palm-guided initialization

When explicitly labelled rigid-palm samples are available, fitting starts with
a longitudinal grasp along the dominant palmar surface. Both axial directions,
two roll orientations and three axial offsets receive the same search budget
for each candidate. The first stage adjusts three translations and a turn about
the palm normal (four DOF). Sampled surface refinement releases tilt (six rigid DOF).
This is a geometric typical-grasp assumption, not a learned most-common pose or
a requirement that the object touches the palm. No scale or hand articulation
is fitted. No palm is inferred from arbitrary or finger-only clouds.

Hand penetration residuals, including the worst local violation, now guide
coarse pose correction and sampled surface refinement. One refinement slot is reserved
for the best palm start; general multistart alignment remains available. The
final exact triangle audit and strict contact/penetration gates still apply.
Result metadata records the selected initialization and palm-start count.


## Passive probing capture

The normal local server records incoming probing telemetry without starting a
probe or reading hardware. Received probing runs appear on the main page.
Capture starts on the first received `thumb_probing` sample and ends on
`probing_complete` or a return to `cylinder_confirmed`. Other states interrupt
the run; 15 seconds without state-bearing telemetry also interrupts it.
Interrupted runs can be opened explicitly but never auto-open as successful.
The receiver may join midway through a probe; it cannot verify that it saw the
physical start. Acquisition times and angle/pressure skew remain unverified.

Completed runs automatically build a point-cloud preview using the existing
deepest-bend-per-rotation reconstruction. Estimation still requires pressing
Estimate. The automatic-open checkbox can be disabled. Captures persist across
page reloads and receiver restarts; active runs at restart become interrupted.

CSV and metadata are saved under
`~/Documents/ChatGPT/hand/probing_situations/received_runs/` by default.
Set `HAND_CAPTURE_DIR` on the receiver to override this folder. Capture is
limited to 128 MiB per run; hitting the limit marks it interrupted. No recordings
are automatically deleted. `GET /api/probing-runs` lists the latest 50 records;
`GET /api/probing-runs/<id>` returns a finished run's CSV.

The sender must POST its state together with six `angles_act` readings and
`tactile_regions` matrices to `http://127.0.0.1:18765/api/state`. Missing regions
remain unknown; partial packets never borrow cached angles or tactile readings.
Optional `source_id` identifies one sender session, `sample_index` is a strictly
increasing integer, and `slip_detected` is boolean. Only one source owns an active
capture. The bundled `logic.twin_publisher` now forwards state/slip metadata and
adds session/sample IDs; the sender process must load this updated code.
The publisher's one-pending-frame policy can drop samples under load. Source
sequence gaps and invalid skipped samples are reported in the capture metadata
and cloud export; receiver capture does not promise every hardware sample.
Scripts that send only angles/pressure need to add state metadata before
automatic capture can work. The receiver remains bound to localhost; a script
on another computer needs an explicit forwarding connection to this PC.


When another computer runs the probing controller, the hand exposes its sensor
readings but not that controller's software state. Use **Start capture** before
their probe and **Finish capture** afterward if the local telemetry has no
reliable probing states. These controls only open/close a receiver recording;
they never send a hardware command. POST `/api/probing-runs` with
`{"action":"start"}` or `{"action":"finish"}` is the equivalent local API.
Manual captures use an explicit capture window for reconstruction, retain the
incoming software state in `reported_state`, and export `capture_mode: manual`.
A manual capture binds to the first sender delivering actual angles and tactile
regions. It stays armed until those readings arrive, then the 15-second
telemetry timeout applies. Local software states do not stop a manual capture.


**Record when fingers close; finish when hand opens** is enabled by default.
After observing all four primary fingers at raw angles >=850, the receiver arms.
At least two primary fingers moving below 800 starts a finger-motion window,
including up to ten preceding coherent samples from that same sender. All four
primary fingers returning to >=850 for three consecutive coherent samples ends
the window. This uses actual angles, not commanded angles, and follows the
existing raw convention (1000=open). A timeout or receiver restart interrupts
the window. If the viewer joins while the hand is closed, it waits for an open
pose; Start capture remains available. Finish capture can end the window before
the hand reopens. Captures export `capture_mode: closure` and the trigger values.
This window includes closing/holding/opening rather than identifying exact
remote probing boundaries. Incoming states are preserved as `reported_state`.
The checkbox controls receiver detection only and sends no hand commands.
POST `/api/probing-runs` with `{"action":"auto","enabled":false}` disables
motion detection for the current receiver session without discarding a capture.


### Larger sensor first

Candidates are tried in descending CAD bounding-envelope volume (open assembly
STLs do not have a reliable solid volume). Quick placement keeps the larger
sensor if every contact and hand constraint meets its existing tolerance;
otherwise it tries the smaller sensor. Any hand penetration over 3 mm (adjustable hand penetration allowance) rejects
a pose; collision counts report violated local hand constraints, not solid
intersection events. A contact or certified-clearance failure also triggers
fallback. If neither fits, the viewer automatically shows the rejected pose with the
lowest fit score. Both rejected candidates remain selectable, with a visible
warning and their contact/collision diagnostics. Displaying a rejected pose
does not make it compatible or confirm its identity.
Detailed validation still fits both candidates and preserves the identity and
ambiguity gates, while the viewer prefers the larger compatible placement.
Size preference is a placement prior, not proof of sensor identity.

### Quick placement (viewer default)

The viewer defaults to Quick placement and a 3 mm position tolerance. This is
a provisional visual pose estimate, not a validated sensor classification.
Estimation settings also offers Detailed validation. API/CLI configurations
retain detailed validation unless `quick_preview: true` is explicitly supplied.

Each cached CAD model has 24,000 deterministic surface samples and a KD tree.
Refinement uses the eight nearest samples to find nearby STL triangles and
projects onto those local surfaces. Ordinary refinement uses local queries;
optional detailed contact-repair rounds query exact triangles for the worst
violations. Final validation uses the full mesh for contact/clearance scoring and triangle
penetration auditing. Quick placement also uses exact final contact queries.
Exact queries reuse the cached sample KD-tree and triangle R-tree: the distance
to an on-surface sample bounds the complete set of possibly nearer triangles.
Triangle projections are batched with bounded working arrays; no watertight
mesh, signed distance or additional dependency is required.

Quick search uses at most 256 spatially spread representative contacts and
256 representative oriented hand limits, eight coarse iterations and fifteen
refinement evaluations. Hand residuals guide initialization, correction and
refinement. Final contact scores use exact triangle distances for every input
contact; hand checks use the full CAD against every input hand patch. Recordings
and exported point clouds are unchanged. The reported pose remains provisional
even if calibrated.


### Collision and orientation search update

Quick placement audits every supplied hand patch against the full sensor
CAD before accepting a placement. Final contact distances are exact;
the separate hand penetration allowance defaults to 3 mm and can
be adjusted in Estimation settings. Rejected poses stay in the fit report.
When neither candidate passes, the better rejected fit is displayed in the
hand viewer with an explicit warning.
During fitting, deeper sampled surface crossings guide the optimizer alongside
nearest-surface distances. Palm initialization covers eight rolls over a full
turn, fallback starts receive hand-aware correction in both modes, and six
orientation-diverse poses are refined by default. Orientation is still inferred
from the measured geometry; sparse contacts can leave several rolls plausible.
These finite local patches still do not certify collision against unsampled
hand volume.


### Estimation point spacing

Estimation settings now offers a separate point spacing, default 2 mm (0 disables
thinning). A deterministic radius search keeps the first original observation
and removes neighbors within that distance. Contact points and hand limits are
reduced separately; hand patches whose normals differ by over 20 degrees remain
separate. Thinning applies to search, not final checks, in both quality modes.
The report shows checked/input counts. The original recording, displayed cloud,
and exports remain intact. Certified clearance constraints are retained.
Increase spacing to reduce search work. Final contact and hand-patch checks
always retain every supplied observation, including nearby distinct errors.

### Adaptive search allocation

Detailed fitting and the full quick-mode fallback screen all initial pose
families cheaply before expensive constrained correction. Screening uses up to
128 contacts, 96 oriented hand patches and 384 CAD collision samples (the quick
fallback retains its existing 256-point screening budgets). General starts get
at most six ICP steps; palm starts get at most three optimization evaluations.
At most max(12, four times the refinement budget) distinct poses are promoted,
plus a reserved empirical grasp prior when supplied.
Pose diversity compares the CAD bounding-box center and rotation; one palm
hypothesis is reserved when available. Promoted poses use the ordinary search
representatives and 1536 collision samples before final surface refinement.
Contact-only ICP retains its established search because its inexpensive rigid
updates do not benefit from pruning expensive hand/direction corrections.

In detailed mode, the first two refined poses can receive up to two contact
repair rounds when at least 90% of original contacts already meet tolerance
and sampled hand limits pass. Each round adds up to 32 worst contact violations
and exact distance constraints. A repair must reduce full-cloud contact
violation and preserve sampled hand feasibility; final exact checks still
decide acceptance. This is a bounded heuristic search, not a global optimum
or infeasibility certificate. Quick-mode identity remains provisional.

Fit metadata includes `search_method`, `starts_evaluated`, `promoted_starts`,
`contact_repair_evaluations`, and full `scored_contact_points`. Cached reports
from older estimator revisions are invalidated. Regression coverage:

```powershell
python -m unittest discover -s tests -p test_adaptive_matching.py -v
python -m unittest discover -s tests -p test_exact_surface_queries.py -v
python -m unittest discover -s tests -p test_point_spacing.py -v
python tests/benchmark_estimation.py --baseline-ref f9de0f7 --frame path/to/cloud.json --output comparison.json
```

The offline benchmark loads each revision's matching and exact-distance code,
warms the CAD indexes outside timing, and reports both candidates plus maximum
and RMS errors on the same original contacts. A single timing is a smoke
comparison; contact fit error is not ground-truth pose accuracy.


### Finger limits in run history

Run clouds retain non-thumb finger surface limits only at each finger's maximum
recorded bend among the selected valid poses. More extended poses no longer
leave extra finger arcs in estimation limits. The display preserves each finger's
initial contact pose separately. Each finger
is selected independently using its actuator calibration, including reversed
calibration; equal maximum bends retain their pressure evidence. Slip and
excluded-state samples cannot select a pose. Measured contact selection, thumb
probing, palm limits and raw recordings are unchanged. Reload the viewer and
reopen or rebuild a run to apply this to an existing history view. API callers
can request `finger_limit_selection: 'all'` to reconstruct the full finger sweep.

## Direction from pressure patches

The estimator uses pressure-weighted local contact planes to guide rotation
in both quick and detailed modes, including coarse pose ranking and refinement.
Each accepted neighborhood needs at least six distinct contacts within 12 mm,
at least 1 mm of independent width, and low out-of-plane variance. Pressure
weights are capped to prevent a single peak from dominating. Sparse, line-like,
and strongly curved neighborhoods contribute no directional term. Planes are
computed before display/search point thinning, with at most 128 neighborhoods.

Their unsigned normals are matched to local CAD triangle normals using a soft
residual with a bounded 3 mm lever arm. STL winding does not determine direction.
Pressure values are not treated as calibrated forces, and a footprint axis is
not assumed to be the sensor axis. A plane leaves rotation about its normal
unresolved; symmetric surfaces may still leave yaw/roll ambiguous. Contact and
hand-penetration acceptance limits remain unchanged. The fit card reports how
many local patches contributed; fit JSON also includes the directional RMS in
mm-equivalent residual units. Previous estimator results must be recomputed.


### Thumb closure first guess

With at least three above-threshold thumb taxels at a pose at least 85%
closed in the configured actuator range, the smaller CAD envelope is tried
first. The running estimate shows this provisional first guess immediately
before alignment. Closure alone never confirms identity or rejects the larger
sensor. A less closed thumb retains the larger-first order. Both candidates
remain available as fallback; detailed validation still fits both.

New run clouds retain the deepest contact-bearing thumb pose separately from
the display pose, after state/slip filtering. Rebuild old clouds to enable this
hint. Live/saved snapshots use their recorded thumb pressures and calibration.
Quick placement first tries eight general orientations and up to eight palm
starts, with three coarse iterations and at most two refinements per candidate.
If no pose passes, it tries the other candidate with the same short budget.
The full search is available through Detailed validation; quick search may miss
a feasible pose, so a rejected quick result does not prove the sensor cannot fit.
It stops refining at the first pose passing the existing
contact, clearance and full-CAD hand checks. The 85% heuristic reflects the
observed grasp behavior, not a calibrated probability or a physical guarantee.

## Pressure-patch direction

Quick and detailed estimation use pressure-weighted local contact planes to guide
orientation during coarse ranking and refinement. The bounded quick search and
thumb-first candidate order are preserved. Each patch needs six distinct points
within 12 mm, at least 1 mm of independent width and a smallest-to-middle PCA
variance ratio below 0.03. Sparse, line-like or strongly curved patches are
ignored. Pressure weights are capped and the unsigned normal residual has a
maximum 3 mm lever arm; it is a soft orientation cue, not a calibrated force.
At most 128 neighborhoods are computed before point thinning. Contact-distance
and hand-collision acceptance checks are unchanged. A patch cannot resolve
rotation about its normal, and symmetric surfaces can remain ambiguous. The fit
card reports the contributing patch count; JSON also reports its mm-equivalent
directional residual. Old estimates are invalidated by the estimator revision.

Quick placement uses one standard centered pose when thumb contacts or a rigid palm are available. It adjusts angle about the CAD center, translates toward the thumb contact boundaries, and refines once before the full hand triangle audit. Detailed validation retains the multistart search. Rejected poses are available for explicit inspection but are never shown automatically as the estimate.

During estimation the viewer follows unvalidated STL trial poses. Switch off Follow live STL position or drag Inspect search poses to pause and inspect the last 256 sampled poses, including candidate switches. Pose updates preserve the camera. History remains available after completion; rejected results are hidden until explicitly inspected.


### First visible surface for inconsistent run previews

Accumulated CSV contacts now retain the outward taxel normal from the same
pose as each selected contact. The estimator validates these optional normals;
older clouds without them retain the existing path. Normals must not be
recovered from the nearest patch in a different pose of the hand sweep.

When the supplied hand patches already contradict the contact tolerance balls,
the inspection fit uses the first CAD surface seen along each recorded normal.
Unsigned distance alone can match a far wall or internal assembly face even
when the model passes through the contact. Visibility rays start beyond the
entire CAD envelope and do not depend on triangle winding or watertightness.

Finger contact group centers initialize the shaft direction; a transverse
circle provides a starting center. Both axial directions, eight rolls and five
axial placements are screened. Five distinct starts are refined in detailed
mode (two in quick mode), using robust point-to-plane steps with visibility
correspondences refreshed between steps. Each region has equal total weight,
recomputed after point selection. Detailed mode finishes with all contacts.
These starts use measured geometry, without sensor-name or run-label priors.

The search score caps errors at 30 mm and assigns ray misses nearest-distance
plus 6 mm. This robust ranking score is distinct from the reported uncapped
RMS over hit rays. Miss count, hit fraction, per-region statistics and the
fraction of all contacts whose first intersection is within tolerance are
reported separately. The final audit queries the complete CAD for every
contact; no missed ray counts as an inlier. Certified clearances and hand
conflict witnesses are still checked exactly.

These poses remain rejected inspection previews: changing the contact fit
does not repair the contradictory hand constraint model or validate the full
hand volume. Consistent inputs retain the existing complete hand audit. No
candidate identity is confirmed by this preview path.


### Local registration near the usual insertion (v17)

Known CAD hashes in left_base_link with oriented contact observations use a
bounded local contact-to-CAD fit before the legacy prism/conflict path. The
recorded grasp prior initializes both candidates independently. Rotation-vector
components are limited to +/-20 degrees and the existing center windows are
retained; this prevents the housing from flipping to the opposite insertion
direction. Quick uses five seeds, detailed seven. The unchanged seed and
intermediate poses compete with the final iterate using region-balanced robust
first-visible-surface evidence. All contacts retain exact final ray and unsigned
triangle-distance metrics, including misses and outliers.

Collision queries use actual finite closed hand CAD links articulated with the
frozen recorded joint pose and calibration. The old unbounded inward prisms do
not reject or steer this path. A bounded correction uses nearest finite-volume
escape directions while retaining contact residuals. Final checks cover 2400
deterministic object-surface samples. Five open hand links currently cannot
provide reliable inside/outside tests and are listed explicitly. This is a
sampled frozen-pose audit, not continuous mesh/mesh collision certification,
not an audit of every recorded swept pose, and not physical calibration.

A placement with at least half the contact rays within tolerance and no sampled
closed-link penetration beyond the hand allowance can be displayed as a
**provisional placement**. This separate placement_usable flag never changes
compatible, classification, strict contact tolerances or calibration gates.
The viewer reports actual contact support and unverified collision coverage.
Sensor identity remains AMBIGUOUS. Source clouds and recordings are unchanged.

Exact first-hit ray queries use a cached triangle BVH, with tests against the
complete triangle reference. No Open3D, SDF training or extra dependency is
required. The pose prior is derived from existing recordings: residual
comparisons on those recordings are engineering regressions, not independent
measurements of 6D pose accuracy.

Candidate scoring balances whole fingers, thumb and palm rather than treating every named tip/nail patch equally. Robust tails retain the influence of poorly matched entire areas; patches with only one or two points do not receive the weight of a whole finger. The viewer explicitly lists poorly matched physical regions and labels their placement Partial. Overall inlier fraction is not a substitute for regional support. Native saved snapshots retain taxel region, normal, link and source metadata during normalization.


### Visible contact registration and cached triangle queries

CSV contacts retain the outward taxel normal from the same observed pose.
Older clouds without complete valid normals retain the existing solver.
When local hand-patch constraints contradict the measured contact balls, the
inspection fit matches CAD surfaces visible from the hand side. It remains
a rejected preview; the legacy hand-patch conflict is not a solid collision
certificate or a confirmed sensor identity.

A strictly parallel normal ray is not a sufficient contact-distance test:
it can miss an edge even when the surface lies inside the positional
tolerance ball. The registration projects to a nearest triangle point and
casts from the hand side toward that point. An earlier occluding CAD surface
replaces a hidden correspondence. Thus far walls and internal assembly faces
cannot masquerade as a nearby externally reachable contact. The final audit
uses exact nearest triangles and the complete CAD for every measured contact.
This is an upper bound on nearest visible distance, not a global visibility
or physical-contact certificate. Normals are geometric directions, not
calibrated force vectors.

The triangle hierarchy is cached on each immutable candidate model. Batched
ray/box tests reduce triangle candidates; batched exact triangle tests retain
the first hit. Segment bounds stop visibility queries beyond their known CAD
target. No new package, STL simplification or geometry rescaling is used.
Winding and open assemblies do not affect first-hit visibility. The separate
parity helper is for watertight hand meshes only.

Finger group centers and a transverse circle initialize the shaft. Detailed
mode checks both axis directions, eight rolls and five axial placements.
Quick mode uses cheap unsigned screening while retaining two axial choices
per direction/roll stratum. Visibility still determines refined poses.
Every region has equal total weight, recomputed after selection. Damped steps
must improve the actual visibility objective, and detailed refinement reaches
all contacts even when a coarse step stalls. The best observed fine pose is
retained instead of blindly taking the last optimizer step.

Preview placement prioritizes the region-balanced fraction of visible contacts
within the requested tolerance, then the robust distance score. Both measures
are reported, including per-region support and uncapped worst/RMS errors.
The robust score caps errors at 30 mm and assigns query misses nearest-distance
plus 6 mm; neither rule turns a failing contact into an inlier. Search nearest
queries use sampled local triangles; final visibility and distance audits use
exact triangles and every contact. No run label or empirical reference pose
is an input to this registration path.

Finite hand-volume checks were additionally performed offline against the
same posed meshes shown by the viewer. They are a separate diagnostic: open
hand links and unsampled CAD regions remain unchecked. The production preview
does not claim complete hand-volume validation. Consistent inputs retain the
existing full hand-patch audit and acceptance gates.

The anchored path uses the shared nearest-visible-triangle correspondence: the tolerance is a positional ball, not an infinitely thin normal ray. Exact final contact distances and occlusion queries retain all contacts. The old normal-ray inlier fraction is retained separately for comparison. The visible contact metric is labelled explicitly in the viewer.


Measured hand motion and sensor-size ambiguity (v18): CSV reconstruction stores
up to 64 measured thumb poses plus the selected contact pose for each closing
finger in `hand_snapshot.motion_samples`. Finite closed hand CAD volumes from
these poses enter the same collision audit and bounded repair as the frozen
pose, including thumb poses without pressure. This remains a sampled audit;
open CAD parts, motion between samples and object stationarity are unverified.
Old JSON exports without these samples retain frozen-pose coverage explicitly.

If both models have usable poses, their balanced visible-contact RMSE differs
by at most one quarter of the positional tolerance, and balanced contact
support differs by at most five percentage points, the result has
`reason=sensor_size_unresolved` and `candidate_comparison`. It has no automatic
`placement_candidate` or `best_candidate`. This is a conservative preview band,
not a calibrated statistical confidence interval. The viewer asks for an
explicit model choice and labels it comparison only. Missing thumb contacts
are stated; a tiny optimizer/prior score advantage cannot decide head size.

### Full-turn shaft rotation search

Shaft roll is periodic and must not inherit the insertion tilt's small prior
window. The local placement path screens a full turn at 15-degree intervals
(30 degrees in quick mode), each at three axial offsets. It refines distinct
angular basins with an independent continuous roll parameter. Roll preserves
the CAD center and shaft direction and receives no orientation-prior penalty.
The unchanged insertion and previous local seeds remain available for the
finite hand audit. The score profile, angular coverage and chosen roll are
reported; screening poses can be previewed during the search.

The independent free registration also screens the finer full-turn grid.
Its previous 45-degree-grid finalists are retained, with additional basins
from the finer grid. All finalists still receive the existing full-contact
exact triangle audit. More starts increase runtime; angular sampling is a
search strategy, not a guarantee of a globally optimal or identifiable roll.
