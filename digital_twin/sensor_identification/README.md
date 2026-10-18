# One-page tactile identification

Start **Start Sensor Identification.cmd** from this Desktop repository. The UI
opens at http://127.0.0.1:18768/. Everything happens on that page.

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
Use the row selector or Previous/Next to inspect CSV frames individually. Only
the selected row is reconstructed. Files remain local; the limit is 50 MB.
Truncated arrays (`...`) are rejected. Means cannot replace per-cell pressure.
Missing/invalid pressure stays unknown. CSV row time is not invented, and CSV
counts are shown as recorded; pressure offsets and failed-read validity are not
supplied by that format. A CSV pose uses the current viewer's explicitly
uncalibrated kinematic model. JSON uses its recorded pose/calibration.

The saved-file hand/point-cloud controls, axes, sensor objects, pressure display
and connection options remain accessible. The old source controls are grouped
under **Advanced display & connection** rather than mixed with the main modes.

Estimate captures all available cell positions. Recognition uses only valid
pressure strictly above its threshold; ordinary non-contact cells are not
hard exclusion boundaries. Gray points are positions with unknown pressure.
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

The report is embedded as the page's estimate view; it does not open another tab.
Its header records which source was frozen. Changing modes while fitting does
not alter that snapshot. The independent CLI and CAD benchmark tests remain.

```powershell
uv sync --extra sensor-identification
uv run --extra sensor-identification python digital_twin/start_sensor_simulation.py
node tests/test_identification_history.js
node tests/test_identification_measurements.js
uv run --extra sensor-identification python -m unittest discover -s tests -p test_identification_service.py -v
```
