# Sensor identification in the current hand twin

Integrated into `check_digital_twin`, based on commit `320c504`.

## Start

Double-click **Start Sensor Identification.cmd** in this repository. The first
launch installs the optional tools. It opens this viewer at
http://127.0.0.1:18768/ (separate from the old prototype on port 18767).
Keep the terminal open while using it.

Under **Sensor identification**, select a CAD sample and click **Identify sensor**.
After fitting, **View fit in 3D** shows both aligned CAD surfaces and contacts.
**Current displayed contacts** uses the viewer's existing live/demo data or
saved-contact playback. The current viewer's hand, sensor objects, saved-file
mode, axes and pressure controls are retained.

The existing telemetry bridge address is left as configured by the current
viewer (normally port 18765). Start that bridge as usual for physical hand
telemetry, or change its existing Bridge address field to this server if your
publisher already targets port 18768. No hardware reads or commands are added.

Manual start:

```powershell
uv sync --extra sensor-identification
uv run --extra sensor-identification python digital_twin/start_sensor_simulation.py
```

To use the default bridge port instead, restart the bridge after installation:

```powershell
uv run --extra sensor-identification python digital_twin/bridge_server.py
```

## Measurement adapter

`measurements.js` consumes `getContactCloud()`, `getTaxelPositions()` and
`ProfileTwin.snapshot()` from the current viewer. The source frame is explicitly
`left_base_link`, with `metres` normalized to `m` at the matcher interface.
Matching itself uses millimetres. Live clouds use the viewer's existing
pose/pressure synchronization checks. Saved playback retains its historical
pose and does not acquire a fake current timestamp. Only recorded active
points are available in saved exports; other cells are unknown, not zero.

No older viewer CAD-frame transform is transplanted. Physical geometry remains
uncalibrated: current displayed hand contacts can return compatibility scores,
but the integration does not enable a confident sensor label for them. CAD
sample inputs have known synthetic geometry and can be classified.

## Matching

The independent matcher tries both STL models with identical multistart rigid
alignment budgets, then refines exact unsigned triangle distances. It never
scales either model. Lower score means better surface compatibility, not a
probability. Both fit, neither fit within the search, poor separation, sparse
contacts, unverified calibration and unstable fitting have separate ambiguity
reasons. Shared shaft contacts commonly cannot distinguish these sensors.
The search is local and does not prove global infeasibility.

Normal non-contact readings are ignored as negative constraints. Only explicitly
certified surface-clearance balls can contribute exclusion evidence. Both CAD
meshes are open, so solid inside/outside collision checks are not inferred.
A measures roughly 36 x 119 x 36 mm; B roughly 68 x 162 x 63 mm. A's companion
STEP declares millimetres; B's physical scale still needs confirmation.
Electrode placement, joint mapping, timing and contact deformation need
calibration before relying on physical identification.

## Test without hardware

```powershell
uv run --extra sensor-identification python -m unittest discover -s tests -p test_sensor_identification.py -v
uv run --extra sensor-identification python -m unittest discover -s tests -p test_identification_service.py -v
node tests/test_identification_measurements.js
```

The current self-contained HTML is extended additively. Existing embedded mesh
and saved-replay code is preserved byte-for-byte; no old viewer is copied over.
The staged `filter_csv.py` file and Git index are outside this change.
