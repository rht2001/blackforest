# RH56DFTP Hand Simulation

Version: 17 October 2026. A lightweight package containing the standalone hand viewer with corrected thumb geometry, a 3D sensor editor, and telemetry displays.

## Goal

The application makes the hand's pose and tactile measurements visible. You can explore the virtual hand without hardware, test the visualization using artificial data, or display measurements from an existing hand-control application.

The 3D view shows hand motion and sensor locations. The 2D view makes the 17 tactile sensor regions and their 1,062 sensing cells easier to inspect.

## Quick start: no installation required

1. Extract the entire ZIP archive.
2. Open `hand_twin_6dof.html` in a browser supporting WebGL2, such as Edge, Chrome, or Firefox.
3. Select “Motion demo”, “Open”, “Close”, or move the six sliders.

This mode works offline. The model and viewer are embedded in the HTML file. Drag to rotate the view and use the mouse wheel to zoom.

Select “Edit sensors” to adjust sensor surfaces. Export the layout as JSON to share it with another person or computer; layouts saved only in your browser are local to that browser.

For the 2D sensor view, open `tactile_2d.html` and select “Simulation”. The interface, status messages, and included scripts are in English.

## Architecture and the bridge server

There are three separate parts in streamed operation:

```text
Existing hand-control application      OR      Demo data generator
(reads measurements from the hand)            (creates artificial measurements)
                    |                                      |
                    +------------------+-------------------+
                                       |
                              Local bridge server
                                       |
                              Browser visualization
                               /                                         3D hand          2D sensor map
```

The **data producer** supplies the hand's current state: joint readings, forces, and tactile measurements. With real hardware, the existing control application remains responsible for communicating with the hand. For a demonstration, the demo generator takes the producer's place.

The **bridge server** is the local meeting point between that producer and the browser. It receives state updates over HTTP, checks their format and allowed ranges, keeps the latest state in memory, and attaches reception timestamps. The browser repeatedly requests that state and updates the visualization. The server also serves the two viewer pages, so both can be opened through a local browser address.

Its purpose is to let an existing acquisition process feed a separate visualization without making the browser communicate directly with the hand. The included background sender keeps the visualization's network requests out of the acquisition application's main workflow and retains the newest pending update.

The bridge listens only on this computer at `127.0.0.1`, using port `18765` by default. It is a current-state relay, not a recording database: it does not save measurement history, and its state is lost when the server stops. It does not read hand sensors, establish a Modbus connection, or issue motor commands. Browser sliders move the virtual model only.

Reception timestamps let the viewer distinguish recent updates from old data. After four seconds without fresh data, the viewer marks the relevant data as stale. A reachable bridge confirms that the local server is running; it does not establish that a physical hand is connected.

**When is it needed?** Use the bridge when displaying data from Python, whether artificial or measured. Opening either HTML file for its built-in offline demonstration does not require the bridge or Python.

## Try the streamed demo without hardware

Requirements: Python 3.10 or newer and a browser. No pip packages or build step are required. Run the following commands from the extracted folder.

In terminal 1:

```powershell
python bridge_server.py
```

Open http://127.0.0.1:18765/ and select “Live from Python”.

In terminal 2:

```powershell
python demo_sender.py --seconds 60
```

The demo sends artificial joint readings, forces, and tactile contacts, identified as simulation data. For the same stream in the 2D view, open http://127.0.0.1:18765/tactile and select “Live bridge”. Stop the server with Ctrl+C.

On Windows, you can alternatively start the combined demo from PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_demo.ps1
```

The launcher opens the browser and sends demo data for up to one hour. Select “Live from Python” in the viewer. Press Ctrl+C in the terminal to stop the demo and its bridge process.

If `python` is unavailable, install Python or use `py` instead of `python` in the two manual commands. If the port is occupied, start both the bridge and demo with `--port 18766`, then open http://127.0.0.1:18766/.

## Connect an existing acquisition application

Start the bridge and have the existing application forward its already-acquired measurements through the included background sender. `twin_sender.py`, `blackforest_adapter.py`, and `tactile_layout.py` must be available to that application. The adapter supports the existing blackforest DataDict format.

Keep a single acquisition path to the physical hand: the viewer should consume the same measurements already collected by the control application. This package contains the visualization connection, not the hardware-control application or a hardware launcher.

The six channels are ordered as follows: little finger, ring finger, middle finger, index finger, thumb bending, and thumb rotation. Joint readings use the 0–1000 register scale, not degrees. The long fingers are open at 1000 and closed at 0. Tactile readings use the 0–4095 raw-value range. The adapter preserves the palm's acquisition order; the viewer handles its spatial arrangement.

## Included files

| File | Purpose |
| --- | --- |
| `hand_twin_6dof.html` | Complete 3D viewer, embedded model, and sensor editor. |
| `tactile_2d.html` | Complete 2D tactile sensor view. |
| `bridge_server.py` | Local state relay and viewer hosting. |
| `tactile_layout.py` | Shared sensor-region definitions and layout helpers. |
| `demo_sender.py` | Artificial streamed data for testing without hardware. |
| `start_demo.ps1` | Windows launcher for the bridge, browser, and demo. |
| `twin_sender.py` | Background connection from an existing Python application. |
| `blackforest_adapter.py` | Converts existing blackforest measurements for visualization. |
| `README.md` | Setup, architecture, and usage guide. |

## Model limitations and verification

This is a geometric digital twin with partly estimated joint relationships and sensor placements. It does not simulate physical forces or collisions. Calibration against the real hand is needed when spatial accuracy matters.

This package contains the files needed to use the application. CAD sources, build tools, hardware control, databases, older versions, and test files are omitted.

Before handover, the bridge/sender tests, adapter tests, and JavaScript tests for all six degrees of freedom passed. The isolated package was also checked for delivery of both viewer pages, demo streaming, all 17 sensor regions, and simulation labeling. Real hardware was not tested.

The English package also passed the sensor-editor and 2D-view behavioral checks. Embedded model and sensor geometry were compared with the original version and are unchanged.


## Live pressure and force display

The twin is a read-only telemetry viewer. `robot.poll()` reads motor force
registers and the tactile cache; `TwinPublisher` posts to the localhost bridge,
and the browser polls every 300 ms. The hardware controller keeps its own
force limits and contact-triggered grasp logic. The twin does not apply forces
or simulate contact dynamics.

Pressure now matches `HandApp.draw`: `max(0, raw - sensor_offsets)` per cell.
Both twin views use the GUI colors: green below 800, yellow from 800, orange
from 2000, red from 3200 (4000-count reference scale). These are sensor counts,
not calibrated pressure units. The 3D viewer also has a selectable live grid
with per-cell readings, min/mean/max, and picking on the hand. Missing/stale
live cells are gray; force readings remain separate in grams. Sensor mounting
and pose remain approximate. Optional baseline capture in the 2D view applies
an additional local correction; leave it off to match the GUI.

Start `uv run python digital_twin/bridge_server.py`, then
`uv run python src/main.py`; open http://127.0.0.1:18765/ and select
**Live from Python**. Restart existing Python/bridge processes and reload the
browser after updating. No hardware connection is required for unit tests.

On `check_digital_twin`, publishing is already enabled by default in `src/main.py`; no `--twin` flag is needed.
