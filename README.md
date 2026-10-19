# Project Name



## Overview

This branch is a receive-only hand telemetry and tactile viewer. Hardware motion
is controlled by the separate Blackforest program. Connecting this application
does not open/close the hand or write angle, position, speed, force, calibration,
or configuration registers. The Python receiver still sends Modbus read requests
to acquire measurements, and publishes them to the local viewer.

The desktop UI shows measured angles. Movement buttons, sliders and saved-pose
execution have been removed; existing position files are preserved. Grasp
detection observes measured motion/contact, captures once all six actuator
readings are stable, and waits for externally driven reopening before rearming.
It also works when attached to an already closed hand with contact.

Use `Start Sensor Identification.cmd` for the shared viewer at
`http://127.0.0.1:18765/`. Virtual hand sliders only change the rendered model.



## Features



## Prerequisites

- Python `>=3.14`

## Installation

```bash
# Clone the repository
git clone <repository-url>
cd <repository-directory>

# Install dependencies
uv sync
```

## Usage

```bash
# Run the main application
uv run blackforest
# or
uv run python src/main.py
```

## Project Structure

```text
├── documents/        # Documentation and CAD / manual assets
├── src/              # Source code
#├── tests/            # Test suite, not sure wether we should do that at all...
├── pyproject.toml    # Project dependencies and metadata
└── README.md
```

## Team & Acknowledgments
