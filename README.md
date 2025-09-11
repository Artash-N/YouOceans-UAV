# PX4 Jetson MAVSDK Utilities

This repository contains lightweight Python utilities for monitoring, debugging, and basic control of a PX4-based drone from a Jetson (or other companion computer) using [MAVSDK-Python 3.10.x](https://mavsdk.mavlink.io/main/en/).

This repository is maintained by You-Oceans.org for the developement of a UAV system for ocean sensor payload drop and retreival
## Overview

- **`monitor.py`**  
  Lean telemetry logger that subscribes to MAVSDK streams (attitude, GPS, battery, mode, etc.) at a configurable rate (default 1 Hz).  
  Logs data to a timestamped CSV file in `~/logs/mav/` and optionally prints a human-readable summary line.

- **`dump_mavlink.py`**  
  Low-level MAVLink sniffer using `pymavlink`. Dumps every incoming MAVLink message (or selected ones) for raw debugging. Useful for protocol-level analysis.

- **`armtest.py`**  
  Minimal safety test: checks basic sensor health, arms the drone at idle, waits 5 seconds, then disarms. Does **not** take off. Designed to confirm companion ↔ Pixhawk serial link and arming logic.

---

## Requirements

- Python ≥3.8
- [MAVSDK-Python 3.10.x](https://mavsdk.mavlink.io/main/en/getting_started/installation.html)
- `pymavlink` (for `dump_mavlink.py`)

Install dependencies in a virtualenv:
```bash
pip install mavsdk==1.* pymavlink
