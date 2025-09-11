## Usage

### monitor.py
Log flight telemetry at 1 Hz (default):

    python monitor.py --port /dev/ttyTHS1 --baud 230400 --hz 1 --verbose

- Output CSV: ~/logs/mav/mavsdk_YYYYMMDD_HHMMSS.csv
- Printed fields (with --verbose):  
  Mode, armed state, battery voltage/SoC, GPS fix/sats, lat/lon/alt, rangefinder (if present), roll/pitch/yaw.

Example line:
    19:12:03 | POSCTL   | ARM    | VBat 23.1V | SoC 51% | GPS f3 s8 | Lat 37.1234567 | Lon -122.1234567 | Alt 12.3 | RF 1.25 | R/P/Y -1.2/0.8/179.9°

---

### dump_mavlink.py
Print all raw MAVLink messages:

    python dump_mavlink.py --port /dev/ttyTHS1 --baud 230400

Optional filters (e.g., only HEARTBEAT and SYS_STATUS):

    python dump_mavlink.py --types HEARTBEAT SYS_STATUS

---

### armtest.py
Arm motors for 5 seconds, then disarm:

    python armtest.py --port /dev/ttyTHS1 --baud 230400

Expected console output:
    -- Connected
    -- Waiting for basic sensor health...
    -- Sensors OK
    -- Arming (idle)
    -- Holding idle for 5s
    -- Disarming
    -- Done

⚠️ Safety note: Props will spin. Run only with propellers removed, or in a safe test environment.

---

## Tips

- Default serial URL: serial:///dev/ttyTHS1:230400  
  (adjust --port and --baud as needed).
- Use --hz in monitor.py to change logging frequency.
- dump_mavlink.py is handy if MAVSDK streams appear empty—verify messages are arriving at the link level.

---

## License
MIT (or specify your own).
