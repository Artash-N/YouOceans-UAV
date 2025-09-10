#!/home/jetson/mavsdk-env/bin/python

# monitor.py — MAVLink state logger with PX4 mode decode + Standard Modes + Rangefinder
import argparse, csv, os, sys, time, math
from datetime import datetime, timezone
from pymavlink import mavutil

DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 115200
DEFAULT_HZ   = 5.0          # print/log rate
REQUEST_HZ   = 10.0         # ask autopilot to send these msgs at this rate

# PX4 main modes (8-bit)
PX4_MAIN = {
    1:"MANUAL",2:"ALTCTL",3:"POSCTL",4:"AUTO",5:"ACRO",6:"OFFBOARD",
    7:"STABILIZED",8:"RATTITUDE",9:"SIMPLE",
}

# PX4 AUTO submodes (8-bit)
PX4_AUTO_SUB = {
    0:"AUTO",1:"AUTO.READY",2:"AUTO.TAKEOFF",3:"AUTO.LOITER",4:"AUTO.MISSION",
    5:"AUTO.RTL",6:"AUTO.LAND",7:"AUTO.RTGS",8:"AUTO.FOLLOW_TARGET",9:"AUTO.PRECLAND",
}

# MAVLink Standard Mode names (if CURRENT_MODE is present)
MAV_STANDARD_MODE = {
    0:"MANUAL",1:"POSITION_HOLD",2:"AUTO",3:"CRUISE",4:"ALTITUDE_HOLD",
    5:"SAFE_RECOVERY",6:"MISSION",7:"LAND",8:"TAKEOFF",
}

def decode_px4_mode(custom_mode: int) -> str:
    if custom_mode == 0: return "UNKNOWN(0)"
    main = (custom_mode >> 16) & 0xFF
    sub  = (custom_mode >> 24) & 0xFF
    if main == 0 and sub == 0:
        main = custom_mode & 0xFF
        sub  = (custom_mode >> 8) & 0xFF
    main_name = PX4_MAIN.get(main)
    if not main_name: return f"UNKNOWN({custom_mode})"
    return PX4_AUTO_SUB.get(sub, f"AUTO.{sub}") if main_name == "AUTO" else main_name

def decode_std_mode(std_raw: int):
    mode_id  = std_raw & 0xFF
    group    = (std_raw >> 8)  & 0xFF
    vehicle  = (std_raw >> 16) & 0xFF
    name = MAV_STANDARD_MODE.get(mode_id, f"UNKNOWN_STD({mode_id})")
    return name, group, vehicle

CSV_FIELDS = [
    "ts_iso","time_boot_ms",
    "mode_name","std_mode_name",
    "base_mode","custom_mode","std_mode_raw",
    "armed",
    "lat_deg","lon_deg","alt_m","agl_m",
    "vx_mps","vy_mps","vz_mps",
    "roll_deg","pitch_deg","yaw_deg",
    "vbatt_V","bat_rem_pct","cpu_load_pct","comm_drop_pct","gps_fix","gps_sats",
    "range_m",
]

def now_iso(): return datetime.now(timezone.utc).astimezone().replace(tzinfo=None).isoformat(timespec="seconds")
def f(v, nd=1): return "-" if v is None else f"{v:.{nd}f}"

def request_msg_rate(m, msg_id: int, hz: float):
    """Ask autopilot to stream a message at ~hz using SET_MESSAGE_INTERVAL; fall back to REQUEST_DATA_STREAM."""
    interval_us = 0 if hz <= 0 else int(1_000_000 / hz)
    try:
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # 511
            0,
            msg_id,        # param1: message id
            interval_us,   # param2: interval (us) (0 for default, -1 to stop)
            0,0,0,0,0
        )
    except Exception:
        pass

    # Legacy fallback: request an appropriate stream that usually carries the message.
    try:
        # Map a few common messages to stream types
        stream = None
        if msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                      mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                      mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT):
            stream = mavutil.mavlink.MAV_DATA_STREAM_POSITION
        elif msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                        mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS):
            stream = mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS
        elif msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,):
            stream = mavutil.mavlink.MAV_DATA_STREAM_EXTRA3  # often where PX4 puts extras
        if stream is not None:
            m.mav.request_data_stream_send(
                m.target_system,
                m.target_component,
                stream,
                int(max(1, hz)),
                1  # start
            )
    except Exception:
        pass

def ensure_streams(m, hz_request: float):
    # Always ask for these; harmless if already streaming
    wanted = [
        mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
    ]
    for mid in wanted:
        request_msg_rate(m, mid, hz_request)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--hz",   type=float, default=DEFAULT_HZ, help="print/log rate")
    ap.add_argument("--req-hz", type=float, default=REQUEST_HZ, help="message request rate from FCU")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    logdir = os.path.expanduser("~/logs/mav"); os.makedirs(logdir, exist_ok=True)
    fname  = os.path.join(logdir, f"mav_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    print(f"[monitor] logging to {fname}")
    print(f"[monitor] connecting: {args.port}  baud={args.baud}  hz={args.hz}")
    m = mavutil.mavlink_connection(args.port, baud=args.baud, autoreconnect=True)
    print("[monitor] waiting for HEARTBEAT…")
    if not m.wait_heartbeat(timeout=10):
        print("[monitor] no heartbeat — check wiring/baud/permissions"); sys.exit(1)
    print(f"[monitor] heartbeat OK from sysid={m.target_system} compid={m.target_component}")

    # Ask the FCU to actually send what we need on this port
    print(f"[monitor] requesting message rates at ~{args.req_hz} Hz (incl. DISTANCE_SENSOR)…")
    ensure_streams(m, args.req_hz)

    state = dict(
        time_boot_ms=None, base_mode=None, custom_mode=None, mode_name=None,
        std_mode_raw=None, std_mode_name=None, armed=0,
        lat_deg=None, lon_deg=None, alt_m=None, agl_m=None,
        vx_mps=None, vy_mps=None, vz_mps=None,
        roll_deg=None, pitch_deg=None, yaw_deg=None,
        vbatt_V=None, bat_rem_pct=None, cpu_load_pct=None, comm_drop_pct=None,
        gps_fix=0, gps_sats=0,
        range_m=None, rf_down_last=None, rf_any_last=None
    )

    ORIENT_DOWN = 25  # MAV_SENSOR_ORIENTATION enum for down-facing

    period = 1.0 / max(0.1, args.hz); next_emit = time.time()

    with open(fname, "w", newline="") as fcsv:
        w = csv.DictWriter(fcsv, fieldnames=CSV_FIELDS); w.writeheader()

        while True:
            try:
                msg = m.recv_match(blocking=True, timeout=0.5)
                if not msg: continue
                tname = msg.get_type()
                if hasattr(msg, "time_boot_ms"):
                    state["time_boot_ms"] = int(msg.time_boot_ms)

                if tname == "HEARTBEAT":
                    state["base_mode"]   = int(msg.base_mode)
                    state["custom_mode"] = int(msg.custom_mode)
                    state["armed"] = 1 if (state["base_mode"] & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
                    state["mode_name"]   = decode_px4_mode(state["custom_mode"])

                elif tname == "CURRENT_MODE":
                    try:
                        std_raw = int(msg.standard_mode)
                        state["std_mode_raw"] = std_raw
                        std_name, grp, veh = decode_std_mode(std_raw)
                        state["std_mode_name"] = std_name
                    except Exception:
                        pass
                    if hasattr(msg, "base_mode"):
                        state["base_mode"] = int(msg.base_mode)
                        state["armed"] = 1 if (state["base_mode"] & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
                    if hasattr(msg, "custom_mode"):
                        cm = int(msg.custom_mode)
                        if not state.get("mode_name") or state["mode_name"].startswith("UNKNOWN"):
                            state["mode_name"] = decode_px4_mode(cm)
                        state["custom_mode"] = cm

                elif tname == "SYS_STATUS":
                    state["cpu_load_pct"] = round(getattr(msg, "load", 0) / 10.0, 1)
                    vb = getattr(msg, "voltage_battery", 0)
                    state["vbatt_V"] = (vb/1000.0) if vb else state["vbatt_V"]
                    br = getattr(msg, "battery_remaining", None)
                    if br is not None and br >= 0: state["bat_rem_pct"] = float(br)
                    state["comm_drop_pct"] = float(getattr(msg, "drop_rate_comm", 0))

                elif tname == "BATTERY_STATUS":
                    br = getattr(msg, "battery_remaining", None)
                    if br is not None and br >= 0: state["bat_rem_pct"] = float(br)
                    volts = getattr(msg, "voltages", None)
                    if volts and isinstance(volts, (list, tuple)) and volts[0] not in (None, 65535):
                        state["vbatt_V"] = volts[0] / 1000.0

                elif tname == "GLOBAL_POSITION_INT":
                    if msg.lat != 0 and msg.lon != 0:
                        state["lat_deg"] = msg.lat / 1e7; state["lon_deg"] = msg.lon / 1e7
                    state["alt_m"] = msg.alt / 1000.0
                    state["agl_m"] = getattr(msg, "relative_alt", 0) / 1000.0
                    state["vx_mps"] = getattr(msg, "vx", 0) / 100.0
                    state["vy_mps"] = getattr(msg, "vy", 0) / 100.0
                    state["vz_mps"] = getattr(msg, "vz", 0) / 100.0

                elif tname == "ATTITUDE":
                    state["roll_deg"]  = math.degrees(msg.roll)
                    state["pitch_deg"] = math.degrees(msg.pitch)
                    state["yaw_deg"]   = math.degrees(msg.yaw)

                elif tname == "GPS_RAW_INT":
                    state["gps_fix"]  = int(getattr(msg, "fix_type", 0))
                    state["gps_sats"] = int(getattr(msg, "satellites_visible", 0))

                elif tname == "DISTANCE_SENSOR":
                    curr_cm = getattr(msg, "current_distance", None)
                    min_cm  = getattr(msg, "min_distance", None)
                    max_cm  = getattr(msg, "max_distance", None)
                    orient  = int(getattr(msg, "orientation", 0))
                    valid = (
                        curr_cm is not None and curr_cm not in (0, 0xFFFF) and
                        min_cm is not None and max_cm is not None and min_cm > 0 and
                        min_cm <= curr_cm <= max_cm
                    )
                    if valid:
                        curr_m = curr_cm / 100.0
                        if orient == ORIENT_DOWN:
                            state["rf_down_last"] = curr_m
                        if state["rf_any_last"] is None or curr_m < state["rf_any_last"]:
                            state["rf_any_last"] = curr_m
                        state["range_m"] = state["rf_down_last"] if state["rf_down_last"] is not None else state["rf_any_last"]

                # periodic log/print
                now = time.time()
                if now >= next_emit:
                    w.writerow({
                        "ts_iso": now_iso(),
                        "time_boot_ms": state["time_boot_ms"],
                        "mode_name": state["mode_name"] or "",
                        "std_mode_name": state["std_mode_name"] or "",
                        "base_mode": state["base_mode"] or 0,
                        "custom_mode": state["custom_mode"] or 0,
                        "std_mode_raw": state["std_mode_raw"] or 0,
                        "armed": state["armed"],
                        "lat_deg": state["lat_deg"], "lon_deg": state["lon_deg"],
                        "alt_m": state["alt_m"], "agl_m": state["agl_m"],
                        "vx_mps": state["vx_mps"], "vy_mps": state["vy_mps"], "vz_mps": state["vz_mps"],
                        "roll_deg": state["roll_deg"], "pitch_deg": state["pitch_deg"], "yaw_deg": state["yaw_deg"],
                        "vbatt_V": state["vbatt_V"], "bat_rem_pct": state["bat_rem_pct"],
                        "cpu_load_pct": state["cpu_load_pct"], "comm_drop_pct": state["comm_drop_pct"],
                        "gps_fix": state["gps_fix"], "gps_sats": state["gps_sats"],
                        "range_m": state["range_m"],
                    })
                    if args.verbose:
                        arm = "ARM" if state["armed"] else "DISARM"
                        std_suffix = ""
                        if state["std_mode_raw"] is not None:
                            std_name, grp, veh = decode_std_mode(state["std_mode_raw"])
                            std_suffix = f" | STD {std_name} [raw=0x{state['std_mode_raw']:X} grp={grp} veh={veh}]"
                        lat = "-" if state["lat_deg"] in (None, 0) else f"{state['lat_deg']:.7f}"
                        lon = "-" if state["lon_deg"] in (None, 0) else f"{state['lon_deg']:.7f}"
                        alt = "-" if state["alt_m"] is None else f"{state['alt_m']:.1f}"
                        rf  = "-" if state["range_m"] is None else f"{state['range_m']:.2f}m"
                        print(
                            f"{datetime.now().strftime('%H:%M:%S')} | {state['mode_name'] or 'UNKNOWN':<8} | {arm:6s}"
                            f" | VBat {f(state['vbatt_V'],2)}V | SoC {f(state['bat_rem_pct'],1)}% | CPU {f(state['cpu_load_pct'],1)}%"
                            f" | GPS f{state['gps_fix']} s{state['gps_sats']}"
                            f" | Lat {lat} | Lon {lon} | Alt {alt} | RF {rf}"
                            f" | R/P/Y {f(state['roll_deg'],1)}/{f(state['pitch_deg'],1)}/{f(state['yaw_deg'],1)}°"
                            f"{std_suffix}"
                        )
                    next_emit = now + period

            except KeyboardInterrupt:
                print("\n[monitor] stopping."); break
            except Exception as e:
                print(f"[monitor] connection/logging error: {e}. Reconnecting in 2s…")
                time.sleep(2)
                try: m.close()
                except: pass
                m = None
                while m is None:
                    try:
                        m = mavutil.mavlink_connection(args.port, baud=args.baud, autoreconnect=True)
                        # Re-request streams after reconnect
                        ensure_streams(m, args.req_hz)
                    except Exception as e2:
                        print(f"[monitor] reconnect failed: {e2}. Retrying…"); time.sleep(2)

if __name__ == "__main__":
    main()

