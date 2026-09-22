"""
Neosky Multilog Analyzer — Merged Desktop Edition v4.1
========================================================
Merges two prior tools into one standalone desktop app, plus a much wider
data extraction pass:

  1) The "rename by folder" step (previously a web chunked-upload service):
     every log discovered under a selected root folder is renamed in place
     to  "<parent_folder_name>_<original_filename>"  before analysis, so a
     flat Excel report still shows which mission/folder each log came from.
     Renaming is a same-directory os.rename() (metadata only, instant even
     for 30GB+ files) and is idempotent/resume-safe: a log already carrying
     its folder prefix is left alone.

  2) The multi-process log analyzer (previously a standalone Tkinter tool):
     every renamed log is parsed with pymavlink and every field of interest
     is reduced with O(1)-memory Welford streaming stats, then streamed into
     a chunked Excel report so raw sample data is never held in memory.

Field coverage note: the ERR-subsystem table and the EV event-ID table below
are carried over from this project's single-log web analyzer (analyzer.py in
andy8901/drone-log-analyzer), where they were empirically cross-checked
against real flight logs rather than guessed. A number of the fields
requested for this report simply do not exist in a standard ArduPilot
flight-controller ".bin" dataflash log (telemetry link/RSSI, gimbal, camera
recording state, payload/GCS status, per-cell battery voltages, EKF
covariance/variance internals) -- those are logged, if at all, by a
companion computer, a ground station, or a smart battery, none of which are
captured in the flight controller's own log. Rather than fabricate a number
for those, this tool fills them with the literal string
"N/A (not in ArduPilot logs)" so the report's column layout still matches
what was asked for, without silently making anything up.

Handles 30 GB+ of ArduPilot .bin logs with:
  • Queue any mix of individual files and multiple folders before starting --
    "Add Folder" can be clicked repeatedly to queue several folders (each
    scanned recursively) in the same run, alongside individually-added files
  • Folder-name-prefixed renaming (mission/folder context preserved in the
    flat report, done once per file, safe to re-run)
  • Multi-process parallel parsing  (N-1 CPU cores)
  • Streaming / chunked Excel output (never OOM)
  • Crash-safe resume  (pick same output folder, ticks resume checkbox)
  • Per-file error log  (bad log != job failure)
  • Memory-capped accumulation  (Welford online stats, zero data stored)
  • Broad field-name fallbacks  (old + new firmware variants)

Windows note: multiprocessing requires the script to be run as a file
(python neosky_multilog_analyzer.py), NOT via interactive interpreter.
"""

import gc
import math
import multiprocessing as mp
import os
import queue
import re
import threading
import traceback
from datetime import datetime, timedelta
from pathlib import Path

import pandas as pd
import tkinter as tk
from tkinter import filedialog, ttk, messagebox

# ── pymavlink ────────────────────────────────────────────────────────────────
try:
    from pymavlink import mavutil
    MAVUTIL_OK = True
except ImportError:
    mavutil = None
    MAVUTIL_OK = False


NA = "N/A (not in ArduPilot logs)"


# ═══════════════════════════════════════════════════════════════════════════════
#  STREAMING STATISTICS  — O(1) memory, Welford online algorithm
# ═══════════════════════════════════════════════════════════════════════════════

class StreamStats:
    __slots__ = ("n", "mean", "M2", "vmin", "vmax")

    def __init__(self):
        self.n    = 0
        self.mean = 0.0
        self.M2   = 0.0
        self.vmin =  math.inf
        self.vmax = -math.inf

    def update(self, x: float):
        if not math.isfinite(x):
            return
        self.n += 1
        delta     = x - self.mean
        self.mean += delta / self.n
        self.M2   += delta * (x - self.mean)
        if x < self.vmin: self.vmin = x
        if x > self.vmax: self.vmax = x

    def get(self, stat: str = "mean", decimals: int = 4) -> float:
        if self.n == 0:
            return 0.0
        if stat == "mean":
            return round(self.mean, decimals)
        if stat == "min":
            return round(self.vmin, decimals) if math.isfinite(self.vmin) else 0.0
        if stat == "max":
            return round(self.vmax, decimals) if math.isfinite(self.vmax) else 0.0
        if stat == "rms":
            var = self.M2 / self.n if self.n >= 1 else 0.0
            return round(math.sqrt(max(0.0, self.mean ** 2 + var)), decimals)
        return 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  ARDUPILOT REFERENCE TABLES
#  (carried over from analyzer.py in this project's single-log web analyzer,
#  where they were validated against real logs rather than guessed)
# ═══════════════════════════════════════════════════════════════════════════════

ERROR_SUBSYSTEMS = {
    1: "MAIN", 2: "RADIO", 3: "COMPASS", 4: "OPTFLOW",
    5: "RADIO FAILSAFE", 6: "BATTERY FAILSAFE", 7: "GPS FAILSAFE",
    8: "GCS FAILSAFE", 9: "FENCE FAILSAFE", 10: "FLIGHT MODE",
    11: "GPS", 12: "CRASH CHECK", 13: "FLIP", 14: "AUTOTUNE",
    15: "PARACHUTE", 16: "EKF CHECK", 17: "EKF/INAV FAILSAFE",
    18: "BAROMETER", 19: "CPU", 20: "ADSB FAILSAFE", 21: "TERRAIN",
    22: "NAVIGATION", 23: "TERRAIN FAILSAFE", 24: "EKF PRIMARY",
    25: "THRUST LOSS CHECK", 26: "SENSORS FAILSAFE", 27: "LEAK FAILSAFE",
    28: "PILOT INPUT", 29: "VIBRATION FAILSAFE", 30: "INTERNAL ERROR",
    31: "DEAD RECKONING FAILSAFE",
}

CRITICAL_SUBSYSTEMS = {5, 6, 7, 8, 9, 12, 13, 16, 17, 25, 26, 27, 29, 30, 31}
FAILSAFE_SUBSYSTEMS = {5, 6, 7, 8, 9}

# ArduPilot's dataflash "EV" message only logs a numeric event ID, no text.
# These specific IDs were cross-checked empirically against real flight logs
# (see analyzer.py). Any ID not in this table is shown as "Event ID <n>".
EV_EVENT_NAMES = {
    10: "ARMED",
    11: "DISARMED",
    15: "AUTO ARMED",
    17: "LAND COMPLETE MAYBE",
    18: "LAND COMPLETE",
    28: "NOT LANDED",
    56: "MOTOR INTERLOCK DISABLED",
    57: "MOTOR INTERLOCK ENABLED",
}

GPS_FIX_TYPES = {
    0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix",
    4: "DGPS", 5: "RTK Float", 6: "RTK Fixed",
}

_GPS_EPOCH = datetime(1980, 1, 6)
_GPS_UTC_LEAP_SECONDS = 18  # stable since Dec 2016 (no leap second added since)

TAKEOFF_ALT_THRESHOLD_M = 1.5


def gps_distance(lat1, lon1, lat2, lon2):
    """Same flat-earth approximation used by analyzer.py -- accurate enough
    for the local distances a single flight covers, and kept identical so
    home-range figures match between the two tools."""
    return math.sqrt((lat2 - lat1) ** 2 + (lon2 - lon1) ** 2) * 111139


def gps_week_to_utc(gwk, gms):
    try:
        return (_GPS_EPOCH + timedelta(weeks=int(gwk), milliseconds=int(gms))
                - timedelta(seconds=_GPS_UTC_LEAP_SECONDS))
    except (TypeError, ValueError, OverflowError):
        return None


# ═══════════════════════════════════════════════════════════════════════════════
#  FIELD-NAME LOOKUP / DATE HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def _first(raw: dict, *keys):
    for k in keys:
        v = raw.get(k)
        if v is not None:
            return v
    return None


_DATE_PATTERNS = (
    (re.compile(r"(20\d{2})[-_]?(\d{2})[-_]?(\d{2})"), "ymd"),
    (re.compile(r"(\d{2})[-_](\d{2})[-_](20\d{2})"), "dmy"),
)


def extract_date_from_filename(name: str):
    """Best-effort date parsed out of the (already folder-prefixed) filename
    itself, per the request to derive Date from the name rather than from
    log content. Returns None if no recognizable date pattern is found."""
    for pattern, order in _DATE_PATTERNS:
        m = pattern.search(name)
        if not m:
            continue
        try:
            if order == "ymd":
                y, mo, d = (int(g) for g in m.groups())
            else:
                d, mo, y = (int(g) for g in m.groups())
            if 1 <= mo <= 12 and 1 <= d <= 31:
                return f"{y:04d}-{mo:02d}-{d:02d}"
        except ValueError:
            continue
    return None


# ═══════════════════════════════════════════════════════════════════════════════
#  FOLDER-PREFIXED RENAME  — same-directory, idempotent, resume-safe
# ═══════════════════════════════════════════════════════════════════════════════

def rename_by_folder(path: str):
    """Rename `path` in place to '<parent_folder>_<original_name>'.

    Returns (new_path, error). error is None on success (including the
    no-op case where the file already carries its folder prefix, which
    makes this safe to call again on a folder that was partly processed
    before a crash).
    """
    parent = os.path.dirname(path)
    folder = os.path.basename(parent) or "LOG"
    base   = os.path.basename(path)
    prefix = f"{folder}_"

    if base.startswith(prefix):
        return path, None  # already renamed

    new_base = prefix + base
    new_path = os.path.join(parent, new_base)

    if os.path.exists(new_path):
        stem, ext = os.path.splitext(new_base)
        i = 1
        while os.path.exists(new_path):
            new_path = os.path.join(parent, f"{stem}_{i}{ext}")
            i += 1

    try:
        os.rename(path, new_path)
        return new_path, None
    except OSError as exc:
        return path, str(exc)


# ═══════════════════════════════════════════════════════════════════════════════
#  SINGLE-LOG PARSER  — runs inside a worker process
# ═══════════════════════════════════════════════════════════════════════════════

def analyze_single_log(path: str):
    """Parse one .bin and return a flat metrics dict, or None on failure."""
    try:
        mlog = mavutil.mavlink_connection(path, robust_parsing=True)
    except Exception as exc:
        return {"_error": f"connection failed: {exc}"}

    ss = {k: StreamStats() for k in (
        "sats", "hdop", "gspeed", "hacc", "vacc",
        "volt", "curr", "power", "balt", "relalt", "galt", "rf_all",
        "ax", "ay", "az", "gx", "gy", "gz",
        "vibe_x", "vibe_y", "vibe_z", "vib",
        "roll", "pitch", "yaw", "roll_des", "pitch_des", "yaw_des",
        "roll_err", "pitch_err", "yaw_err", "alt_des", "alt_err",
        "gspeed_des",
        "rc_roll", "rc_pitch", "rc_throttle", "rc_yaw", "rc_rssi",
        "esc_volt", "esc_curr", "esc_temp",
        "mout1", "mout2", "mout3", "mout4",
    )}
    rf_inst    = {i: StreamStats() for i in range(6)}
    motor_rpm  = {i: StreamStats() for i in range(4)}  # ESC Instance 0-3 -> Motor1-4

    lat = lon = None                      # first GPS fix (kept for back-compat)
    home_lat = home_lon = None
    prev_lat = prev_lon = None
    cur_lat = cur_lon = None
    distance_travelled = 0.0
    max_home_distance = 0.0
    last_fix_status = None
    last_course = None

    t1 = t2 = None
    log_start_utc = log_end_utc = None

    home_baro = None
    last_alt_t = last_alt_val = None
    max_climb_rate = 0.0
    max_descent_rate = 0.0

    armed_since = None
    armed_usec_total = 0
    first_arm_t = None
    last_disarm_t = None
    arm_count = 0
    disarm_count = 0
    emergency_disarm_count = 0

    takeoff_t = landing_t = None
    takeoff_lat = takeoff_lon = None
    landing_lat = landing_lon = None

    modes_seen = []
    mode_change_count = 0
    rtl_events = 0
    autoland_events = 0

    param_change_count = 0

    ekf_err_count = 0
    comp_err_count = 0
    gps_glitch_count = 0
    gps_glitch_open_t = None
    gps_glitch_total = 0.0
    failsafe_count = 0
    failsafe_types = []
    failsafe_open_t = None
    failsafe_total = 0.0
    radio_fs_count = 0
    radio_fs_open_t = None
    radio_fs_total = 0.0

    clip0_last = clip1_last = clip2_last = 0

    total_critical = 0
    total_warning = 0
    critical_msgs, warning_msgs, error_msgs, event_msgs, all_msgs = [], [], [], [], []
    critical_ts = []
    thrust_loss_msgs = []
    reboot_count = 0
    firmware_version = None
    gps_fw_info = None
    lidar_6 = "Missing"

    MSG_CAP = 20

    while True:
        try:
            msg = mlog.recv_match(blocking=False)
        except Exception:
            break
        if msg is None:
            break

        mtype = msg.get_type()
        if mtype in ("BAD_DATA", "UNKNOWN") or mtype is None:
            continue
        try:
            raw: dict = msg.to_dict()
        except Exception:
            continue

        # Always normalize to microseconds -- TimeUS is already microseconds;
        # TimeMS (only used when TimeUS is absent) is milliseconds and needs
        # scaling. Guessing the unit from magnitude (the previous approach)
        # misclassifies short flights, since a short flight's TimeUS value
        # can itself be numerically small.
        t = raw.get("TimeUS")
        if t is None:
            t_ms = raw.get("TimeMS")
            t = t_ms * 1000 if t_ms is not None else None
        if t:
            if t1 is None: t1 = t
            t2 = t

        # ── Rangefinder ──────────────────────────────────────────────────
        if mtype in ("RFND", "RNGF", "RNG"):
            dist = _first(raw, "Dist", "Distance", "D")
            inst = int(_first(raw, "Instance", "I", "Inst") or 0)
            if dist is not None and dist > 0:
                ss["rf_all"].update(float(dist))
                if inst in rf_inst:
                    rf_inst[inst].update(float(dist))

        # ── Error / failsafe events ─────────────────────────────────────
        elif mtype == "ERR":
            sub = raw.get("Subsys", -1)
            code = raw.get("ECode", raw.get("Code", 0)) or 0
            t_sec = (t - t1) / 1e6 if (t and t1) else None

            if sub == 11:  # GPS
                if code != 0:
                    gps_glitch_count += 1
                    if gps_glitch_open_t is None:
                        gps_glitch_open_t = t_sec
                elif gps_glitch_open_t is not None and t_sec is not None:
                    gps_glitch_total += (t_sec - gps_glitch_open_t)
                    gps_glitch_open_t = None
            elif sub == 16 and code != 0:  # EKF CHECK
                ekf_err_count += 1
            elif sub == 3 and code != 0:  # COMPASS
                comp_err_count += 1

            if sub == 5:  # RADIO FAILSAFE
                if code != 0:
                    radio_fs_count += 1
                    if radio_fs_open_t is None:
                        radio_fs_open_t = t_sec
                elif radio_fs_open_t is not None and t_sec is not None:
                    radio_fs_total += (t_sec - radio_fs_open_t)
                    radio_fs_open_t = None

            if sub in FAILSAFE_SUBSYSTEMS:
                if code != 0:
                    failsafe_count += 1
                    name = ERROR_SUBSYSTEMS.get(sub, str(sub))
                    if name not in failsafe_types:
                        failsafe_types.append(name)
                    if failsafe_open_t is None:
                        failsafe_open_t = t_sec
                elif failsafe_open_t is not None and t_sec is not None:
                    failsafe_total += (t_sec - failsafe_open_t)
                    failsafe_open_t = None

            if code != 0:
                name = ERROR_SUBSYSTEMS.get(sub, f"SUBSYSTEM {sub}")
                severity = "CRITICAL" if sub in CRITICAL_SUBSYSTEMS else "WARNING"
                text = f"{severity}: {name} ERROR (code {code})"
                if severity == "CRITICAL":
                    total_critical += 1
                    if len(critical_msgs) < MSG_CAP: critical_msgs.append(text)
                    if len(critical_ts) < MSG_CAP and t_sec is not None:
                        critical_ts.append(round(t_sec, 1))
                else:
                    total_warning += 1
                    if len(warning_msgs) < MSG_CAP: warning_msgs.append(text)
                if len(error_msgs) < MSG_CAP: error_msgs.append(text)

        elif mtype == "MSG":
            txt = raw.get("Message", "")
            if txt:
                if len(all_msgs) < MSG_CAP: all_msgs.append(txt)
                low = txt.lower()

                if "thrust loss" in low and len(thrust_loss_msgs) < MSG_CAP:
                    thrust_loss_msgs.append(txt)

                if "disarm" in low:
                    if armed_since is not None:
                        armed_usec_total += (t - armed_since) if t else 0
                        armed_since = None
                        disarm_count += 1
                        last_disarm_t = t
                    if "crash" in low:
                        emergency_disarm_count += 1
                        if len(critical_msgs) < MSG_CAP:
                            critical_msgs.append(txt)
                elif "armed" in low and "disarmed" not in low:
                    if armed_since is None:
                        armed_since = t
                        arm_count += 1
                        if first_arm_t is None:
                            first_arm_t = t

                if firmware_version is None and any(
                    v in txt for v in ("ArduCopter", "ArduPlane", "ArduRover", "ArduSub", "ArduBlimp")
                ):
                    firmware_version = txt
                    reboot_count += 1
                if gps_fw_info is None and "GPS" in txt.upper():
                    gps_fw_info = txt

        elif mtype == "EV":
            eid = raw.get("Id")
            name = EV_EVENT_NAMES.get(eid)
            if len(event_msgs) < MSG_CAP:
                event_msgs.append(name or f"Event ID {eid}")

            if eid == 10:
                if armed_since is None:
                    armed_since = t
                    arm_count += 1
                    if first_arm_t is None:
                        first_arm_t = t
            elif eid == 11:
                if armed_since is not None:
                    armed_usec_total += (t - armed_since) if t else 0
                    armed_since = None
                    disarm_count += 1
                    last_disarm_t = t

        # ── Flight mode ──────────────────────────────────────────────────
        elif mtype == "MODE":
            mode_name = mlog.flightmode or f"MODE {raw.get('Mode')}"
            if mode_name not in modes_seen:
                modes_seen.append(mode_name)
            mode_change_count += 1
            if mode_name == "RTL":
                rtl_events += 1
            elif mode_name == "LAND":
                autoland_events += 1

        # ── Parameters ───────────────────────────────────────────────────
        elif mtype == "PARM":
            param_change_count += 1

        # ── GPS ───────────────────────────────────────────────────────────
        # Exact match only (not e.g. "GPS2"/"GPS_RAW_INT2"): a secondary GPS
        # instance can disagree with the primary receiver, and merging both
        # into one running home-point / distance-travelled / sat-count
        # stream would corrupt all three. analyzer.py's own single-log
        # tool makes the same choice (GPS/GPS_RAW_INT only).
        elif mtype in ("GPS", "GPS_RAW_INT"):
            gwk, gms = raw.get("GWk"), raw.get("GMS")
            if gwk is not None and gms is not None:
                utc = gps_week_to_utc(gwk, gms)
                if utc is not None:
                    if log_start_utc is None:
                        log_start_utc = utc
                    log_end_utc = utc

            status = raw.get("Status")
            if status is not None:
                last_fix_status = status

            rlat = _first(raw, "Lat")
            rlng = _first(raw, "Lng", "Lon")
            if rlat and rlng:
                latf, lonf = rlat / 1e7, rlng / 1e7
                cur_lat, cur_lon = latf, lonf
                if lat is None:
                    lat, lon = latf, lonf
                if home_lat is None and (status is None or status >= 3):
                    home_lat, home_lon = latf, lonf
                if prev_lat is not None:
                    distance_travelled += gps_distance(prev_lat, prev_lon, latf, lonf)
                prev_lat, prev_lon = latf, lonf
                if home_lat is not None:
                    d_home = gps_distance(home_lat, home_lon, latf, lonf)
                    if d_home > max_home_distance:
                        max_home_distance = d_home

            s = _first(raw, "NSats", "Sats", "NumSats")
            if s is not None: ss["sats"].update(float(s))
            h = _first(raw, "HDop", "HDOP", "Hdop")
            if h is not None: ss["hdop"].update(float(h))
            a = _first(raw, "Alt", "RelAlt")
            if a is not None: ss["galt"].update(float(a))
            spd = _first(raw, "Spd", "GroundSpeed")
            if spd is not None: ss["gspeed"].update(float(spd))
            crs = _first(raw, "GCrs", "Crs")
            if crs is not None: last_course = float(crs)

        elif mtype == "GPA":
            hacc = _first(raw, "HAcc")
            vacc = _first(raw, "VAcc")
            if hacc is not None: ss["hacc"].update(float(hacc))
            if vacc is not None: ss["vacc"].update(float(vacc))

        # ── IMU (raw accel/gyro) ─────────────────────────────────────────
        elif "IMU" in mtype:
            ax = _first(raw, "AccX", "ax") or 0.0
            ay = _first(raw, "AccY", "ay") or 0.0
            az = _first(raw, "AccZ", "az") or 0.0
            gx = _first(raw, "GyrX", "gx")
            gy = _first(raw, "GyrY", "gy")
            gz = _first(raw, "GyrZ", "gz")
            ss["ax"].update(float(ax)); ss["ay"].update(float(ay)); ss["az"].update(float(az))
            if gx is not None: ss["gx"].update(float(gx))
            if gy is not None: ss["gy"].update(float(gy))
            if gz is not None: ss["gz"].update(float(gz))

        # ── VIBE (ArduPilot's actual vibration metric + clip counters) ───
        elif mtype == "VIBE":
            vx, vy, vz = raw.get("VibeX"), raw.get("VibeY"), raw.get("VibeZ")
            if vx is not None: ss["vibe_x"].update(float(vx))
            if vy is not None: ss["vibe_y"].update(float(vy))
            if vz is not None: ss["vibe_z"].update(float(vz))
            if vx is not None and vy is not None and vz is not None:
                ss["vib"].update(math.sqrt(float(vx) ** 2 + float(vy) ** 2 + float(vz) ** 2))
            if raw.get("Clip0") is not None: clip0_last = raw["Clip0"]
            if raw.get("Clip1") is not None: clip1_last = raw["Clip1"]
            if raw.get("Clip2") is not None: clip2_last = raw["Clip2"]

        # ── Battery ───────────────────────────────────────────────────────
        elif "BAT" in mtype:
            v = _first(raw, "Volt", "V", "Voltage")
            c = _first(raw, "Curr", "I", "Current")
            if v is not None: ss["volt"].update(float(v))
            if c is not None: ss["curr"].update(float(c))
            if v is not None and c is not None:
                ss["power"].update(float(v) * float(c))

        # ── Barometer (+ derived relative alt / climb-descent rate) ─────
        elif mtype in ("BARO", "BAR2"):
            a = _first(raw, "Alt", "Altitude")
            if a is not None:
                a = float(a)
                ss["balt"].update(a)
                if home_baro is None:
                    home_baro = a
                rel = a - home_baro
                ss["relalt"].update(rel)

                if last_alt_t is not None and t is not None:
                    dt = (t - last_alt_t) / 1e6
                    if dt > 0:
                        rate = (a - last_alt_val) / dt
                        if rate > max_climb_rate: max_climb_rate = rate
                        if rate < max_descent_rate: max_descent_rate = rate
                last_alt_t, last_alt_val = t, a

                if armed_since is not None:
                    if rel > TAKEOFF_ALT_THRESHOLD_M:
                        if takeoff_t is None:
                            takeoff_t = t
                            takeoff_lat, takeoff_lon = cur_lat, cur_lon
                        landing_t = t
                        landing_lat, landing_lon = cur_lat, cur_lon

        # ── CTUN (desired-altitude target only) ──────────────────────────
        # Deliberately does NOT also update balt/relalt/climb-descent-rate:
        # CTUN's own "Alt"/"BAlt" fields are a different (EKF-blended, not
        # raw-pressure) altitude signal than the standalone BARO message.
        # Feeding both into one shared last-sample derivative produced
        # spurious multi-thousand m/s "climb rate" spikes whenever a CTUN
        # and a BARO message landed at nearly the same timestamp with a
        # slightly different value -- confirmed against a real flight log.
        # BARO/BAR2 alone (matching analyzer.py's precedent) stays the sole
        # source for altitude/climb-rate/takeoff-landing detection.
        elif mtype == "CTUN":
            dalt = _first(raw, "DSAlt", "DAlt", "DesAlt")
            if dalt is not None:
                dalt = float(dalt)
                ss["alt_des"].update(dalt)
                if last_alt_val is not None:
                    ss["alt_err"].update(abs(last_alt_val - dalt))

        # ── NTUN (desired ground speed, when logged) ─────────────────────
        elif mtype == "NTUN":
            dspd = _first(raw, "DVel", "DesSpeed", "DesGSpeed")
            if dspd is not None: ss["gspeed_des"].update(float(dspd))

        # ── Attitude (actual vs desired, tracking error) ─────────────────
        elif mtype == "ATT":
            roll  = _first(raw, "Roll")
            pitch = _first(raw, "Pitch")
            yaw   = _first(raw, "Yaw")
            droll  = _first(raw, "DesRoll")
            dpitch = _first(raw, "DesPitch")
            dyaw   = _first(raw, "DesYaw")

            if roll is not None: ss["roll"].update(float(roll))
            if pitch is not None: ss["pitch"].update(float(pitch))
            if yaw is not None: ss["yaw"].update(float(yaw))
            if droll is not None: ss["roll_des"].update(float(droll))
            if dpitch is not None: ss["pitch_des"].update(float(dpitch))
            if dyaw is not None: ss["yaw_des"].update(float(dyaw))

            if roll is not None and droll is not None:
                ss["roll_err"].update(abs(float(roll) - float(droll)))
            if pitch is not None and dpitch is not None:
                ss["pitch_err"].update(abs(float(pitch) - float(dpitch)))
            if yaw is not None and dyaw is not None:
                # Yaw wraps 0-360 (unlike roll/pitch), so e.g. actual=358 vs
                # desired=2 is really a 4 deg error, not 356 -- take the
                # minor arc around the circle.
                yaw_diff = abs(float(yaw) - float(dyaw))
                yaw_diff = min(yaw_diff, 360.0 - yaw_diff)
                ss["yaw_err"].update(yaw_diff)

        # ── RC input / link ───────────────────────────────────────────────
        elif mtype == "RCIN":
            for ch, key in ((1, "rc_roll"), (2, "rc_pitch"), (3, "rc_throttle"), (4, "rc_yaw")):
                v = raw.get(f"C{ch}")
                if v is not None: ss[key].update(float(v))

        elif mtype == "RSSI":
            v = _first(raw, "RSSI", "RXRSSI")
            if v is not None: ss["rc_rssi"].update(float(v))

        # ── Motor / ESC outputs & telemetry ──────────────────────────────
        elif mtype == "RCOU":
            for ch in (1, 2, 3, 4):
                v = raw.get(f"C{ch}")
                if v is not None: ss[f"mout{ch}"].update(float(v))

        elif mtype == "ESC":
            inst = int(_first(raw, "Instance", "I") or 0)
            rpm = raw.get("RPM")
            if rpm is not None and inst in motor_rpm:
                motor_rpm[inst].update(float(rpm))
            v, c, tmp = raw.get("Volt"), raw.get("Curr"), raw.get("Temp")
            if v is not None: ss["esc_volt"].update(float(v))
            if c is not None: ss["esc_curr"].update(float(c))
            if tmp is not None: ss["esc_temp"].update(float(tmp))

        # ── 6-side Proximity ──────────────────────────────────────────────
        elif mtype in ("PRX", "DISTAD", "PRX1"):
            valid = sum(1 for i in range(1, 7) if raw.get(f"D{i}") is not None)
            if valid >= 6:
                lidar_6 = "OK"
            elif valid > 0:
                lidar_6 = f"Partial ({valid}/6)"

    # Vehicle still armed / airborne at EOF: close out the open interval.
    if armed_since is not None and t2 is not None:
        armed_usec_total += (t2 - armed_since)

    if t1 is None or t2 is None:
        return {"_error": "no timestamp data — empty or corrupt"}

    delta   = t2 - t1
    dur_sec = round(delta / 1e6, 2)  # t1/t2 are always microseconds now
    armed_sec = round(armed_usec_total / 1e6, 2)

    def rel_or_utc(t_us, label_utc=True):
        if t_us is None:
            return "N/A"
        if log_start_utc is not None and t1 is not None:
            return (log_start_utc + timedelta(seconds=(t_us - t1) / 1e6)).strftime("%Y-%m-%d %H:%M:%S UTC")
        return f"+{round((t_us - t1) / 1e6, 2)}s" if t1 is not None else "N/A"

    filename = os.path.basename(path)
    date_str = extract_date_from_filename(filename)
    if date_str is None:
        try:
            date_str = datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d")
        except OSError:
            date_str = "N/A"

    motor_out_means = [ss[f"mout{n}"].get("mean") for n in (1, 2, 3, 4)]
    nonzero_motor_means = [m for m in motor_out_means if m]
    motor_imbalance = round(max(nonzero_motor_means) - min(nonzero_motor_means), 2) if nonzero_motor_means else 0.0

    report = {
        "Log Name": filename,
        "Date": date_str,
        "Log Start Time": (log_start_utc.strftime("%Y-%m-%d %H:%M:%S UTC") if log_start_utc else "N/A"),
        "Log End Time": (log_end_utc.strftime("%Y-%m-%d %H:%M:%S UTC") if log_end_utc else "N/A"),
        "Overall Endurance / Log Duration (sec)": dur_sec,
        "Arm Start Time": rel_or_utc(first_arm_t),
        "Disarm End Time": rel_or_utc(last_disarm_t),
        "Armed Flight Duration / Flight Endurance (sec)": armed_sec,
        "Flight Time (min)": round(armed_sec / 60, 2),
        "Arm Events Count": arm_count,
        "Disarm Events Count": disarm_count,
        "Flight Mode": " -> ".join(modes_seen) if modes_seen else "N/A",
        "Mode Change Count": mode_change_count,
        "Takeoff Time": rel_or_utc(takeoff_t),
        "Landing Time": rel_or_utc(landing_t),
        "Takeoff Latitude": takeoff_lat,
        "Takeoff Longitude": takeoff_lon,
        "Landing Latitude": landing_lat,
        "Landing Longitude": landing_lon,
        "Latitude": lat,
        "Longitude": lon,
        "GPS Fix Type": GPS_FIX_TYPES.get(last_fix_status, f"Unknown ({last_fix_status})") if last_fix_status is not None else "N/A",
        "Avg Sat Count": ss["sats"].get("mean"),
        "Min Sat Count": ss["sats"].get("min"),
        "Max Sat Count": ss["sats"].get("max"),
        "Max HDOP": ss["hdop"].get("max"),
        "Avg HDOP": ss["hdop"].get("mean"),
        "GPS Accuracy Horizontal": ss["hacc"].get("mean"),
        "GPS Accuracy Vertical": ss["vacc"].get("mean"),
        "GPS Ground Speed Avg": ss["gspeed"].get("mean"),
        "GPS Ground Speed Max": ss["gspeed"].get("max"),
        "GPS Course": round(last_course, 2) if last_course is not None else "N/A",
        "GPS Glitch Count": gps_glitch_count,
        "GPS Glitch Duration (sec)": round(gps_glitch_total, 2),
        "Distance Travelled (m)": round(distance_travelled, 1),
        "Max Home Distance (m)": round(max_home_distance, 1),
        "Max Baro Alt (m)": ss["balt"].get("max"),
        "Avg Baro Alt (m)": ss["balt"].get("mean"),
        "Max GPS Alt (m)": ss["galt"].get("max"),
        "Avg GPS Alt (m)": ss["galt"].get("mean"),
        "Max Relative Altitude (m)": ss["relalt"].get("max"),
        "Avg Relative Altitude (m)": ss["relalt"].get("mean"),
        "Max Climb Rate (m/s)": round(max_climb_rate, 2),
        "Max Descent Rate (m/s)": round(max_descent_rate, 2),
        "Avg Rangefinder (m)": ss["rf_all"].get("mean"),
        "Min Rangefinder (m)": ss["rf_all"].get("min"),
        "Max Rangefinder (m)": ss["rf_all"].get("max"),
    }

    for i in range(6):
        report[f"RFND_{i}_Avg (m)"] = rf_inst[i].get("mean")
        report[f"RFND_{i}_Max (m)"] = rf_inst[i].get("max")
        report[f"RFND_{i}_Min (m)"] = rf_inst[i].get("min")

    report.update({
        "Lidar / 6-Side Status": lidar_6,
        "Min Voltage (V)": ss["volt"].get("min"),
        "Max Voltage (V)": ss["volt"].get("max"),
        "Avg Voltage (V)": ss["volt"].get("mean"),
        "Avg Current (A)": ss["curr"].get("mean"),
        "Max Current (A)": ss["curr"].get("max"),
        "Battery SOC (%)": NA,
        "Consumed Battery (mAh)": NA,
        "Battery Temperature": NA,
        "Min Cell Voltage": NA,
        "Max Cell Voltage": NA,
        "Cell Voltage Difference": NA,
        "Battery Resistance": NA,
        "Max Power (W)": ss["power"].get("max"),
        "Battery Failsafe": "Yes" if radio_fs_count == 0 and "BATTERY FAILSAFE" in failsafe_types else "No",

        "ax_avg": ss["ax"].get("mean"), "ay_avg": ss["ay"].get("mean"), "az_avg": ss["az"].get("mean"),
        "ax_max": ss["ax"].get("max"), "ay_max": ss["ay"].get("max"), "az_max": ss["az"].get("max"),
        "gx_avg": ss["gx"].get("mean"), "gy_avg": ss["gy"].get("mean"), "gz_avg": ss["gz"].get("mean"),
        "gx_max": ss["gx"].get("max"), "gy_max": ss["gy"].get("max"), "gz_max": ss["gz"].get("max"),

        "Vibration X Avg": ss["vibe_x"].get("mean"),
        "Vibration Y Avg": ss["vibe_y"].get("mean"),
        "Vibration Z Avg": ss["vibe_z"].get("mean"),
        "vib_avg": ss["vib"].get("mean"),
        "vib_max": ss["vib"].get("max"),
        "vib_rms": ss["vib"].get("rms"),
        "IMU Clipping X": clip0_last,
        "IMU Clipping Y": clip1_last,
        "IMU Clipping Z": clip2_last,

        "Roll Max": ss["roll"].get("max"), "Roll Min": ss["roll"].get("min"),
        "Pitch Max": ss["pitch"].get("max"), "Pitch Min": ss["pitch"].get("min"),
        "Yaw Max": ss["yaw"].get("max"), "Yaw Min": ss["yaw"].get("min"),
        "Desired Roll Max": ss["roll_des"].get("max"),
        "Desired Pitch Max": ss["pitch_des"].get("max"),
        "Desired Yaw Max": ss["yaw_des"].get("max"),
        "Desired Altitude Max": ss["alt_des"].get("max"),
        "Roll Tracking Error": ss["roll_err"].get("max"),
        "Pitch Tracking Error": ss["pitch_err"].get("max"),
        "Yaw Tracking Error": ss["yaw_err"].get("max"),
        "Altitude Tracking Error": ss["alt_err"].get("max"),
        "Actual Climb Rate": round(max_climb_rate, 2),
        "Desired Climb Rate": NA,
        "Desired Ground Speed": ss["gspeed_des"].get("mean"),
        "Actual Ground Speed": ss["gspeed"].get("mean"),

        "RC Roll Input": ss["rc_roll"].get("mean"),
        "RC Pitch Input": ss["rc_pitch"].get("mean"),
        "RC Throttle Input": ss["rc_throttle"].get("mean"),
        "RC Yaw Input": ss["rc_yaw"].get("mean"),
        "RC RSSI": ss["rc_rssi"].get("mean"),
        "RC Link Quality": NA,
        "RC Failsafe": "Yes" if radio_fs_count > 0 else "No",
        "RC Signal Loss Count": radio_fs_count,
        "RC Signal Loss Duration (sec)": round(radio_fs_total, 2),
        "RC Override Events": NA,
    })

    for n in range(1, 5):
        report[f"Motor {n} RPM Avg"] = motor_rpm[n - 1].get("mean")
        report[f"Motor {n} RPM Min"] = motor_rpm[n - 1].get("min")
        report[f"Motor {n} RPM Max"] = motor_rpm[n - 1].get("max")
    for n in range(1, 5):
        report[f"Motor {n} Output Avg"] = ss[f"mout{n}"].get("mean")
        report[f"Motor {n} Output Max"] = ss[f"mout{n}"].get("max")

    report.update({
        "Motor Output Imbalance": motor_imbalance,
        "Motor/ESC Error Flags": NA,
        "ESC Voltage": ss["esc_volt"].get("mean"),
        "ESC Current": ss["esc_curr"].get("mean"),
        "ESC Temperature": ss["esc_temp"].get("mean"),
        "ESC RPM Error": NA,
        "Thrust Saturation Events": NA,
        "Potential Thrust Loss": " | ".join(thrust_loss_msgs) or "None",

        "EKF Error Count": ekf_err_count,
        "EKF Reset Count": NA,
        "EKF Lane Switches": NA,
        "Active EKF Lane": NA,
        "EKF Source": NA,
        "EKF Velocity Variance": NA,
        "EKF Position Variance": NA,
        "EKF Height Variance": NA,
        "EKF Compass Variance": NA,
        "EKF Yaw Variance": NA,
        "Yaw Reset Events": NA,
        "Compass Error Count": comp_err_count,
        "Compass Variance": NA,
        "Compass Health": NA,

        "Failsafe Event Count": failsafe_count,
        "Failsafe Type": " | ".join(failsafe_types) if failsafe_types else "None",
        "Failsafe Duration (sec)": round(failsafe_total, 2),
        "RTL Events": rtl_events,
        "Auto Land Events": autoland_events,
        "Emergency Stop / Disarm Events": emergency_disarm_count,

        "Telemetry RSSI": NA,
        "Telemetry Link Quality": NA,
        "Telemetry Packet Loss": NA,
        "Telemetry Link Loss Count": NA,
        "Telemetry Link Loss Duration": NA,
        "GCS Connection Status": NA,
        "GCS Latency": NA,
        "Gimbal Pitch": NA,
        "Gimbal Yaw": NA,
        "Camera Recording Status": NA,
        "Payload Communication Status": NA,
        "Payload Health": NA,
        "Sensor Health": NA,

        "Critical Messages": " | ".join(critical_msgs) or (" | ".join(all_msgs[-5:]) or "None"),
        "Error Messages": " | ".join(error_msgs) or "None",
        "Warning Messages": " | ".join(warning_msgs) or "None",
        "Event Messages": " | ".join(event_msgs) or "None",
        "Parameter Change Events": param_change_count,
        "Reboot Events": reboot_count,
        "Firmware Version": firmware_version or "N/A",
        "Flight Controller Version": firmware_version or "N/A",
        "GPS Module / Firmware Information": gps_fw_info or "N/A",
        "Total Error Count": total_critical + total_warning,
        "Total Warning Count": total_warning,
        "Critical Event Timestamp(s)": ", ".join(str(x) for x in critical_ts) if critical_ts else "None",
    })

    return report


# ═══════════════════════════════════════════════════════════════════════════════
#  WORKER  — top-level so multiprocessing can pickle it on Windows
# ═══════════════════════════════════════════════════════════════════════════════

def _worker(args):
    path, resume_set = args
    basename = os.path.basename(path)
    if basename in resume_set:
        return None, basename, "SKIPPED"
    try:
        result = analyze_single_log(path)
        if result is None:
            return None, basename, "EMPTY"
        if "_error" in result:
            return None, basename, f"ERROR: {result['_error']}"
        return result, basename, "OK"
    except Exception:
        return None, basename, f"EXCEPTION: {traceback.format_exc()}"


# ═══════════════════════════════════════════════════════════════════════════════
#  CHUNKED EXCEL WRITER
# ═══════════════════════════════════════════════════════════════════════════════

class ChunkedExcelWriter:
    CHUNK_ROWS = 200   # lower = safer on low RAM

    def __init__(self, path: str):
        self.path   = path
        self.buffer: list = []
        self._wrote_header = False

    def add(self, row: dict):
        self.buffer.append(row)
        if len(self.buffer) >= self.CHUNK_ROWS:
            self.flush()

    def flush(self):
        if not self.buffer:
            return
        df = pd.DataFrame(self.buffer)
        if not self._wrote_header or not os.path.exists(self.path):
            df.to_excel(self.path, index=False, engine="openpyxl")
            self._wrote_header = True
        else:
            with pd.ExcelWriter(self.path, engine="openpyxl",
                                mode="a", if_sheet_exists="overlay") as ew:
                existing_rows = len(pd.read_excel(self.path, engine="openpyxl"))
                df.to_excel(ew, index=False, header=False,
                            startrow=existing_rows + 1)
        self.buffer.clear()
        gc.collect()


# ═══════════════════════════════════════════════════════════════════════════════
#  GUI
# ═══════════════════════════════════════════════════════════════════════════════

class NeoskyMultilogAnalyzer:
    WORKERS = max(1, mp.cpu_count() - 1)
    LOG_EXT = ".bin"

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Neosky Multilog Analyzer  v4.1")
        self.root.geometry("760x640")
        self.root.configure(bg="#f0f3f4")
        self.gui_queue: queue.Queue = queue.Queue()
        self._active   = False
        self._pool     = None          # reference so Stop can terminate it
        self._pending_folders: list = []   # folders queued via "Add Folder" (multiple allowed)
        self._pending_files: list = []     # individually-picked files queued via "Add Files"
        self._build_ui()
        self.root.after(150, self._drain_queue)

    # ── UI ───────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # header
        hdr = tk.Frame(self.root, bg="#1a252f", height=65)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)
        tk.Label(hdr, text="NEOSKY MULTILOG ANALYZER",
                 fg="#ecf0f1", bg="#1a252f",
                 font=("Segoe UI", 15, "bold")).pack(side="left", padx=20, pady=15)
        tk.Label(hdr, text=f"v4.1  —  {self.WORKERS} parallel workers",
                 fg="#7f8c8d", bg="#1a252f",
                 font=("Segoe UI", 9)).pack(side="right", padx=18, pady=20)

        body = tk.Frame(self.root, bg="#f0f3f4")
        body.pack(expand=True, fill="both", padx=28, pady=16)

        # ── output folder ──
        frm_out = tk.Frame(body, bg="#f0f3f4")
        frm_out.pack(fill="x", pady=(0, 6))
        tk.Label(frm_out, text="Report output folder:", bg="#f0f3f4",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        self._out_var = tk.StringVar(value=str(Path.home() / "Downloads"))
        tk.Entry(frm_out, textvariable=self._out_var, width=44,
                 font=("Segoe UI", 9)).pack(side="left", padx=8)
        tk.Button(frm_out, text="Browse…", command=self._pick_output_folder,
                  bg="#34495e", fg="white", relief="flat",
                  font=("Segoe UI", 9), cursor="hand2",
                  padx=8).pack(side="left")

        # ── resume checkbox ──
        self._resume_var = tk.BooleanVar(value=True)
        tk.Checkbutton(body,
                       text="Resume: skip logs already present in the output report",
                       variable=self._resume_var,
                       bg="#f0f3f4", font=("Segoe UI", 9)).pack(anchor="w", pady=(0, 2))

        tk.Label(body,
                 text="Each log is first renamed in place to "
                      "\"<folder name>_<log name>\" (skipped if already "
                      "renamed), then parsed and added as a row in the Excel report. "
                      "Fields with no source in a standard ArduPilot .bin log are "
                      "marked \"N/A (not in ArduPilot logs)\" rather than guessed.",
                 bg="#f0f3f4", fg="#5d6d7e", font=("Segoe UI", 8, "italic"),
                 wraplength=700, justify="left").pack(anchor="w", pady=(0, 10))

        # ── selection queue (multiple folders and/or files can be added) ──
        tk.Label(body,
                 text="Add one or more folders and/or files, then start. "
                      "Each \"Add Folder\" click queues another folder (scanned recursively) "
                      "without clearing what's already queued.",
                 bg="#f0f3f4", fg="#5d6d7e", font=("Segoe UI", 8, "italic"),
                 wraplength=700, justify="left").pack(anchor="w", pady=(0, 4))

        add_row = tk.Frame(body, bg="#f0f3f4")
        add_row.pack()
        self.btn_add_files = tk.Button(
            add_row, text="+  ADD FILES (.bin)",
            command=self._add_files,
            height=2, width=20,
            bg="#2980b9", fg="white",
            font=("Segoe UI", 10, "bold"),
            relief="flat", cursor="hand2",
        )
        self.btn_add_files.pack(side="left", padx=4)

        self.btn_add_folder = tk.Button(
            add_row, text="📁  ADD FOLDER (recursive)",
            command=self._add_folder,
            height=2, width=24,
            bg="#2980b9", fg="white",
            font=("Segoe UI", 10, "bold"),
            relief="flat", cursor="hand2",
        )
        self.btn_add_folder.pack(side="left", padx=4)

        self.btn_clear = tk.Button(
            add_row, text="✕  CLEAR",
            command=self._clear_selection,
            height=2, width=10,
            bg="#7f8c8d", fg="white",
            font=("Segoe UI", 10, "bold"),
            relief="flat", cursor="hand2",
        )
        self.btn_clear.pack(side="left", padx=4)

        self.lbl_selection = tk.Label(body, text="Nothing queued yet.",
                                      bg="#f0f3f4", fg="#2c3e50",
                                      font=("Segoe UI", 9, "bold"))
        self.lbl_selection.pack(pady=(6, 8))

        # ── action buttons ──
        btn_row = tk.Frame(body, bg="#f0f3f4")
        btn_row.pack()
        self.btn_start = tk.Button(
            btn_row, text="▶   START ANALYSIS",
            command=self._start_analysis,
            height=2, width=22,
            bg="#27ae60", fg="white",
            font=("Segoe UI", 10, "bold"),
            relief="flat", cursor="hand2",
        )
        self.btn_start.pack(side="left", padx=4)

        self.btn_stop = tk.Button(
            btn_row, text="⏹  STOP",
            command=self._stop,
            height=2, width=10,
            bg="#c0392b", fg="white",
            font=("Segoe UI", 10, "bold"),
            relief="flat", cursor="hand2",
            state="disabled",
        )
        self.btn_stop.pack(side="left", padx=4)

        # ── progress label + bar ──
        self.lbl_progress = tk.Label(body, text="Ready — add folders/files above, then start.",
                                     bg="#f0f3f4", font=("Segoe UI", 10))
        self.lbl_progress.pack(pady=(14, 2))

        self.pbar = ttk.Progressbar(body, length=680, mode="determinate")
        self.pbar.pack(pady=4)

        self.lbl_detail = tk.Label(body, text="",
                                   bg="#f0f3f4", fg="#5d6d7e",
                                   font=("Segoe UI", 9, "italic"))
        self.lbl_detail.pack()

        # ── live log ──
        log_frm = tk.Frame(body, bg="#f0f3f4")
        log_frm.pack(fill="both", expand=True, pady=(12, 0))
        tk.Label(log_frm, text="Processing log:",
                 bg="#f0f3f4", font=("Segoe UI", 9, "bold"),
                 anchor="w").pack(anchor="w")
        inner = tk.Frame(log_frm)
        inner.pack(fill="both", expand=True)
        self.log_box = tk.Text(inner, font=("Consolas", 8),
                               bg="#1e272e", fg="#dfe6e9",
                               state="disabled", relief="flat",
                               wrap="none")
        sb_y = tk.Scrollbar(inner, orient="vertical",   command=self.log_box.yview)
        sb_x = tk.Scrollbar(inner, orient="horizontal", command=self.log_box.xview)
        self.log_box.configure(yscrollcommand=sb_y.set, xscrollcommand=sb_x.set)
        sb_y.pack(side="right",  fill="y")
        sb_x.pack(side="bottom", fill="x")
        self.log_box.pack(fill="both", expand=True)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _pick_output_folder(self):
        folder = filedialog.askdirectory(title="Select output folder")
        if folder:
            self._out_var.set(folder)

    def _log(self, text: str):
        self.log_box.config(state="normal")
        self.log_box.insert("end", text + "\n")
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    def _log_clear(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.config(state="disabled")

    def _set_running(self, running: bool):
        if running:
            self.btn_add_files.config(state="disabled", bg="#7f8c8d", cursor="arrow")
            self.btn_add_folder.config(state="disabled", bg="#7f8c8d", cursor="arrow")
            self.btn_clear.config(state="disabled", bg="#7f8c8d", cursor="arrow")
            self.btn_start.config(state="disabled", bg="#7f8c8d", cursor="arrow")
            self.btn_stop.config(state="normal",   bg="#c0392b", cursor="hand2")
        else:
            self.btn_add_files.config(state="normal",   bg="#2980b9", cursor="hand2")
            self.btn_add_folder.config(state="normal",   bg="#2980b9", cursor="hand2")
            self.btn_clear.config(state="normal",   bg="#7f8c8d", cursor="hand2")
            self.btn_start.config(state="normal",   bg="#27ae60", cursor="hand2")
            self.btn_stop.config(state="disabled", bg="#7f8c8d", cursor="arrow")

    # ── selection queue ──────────────────────────────────────────────────────

    def _update_selection_label(self):
        nf, nfl = len(self._pending_files), len(self._pending_folders)
        if nf == 0 and nfl == 0:
            self.lbl_selection.config(text="Nothing queued yet.")
        else:
            self.lbl_selection.config(
                text=f"Queued: {nfl} folder(s) (scanned recursively) + {nf} individual file(s)")

    def _add_files(self):
        if not MAVUTIL_OK:
            messagebox.showerror("Missing dependency",
                                 "pymavlink is not installed.\n\n"
                                 "Run:  pip install pymavlink")
            return
        files = filedialog.askopenfilenames(
            title="Add ArduPilot .bin log files (you can add more afterwards)",
            filetypes=[("ArduPilot Logs", "*.bin"), ("All files", "*.*")],
        )
        if not files:
            return
        added = 0
        for f in files:
            if f not in self._pending_files:
                self._pending_files.append(f)
                added += 1
        self._log(f"+ {added} file(s) added to the queue.")
        self._update_selection_label()

    def _add_folder(self):
        if not MAVUTIL_OK:
            messagebox.showerror("Missing dependency",
                                 "pymavlink is not installed.\n\n"
                                 "Run:  pip install pymavlink")
            return
        folder = filedialog.askdirectory(
            title="Add a folder of logs (scanned recursively) — click Add Folder again for more")
        if not folder:
            return
        if folder in self._pending_folders:
            self._log(f"(already queued) {folder}")
            return
        self._pending_folders.append(folder)
        self._log(f"+ folder queued: {folder}")
        self._update_selection_label()

    def _clear_selection(self):
        self._pending_folders.clear()
        self._pending_files.clear()
        self._update_selection_label()
        self._log_clear()
        self._log("Selection cleared.")

    def _start_analysis(self):
        if not self._pending_folders and not self._pending_files:
            messagebox.showwarning("Nothing selected",
                                   "Add at least one folder or file first.")
            return

        files = list(self._pending_files)
        for folder in self._pending_folders:
            for dirpath, _dirnames, filenames in os.walk(folder):
                for fn in filenames:
                    if fn.lower().endswith(self.LOG_EXT):
                        files.append(os.path.join(dirpath, fn))

        # De-dupe (a file could be reachable both directly and via a queued
        # folder) while preserving the order picked.
        seen = set()
        unique_files = []
        for f in files:
            key = os.path.realpath(f)
            if key not in seen:
                seen.add(key)
                unique_files.append(f)

        if not unique_files:
            messagebox.showwarning("No logs found",
                                   f"No {self.LOG_EXT} files found in the queued folders/files.")
            return
        self._begin(unique_files)

    def _begin(self, files: list):
        out_dir = self._out_var.get().strip() or str(Path.home())
        os.makedirs(out_dir, exist_ok=True)

        # Fixed filename so resume works across sessions
        out_path = os.path.join(out_dir, "Neosky_Report.xlsx")
        err_path = os.path.join(out_dir, "Neosky_Errors.txt")

        self._active = True
        self._set_running(True)
        self.pbar["value"]   = 0
        self.pbar["maximum"] = len(files)
        self._log_clear()
        self._log(f"Output → {out_path}")
        self._log(f"Files selected: {len(files)}   Workers: {self.WORKERS}")
        self.lbl_progress.config(text=f"Starting… 0 / {len(files)}")

        threading.Thread(
            target=self._run_batch,
            args=(files, out_path, err_path, self._resume_var.get()),
            daemon=True,
        ).start()

    def _stop(self):
        self._active = False
        if self._pool is not None:
            try:
                self._pool.terminate()
            except Exception:
                pass
        self._log("⚠  Stop requested.")

    # ── batch worker (background thread) ─────────────────────────────────────

    def _rename_files(self, files: list) -> tuple:
        """Rename each log to '<folder>_<name>' in place. Returns
        (renamed_paths, error_lines). A file that fails to rename is still
        analyzed under its original path so the batch keeps going."""
        renamed = []
        errors  = []
        self.gui_queue.put({"type": "LOG", "text": "Renaming logs by folder…"})
        for f in files:
            if not self._active:
                renamed.append(f)
                continue
            new_path, err = rename_by_folder(f)
            if err:
                errors.append(f"{os.path.basename(f)}: RENAME FAILED — {err}")
                self.gui_queue.put({"type": "LOG",
                    "text": f"✗ rename failed: {os.path.basename(f)} ({err})"})
            elif new_path != f:
                self.gui_queue.put({"type": "LOG",
                    "text": f"↳ renamed: {os.path.basename(f)} → {os.path.basename(new_path)}"})
            renamed.append(new_path)
        return renamed, errors

    def _run_batch(self, files: list, out_path: str, err_path: str, resume: bool):
        files, rename_errors = self._rename_files(files)

        # Build resume set
        resume_set: set = set()
        if resume and os.path.exists(out_path):
            try:
                df_ex = pd.read_excel(out_path, usecols=["Log Name"], engine="openpyxl")
                resume_set = set(df_ex["Log Name"].dropna().astype(str))
                self.gui_queue.put({"type": "LOG",
                    "text": f"Resume: {len(resume_set)} logs already done — skipping them."})
            except Exception as exc:
                self.gui_queue.put({"type": "LOG",
                    "text": f"Could not read existing report ({exc}). Starting fresh."})

        writer      = ChunkedExcelWriter(out_path)
        error_lines: list = list(rename_errors)
        done        = 0
        total       = len(files)

        try:
            # Use 'fork' on Linux/macOS (fast), 'spawn' on Windows (required)
            ctx_method = "spawn" if os.name == "nt" else "fork"
            ctx = mp.get_context(ctx_method)
            self._pool = ctx.Pool(processes=self.WORKERS)

            args_iter = ((f, resume_set) for f in files)
            for result, name, status in self._pool.imap_unordered(
                    _worker, args_iter, chunksize=2):

                if not self._active:
                    break

                done += 1
                self.gui_queue.put({
                    "type":   "PROGRESS",
                    "value":  done,
                    "total":  total,
                    "name":   name,
                    "status": status,
                })

                if status == "OK" and result:
                    writer.add(result)
                elif status not in ("SKIPPED", "OK"):
                    error_lines.append(f"{name}: {status}")

        except Exception as exc:
            self.gui_queue.put({"type": "LOG",
                "text": f"Pool error: {exc}\n{traceback.format_exc()}"})
        finally:
            if self._pool:
                self._pool.close()
                self._pool.join()
                self._pool = None

        writer.flush()

        if error_lines:
            with open(err_path, "w", encoding="utf-8") as f:
                f.write("\n".join(error_lines))

        ok_count  = done - (len(error_lines) - len(rename_errors))
        if os.path.exists(out_path) and ok_count > 0:
            self.gui_queue.put({
                "type":    "DONE",
                "out":     out_path,
                "ok":      ok_count,
                "errors":  len(error_lines),
                "err_log": err_path if error_lines else None,
                "total":   total,
            })
        else:
            self.gui_queue.put({"type": "ERROR",
                "text": "No results saved — all files failed or were skipped.\n"
                        f"Check {err_path} for details."})

    # ── GUI queue drain (main thread, every 150 ms) ───────────────────────────

    def _drain_queue(self):
        try:
            while True:
                msg = self.gui_queue.get_nowait()
                t   = msg["type"]

                if t == "PROGRESS":
                    v, total = msg["value"], msg["total"]
                    name, status = msg["name"], msg["status"]
                    self.pbar["value"] = v
                    pct = int(v / total * 100) if total else 0
                    self.lbl_progress.config(
                        text=f"Processing: {v} / {total}  ({pct}%)")
                    icon = {"OK": "✓", "SKIPPED": "⊘", "EMPTY": "∅"}.get(
                        status.split(":")[0], "✗")
                    self._log(f"{icon} [{v}/{total}] {name}  →  {status}")
                    self.lbl_detail.config(text=f"Last: {name}")

                elif t == "LOG":
                    self._log(msg["text"])

                elif t == "DONE":
                    self._active = False
                    self._set_running(False)
                    self.lbl_progress.config(
                        text=f"Done! {msg['ok']}/{msg['total']} logs processed.")
                    summary = (f"✅  Report saved:\n{msg['out']}\n\n"
                               f"Processed: {msg['ok']}  |  Errors: {msg['errors']}")
                    if msg["err_log"]:
                        summary += f"\n\nError details:\n{msg['err_log']}"
                    messagebox.showinfo("Complete", summary)

                elif t == "ERROR":
                    self._active = False
                    self._set_running(False)
                    self.lbl_progress.config(text="Failed — see log.")
                    messagebox.showwarning("No output", msg["text"])

        except queue.Empty:
            pass
        finally:
            self.root.after(150, self._drain_queue)


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    mp.freeze_support()          # required for Windows PyInstaller / spawn
    root = tk.Tk()
    app  = NeoskyMultilogAnalyzer(root)
    root.mainloop()
