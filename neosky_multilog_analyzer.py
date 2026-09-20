"""
Neosky Multilog Analyzer — Merged Desktop Edition v3.0
========================================================
Merges two prior tools into one standalone desktop app:

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

Handles 30 GB+ of ArduPilot .bin logs with:
  • Select individual files, OR select a root folder and recurse into it
  • Folder-name-prefixed renaming (mission/folder context preserved in the
    flat report, done once per file, safe to re-run)
  • Multi-process parallel parsing  (N-1 CPU cores)
  • Streaming / chunked Excel output (never OOM)
  • Crash-safe resume  (pick same output folder, ticks resume checkbox)
  • Per-file error log  (bad log != job failure)
  • Memory-capped accumulation  (Welford online stats, zero data stored)
  • Correct vib_rms calculation
  • Broad field-name fallbacks  (old + new firmware variants)

Windows note: multiprocessing requires the script to be run as a file
(python neosky_multilog_analyzer.py), NOT via interactive interpreter.
"""

import gc
import math
import multiprocessing as mp
import os
import queue
import threading
import traceback
from datetime import datetime
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
#  FIELD-NAME LOOKUP HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def _first(raw: dict, *keys):
    for k in keys:
        v = raw.get(k)
        if v is not None:
            return v
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
        "sats", "hdop", "volt", "curr", "balt", "galt", "rf_all",
        "ax", "ay", "az", "gx", "gy", "gz", "vib",
        "roll", "pitch", "yaw",
    )}
    rf_inst  = {i: StreamStats() for i in range(6)}
    lat = lon = t1 = t2 = None
    diag = {"thrust_loss": [], "ekf_err": 0, "comp_err": 0,
            "gps_glitch": 0, "lane_switch": 0, "lidar_6": "Missing"}
    last_msgs: list = []

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

        t = raw.get("TimeUS") or raw.get("TimeMS")
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

        # ── Error events ──────────────────────────────────────────────────
        elif mtype == "ERR":
            sub = raw.get("Subsys", -1)
            if sub == 16: diag["ekf_err"]   += 1
            elif sub == 2: diag["comp_err"]  += 1
            elif sub == 11: diag["gps_glitch"] += 1

        elif mtype == "STATUSTEXT":
            txt = raw.get("Text", "")
            if txt:
                last_msgs.append(txt)
                if len(last_msgs) > 10: last_msgs.pop(0)
                if "thrust loss" in txt.lower():
                    diag["thrust_loss"].append(txt)

        elif mtype == "EV":
            if raw.get("Id") in (10, 11, 12):
                diag["lane_switch"] += 1

        # ── GPS ───────────────────────────────────────────────────────────
        elif "GPS" in mtype:
            if lat is None:
                rlat = _first(raw, "Lat")
                rlng = _first(raw, "Lng", "Lon")
                if rlat and rlng:
                    lat = rlat / 1e7
                    lon = rlng / 1e7
            s = _first(raw, "NSats", "Sats", "NumSats")
            if s is not None: ss["sats"].update(float(s))
            h = _first(raw, "HDop", "HDOP", "Hdop")
            if h is not None: ss["hdop"].update(float(h))
            a = _first(raw, "Alt", "RelAlt")
            if a is not None: ss["galt"].update(float(a))

        # ── IMU ───────────────────────────────────────────────────────────
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
            ss["vib"].update(math.sqrt(float(ax)**2 + float(ay)**2 + float(az)**2))

        # ── Battery ───────────────────────────────────────────────────────
        elif "BAT" in mtype:
            v = _first(raw, "Volt", "V", "Voltage")
            c = _first(raw, "Curr", "I", "Current")
            if v is not None: ss["volt"].update(float(v))
            if c is not None: ss["curr"].update(float(c))

        # ── Barometer ─────────────────────────────────────────────────────
        elif mtype in ("BARO", "BAR2", "CTUN"):
            a = _first(raw, "Alt", "BAlt", "Altitude")
            if a is not None: ss["balt"].update(float(a))

        # ── Attitude ──────────────────────────────────────────────────────
        elif mtype == "ATT":
            r = _first(raw, "Roll",  "DesRoll")
            p = _first(raw, "Pitch", "DesPitch")
            y = _first(raw, "Yaw",   "DesYaw")
            if r is not None: ss["roll"].update(float(r))
            if p is not None: ss["pitch"].update(float(p))
            if y is not None: ss["yaw"].update(float(y))

        # ── 6-side Proximity ──────────────────────────────────────────────
        elif mtype in ("PRX", "DISTAD", "PRX1"):
            valid = sum(1 for i in range(1, 7) if raw.get(f"D{i}") is not None)
            if valid >= 6:
                diag["lidar_6"] = "OK"
            elif valid > 0:
                diag["lidar_6"] = f"Partial ({valid}/6)"

    if t1 is None or t2 is None:
        return {"_error": "no timestamp data — empty or corrupt"}

    delta   = t2 - t1
    dur_sec = round(delta / 1e6 if delta > 1e9 else delta / 1e3, 2)

    report = {
        "Log Name":              os.path.basename(path),
        "Flight Duration (sec)": dur_sec,
        "Latitude":              lat,
        "Longitude":             lon,

        "Avg Sat Count":         ss["sats"].get("mean"),
        "Max HDOP":              ss["hdop"].get("max"),

        "Max Baro Alt (m)":      ss["balt"].get("max"),
        "Avg Baro Alt (m)":      ss["balt"].get("mean"),
        "Max GPS Alt (m)":       ss["galt"].get("max"),

        "Avg Rangefinder (m)":   ss["rf_all"].get("mean"),
        "Min Rangefinder (m)":   ss["rf_all"].get("min"),
        "Max Rangefinder (m)":   ss["rf_all"].get("max"),

        "Min Voltage (V)":       ss["volt"].get("min"),
        "Max Voltage (V)":       ss["volt"].get("max"),
        "Avg Voltage (V)":       ss["volt"].get("mean"),
        "Avg Current (A)":       ss["curr"].get("mean"),
        "Max Current (A)":       ss["curr"].get("max"),

        "ax_avg": ss["ax"].get("mean"),  "ay_avg": ss["ay"].get("mean"),  "az_avg": ss["az"].get("mean"),
        "ax_max": ss["ax"].get("max"),   "ay_max": ss["ay"].get("max"),   "az_max": ss["az"].get("max"),
        "gx_avg": ss["gx"].get("mean"),  "gy_avg": ss["gy"].get("mean"),  "gz_avg": ss["gz"].get("mean"),

        "vib_avg": ss["vib"].get("mean"),
        "vib_max": ss["vib"].get("max"),
        "vib_rms": ss["vib"].get("rms"),

        "roll_max":  ss["roll"].get("max"),
        "pitch_max": ss["pitch"].get("max"),
        "yaw_max":   ss["yaw"].get("max"),

        "Potential Thrust Loss": " | ".join(diag["thrust_loss"]) or "None",
        "EKF Error Count":       diag["ekf_err"],
        "Compass Error Count":   diag["comp_err"],
        "GPS Glitch Count":      diag["gps_glitch"],
        "EKF Lane Switches":     diag["lane_switch"],
        "Lidar (6-Side Status)": diag["lidar_6"],
    }

    for i in range(6):
        report[f"RFND_{i}_Avg (m)"] = rf_inst[i].get("mean")
        report[f"RFND_{i}_Max (m)"] = rf_inst[i].get("max")
        report[f"RFND_{i}_Min (m)"] = rf_inst[i].get("min")

    report["Critical Messages"] = " | ".join(last_msgs[-5:])
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
        self.root.title("Neosky Multilog Analyzer  v3.0")
        self.root.geometry("760x640")
        self.root.configure(bg="#f0f3f4")
        self.gui_queue: queue.Queue = queue.Queue()
        self._active   = False
        self._pool     = None          # reference so Stop can terminate it
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
        tk.Label(hdr, text=f"v3.0  —  {self.WORKERS} parallel workers",
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
                      "renamed), then parsed and added as a row in the Excel report.",
                 bg="#f0f3f4", fg="#5d6d7e", font=("Segoe UI", 8, "italic"),
                 wraplength=700, justify="left").pack(anchor="w", pady=(0, 10))

        # ── action buttons ──
        btn_row = tk.Frame(body, bg="#f0f3f4")
        btn_row.pack()
        self.btn_files = tk.Button(
            btn_row, text="▶   SELECT FILES (.bin)",
            command=self._pick_files,
            height=2, width=22,
            bg="#2980b9", fg="white",
            font=("Segoe UI", 10, "bold"),
            relief="flat", cursor="hand2",
        )
        self.btn_files.pack(side="left", padx=4)

        self.btn_folder = tk.Button(
            btn_row, text="📁  SELECT FOLDER (recursive)",
            command=self._pick_source_folder,
            height=2, width=26,
            bg="#2980b9", fg="white",
            font=("Segoe UI", 10, "bold"),
            relief="flat", cursor="hand2",
        )
        self.btn_folder.pack(side="left", padx=4)

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
        self.lbl_progress = tk.Label(body, text="Ready — select .bin files or a folder to begin.",
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
            self.btn_files.config(state="disabled", bg="#7f8c8d", cursor="arrow")
            self.btn_folder.config(state="disabled", bg="#7f8c8d", cursor="arrow")
            self.btn_stop.config(state="normal",   bg="#c0392b", cursor="hand2")
        else:
            self.btn_files.config(state="normal",   bg="#2980b9", cursor="hand2")
            self.btn_folder.config(state="normal",   bg="#2980b9", cursor="hand2")
            self.btn_stop.config(state="disabled", bg="#7f8c8d", cursor="arrow")

    # ── selection entry points ──────────────────────────────────────────────

    def _pick_files(self):
        if not MAVUTIL_OK:
            messagebox.showerror("Missing dependency",
                                 "pymavlink is not installed.\n\n"
                                 "Run:  pip install pymavlink")
            return
        files = filedialog.askopenfilenames(
            title="Select ArduPilot .bin log files",
            filetypes=[("ArduPilot Logs", "*.bin"), ("All files", "*.*")],
        )
        if not files:
            return
        self._begin(list(files))

    def _pick_source_folder(self):
        if not MAVUTIL_OK:
            messagebox.showerror("Missing dependency",
                                 "pymavlink is not installed.\n\n"
                                 "Run:  pip install pymavlink")
            return
        folder = filedialog.askdirectory(title="Select root folder of logs (scanned recursively)")
        if not folder:
            return

        files = []
        for dirpath, _dirnames, filenames in os.walk(folder):
            for fn in filenames:
                if fn.lower().endswith(self.LOG_EXT):
                    files.append(os.path.join(dirpath, fn))

        if not files:
            messagebox.showwarning("No logs found",
                                   f"No {self.LOG_EXT} files found under:\n{folder}")
            return
        self._begin(files)

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
