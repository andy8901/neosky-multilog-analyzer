# Neosky Multilog Analyzer

Standalone desktop tool for batch-processing ArduPilot `.bin` flight logs.

## What it does

1. **Rename by folder** — every log found (either explicitly selected or
   discovered by recursively scanning a chosen root folder) is renamed in
   place to `<parent_folder_name>_<original_filename>`, so a flat report
   still shows which mission/folder each log came from. Renaming is a
   same-directory `os.rename()` (metadata only, instant even for 30GB+
   files) and is idempotent — a log that already carries its folder prefix
   is left alone, so re-running on a partially processed folder is safe.

2. **Extract to Excel** — every renamed log is parsed with `pymavlink` and
   reduced with O(1)-memory Welford streaming statistics (GPS, IMU,
   battery, rangefinder, vibration, attitude, and diagnostic error counts),
   then streamed into a chunked Excel report so raw sample data is never
   held in memory.

## Features

- Select individual files, or select a root folder and recurse into it
- Multi-process parallel parsing (N-1 CPU cores)
- Streaming / chunked Excel output (never OOM)
- Crash-safe resume (pick the same output folder, tick the resume checkbox)
- Per-file error log (a bad log doesn't fail the whole job)
- Broad field-name fallbacks for old and new ArduPilot firmware variants

## Requirements

```
pip install -r requirements.txt
```

Tkinter ships with standard Python installers on Windows/macOS. On Linux,
install your distro's Tk package if it's missing (e.g. `apt install
python3-tk`).

## Running

```
python neosky_multilog_analyzer.py
```

Windows note: multiprocessing requires running this as a script
(`python neosky_multilog_analyzer.py`), not via the interactive
interpreter.
