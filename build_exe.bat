@echo off
REM Builds the standalone Windows .exe. Run this ON WINDOWS from the repo
REM root, with your virtualenv activated and both requirements files
REM installed:
REM     pip install -r requirements.txt -r requirements-desktop.txt
REM
REM Uses "python -m PyInstaller" rather than the bare "pyinstaller" command:
REM the latter depends on your venv's Scripts\ folder being on PATH, which
REM isn't always reliable on Windows even with the venv activated, and fails
REM with a plain "not recognized" error that's easy to mistake for a real
REM build failure. "python -m PyInstaller" instead goes through the same
REM Python interpreter already resolved on PATH, so it works whenever
REM PyInstaller is installed for that interpreter, period.

python -c "import pandas, openpyxl, pymavlink, PyInstaller" 2>NUL
if errorlevel 1 (
    echo.
    echo ERROR: one or more required packages are not installed for THIS
    echo "python" ^(the one currently on PATH^). This is the exact failure
    echo mode that produces a .exe that builds "successfully" but then
    echo crashes with "ModuleNotFoundError" when you run it -- PyInstaller
    echo bundles whatever the interpreter it runs under can see, so if the
    echo wrong Python ^(not your activated venv's^) ends up on PATH, the
    echo build silently omits packages that ARE installed, just not there.
    echo.
    echo Run this first, then re-run build_exe.bat:
    echo     pip install -r requirements.txt -r requirements-desktop.txt
    echo.
    echo If that still doesn't fix it, run "where.exe python" and confirm
    echo the first line points inside THIS project's venv\Scripts\ folder --
    echo if it doesn't, your venv isn't actually active in this terminal.
    exit /b 1
)

python -m PyInstaller --onefile --windowed --name NeoskyMultilogAnalyzer ^
    --hidden-import=pymavlink.dialects.v20.ardupilotmega ^
    --hidden-import=pymavlink.dialects.v10.ardupilotmega ^
    --collect-submodules pymavlink ^
    neosky_multilog_analyzer.py

if errorlevel 1 (
    echo.
    echo BUILD FAILED. Scroll up for the actual PyInstaller error -- common
    echo causes are a missing dependency ^(re-run the pip install above^) or
    echo antivirus quarantining a file mid-build.
    exit /b 1
)

if not exist "dist\NeoskyMultilogAnalyzer.exe" (
    echo.
    echo PyInstaller reported success but dist\NeoskyMultilogAnalyzer.exe is
    echo missing. Something unexpected happened -- scroll up for details.
    exit /b 1
)

echo.
echo Build finished. The .exe is in dist\NeoskyMultilogAnalyzer.exe
echo Share that single file -- no Python install needed on the receiving
echo machine. It writes Neosky_Report.xlsx / Neosky_Errors.txt to whatever
echo output folder is picked in the app.
