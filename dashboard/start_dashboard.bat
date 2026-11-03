@echo off
REM Fallback launcher: serves the dashboard on http://localhost:8765 (a secure context for Web Serial)
REM Normally you can just double-click FaultIsolation_Dashboard.html in Chrome/Edge instead.
cd /d "%~dp0"
start "" "http://localhost:8765/FaultIsolation_Dashboard.html"
python -m http.server 8765
