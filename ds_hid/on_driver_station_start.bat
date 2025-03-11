@echo off
start "" "C:\Users\Public\wpilib\2025\elastic\elastic_dashboard.exe"
start cmd /k "cd /d "%~dp0" && python ds_hid.py"
