@echo off
setlocal
cd /d "%~dp0"
set "PATH=%USERPROFILE%\.cargo\bin;%USERPROFILE%\.rustup\toolchains\stable-x86_64-pc-windows-msvc\bin;%PATH%"

cargo build %*
if errorlevel 1 goto :done

if not exist "target\debug\ra2engine.exe" goto :done

rem Purge intermediates for good: rd /s /q and del /f /q never use the Recycle Bin.
if exist "target\debug\incremental" rd /s /q "target\debug\incremental"
del /f /q /s "target\debug\deps\*.o"   >nul 2>nul
del /f /q /s "target\debug\deps\*.d"   >nul 2>nul
del /f /q /s "target\debug\deps\*.pdb" >nul 2>nul
del /f /q /s "target\debug\*.pdb"      >nul 2>nul

echo build ok, intermediates deleted

:done
endlocal
