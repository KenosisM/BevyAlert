@echo off
setlocal
cd /d "%~dp0"
for %%I in ("%~dp0..\.tools") do set "BEVY_ALERT_TOOLS=%%~fI"

rem Limit cold-build memory use; an explicit --jobs argument takes precedence.
if not defined CARGO_BUILD_JOBS set "CARGO_BUILD_JOBS=1"

rem All paths are process-local. rust-toolchain.toml selects the pinned Rust.
rem BEVY_ALERT_CARGO can point to another verified cargo.exe.
if defined BEVY_ALERT_CARGO goto :build
if exist "%BEVY_ALERT_TOOLS%\cargo\bin\rustup.exe" goto :local_tools
if defined CARGO_HOME if exist "%CARGO_HOME%\bin\cargo.exe" goto :cargo_home
if exist "%USERPROFILE%\.cargo\bin\cargo.exe" goto :user_tools
echo Rust 1.97.1 is required. See docs\BUILD.md for installation instructions.
exit /b 1

:local_tools
set "CARGO_HOME=%BEVY_ALERT_TOOLS%\cargo"
set "RUSTUP_HOME=%BEVY_ALERT_TOOLS%\rustup"
set "BEVY_ALERT_RUSTUP=%BEVY_ALERT_TOOLS%\cargo\bin\rustup.exe"
goto :resolve_toolchain

:cargo_home
set "BEVY_ALERT_RUSTUP=%CARGO_HOME%\bin\rustup.exe"
goto :resolve_toolchain

:user_tools
set "BEVY_ALERT_RUSTUP=%USERPROFILE%\.cargo\bin\rustup.exe"

:resolve_toolchain
if not exist "%BEVY_ALERT_RUSTUP%" (
    echo rustup.exe was not found. See docs\BUILD.md.
    exit /b 1
)
rem Resolve the pinned toolchain to native executables without relying on PATH.
for /f "delims=" %%C in ('""%BEVY_ALERT_RUSTUP%" which cargo"') do set "BEVY_ALERT_CARGO=%%C"
for /f "delims=" %%C in ('""%BEVY_ALERT_RUSTUP%" which rustc"') do set "RUSTC=%%C"
for /f "delims=" %%C in ('""%BEVY_ALERT_RUSTUP%" which rustdoc"') do set "RUSTDOC=%%C"
if not defined BEVY_ALERT_CARGO exit /b 1
if not defined RUSTC exit /b 1
if not defined RUSTDOC exit /b 1

:build
if not exist "%BEVY_ALERT_CARGO%" (
    echo cargo.exe was not found at the configured path.
    exit /b 1
)
"%BEVY_ALERT_CARGO%" build --locked %*
exit /b %errorlevel%
