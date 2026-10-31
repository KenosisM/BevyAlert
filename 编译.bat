@echo off
rem Build without deleting any files.
call "%~dp0build.cmd" %*
exit /b %errorlevel%
