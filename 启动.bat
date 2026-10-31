@echo off
rem ===========================================================================
rem  ra2engine 演示启动器 —— 双击即可运行
rem
rem  【用法】
rem    1) 直接双击本文件           用 rulesmd.ini 里写的地图启动
rem    2) 把某个 .map 拖到本文件上  改用该地图（其余仍按 rulesmd.ini）
rem
rem  【调参】所有可变项都在 rulesmd.ini 里：地图 / 游戏目录 / 单位 / 数量 /
rem          视口缩放与相机位置 / 光源 / 亮度。改那个文件就行，本 bat 不用动。
rem
rem  【教训，别踩】
rem    - 工作目录必须是工程根：本脚本自己 cd 过来，双击也能跑。
rem    - 路径带空格必须加引号（下面已经加了）。
rem    - 首次运行要等 MIX 索引（约 1 秒）+ 地形装配（约 2 秒）才出窗口，
rem      这期间窗口没反应是正常的，别以为死了。
rem    - exe 不存在会自动编译，首次编译要几分钟，那是真的在编。
rem ===========================================================================

cd /d "%~dp0"

set "EXE=target\debug\ra2engine.exe"

rem 拖放：把 .map 拖到本 bat 上时 %~1 就是那个文件，覆盖 rulesmd.ini 的 Map
set "DRAG="
if not "%~1"=="" set "DRAG="%~1""

if not exist "%EXE%" goto :build

:run
echo.
echo   可调项   rulesmd.ini（地图 / 单位 / 视口 / 光源 / 亮度）
echo.
echo   启动中…… 首次要等 2~3 秒出窗口
echo.
"%EXE%" game %DRAG%
echo.
echo   游戏已退出。
pause
exit /b 0

:build
echo.
echo   找不到 %EXE%，先编译一次（首次要几分钟，请耐心等）
echo.
call "%~dp0build.cmd"
if errorlevel 1 (
  echo.
  echo   [x] 编译失败，请把上面的报错发出来。
  pause
  exit /b 1
)
goto :run
