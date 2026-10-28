@echo off
rem Build plugin\xpbench\64\win.xpl with MSVC. Usage: plugin\build.bat [XPLANE_DIR]
rem Fetches the X-Plane SDK (BSD-style licence, not vendored) into plugin\sdk on first run.
setlocal
set HERE=%~dp0
if not exist "%HERE%sdk\SDK\CHeaders" (
  python "%HERE%fetch_sdk.py" "%HERE%sdk" || exit /b 1
)
set VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -products * -latest -property installationPath`) do set VS=%%i
call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
if not exist "%HERE%out\xpbench\64" mkdir "%HERE%out\xpbench\64"
cl /nologo /O2 /W3 /LD /DIBM=1 /DXPLM200 /DXPLM210 /DXPLM300 /DXPLM301 /DXPLM302 /DXPLM303 /DXPLM400 /DXPLM410 ^
   /I"%HERE%sdk\SDK\CHeaders\XPLM" "%HERE%xpbench.c" ^
   /Fo"%HERE%out\\" /Fe"%HERE%out\xpbench\64\win.xpl" ^
   /link "%HERE%sdk\SDK\Libraries\Win\XPLM_64.lib" || exit /b 1
del "%HERE%out\xpbench\64\win.exp" "%HERE%out\xpbench\64\win.lib" 2>nul
if not "%~1"=="" (
  if not exist "%~1\Resources\plugins\xpbench\64" mkdir "%~1\Resources\plugins\xpbench\64"
  copy /y "%HERE%out\xpbench\64\win.xpl" "%~1\Resources\plugins\xpbench\64\win.xpl" >nul && echo installed to %~1\Resources\plugins\xpbench
)
echo built %HERE%out\xpbench\64\win.xpl
