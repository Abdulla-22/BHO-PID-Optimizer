@echo off
rem Builds packaging\dist\BHO\BHO.exe (onedir) and, if Inno Setup is installed, packaging\installer\BHO_Setup.exe.
rem All build files and outputs stay inside the packaging folder.
rem Run on Windows with 64-bit Python 3.10-3.12 in PATH. Can be started from any location.
cd /d "%~dp0.."
set APP=%CD%
set PKG=%APP%\packaging

python -m venv "%PKG%\.build_venv" || goto :err
call "%PKG%\.build_venv\Scripts\activate.bat" || goto :err
python -m pip install --upgrade pip || goto :err
pip install pyinstaller customtkinter numpy scipy pandas matplotlib pillow openpyxl pyserial || goto :err

set ICON=
if exist "%PKG%\app.ico" set ICON=--icon "%PKG%\app.ico"

pyinstaller --noconfirm --clean --noconsole --onedir --noupx --name BHO ^
  --distpath "%PKG%\dist" --workpath "%PKG%\build" --specpath "%PKG%" ^
  --version-file "%PKG%\version_info.txt" %ICON% ^
  --collect-all customtkinter ^
  --hidden-import openpyxl ^
  --add-data "%APP%\images;images" ^
  "%APP%\BHO.py" || goto :err

python "%PKG%\collect_licenses.py" --output "%PKG%\dist\BHO" || goto :err

set ISCC=%ProgramFiles(x86)%\Inno Setup 6\ISCC.exe
if exist "%ISCC%" (
  "%ISCC%" "%PKG%\BHO_Setup.iss" || goto :err
  echo Installer: %PKG%\installer\BHO_Setup.exe
) else (
  echo Inno Setup 6 not found - skipping installer. Exe: %PKG%\dist\BHO\BHO.exe
)
pause
exit /b 0
:err
echo BUILD FAILED
pause
exit /b 1
