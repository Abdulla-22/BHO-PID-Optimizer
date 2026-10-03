; Inno Setup script (paths are relative to this file). Build with: ISCC BHO_Setup.iss (after build_exe.bat)
; Optional: place vc_redist.x64.exe next to this file to bundle the VC++ runtime.
#define AppName "BHO PID Optimizer"
#define AppVersion "1.0.1"

[Setup]
AppId={{6F1B0C2E-4A7D-4E55-9B1A-3C8D2E7A5F10}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher=Ali Hubail, Abdulla Yusuf, Mujtaba Obaid
LicenseFile=..\..\LICENSE
DefaultDirName={autopf}\BHO
DefaultGroupName={#AppName}
OutputDir=installer
OutputBaseFilename=BHO_Setup
Compression=lzma2
SolidCompression=yes
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
PrivilegesRequiredOverridesAllowed=dialog
UninstallDisplayIcon={app}\BHO.exe
WizardStyle=modern
#ifexist "app.ico"
SetupIconFile=app.ico
#endif

[Files]
Source: "dist\BHO\*"; DestDir: "{app}"; Flags: recursesubdirs ignoreversion createallsubdirs
#ifexist "vc_redist.x64.exe"
Source: "vc_redist.x64.exe"; DestDir: "{tmp}"; Flags: deleteafterinstall
#endif

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\BHO.exe"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\BHO.exe"; Tasks: desktopicon

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; Flags: unchecked

[Run]
#ifexist "vc_redist.x64.exe"
Filename: "{tmp}\vc_redist.x64.exe"; Parameters: "/install /quiet /norestart"; StatusMsg: "Installing Visual C++ runtime..."; Flags: waituntilterminated
#endif
Filename: "{app}\BHO.exe"; Description: "Launch {#AppName}"; Flags: nowait postinstall skipifsilent
