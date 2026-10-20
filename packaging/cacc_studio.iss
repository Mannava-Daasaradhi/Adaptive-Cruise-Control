; CACC Studio Windows installer (Inno Setup 6).
; Built by scripts/build_exe.py after the PyInstaller one-folder build:
;   iscc /DAppVersion=0.6.0 /DSourceDir=..\dist\CACC-Studio /DOutDir=..\dist packaging\cacc_studio.iss
; Per-user install (no admin prompt), Start-menu + optional desktop shortcut,
; clean uninstall.

#ifndef AppVersion
  #define AppVersion "0.0.0"
#endif
#ifndef SourceDir
  #define SourceDir "..\dist\CACC-Studio"
#endif
#ifndef OutDir
  #define OutDir "..\dist"
#endif

[Setup]
AppId={{6C2D9B7E-3F1A-4E8B-9C55-CACC57D10001}
AppName=CACC Studio
AppVersion={#AppVersion}
AppPublisher=Mannava Daasaradhi
AppPublisherURL=https://github.com/Mannava-Daasaradhi/Adaptive-Cruise-Control
DefaultDirName={localappdata}\Programs\CACC Studio
DefaultGroupName=CACC Studio
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
OutputDir={#OutDir}
OutputBaseFilename=CACC-Studio-{#AppVersion}-setup
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
LicenseFile=..\LICENSE
UninstallDisplayIcon={app}\CACC-Studio.exe

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Files]
Source: "{#SourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\CACC Studio"; Filename: "{app}\CACC-Studio.exe"
Name: "{group}\Uninstall CACC Studio"; Filename: "{uninstallexe}"
Name: "{userdesktop}\CACC Studio"; Filename: "{app}\CACC-Studio.exe"; Tasks: desktopicon

[Run]
Filename: "{app}\CACC-Studio.exe"; Description: "{cm:LaunchProgram,CACC Studio}"; Flags: nowait postinstall skipifsilent
