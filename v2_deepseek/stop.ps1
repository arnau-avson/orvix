# stop.ps1 — apaga el emulador y cierra las ventanas que abrió start.ps1.
#
# Uso:  cd C:\Users\arnau\Desktop\pegasus ;  .\stop.ps1

$ErrorActionPreference = 'SilentlyContinue'

$Sdk = "$env:LOCALAPPDATA\Android\Sdk"
$env:Path = "$env:Path;$Sdk\platform-tools;$Sdk\emulator"

Write-Host "[*] Pidiendo al emulador que se apague (adb emu kill)..." -ForegroundColor Cyan
adb -s emulator-5554 emu kill | Out-Null
Start-Sleep -Seconds 2

Write-Host "[*] Matando procesos residuales del emulador..." -ForegroundColor Cyan
Get-Process emulator             -ErrorAction SilentlyContinue | Stop-Process -Force
Get-Process qemu-system-x86_64   -ErrorAction SilentlyContinue | Stop-Process -Force
Get-Process crashpad_handler     -ErrorAction SilentlyContinue | Stop-Process -Force

Write-Host "[OK] Emulador apagado." -ForegroundColor Green
Write-Host "Cierra manualmente las ventanas del receptor (recv.py) y del streaming"
Write-Host "(Ctrl-C dentro de cada una, o cerrar la ventana directamente)."