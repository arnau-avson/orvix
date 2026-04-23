# setup_oppo_avd.ps1 - crea el AVD del OPPO A5 Pro 5G (n92u) escribiendo
# los ficheros del AVD directamente, sin depender de avdmanager.
#
# Un AVD del emulador Android es solo dos ficheros:
#   ~/.android/avd/<name>.ini           pointer
#   ~/.android/avd/<name>.avd/config.ini hw config
# (userdata.img se autogenera en el primer boot)
#
# Hardware aproximado del OPPO A5 Pro 5G (form factor):
#   Display:  6.67"  1080 x 2412 LCD-LTPS  ~393 ppi  (mapeado a xxhdpi/480)
#   RAM:      8 GB
#   Cores:    6
#   Storage:  6 GB internal
#
# La imagen sigue siendo AOSP/Google Play x86_64 (no ColorOS).

$ErrorActionPreference = 'Stop'

$Sdk        = "$env:LOCALAPPDATA\Android\Sdk"
$ImgPath    = "$Sdk\system-images\android-36.1\google_apis_playstore\x86_64"
$AvdName    = "oppo_a5_pro_5g_n92u"
$AvdHome    = "$env:USERPROFILE\.android\avd"
$AvdDir     = "$AvdHome\$AvdName.avd"
$AvdPointer = "$AvdHome\$AvdName.ini"
$AvdCfg     = "$AvdDir\config.ini"

if (-not (Test-Path $ImgPath)) {
    throw "no encuentro la system image en: $ImgPath"
}

if (Test-Path $AvdDir) {
    Write-Host "[*] Limpiando AVD previo en $AvdDir..." -ForegroundColor Cyan
    Remove-Item -Recurse -Force $AvdDir
}
if (Test-Path $AvdPointer) {
    Remove-Item -Force $AvdPointer
}

Write-Host "[*] Creando carpetas del AVD..." -ForegroundColor Cyan
New-Item -ItemType Directory -Force -Path $AvdHome | Out-Null
New-Item -ItemType Directory -Force -Path $AvdDir  | Out-Null

# 1) pointer .ini en ~/.android/avd/
$pointer = @"
avd.ini.encoding=UTF-8
path=$AvdDir
path.rel=avd\$AvdName.avd
target=android-36.1
"@
Set-Content -Path $AvdPointer -Value $pointer -Encoding ASCII -Force
Write-Host "[+] pointer escrito: $AvdPointer" -ForegroundColor Green

# 2) config.ini con specs del OPPO
$cfg = @"
AvdId=$AvdName
avd.ini.displayname=OPPO A5 Pro 5G (n92u)
avd.ini.encoding=UTF-8
abi.type=x86_64
hw.cpu.arch=x86_64
hw.cpu.ncore=6
hw.ramSize=8192
vm.heapSize=512
disk.dataPartition.size=6G
hw.lcd.width=1080
hw.lcd.height=2412
hw.lcd.density=480
hw.lcd.depth=24
hw.initialOrientation=portrait
hw.gpu.enabled=yes
hw.gpu.mode=swiftshader_indirect
hw.keyboard=yes
hw.mainKeys=no
hw.trackBall=no
hw.dPad=no
hw.gsmModem=yes
hw.gps=yes
hw.battery=yes
hw.accelerometer=yes
hw.gyroscope=yes
hw.audioInput=yes
hw.audioOutput=yes
hw.sensors.proximity=yes
hw.sensors.magnetic_field=yes
hw.sensors.orientation=yes
hw.sensors.temperature=yes
hw.sensors.pressure=yes
hw.sensors.humidity=yes
hw.sensors.light=yes
hw.sensors.heart_rate=no
hw.sdCard=no
image.sysdir.1=system-images\android-36.1\google_apis_playstore\x86_64\
tag.id=google_apis_playstore
tag.display=Google Play
tag.ids=google_apis_playstore
tag.displaynames=Google Play
target=android-36.1
PlayStore.enabled=true
showDeviceFrame=no
runtime.network.latency=none
runtime.network.speed=full
fastboot.forceColdBoot=yes
fastboot.forceFastBoot=no
"@
Set-Content -Path $AvdCfg -Value $cfg -Encoding ASCII -Force
Write-Host "[+] config.ini escrito: $AvdCfg" -ForegroundColor Green

Write-Host ""
Write-Host "[OK] AVD '$AvdName' creado." -ForegroundColor Green
Write-Host "    display: 1080x2412 @ xxhdpi (480), 8 GB RAM, 6 cores"
Write-Host ""
Write-Host "Verificalo con:  emulator -list-avds"
