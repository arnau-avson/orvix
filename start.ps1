# start.ps1 - orquesta todo el flujo de Lens de una vez:
#   1) crea el AVD del OPPO si no existe
#   2) compila el binario nativo (mobile/lens.c)
#   3) abre el emulador
#   4) espera al boot
#   5) abre el backend HTTP en una ventana
#   6) configura adb reverse tcp:8080 -> tu PC
#   7) push del binario nativo a /data/local/tmp/lens
#   8) abre el streaming de screenshots (binario C) en una ventana
#
# Uso:  cd C:\Users\arnau\Desktop\pegasus ;  .\start.ps1

$ErrorActionPreference = 'Stop'

# ---- configuracion -----------------------------------------------------------
$ProjectDir   = "C:\Users\arnau\Desktop\pegasus"
$MobileDir    = "$ProjectDir\mobile"
$ServerDir    = "$ProjectDir\server"

$Sdk          = "$env:LOCALAPPDATA\Android\Sdk"
$Ndk          = "$Sdk\ndk\28.2.13676358"
$Clang        = "$Ndk\toolchains\llvm\prebuilt\windows-x86_64\bin\x86_64-linux-android30-clang.cmd"

$Avd          = "oppo_a5_pro_5g_n92u"
$ReceiverPort = 8080
$IntervalMs   = 1000

# ---- preparacion de PATH (sesion actual) -------------------------------------
$env:ANDROID_HOME      = $Sdk
$env:ANDROID_SDK_ROOT  = $Sdk
$env:Path = "$env:Path;$Sdk\platform-tools;$Sdk\emulator"

Set-Location $ProjectDir

function Step($msg) { Write-Host "[*] $msg" -ForegroundColor Cyan }
function Ok($msg)   { Write-Host "[+] $msg" -ForegroundColor Green }

# ---- 1) AVD del OPPO: crear si no existe -------------------------------------
$avdConfig = "$env:USERPROFILE\.android\avd\$Avd.avd\config.ini"
if (-not (Test-Path $avdConfig)) {
    Step "AVD '$Avd' no existe, creandolo..."
    & "$ProjectDir\setup_oppo_avd.ps1"
    if (-not (Test-Path $avdConfig)) { throw "el AVD no se creo correctamente" }
} else {
    Ok "AVD '$Avd' ya existe."
}

# ---- 2) compilar el binario nativo -------------------------------------------
Step "compilando mobile/lens.c -> mobile/lens (NDK x86_64)..."
& $Clang -O3 -Wall -Wextra -o "$MobileDir\lens.tmp" "$MobileDir\lens.c"
if (-not (Test-Path "$MobileDir\lens.tmp")) { throw "compilacion fallo" }
Move-Item -Force "$MobileDir\lens.tmp" "$MobileDir\lens"
$size = (Get-Item "$MobileDir\lens").Length
Ok "binario nativo listo ($size bytes)."

# ---- 3) emulador (solo si no hay uno corriendo) ------------------------------
$existing = (& adb devices) -match "^emulator-\d+\s+device"
if ($existing) {
    Ok "emulador ya en marcha, reutilizo el existente."
} else {
    Step "abriendo ventana del emulador (AVD: $Avd)..."
    Start-Process powershell -ArgumentList @(
        "-NoExit", "-Command",
        "cd '$ProjectDir'; " +
        "`$env:Path = '$env:Path'; " +
        "emulator -avd $Avd -gpu swiftshader_indirect -no-snapshot-load"
    ) | Out-Null
}

# ---- 4) esperar al boot ------------------------------------------------------
Step "esperando al boot del emulador (1a vez puede tardar varios minutos)..."
& adb wait-for-device
$booted = ""
$tries = 0
while ($booted -ne "1" -and $tries -lt 240) {
    Start-Sleep -Seconds 2
    $booted = ((& adb shell getprop sys.boot_completed) | Out-String).Trim()
    $tries++
    if ($tries % 5 -eq 0) {
        Write-Host "    sys.boot_completed = '$booted' (intento $tries / 240)"
    }
}
if ($booted -ne "1") { throw "el emulador no termino de arrancar tras 8 min" }
Ok "emulador booteado."

# ---- 5) backend HTTP en ventana propia ---------------------------------------
Step "abriendo backend HTTP (server\recv_http.py, puerto $ReceiverPort)..."
Start-Process powershell -ArgumentList @(
    "-NoExit", "-Command",
    "cd '$ServerDir'; python recv_http.py $ReceiverPort"
) | Out-Null
Start-Sleep -Seconds 2

# ---- 6) tunel adb reverse: movil -> tu PC ------------------------------------
Step "configurando adb reverse tcp:$ReceiverPort -> tu PC..."
& adb reverse tcp:$ReceiverPort tcp:$ReceiverPort | Out-Null
Ok "tunel listo."

# ---- 7) push del binario nativo ----------------------------------------------
Step "subiendo binario nativo a /data/local/tmp/lens..."
& adb push "$MobileDir\lens" /data/local/tmp/ | Out-Null
& adb shell chmod 755 /data/local/tmp/lens
Ok "binario nativo en el dispositivo."

# ---- 8) lanzar streaming de screenshots desde el binario nativo --------------
$EndpointUrl = "http://127.0.0.1:$ReceiverPort/upload"
Step "abriendo ventana del streaming de screenshots + taps ($IntervalMs ms)..."
Start-Process powershell -ArgumentList @(
    "-NoExit", "-Command",
    "cd '$ProjectDir'; " +
    "`$env:Path = '$env:Path'; " +
    "adb shell /data/local/tmp/lens tap-stream $EndpointUrl $IntervalMs"
) | Out-Null

Write-Host ""
Ok "todo en marcha."
Write-Host "  - Ventana 1: emulador OPPO A5 Pro 5G (n92u)"
Write-Host "  - Ventana 2: backend HTTP en puerto $ReceiverPort"
Write-Host "                 screenshots:    $ServerDir\screenshot\"
Write-Host "  - Ventana 3: streaming nativo lens tap-stream -> $EndpointUrl cada $IntervalMs ms"
Write-Host ""
Write-Host "Status:  http://localhost:$ReceiverPort/"
Write-Host ""
Write-Host "Para parar todo:  .\stop.ps1"
