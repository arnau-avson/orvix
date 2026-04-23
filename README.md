# Orvix

Captura periódica de pantalla del dispositivo Android + detección de taps con
punto rojo sobre cada pulsación + etiquetado con el package de la app en
foreground. Todo se envia a un backend HTTP local que guarda los PNG en disco.

Arquitectura:

- **`mobile/lens.c`**: binario nativo en C (compilado con el NDK) que corre
  dentro del emulador. Llama a `screencap` periódicamente, parsea los taps via
  `getevent -l`, averigua la app en foreground con `dumpsys window`, y hace
  `POST /upload` al backend.
- **`server/recv_http.py`**: backend HTTP con stdlib. Recibe PNGs, dibuja el
  círculo rojo en el punto del tap (usando Pillow) y los guarda.

> *La carpeta del proyecto sigue llamandose `pegasus/` por inercia; renombrala
> a `orvix/` cuando quieras.*

## Estructura

```
pegasus/
├── mobile/
│   ├── lens.c                        ← fuente C
│   ├── lens                          ← binario compilado (NDK x86_64)
│   └── Makefile
│
├── server/
│   ├── recv_http.py                  ← backend HTTP
│   └── screenshot/                   ← PNGs recibidos (se crea al primer upload)
│
├── start.ps1                         ← lanza todo: emulador + backend + streaming
├── stop.ps1                          ← apaga el emulador
├── setup_oppo_avd.ps1                ← crea AVD del OPPO A5 Pro 5G n92u
├── TODO.md                           ← pendientes (portar a dispositivo real)
└── README.md
```

## Setup inicial (una sola vez)

Requisitos:
- Android SDK + NDK (28.2.13676358) instalados en `%LOCALAPPDATA%\Android\Sdk`.
- Python 3 con Pillow (`pip install Pillow`).

El resto lo prepara `start.ps1`:
- Crea el AVD del OPPO si no existe.
- Compila el binario nativo con el clang del NDK.

## Lanzar todo de una vez

```powershell
cd C:\Users\arnau\Desktop\pegasus
.\start.ps1
```

Hace en orden:

1. Crea el AVD del OPPO si no existe (`setup_oppo_avd.ps1`).
2. Compila `mobile/lens.c` -> `mobile/lens`.
3. **Ventana 1**: arranca el emulador.
4. Espera al boot (hasta 8 min la 1a vez con AVD nuevo).
5. **Ventana 2**: arranca el backend HTTP en `0.0.0.0:8080`.
6. Configura `adb reverse tcp:8080 tcp:8080` (el binario vera el backend en
   `127.0.0.1:8080`, sin importar el estado de la wifi virtual).
7. Sube el binario nativo a `/data/local/tmp/lens` y le da permisos.
8. **Ventana 3**: lanza `lens tap-stream` — captura cada 1000 ms + captura
   inmediata en cada tap con las coords.

Cada captura se guarda en `server/screenshot/` con un nombre del estilo:

```
shot_<timestamp>_<n>_<package>_[tap_<x>_<y>].png
```

Ejemplos:
- `shot_1776948750_00000_com.android.launcher3.png` (captura periódica)
- `shot_1776948751_00001_com.android.chrome_tap_540_1200.png` (con tap)

## Parar

```powershell
cd C:\Users\arnau\Desktop\pegasus
.\stop.ps1
```

Apaga el emulador. Las ventanas del backend y el streaming las cierras a mano.

## Que manda `lens` exactamente

Cada captura va como `POST /upload` al backend con:
- Cuerpo: el PNG crudo devuelto por `screencap -p`.
- Query params opcionales:
  - `x`, `y`: coords RAW del touchscreen (solo en capturas disparadas por tap).
  - `app`: package de la app en foreground, extraido de `dumpsys window`.

El servidor:
- Escala las coords RAW (0..32767 en el emulador) a pixeles de la imagen.
- Dibuja un circulo rojo translucido en el punto del tap.
- Guarda el PNG con un nombre que incluye timestamp, contador, package y tap.

Inspeccionar el status en directo:
- <http://localhost:8080/>

## Notas

### El primer boot del AVD es lento

Un AVD recien creado tarda 3-8 min en su primer boot frio (debe inicializar
`userdata.img`, ejecutar `init.rc`, dexopt). Despues va mucho mas rapido.

### `adb reverse` y por que `127.0.0.1:8080` desde el movil

El emulador moderno usa `virtio_wifi`. El alias clasico `10.0.2.2` puede no ser
enrutable en condiciones especificas. `adb reverse tcp:X tcp:Y` tuneliza via la
propia conexion adb, asi que el binario conecta a `127.0.0.1:8080` dentro del
emulador y termina en el puerto 8080 de tu PC.

### Pantalla negra al arrancar el emulador

API 35+ tiene un bug del renderer ANGLE. `start.ps1` ya lo evita lanzando con
`-gpu swiftshader_indirect`.

### Apps con `FLAG_SECURE`

Bancos, Netflix, contraseñas y similares bloquean `screencap`. En esas vistas
el PNG sale en negro; no hay forma de saltarlo desde shell.

### Portar a dispositivo fisico

Ver [TODO.md](TODO.md) — hay varias cosas que asumen emulador ranchu (rango de
coords del touchscreen, acceso a `/dev/input/event*` desde `shell`) y que no
funcionan igual en un movil real.
