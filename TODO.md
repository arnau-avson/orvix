# Pendientes

## Portar a dispositivo fisico (cuando salgamos del emulador)

### 0. Permisos para leer `/dev/input/event*` (bloqueante)

En el emulador, el usuario `shell` puede leer los eventos del touchscreen porque
SELinux es permisivo con builds de desarrollo. **En un movil real no-rooteado,
`adb shell` NO tiene permiso** (SELinux en `user` builds bloquea la lectura de
`/dev/input/event*`).

Opciones cuando llegue el momento:

- **Root** (o custom ROM con SELinux permisivo): trivial pero implica bootloader
  unlock y, en el caso del OPPO objetivo, casi seguro perdida de garantia/banking
  apps y puede no estar permitido por el fabricante.
- **Firmar el binario como `system`** en una ROM personalizada: misma historia.
- **Reescribir como app Android** con `AccessibilityService` (captura taps sin
  root, pero requiere que el usuario active el servicio en Ajustes manualmente)
  o un overlay con `TYPE_APPLICATION_OVERLAY`. Esto implicaria reintroducir una
  app Java (similar a la que borramos), y `screencap` tambien hay que sustituirlo
  por `MediaProjection` con consentimiento del usuario.
- **Magisk/Shizuku** — Shizuku es interesante: da permisos elevados a apps sin
  necesidad de root si el dispositivo tiene activado el modo desarrollador, pero
  requiere que el usuario lo configure.

Sin una de estas opciones, este flujo no portara a movil real.

### 1. Rango de coordenadas del touchscreen (`RAW_TOUCH_MAX`)

En el emulador ranchu el touchscreen `virtio_input_multi_touch_*` reporta coords
en rango fijo `0..32767` en ambos ejes. Por eso [server/recv_http.py](server/recv_http.py)
hoy hace:

```python
RAW_TOUCH_MAX = 32767
x = int(raw_x * w / RAW_TOUCH_MAX)
y = int(raw_y * h / RAW_TOUCH_MAX)
```

En un movil real ese max **es distinto para cada dispositivo** (cada chip
touchscreen tiene su propia resolucion raw). Para averiguarlo en el dispositivo:

```
adb shell getevent -lp
```

Y buscar en el dispositivo touchscreen las lineas:

```
ABS_MT_POSITION_X  : value 0, min 0, max <MX>, fuzz ..., flat ..., resolution ...
ABS_MT_POSITION_Y  : value 0, min 0, max <MY>, fuzz ..., flat ..., resolution ...
```

`<MX>` y `<MY>` son normalmente distintos entre si (no como en el emulador) y
distintos a 32767.

### 2. Solucion: pasar el max dinamicamente

Plan cuando toque:

- En [mobile/lens.c](mobile/lens.c), al arrancar `cmd_tap_stream`, hacer
  `ioctl(fd, EVIOCGABS(ABS_MT_POSITION_X), &abs)` sobre el fd del touchscreen
  para obtener `abs.maximum`. Idem con Y. Esto requiere abrir `/dev/input/eventN`
  directamente en vez de pasar por el subproceso `getevent -l`, o bien parsear
  la salida de `getevent -lp` una vez al inicio.
- Anadir los max a la URL del POST: `/upload?x=...&y=...&mx=<MX>&my=<MY>`.
- En [server/recv_http.py](server/recv_http.py), leer `mx` y `my` del query
  string en `do_POST` y pasarlos a `draw_tap_overlay` en lugar de usar la
  constante global.

### 3. Identificar el touchscreen correcto

En el emulador sale primero `virtio_input_multi_touch_1` como `/dev/input/event2`,
pero en un movil real el touchscreen suele llamarse `<fabricante>_ts`, `synaptics_dsx`,
`goodix_ts`, `focaltech_ts`, etc. Actualmente [mobile/lens.c](mobile/lens.c:237)
lanza `getevent -l` sin filtrar dispositivo: lee TODOS los `event*`. Funciona
porque ignoramos eventos sin `ABS_MT_POSITION_*`, pero es fragil si hay otro
periferico con multi-touch (stylus, pad externo).

Mejor filtrar por nombre: al arrancar, hacer `getevent -lp` (o `ioctl(EVIOCGNAME)`),
elegir el primer device cuyo nombre contenga `touch`, y pasarle ese path como
argumento a `getevent -l /dev/input/eventN`.

### 4. Detectar BTN_TOUCH en dispositivo real

El codigo actual detecta taps por `ABS_MT_TRACKING_ID` (necesario en el emulador,
que no emite `BTN_TOUCH`). La rama de `BTN_TOUCH` sigue ahi por si un dispositivo
real lo emite, pero hay que confirmar con `getevent -l` en el movil que
`ABS_MT_TRACKING_ID` sigue apareciendo — en protocolo B es obligatorio, asi que
deberia funcionar.
