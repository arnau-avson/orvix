# Pendientes

## Portar a dispositivo fisico (cuando salgamos del emulador)

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
