# Hardware para testing

Listado de componentes para probar el robot por etapas. Empezar por el **Tier 1** (lo más barato) y subir solo cuando funcione cada nivel — así aíslas problemas.

---

## Tier 0 — Solo software (lo que ya tienes)

Cualquier ordenador con Python 3.10+. Ejecuta `run_simulation.py` y `run_robot.py` con mocks. **No necesitas nada físico**.

---

## Tier 1 — Percepción real (cámara)

**Objetivo:** validar que el detector de semáforos y obstáculos funciona con frames de cámara reales (no imágenes guardadas).

| Componente | Modelo recomendado | Por qué |
|------------|-------------------|---------|
| Webcam USB | **Logitech C920 / C922** | 1080p, autofocus, gran angular, plug-and-play en cualquier OS. Lo que usan la mayoría de papers. |
| Alternativa más barata | Cualquier webcam UVC ≥720p | Funciona. Calidad de imagen importa para semáforos lejanos. |

**Test:**
```python
from delivery_robot.hardware import OpenCVCamera
from delivery_robot.perception import YOLOTrafficLightDetector, classify_state

cam = OpenCVCamera(0)               # índice 0 = primera webcam
detector = YOLOTrafficLightDetector(model_size="m")
while True:
    frame = cam.read()
    detections = detector.detect(frame)
    # ...
```

Sal a la calle con el portátil + webcam y apunta a un cruce con semáforo. Coste: **~70€**.

---

## Tier 2 — Localización real (GPS)

**Objetivo:** validar que el robot sabe dónde está y puede hacer el tracker proyectar pose contra ruta.

| Componente | Modelo recomendado | Por qué |
|------------|-------------------|---------|
| Módulo GPS | **u-blox NEO-M8N** (con antena externa activa) | Soporta GPS+GLONASS+Galileo, accuracy ~2.5m, NMEA por UART a 9600 baud (lo que parsea `NMEASerialGPS`). |
| Alternativa más barata | u-blox NEO-6M | Solo GPS, ~5m accuracy. Sirve para empezar. |
| Adaptador USB-TTL | **CP2102 / CH340** | Conecta el TX del GPS a USB del ordenador. ~3€. |
| Antena GPS | Antena activa con cable SMA, 3m | Mejora muchísimo la calidad de fix. La integrada de los módulos baratos es decepcionante. |

**Test:**
```python
from delivery_robot.hardware import NMEASerialGPS

gps = NMEASerialGPS("/dev/ttyUSB0")  # Linux/Mac;  "COM3" en Windows
pose = gps.get_pose()
print(pose)
```

Antes de probar al aire libre déjalo 5 min con vista al cielo para que adquiera fix. Coste: **~25-40€**.

---

## Tier 3 — Sensor hub (IMU + encoders) vía microcontrolador

**Objetivo:** dar al robot heading estable a baja velocidad (donde el GPS heading falla) y odometría.

| Componente | Modelo recomendado | Por qué |
|------------|-------------------|---------|
| Microcontrolador | **ESP32 DevKit** (o Arduino Nano) | ESP32 es más potente y trae WiFi/BT. Nano sirve perfectamente y es plug-and-play en el IDE de Arduino. |
| IMU | **BNO055** (Adafruit/clones) | Trae fusión de sensores ON-CHIP (heading absoluto vía magnetómetro), calibración guardada. ~25€. |
| IMU alternativa barata | MPU6050 | Solo accel+gyro. Heading hay que calcularlo a mano y drifta. ~3€. |
| Encoders rotatorios | (vienen con los motores del Tier 4) | Cuentan revoluciones para odometría — el ESP32 lee y reenvía deltas. |

**Firmware mínimo del ESP32** (lo que falta escribir — el contrato Python ya está):
- Leer BNO055 por I2C cada 20ms
- Leer encoders por interrupciones
- Imprimir cada 100ms: `$IMU,ax,ay,az,gx,gy,gz,heading\n`
- Escuchar comandos `go/wait/stop/estop` desde Python (cuando lleguen los motores)

**Test:**
```python
from delivery_robot.hardware import NMEASerialGPS, SerialIMU, GPSIMULocalizer

gps = NMEASerialGPS("/dev/ttyUSB0")
imu = SerialIMU("/dev/ttyUSB1")
loc = GPSIMULocalizer(gps, imu)
pose = loc.get_pose()  # heading fusionado
```

Coste: **~35€** (ESP32 + BNO055 + cables).

---

## Tier 4 — Plataforma móvil (motores + chasis)

**Objetivo:** un robot que se mueve. A esta altura ya tienes percepción + localización; solo falta que el `decision.action` mueva ruedas.

| Componente | Modelo recomendado | Notas |
|------------|-------------------|-------|
| Motores DC con encoder | **JGB37-520 12V con encoder hall** (pareja, ~178 RPM) | A 178 RPM con rueda 100mm da ~0.93 m/s sin reducción adicional. Para 1.4 m/s usar reducción 1:30 o motores 24V. |
| Driver de motor | **BTS7960 43A** (uno por motor) | Maneja la corriente sin sufrir. L298N solo aguanta motores juguete. |
| Chasis 4 ruedas | Kit "smart car 4WD" o aluminio cortado a medida | ~30x40cm es suficiente para llevar el portátil/Pi + batería. |
| Ruedas | **100-150mm de diámetro, goma** | Más grande = mejor sobre baldosa de acera. |
| Caster (opcional) | Si vas a 2 motores + 2 ruedas locas | Más fácil de conducir que 4 motorizadas. |
| MCU motor (puede ser otro ESP32 separado o el mismo del Tier 3) | ESP32 / Arduino Mega | Recomiendo separar sensor-hub MCU de motor-MCU para no perder samples del IMU si el motor PID es exigente. |

**Firmware del motor MCU** (también falta escribir):
- Leer comandos `go <mmps>`, `wait`, `stop`, `estop` por serial
- Cerrar PID sobre encoders para alcanzar velocidad pedida
- Implementar `estop` con latch (no salir hasta reset físico)

**Test:**
```python
from delivery_robot.hardware import SerialMotorController
from delivery_robot.navigation import NavigationAction

motors = SerialMotorController("/dev/ttyACM2")
motors.execute(NavigationAction.GO)  # debería arrancar
time.sleep(2)
motors.execute(NavigationAction.STOP)
```

Coste: **~80-150€** (motores + drivers + chasis + ESP32).

---

## Tier 5 — Robot completo, listo para campo

**Objetivo:** desplegar en acera real con autonomía completa.

| Componente | Modelo recomendado | Notas |
|------------|-------------------|-------|
| Cerebro embebido | **Raspberry Pi 5 (8GB)** | YOLOv8n a ~10 Hz, suficiente para 5 km/h. Para más FPS añadir Coral USB Accelerator (~70€). |
| Alternativa potente | **Jetson Orin Nano** | YOLOv8m a 30+ Hz pero ~250€. |
| Acelerador (opcional) | **Google Coral USB** | Lleva el inference de YOLOv8n a 30 Hz en Pi. |
| Almacenamiento | microSD A2 64-128GB **o** SSD NVMe vía USB | SSD recomendado para no morir el sistema con writes. |
| Disipación | Caja con ventilador para Pi5 | Termal throttling baja FPS hasta la mitad sin esto. |
| Batería principal | **LiFePO4 12V 20Ah** | LiFePO4 es más segura que LiPo (no se incendia tan fácil), tolera más ciclos. |
| Reductor 12V → 5V/5A | Buck converter aislado | Para alimentar la Pi sin ruido de los motores. |
| Distribución de potencia | Placa de distribución + fusibles | Importante: motor 12V y Pi 5V con masas comunes pero líneas separadas. |
| Botón E-STOP físico | **Seta NC en serie con la línea de motores** | Cortar físicamente la corriente de motores, no solo software. **Mandatory por seguridad**. |
| Luces de visibilidad | LED frontal blanco + trasero rojo + intermitentes | Aceras de noche / sombras. |
| Compartimento de carga | Caja con tapa cerrable, ~30L | El "delivery" parte. |
| Suspensión | Amortiguadores entre chasis y ruedas | Las aceras tienen escalones de varios cm. |

Coste estimado completo: **~600-1000€** según calidad.

---

## Recomendación de orden

1. **Empieza por Tier 1** — sal a la calle con el portátil y webcam, valida que el detector ve semáforos reales con buena precisión.
2. **Tier 2** — añade GPS, valida que el tracker proyecta tu paseo correctamente sobre la ruta planificada.
3. **Tier 3** — ESP32 + IMU, escribe el firmware mínimo, verifica que `GPSIMULocalizer` da heading estable cuando paras.
4. **Skip Tier 4 si quieres validar el cerebro primero** — puedes hacer el Tier 5 (Raspberry Pi) y ejecutar `run_robot.py` en estático: el robot "decide" pero no se mueve. Sirve para verificar latencias y temperaturas.
5. **Tier 4 + 5** — junta todo en chasis, prueba en interior controlado antes de salir a acera, **siempre** con E-STOP accesible.

---

## Hardware que NO necesitas (tentaciones a evitar al principio)

- **LIDAR** — caro (300€+), añade complejidad. La cámara monocular con YOLO es suficiente para validar la lógica.
- **Cámara estéreo / depth** — útil para detección de bordillos, pero el algoritmo no está implementado aún. Dejar para fase 2.
- **RTK GPS** (~500€) — accuracy cm. Mata las dudas de localización pero el robot va por aceras anchas; M8N + IMU es suficiente.
- **ROS** — tentador como infraestructura, pero todo el stack actual es Python plano y funciona. Migrar a ROS solo si en algún momento necesitas integrar con simuladores tipo Gazebo o multi-robot.

---

## Resumen económico

| Fase | Coste acumulado | Qué validas |
|------|-----------------|-------------|
| Tier 0 | 0€ | Toda la lógica software |
| Tier 1 | ~70€ | Percepción con frames reales |
| Tier 2 | ~110€ | Localización GPS |
| Tier 3 | ~145€ | Heading fusionado GPS+IMU |
| Tier 4 | ~250€ | Movimiento real, comandos motor |
| Tier 5 | ~700-1000€ | Robot autónomo completo |
