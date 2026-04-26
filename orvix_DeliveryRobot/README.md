# orvix_DeliveryRobot

Robot delivery autónomo para entregas urbanas por **acera** (no por carretera). Stack 100% Python.

---

## Objetivo

Robot tipo Starship/Serve: recorre aceras de un punto A a un punto B a velocidad peatonal (~5 km/h), respetando semáforos peatonales, esquivando obstáculos (personas, perros, mobiliario urbano), y completando misiones pickup → dropoff sin intervención humana.

**Decisiones de diseño:**
- Aceras, nunca calzada → grafo OSM filtrado a `footway/pedestrian/path/living_street`
- Velocidad humana → no necesita planificación en milisegundos, ciclo a ~5 Hz es suficiente
- Cámara mirando al frente como único sensor visual → percepción basada en YOLOv8 + heurísticas HSV
- Microcontrolador como hub de sensores y motores → desacopla timing de Linux

---

## Estado del software

### Completado

| Módulo | Qué hace |
|--------|----------|
| `models`, `geocoder`, `map_loader`, `router` | Routing A→B sobre grafo peatonal OpenStreetMap (modo estricto = solo footways/pedestrian) |
| `traffic_lights` | Identifica semáforos OSM en la ruta + dirección de cruce |
| `geometry` | Bearing, haversine, proyección punto→segmento |
| `export` | GeoJSON de la ruta |
| `localization/` | `Pose`, `LocalizationProvider`, `RouteSimulator`, `RouteTracker` (proyecta pose sobre ruta + emite eventos de aproximación) |
| `perception/detector` | YOLOv8 traffic-light con 5 filtros encadenados (conf, aspect ratio, tamaño, máscara overlays Street View, cross-check con `stop_sign`) |
| `perception/classifier` | HSV → `red`/`yellow`/`green`/`unknown` con filtro de blancos condicional (mata "Dirección prohibida" sin sacrificar semáforos amarillos sobre fondo claro) |
| `perception/sensor` | `ImageSensor` que selecciona el primario y valida estado |
| `perception/obstacles` | YOLOv8 sobre 15 clases COCO relevantes para acera + `should_stop()` con umbrales de tamaño y posición |
| `fusion/` | Voto temporal por ventana deslizante para semáforos y obstáculos (mata flicker de un solo frame) |
| `navigation/orchestrator` | State machine: `IDLE`/`WALKING`/`APPROACHING_CROSSING`/`WAITING_AT_CROSSING`/`CROSSING`/`STOPPED_FOR_OBSTACLE`/`OFF_ROUTE`/`ARRIVED`/`ERROR` con prioridad ARRIVED > OFF_ROUTE > OBSTACLE > LIGHT > WALKING |
| `mission/manager` | Geocode + plan + execute + replanning automático (cap 3 replans) + lifecycle `PENDING`→`COMPLETED` |
| `hardware/camera` | `OpenCVCamera` (USB/IP/video) + `ImageSequenceCamera` + `BlankCamera` |
| `hardware/gps` | `NMEASerialGPS` (parser GPGGA + GPRMC) + `MockGPS` |
| `hardware/imu` | `SerialIMU` (protocolo ASCII `$IMU,...`) + `MockIMU` |
| `hardware/fusion` | `GPSIMULocalizer` con filtro complementario en heading (GPS≥1m/s confía GPS, parado confía IMU) |
| `hardware/motors` | `SerialMotorController` (protocolo `go/wait/stop/estop/turn`) + `MockMotorController` |
| `hardware/robot` | `Robot.run_mission()` — bucle integrado con safety estop ante cualquier excepción |

### Lo que falta

- **Firmware Arduino/ESP32** que implemente los protocolos serie definidos por los adapters (`$IMU,...` para IMU, `go/wait/stop/estop` para motores). El contrato Python ↔ MCU está fijado; falta el lado MCU.
- **Logging estructurado** (Python `logging` + JSON) y soporte de replay desde logs para depurar incidentes.
- **Tests automáticos** (pytest) sobre orquestador, fusión, geometría, parser NMEA. Ahora todo se valida con scripts manuales.
- **Comportamientos de recuperación**: pérdida de GPS prolongada (>30s), semáforo `unknown` perpetuo, obstáculo bloqueando >5 min (¿avisar a operador remoto?).
- **Comunicación con backend**: recepción de misiones desde fleet management (REST/MQTT), telemetría en vivo, ETA, alertas.
- **Detección de pasos de cebra y bordillos**: ahora el robot sabe que hay un semáforo cerca, pero no exactamente dónde está la línea de stop. Necesitaría modelo entrenado en zebra crossings + sensor de profundidad para curbs.

---

## Arquitectura

```
delivery_robot/
├── models.py              Point, Step, Route
├── geocoder.py            Direcciones <-> coordenadas (Nominatim)
├── map_loader.py          Grafo peatonal OSM (modo estricto opcional)
├── router.py              A* sobre grafo peatonal
├── traffic_lights.py      Semáforos sobre la ruta + sensor ABC
├── geometry.py            Bearing, distancia, proyección
├── export.py              GeoJSON
├── localization/
│   ├── models.py          Pose
│   ├── provider.py        LocalizationProvider, RouteSimulator
│   └── tracker.py         RouteTracker
├── perception/
│   ├── detector.py        YOLOTrafficLightDetector
│   ├── classifier.py      HSV state classifier
│   ├── sensor.py          ImageSensor
│   ├── overlays.py        Máscaras Street View
│   └── obstacles.py       ObstacleDetector + should_stop
├── fusion/
│   ├── temporal.py        TemporalStateVoter
│   ├── traffic_light.py   FusedTrafficLightSensor
│   └── obstacle.py        FusedObstacleGate
├── navigation/
│   ├── states.py          NavigationState, NavigationAction
│   ├── decision.py        NavigationDecision
│   └── orchestrator.py    NavigationOrchestrator
├── mission/
│   ├── models.py          Mission, MissionStatus
│   └── manager.py         MissionManager
└── hardware/
    ├── camera.py          OpenCVCamera, ImageSequenceCamera, BlankCamera
    ├── gps.py             NMEASerialGPS, MockGPS
    ├── imu.py             SerialIMU, MockIMU
    ├── fusion.py          GPSIMULocalizer
    ├── motors.py          SerialMotorController, MockMotorController
    └── robot.py           Robot
```

---

## Scripts ejecutables

| Script | Qué prueba |
|--------|-----------|
| `main.py` | Routing + GeoJSON de la ruta |
| `test_perception.py <img>` | Detección semáforo + clasificación HSV |
| `test_obstacles.py <img>` | Detección obstáculos + decisión stop |
| `test_localization.py` | Pose simulada + tracker + aproximación |
| `run_simulation.py` | **Demo end-to-end** — 6 estados del state machine |
| `run_simulation.py --real` | Demo con YOLO real + cycling de imágenes |
| `run_robot.py` | Demo end-to-end con la capa hardware (mocks) |

---

## Setup

```bash
pip install -r requirements.txt
python run_simulation.py
```

Para probar la pipeline completa con hardware real, ver [HARDWARE.md](HARDWARE.md).

---

## Protocolos serie definidos

El lado Python está completo. El firmware del MCU debe implementar:

- **GPS → Python** (NMEA-0183 estándar — sale de fábrica del módulo u-blox):
  ```
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  ```

- **IMU → Python** (firmware MCU lo emite):
  ```
  $IMU,<ax_g>,<ay_g>,<az_g>,<gx_dps>,<gy_dps>,<gz_dps>,<heading_deg_or_empty>\n
  Ejemplo: $IMU,0.01,-0.02,0.99,0.5,-0.3,0.1,247.5
  ```

- **Python → Motores** (firmware MCU lo recibe):
  ```
  go <speed_mmps>\n     (e.g. "go 1400" → 1.4 m/s adelante)
  wait\n                (motores OFF, sin freno)
  stop\n                (frenado activo)
  estop\n               (parada de emergencia, latch hasta reset)
  turn <heading_deg>\n  (rotar a heading absoluto, opcional)
  ```
