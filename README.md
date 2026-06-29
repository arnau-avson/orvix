# ORVIX — Open Robotics for Versatile Intelligent eXploration

Monorepo de plataformas robóticas autónomas de bajo coste, desarrollado como Trabajo de Fin de Grado (TFG). El proyecto contiene dos vehículos complementarios — un dron cuadricóptero y un robot terrestre de reparto — que comparten principios de diseño: autonomía basada en visión, navegación GPS, detección de objetos con YOLO e integración hardware progresiva validada primero en simulación.

```
orvix/
├── orvix_WarDrone/   Cuadricóptero autónomo · ROS 2 + PX4 + MAVSDK
├── orvix_WarTank/    Robot de reparto en acera · Python puro + OSM
└── README.md         ← este archivo
```

---

## Índice

1. [Visión general](#visión-general)
2. [orvix_WarDrone](#orvix_wardrone)
3. [orvix_WarTank](#orvix_wartank)
4. [Tabla comparativa](#tabla-comparativa)
5. [Tecnologías comunes](#tecnologías-comunes)
6. [Requisitos previos](#requisitos-previos)
7. [Inicio rápido](#inicio-rápido)
8. [Tests](#tests)
9. [Estado del proyecto](#estado-del-proyecto)
10. [Licencia](#licencia)

---

## Visión general

| | **WarDrone** | **WarTank** |
|---|---|---|
| **Plataforma** | Cuadricóptero FPV 5" | Robot 4WD de acera |
| **Dominio** | Navegación aérea, seguimiento de objetivos | Reparto autónomo urbano en zonas peatonales |
| **Middleware** | ROS 2 Humble | Python puro (sin ROS) |
| **Controlador de vuelo / motor** | PX4 vía MAVSDK | MCU (ESP32/Arduino) vía serial ASCII |
| **Simulación** | PX4 SITL + Gazebo (Docker) | Simulador Python integrado |
| **Visión** | YOLO11n (detección + tracking SORT) | YOLOv8n (semáforos + obstáculos + HSV) |
| **Navegación** | Waypoints GPS + VIO (VINS-Fusion) | Rutas A* sobre grafo peatonal de OSM |
| **Coste estimado** | ~250–500 € | ~250–1000 € (según tier) |

---

## orvix_WarDrone

Stack de autonomía ROS 2 para un cuadricóptero de bajo coste que opera sobre un autopiloto PX4 con companion computer (Raspberry Pi 5 / Jetson Orin Nano).

### Arquitectura

```
 PX4 Autopilot (Pixhawk)
       ↕ MAVLink (UDP / Serial)
 ┌─────────────────────────────────────────────────────────┐
 │  wardrone_driver        MAVSDK Bridge                   │
 │    ├─ Publica telemetría, estado, batería, GPS          │
 │    ├─ Recibe cmd_velocity, cmd_goto_global, VIO pose    │
 │    └─ Action servers: Takeoff, Land                     │
 ├─────────────────────────────────────────────────────────┤
 │  wardrone_mission       Mission Controller (FSM)        │
 │    └─ 10 estados: IDLE → PREFLIGHT → TAKEOFF →          │
 │       NAVIGATE → SEARCH → TRACK → RTL → LAND → DONE    │
 ├─────────────────────────────────────────────────────────┤
 │  wardrone_navigation    Waypoint Navigator + Safety     │
 │    ├─ Ejecuta misiones (waypoints YAML)                 │
 │    ├─ Safety monitor: batería, GPS, link loss           │
 │    ├─ Obstacle detector (MOG2+YOLO, 10 sectores)       │
 │    └─ Obstacle avoidance (clasificación + geométrica)   │
 ├─────────────────────────────────────────────────────────┤
 │  wardrone_vision        Cámara + YOLO + Tracker         │
 │    ├─ Camera node (Gazebo / V4L2 / CSI)                 │
 │    ├─ Detector node (YOLO11n, clases configurables)     │
 │    └─ Tracker node (SORT + lock-on PID pursuit)         │
 ├─────────────────────────────────────────────────────────┤
 │  wardrone_vio           Visual-Inertial Odometry        │
 │    └─ Bridge VINS-Fusion / ground truth → PX4 EKF2      │
 ├─────────────────────────────────────────────────────────┤
 │  wardrone_interfaces    Msgs (11), Srvs (4), Actions (4)│
 │  wardrone_bringup       Launch files + YAML configs     │
 └─────────────────────────────────────────────────────────┘
```

### Paquetes ROS 2

| Paquete | Descripción |
|---|---|
| `wardrone_bringup` | Launch files (SITL full, navigation, vision, hardware real), configuración YAML y misiones de ejemplo |
| `wardrone_interfaces` | 11 mensajes (`Telemetry`, `Detection`, `Obstacle`, `MissionState`, `Waypoint`…), 4 servicios (`Arm`, `SetFlightMode`, `LoadMission`, `SetTrackingTarget`), 4 acciones (`Takeoff`, `Land`, `GoToWaypoint`, `ExecuteMission`) |
| `wardrone_driver` | Bridge MAVSDK ↔ ROS 2 — telemetría a 10 Hz, conversión ENU ↔ NED, action servers de despegue y aterrizaje |
| `wardrone_mission` | Máquina de estados genérica con guardas, callbacks de entrada/salida, transiciones globales e historial; controlador de misión con 10 estados y eventos |
| `wardrone_navigation` | Navegador de waypoints (Haversine, radio de aceptación configurable), monitor de seguridad (batería 30%/15%, pérdida de enlace, calidad GPS), detector de obstáculos (MOG2 + YOLO, 10 sectores), evasión reactiva (clasificación + geométrica) |
| `wardrone_vision` | Nodo de cámara (Gazebo/V4L2/CSI), detector YOLO11n, tracker SORT con lock-on pursuit (PID yaw + forward), wrapper YOLO reutilizable |
| `wardrone_vio` | Bridge de odometría visual (VINS-Fusion o ground truth de Gazebo) a pose PX4, transformación cámara → body → map a 30 Hz |

### Funcionalidades

| Funcionalidad | Estado |
|---|---|
| Navegación por waypoints GPS | Completa |
| Despegue / aterrizaje autónomo | Completa |
| Máquina de estados de misión (10 estados) | Completa |
| Telemetría PX4 a 10 Hz | Completa |
| Odometría visual (VIO) para vuelo sin GPS | Completa |
| Detección YOLO multi-clase | Completa |
| Tracking de objetos (SORT) | Completa |
| Persecución lock-on (PID) | Completa |
| Monitor de seguridad (batería, GPS, enlace) | Completa |
| Detección de obstáculos (MOG2 + YOLO, 10 sectores) | Completa |
| Evasión de obstáculos (clasificación + geométrica) | Completa |
| Control de velocidad por waypoint | Completa |

### Simulación con Docker

```bash
cd orvix_WarDrone
docker compose build
docker compose up                          # PX4 SITL + Gazebo + stack ROS 2
docker compose run wardrone-dev bash       # Shell interactiva
docker compose run wardrone-dev colcon-build
docker compose run wardrone-dev test-unit
```

---

## orvix_WarTank

Robot de reparto autónomo para aceras, 100% Python. Navega de punto A a punto B por rutas peatonales de OpenStreetMap a ~5 km/h, respetando semáforos, evitando obstáculos y gestionando cruces de calzada.

### Arquitectura

```
 ┌───────────────────────────────────────────────────────────┐
 │  Routing Layer                                            │
 │    geocoder → map_loader → router (A*) → traffic_lights   │
 │    crossings · buildings · export (GeoJSON)                │
 ├───────────────────────────────────────────────────────────┤
 │  Perception Layer                                         │
 │    camera (OpenCV) → YOLOv8 detector → HSV classifier     │
 │    obstacle detector (15 clases COCO)                     │
 ├───────────────────────────────────────────────────────────┤
 │  Fusion Layer                                             │
 │    TemporalStateVoter (ventana deslizante + mayoría)       │
 │    FusedTrafficLightSensor · FusedObstacleGate             │
 ├───────────────────────────────────────────────────────────┤
 │  Localization Layer                                       │
 │    GPS (NMEA) + IMU → GPSIMULocalizer (filtro complem.)   │
 │    RouteTracker (proyección sobre polilínea de ruta)       │
 ├───────────────────────────────────────────────────────────┤
 │  Navigation Layer (State Machine — 9 estados)             │
 │    IDLE → WALKING → APPROACHING_CROSSING →                │
 │    WAITING_AT_CROSSING → CROSSING → ARRIVED               │
 │         ↕ STOPPED_FOR_OBSTACLE   ↕ OFF_ROUTE → ERROR      │
 ├───────────────────────────────────────────────────────────┤
 │  Mission Layer                                            │
 │    MissionManager: geocode → plan → execute → replan (×3) │
 ├───────────────────────────────────────────────────────────┤
 │  Hardware Layer                                           │
 │    camera · gps · imu · motors (serial ASCII a MCU)       │
 │    Robot.run_mission() — bucle principal a 5-10 Hz        │
 ├───────────────────────────────────────────────────────────┤
 │  Observability                                            │
 │    Logging JSONL estructurado + replay para diagnóstico    │
 └───────────────────────────────────────────────────────────┘
```

### Módulos

| Módulo | Descripción |
|---|---|
| `delivery_robot/models.py` | Tipos de dominio: `Point(lat, lon)`, `Step`, `Route` |
| `delivery_robot/geocoder.py` | Geocodificación bidireccional vía Nominatim |
| `delivery_robot/map_loader.py` | Carga de grafos OSM peatonales (osmnx), caché en GraphML |
| `delivery_robot/router.py` | A* con heurística Haversine sobre grafo peatonal |
| `delivery_robot/traffic_lights.py` | Detección de semáforos desde tags OSM, anotación de ruta |
| `delivery_robot/geometry.py` | Haversine, bearing, proyección sobre segmento |
| `delivery_robot/crossings.py` | Detección geométrica de cruces de calzada |
| `delivery_robot/buildings.py` | Validación de ruta contra polígonos de edificios OSM |
| `delivery_robot/export.py` | Exportación GeoJSON (ruta, semáforos, cruces, conflictos) |
| `delivery_robot/perception/` | Detector YOLO de semáforos, clasificador HSV (rojo/amarillo/verde), detector de obstáculos (15 clases) |
| `delivery_robot/fusion/` | Votación temporal por ventana deslizante, fusión de sensores |
| `delivery_robot/localization/` | Tracker de ruta, simulador, fusión GPS+IMU |
| `delivery_robot/navigation/` | Orquestador con 9 estados, monitor de recuperación (stuck detection) |
| `delivery_robot/mission/` | Ciclo de vida de misión, replanificación automática (máx. 3 intentos) |
| `delivery_robot/hardware/` | Adaptadores: cámara (OpenCV), GPS (NMEA), IMU (serial), motores (serial ASCII) |
| `delivery_robot/observability.py` | Logging JSONL estructurado + utilidad de replay |

### Funcionalidades

| Funcionalidad | Estado |
|---|---|
| Routing A* sobre aceras OSM | Completa |
| Geocodificación de direcciones | Completa |
| Detección de semáforos (YOLO + HSV) | Completa |
| Fusión temporal multi-frame | Completa |
| Detección de obstáculos (15 clases COCO) | Completa |
| Máquina de estados de navegación (9 estados) | Completa |
| Fusión GPS + IMU (filtro complementario) | Completa |
| Tracking de ruta (proyección sobre polilínea) | Completa |
| Gestión de misión con replanificación | Completa |
| Detección de cruces de calzada | Completa |
| Validación contra edificios | Completa |
| Logging JSONL + replay | Completa |
| Firmware MCU (ESP32/Arduino) | Pendiente |
| Backend de comunicación (REST/MQTT) | Pendiente |
| Detección de paso de cebra por visión | Pendiente |

### Tiers de hardware

| Tier | Componentes | Coste | Valida |
|---|---|---|---|
| 0 | Nada (solo software) | 0 € | Lógica, routing, máquina de estados |
| 1 | Webcam USB | ~70 € | Percepción real (semáforos, obstáculos) |
| 2 | GPS u-blox NEO-M8N | ~110 € | Localización, tracker, proyección |
| 3 | ESP32 + IMU BNO055 | ~145 € | Fusión GPS+IMU, heading estable |
| 4 | Motores + drivers + chasis 4WD | ~250 € | Movimiento, bucle de decisión |
| 5 | Raspberry Pi 5 + batería + E-STOP | ~700–1000 € | Robot autónomo completo |

### Ejecución

```bash
cd orvix_WarTank
pip install -r requirements.txt

python main.py                    # Demo de routing (genera route.geojson)
python run_simulation.py          # Simulación completa (9 estados)
python run_simulation.py --real   # Simulación con inferencia YOLO real
python run_robot.py               # End-to-end con adaptadores hardware
```

---

## Tabla comparativa

| Aspecto | WarDrone | WarTank |
|---|---|---|
| **Lenguaje** | Python 3.10+ | Python 3.10+ |
| **Middleware** | ROS 2 Humble (ament_python) | Ninguno (puro Python) |
| **Detección de objetos** | YOLO11n (ultralytics) | YOLOv8n/m/l (ultralytics) |
| **Tracking** | SORT (IoU assignment) | — |
| **Navegación** | Waypoints GPS + VIO | A* sobre grafo OSM peatonal |
| **Máquina de estados** | 10 estados (genérica con guardas) | 9 estados (prioridad jerárquica) |
| **Seguridad** | Monitor: batería, GPS, enlace → RTL/LAND | Monitor: stuck detection → ERROR |
| **Simulación** | PX4 SITL + Gazebo (Docker) | Simulador Python integrado |
| **Containerización** | Docker Compose multi-servicio | — |
| **Protocolos HW** | MAVLink (UDP/Serial) | NMEA, $IMU, ASCII (Serial) |
| **Tests** | pytest (13 archivos) | pytest (12 archivos, 72 tests) |
| **Companion computer** | Raspberry Pi 5 / Jetson Orin Nano | Raspberry Pi 5 |

---

## Tecnologías comunes

- **Python 3.10+** — lenguaje principal en ambos proyectos
- **ultralytics (YOLO)** — detección de objetos en tiempo real
- **OpenCV** — captura de cámara y procesamiento de imagen
- **pytest** — framework de testing
- **Haversine / geometría GPS** — cálculos de distancia y bearing
- **Máquinas de estados** — control de misión basado en eventos
- **Adaptadores hardware** — abstracción de sensores con mocks para testing
- **Docker** (WarDrone) / **venv** (WarTank) — entornos aislados

---

## Requisitos previos

### WarDrone

- Docker y Docker Compose
- (Opcional) ROS 2 Humble instalado localmente
- (Opcional) PX4 Autopilot SITL

### WarTank

- Python 3.10+
- Dependencias: `pip install -r orvix_WarTank/requirements.txt`
- (Opcional) Webcam USB para percepción real
- (Opcional) GPS u-blox + IMU + motores para hardware real

---

## Inicio rápido

### WarDrone (simulación SITL)

```bash
cd orvix_WarDrone
docker compose build
docker compose up
# En otra terminal:
docker compose run wardrone-dev bash
source install/setup.bash
ros2 launch wardrone_bringup sitl_full.launch.py
```

### WarTank (simulación Python)

```bash
cd orvix_WarTank
pip install -r requirements.txt
python run_simulation.py
```

---

## Tests

```bash
# WarDrone — tests unitarios
cd orvix_WarDrone
docker compose run wardrone-dev test-unit
# o localmente:
colcon test --packages-select wardrone_driver wardrone_mission wardrone_navigation wardrone_vision wardrone_vio

# WarTank — 72 tests (~2.4s)
cd orvix_WarTank
pytest tests/ -v
```

---

## Estado del proyecto

| Fase | WarDrone | WarTank |
|---|---|---|
| Diseño de arquitectura | Completa | Completa |
| Implementación del software | ~95% | ~90% |
| Simulación validada | En progreso | Completa |
| Hardware construido | Pendiente | Pendiente (firmware MCU) |
| Integración en hardware real | Pendiente | Pendiente |
| Evaluación y métricas | Pendiente | Pendiente |

---

## Licencia

Proyecto académico (TFG). Consultar con el autor para términos de uso.
