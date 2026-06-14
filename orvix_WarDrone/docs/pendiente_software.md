# Software Pendiente -- WarDrone

> Funcionalidades que faltan o estan incompletas para vuelo autonomo completo.

---

## 1. Aterrizaje automatico al completar mision

- **Estado**: Esqueleto (action client creado pero nunca invocado)
- **Problema**: El driver PX4 tiene la accion `Land` implementada y funcional, pero el mission controller nunca llama a `send_goal()`. Cuando la mision termina, el drone se queda en hover indefinidamente.
- **Que falta**:
  - Conectar el estado `NAVIGATE -> RTL -> LAND -> DONE` en el mission controller
  - Invocar la accion `Land` cuando se completa el ultimo waypoint
  - Gestionar el evento `LANDED` para transicionar a `DONE`
- **Archivos afectados**: `src/wardrone_mission/wardrone_mission/mission_controller_node.py`
- **Hardware necesario**: Ninguno (solo software)

---

## 2. Control de velocidad crucero

- **Estado**: Parcial (velocidad parseada pero no enviada al vehiculo)
- **Problema**: El campo `speed_m_s` se lee del YAML de mision y se usa para calcular ETA, pero no se transmite a PX4. El drone vuela a la velocidad por defecto de PX4.
- **Que falta**:
  - Enviar setpoint de velocidad a PX4 via MAVSDK (`set_velocity_ned` o parametro en `goto`)
  - Publicar un mensaje de tipo `TwistStamped` o usar la API de velocidad de MAVSDK
  - Aplicar la velocidad por waypoint (ya disponible en `WaypointData.speed_m_s`)
- **Archivos afectados**: `src/wardrone_navigation/wardrone_navigation/waypoint_navigator_node.py`, `src/wardrone_driver/wardrone_driver/mavsdk_bridge_node.py`
- **Hardware necesario**: Ninguno (solo software)

---

## 3. Deteccion de obstaculos

- **Estado**: No implementado
- **Problema**: La vision (YOLO) solo detecta personas, coches, motos, etc. como objetivos de seguimiento. No hay deteccion de obstaculos (arboles, edificios, paredes, cables, otros drones).
- **Que falta**:
  - Sensor de distancia que proporcione profundidad (LiDAR, ultrasonico, o camara de profundidad)
  - Nodo ROS que publique distancias/pointcloud de obstaculos
  - Procesamiento para clasificar zonas libres vs zonas bloqueadas
- **Archivos afectados**: Nuevo nodo en `src/wardrone_navigation/` o `src/wardrone_vision/`
- **Hardware necesario**:
  - Opcion A: **LiDAR ligero** (TFmini Plus ~25 EUR, rango 12m, 1D frontal)
  - Opcion B: **Sensor ultrasonico** (HC-SR04 ~3 EUR, rango 4m, solo frontal, poco fiable en exterior)
  - Opcion C: **Camara de profundidad** (Intel RealSense D435 ~200 EUR, pointcloud 3D completa)
  - Recomendacion: TFmini Plus por coste/peso/rango

---

## 4. Evasion reactiva de obstaculos

- **Estado**: No implementado
- **Problema**: El waypoint navigator publica posiciones GPS sin comprobar si hay algo en el camino. Si hay un obstaculo, el drone se estrella.
- **Que falta**:
  - Logica de frenado de emergencia cuando se detecta obstaculo a distancia minima
  - Algoritmo de desvio (potential fields, VFH, o desvio lateral simple)
  - Logica de reanudacion de ruta original tras esquivar
  - Priorizacion: safety monitor debe poder interrumpir waypoint navigator
- **Archivos afectados**: `src/wardrone_navigation/wardrone_navigation/safety_monitor_node.py`, nuevo nodo de evasion
- **Hardware necesario**: Depende del punto 3 (sensor de distancia)

---

## 5. Deteccion de obstaculos por detras y laterales

- **Estado**: No implementado
- **Problema**: Aunque se implemente la evasion frontal, el drone no detectaria objetos que se acercan por detras o por los lados (otros drones, pajaros, etc.).
- **Que falta**:
  - Sensores de distancia adicionales (trasero y/o laterales)
  - Fusion de datos de multiples sensores en un mapa de ocupacion local
  - Evasion omnidireccional (no solo frontal)
- **Archivos afectados**: Nuevo nodo de fusion de sensores
- **Hardware necesario**:
  - Minimo: 1 sensor trasero adicional (TFmini Plus ~25 EUR)
  - Ideal: 4 sensores (frontal, trasero, izquierda, derecha) o 1 LiDAR 360 (RPLiDAR A1 ~100 EUR)

---

## Resumen

| # | Funcionalidad | Solo software | Hardware extra |
|---|--------------|---------------|----------------|
| 1 | Aterrizaje automatico | Si | No |
| 2 | Control de velocidad | Si | No |
| 3 | Deteccion de obstaculos | No | Sensor de distancia (~25 EUR) |
| 4 | Evasion reactiva | Parcial (necesita punto 3) | Sensor de distancia |
| 5 | Deteccion omnidireccional | No | Sensores adicionales (~25-100 EUR) |

---

## Orden de implementacion recomendado

1. **Aterrizaje automatico** -- solo software, se puede hacer ya
2. **Control de velocidad** -- solo software, se puede hacer ya
3. **Deteccion frontal + evasion** -- requiere comprar sensor (TFmini Plus)
4. **Deteccion trasera/lateral** -- requiere sensores adicionales
