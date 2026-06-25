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

- **Estado**: IMPLEMENTADO
- **Solucion**: Nodo `obstacle_detector_node` que procesa imagenes de hasta 8 camaras (front, front_right, right, rear_right, rear, rear_left, left, front_left) usando:
  - Sustraccion de fondo (MOG2) + analisis de contornos para deteccion rapida de movimiento
  - Clasificacion YOLO bajo demanda para identificar el tipo de obstaculo (pajaro, drone, vehiculo, etc.)
  - Estimacion de distancia monocular usando tamanos conocidos de objetos
  - Estimacion de velocidad de aproximacion mediante tasa de expansion del bounding box
  - Calculo de tiempo de colision (TTC) para detectar objetos que se acercan a alta velocidad
- **Archivos**:
  - `src/wardrone_navigation/wardrone_navigation/obstacle_detector_node.py`
  - `src/wardrone_interfaces/msg/Obstacle.msg`
  - `src/wardrone_interfaces/msg/ObstacleArray.msg`
  - `src/wardrone_bringup/config/obstacle_params.yaml`
- **Hardware necesario**:
  - Opcion A (economica): 8x OV5647 (~5 EUR cada una, 62deg FOV) = ~40 EUR
  - Opcion B (menos camaras): 4x IMX219 gran angular 160deg (~12 EUR cada una) = ~48 EUR
  - Conexion via USB o multiplexor CSI (ej: Arducam Multi-Camera Adapter)

---

## 4. Evasion reactiva de obstaculos

- **Estado**: IMPLEMENTADO
- **Solucion**: Nodo `obstacle_avoidance_node` con maquina de estados (CLEAR -> MONITORING -> AVOIDING -> RESUMING) y maniobras de evasion inteligentes basadas en el tipo de obstaculo:
  - **Edificio/arbol** -> deslizamiento lateral (son altos, subir no ayuda)
  - **Pajaro/animal** -> subir verticalmente (vuelan a altitud similar, subir los evita)
  - **Otro drone** -> deslizamiento lateral rapido (son agiles, salir de su trayectoria)
  - **Vehiculo/persona** -> subir (estan en el suelo)
  - **Desconocido** -> logica geometrica basada en sector y rutas disponibles
  - **Emergencia** -> parada inmediata (hover) cuando esta demasiado cerca
  - Reanudacion automatica de la ruta original tras esquivar
  - Escape diagonal cuando tanto subir como lateral estan bloqueados
- **Archivos**:
  - `src/wardrone_navigation/wardrone_navigation/obstacle_avoidance_node.py`
  - `src/wardrone_navigation/wardrone_navigation/safety_monitor_node.py` (integrado)
  - `src/wardrone_bringup/config/obstacle_params.yaml`
- **Hardware necesario**: Mismo que punto 3 (camaras)

---

## 5. Deteccion de obstaculos por detras y laterales

- **Estado**: IMPLEMENTADO (integrado en punto 3)
- **Solucion**: El `obstacle_detector_node` soporta 8 sectores de camara simultaneos cubriendo 360 grados:
  - FRONT, FRONT_RIGHT, RIGHT, REAR_RIGHT, REAR, REAR_LEFT, LEFT, FRONT_LEFT
  - Cada sector tiene su propio background subtractor y tracker de contornos independiente
  - Deteccion especial de objetos que se acercan a alta velocidad desde cualquier direccion
  - El nodo de evasion toma decisiones omnidireccionales considerando todos los sectores bloqueados
- **Archivos**: Mismos que punto 3
- **Hardware necesario**: Mismas camaras (8 sectores = cobertura completa 360deg)

---

## Resumen

| # | Funcionalidad | Estado | Hardware extra |
|---|--------------|--------|----------------|
| 1 | Aterrizaje automatico | Pendiente (solo software) | No |
| 2 | Control de velocidad | Pendiente (solo software) | No |
| 3 | Deteccion de obstaculos | **IMPLEMENTADO** | Camaras (~40-48 EUR) |
| 4 | Evasion reactiva | **IMPLEMENTADO** | Mismas camaras |
| 5 | Deteccion omnidireccional | **IMPLEMENTADO** | Mismas camaras (8 sectores) |

---

## Orden de implementacion recomendado

1. **Aterrizaje automatico** -- solo software, se puede hacer ya
2. **Control de velocidad** -- solo software, se puede hacer ya
3. ~~**Deteccion frontal + evasion**~~ -- HECHO
4. ~~**Deteccion trasera/lateral**~~ -- HECHO

---

## Hardware pendiente de montar (para puntos 3-5)

Para activar la deteccion de obstaculos, montar las camaras y publicar en los topics:
```
/wardrone/obstacle_cam/front/image_raw
/wardrone/obstacle_cam/front_right/image_raw
/wardrone/obstacle_cam/right/image_raw
/wardrone/obstacle_cam/rear_right/image_raw
/wardrone/obstacle_cam/rear/image_raw
/wardrone/obstacle_cam/rear_left/image_raw
/wardrone/obstacle_cam/left/image_raw
/wardrone/obstacle_cam/front_left/image_raw
```

Se pueden activar/desactivar sectores individuales en `obstacle_params.yaml` -> `enabled_sectors`.
