# TFG — Dron autónomo de bajo coste (plataforma estilo FPV)

> Documento vivo de proyecto. Última actualización: 2026-06-14
> Mantén la sección **Bitácora** al día y marca las casillas a medida que avances.

---

## 1. Resumen del proyecto

Diseño, construcción y programación de una plataforma de dron cuadricóptero
**autónomo y de bajo coste**, inspirada en los drones FPV de bajo coste usados en
Ucrania, con foco en el **software de autonomía y visión por computador**.

El dron físico es el banco de pruebas; la **contribución evaluable es la capa de
software** (navegación autónoma, navegación sin GPS y seguimiento de objetivo por
visión) ejecutada sobre una plataforma cuyo coste se trata como métrica de diseño.

- **Ámbito**: Informática / Software
- **Núcleo de la tesis**: stack de software de autonomía + integración optimizada en coste
- **Inspiración**: barato, fabricable en serie, capaz sin GPS, guiado por visión

---

## 2. Alcance

### Dentro del alcance
- Plataforma cuadricóptero funcional (frame, motores, FC, companion computer).
- Stack de software propio sobre ROS 2 para misiones autónomas.
- Navegación por waypoints y navegación sin GPS (odometría visual-inercial).
- Detección y seguimiento de objetivo por visión por computador.
- Validación en simulación (PX4 SITL + Gazebo) antes del vuelo real.
- Evaluación cuantitativa: error de navegación, precisión de seguimiento, latencia, coste.

### Fuera del alcance
- Cualquier carga útil o mecanismo de armamento. El proyecto trata exclusivamente la
  plataforma y su software; no la weaponización, que ni aporta valor académico ni
  procede en un TFG.
- Vuelo BVLOS no autorizado (ver sección Legal).

---

## 3. Arquitectura del sistema

Arquitectura de **dos cerebros**, estándar en drones autónomos:

| Capa | Hardware | Software | ¿Lo programo yo? |
|------|----------|----------|------------------|
| Control de vuelo (FC) | Placa clase Pixhawk | PX4 o ArduPilot (C++) | No — se configura |
| Companion computer | Raspberry Pi 5 / Jetson Orin Nano | **Mi stack (ROS 2)** | **Sí — núcleo del TFG** |

Comunicación entre ambas mediante el protocolo **MAVLink**.

```
[Cámara] --> [Companion computer: visión + autonomía (ROS 2)] --MAVLink--> [FC: estabilización + motores]
```

---

## 4. Stack tecnológico

- **Lenguajes**: Python (lógica de alto nivel, nodos, prototipado) + C++ (visión y control crítico).
- **Framework**: ROS 2 (Humble o Jazzy).
- **Comunicación con la FC**: MAVSDK o pymavlink (MAVLink).
- **Visión**: OpenCV + modelo ligero (YOLO11n / YOLOv8n).
- **Simulación**: PX4 SITL + Gazebo.
- **Estación de tierra**: QGroundControl.
- **Firmware de vuelo**: PX4 o ArduPilot (decidir en Fase 0).

> Verificar versiones actuales de ROS 2, PX4 y los modelos YOLO al iniciar, ya que evolucionan.

---

## 5. Plan por fases

- [ ] **Fase 0 — Preparación y decisiones**
  - [ ] Elegir firmware de vuelo (PX4 vs ArduPilot) y justificarlo.
  - [ ] Elegir companion computer (Raspberry Pi 5 vs Jetson) según necesidades de ML.
  - [ ] Montar entorno de desarrollo (ROS 2, PX4 SITL, Gazebo).
- [ ] **Fase 1 — Simulación primero** *(empezamos aquí)*
  - [ ] Volar misiones por waypoints en SITL + Gazebo.
  - [ ] Desarrollar el stack de software completo en virtual.
- [ ] **Fase 2 — Construcción de la plataforma física**
  - [ ] Compra de componentes (ver BOM).
  - [ ] Ensamblaje y cableado.
- [ ] **Fase 3 — Bring-up**
  - [ ] Vuelo manual estable.
  - [ ] Integración del companion computer (enlace MAVLink).
- [ ] **Fase 4 — Porte a hardware real**
  - [ ] Portar software validado en sim.
  - [ ] Pruebas incrementales (atado/interior primero).
- [ ] **Fase 5 — Evaluación y memoria**
  - [ ] Recoger métricas.
  - [ ] Redactar resultados y conclusiones.

---

## 6. Roadmap de SOFTWARE (prioridad de arranque)

> Todo esto se desarrolla y valida primero en simulación.

### Hito S0 — Entorno listo
- [ ] Instalar ROS 2 y verificar con nodos de ejemplo (talker/listener).
- [ ] Levantar PX4 SITL + Gazebo y volar en modo manual/asistido.
- [ ] Conectar QGroundControl al SITL.

### Hito S1 — Comunicación con la FC
- [ ] "Hola mundo" MAVLink: leer telemetría (posición, batería, modo) desde Python.
- [ ] Enviar comandos básicos: armar, despegar, aterrizar (en SITL).
- [ ] Nodo ROS 2 que publique telemetría y exponga servicios de comando.

### Hito S2 — Navegación por waypoints
- [ ] Cargar y ejecutar una misión de waypoints en SITL.
- [ ] Lógica propia de misión (no la nativa): generar y enviar waypoints desde mi nodo.
- [ ] Manejo de eventos: fallo de enlace, batería baja, retorno seguro.

### Hito S3 — Navegación sin GPS (VIO)
- [ ] Integrar una fuente de odometría visual-inercial (p. ej. VINS-Fusion) en sim.
- [ ] Alimentar la pose estimada a la FC como sustituto del GPS.
- [ ] Medir deriva/error frente a la verdad-terreno del simulador.

### Hito S4 — Visión: detección y seguimiento
- [ ] Pipeline de cámara en ROS 2 (suscripción al stream simulado).
- [ ] Detección de objeto con YOLO ligero (clase de prueba, p. ej. un vehículo en Gazebo).
- [ ] Tracker que mantenga el objetivo entre frames.
- [ ] Convertir la posición del objetivo en comandos de velocidad hacia la FC ("lock-on").
- [ ] Comportamiento ante pérdida del objetivo (búsqueda / hover).

### Hito S5 — Integración y robustez
- [ ] Máquina de estados que combine navegación + visión.
- [ ] Pruebas de estrés en sim (latencias, pérdida de detección, viento).
- [ ] Logging de métricas para la memoria.

---

## 7. Presupuesto / Lista de materiales (BOM)

> Estimación orientativa; el coste es una **métrica de diseño** del proyecto.

| Componente | Estimación (€) | Notas |
|-----------|----------------|-------|
| Frame 5" fibra de carbono | 20–30 | |
| 4× motores brushless + ESCs | 40–60 | |
| Controladora de vuelo (Pixhawk-class) | 40–120 | Necesaria para autonomía |
| Companion computer | 60–250 | RPi 5 (barato) vs Jetson Orin Nano (ML) |
| Cámara | 15–40 | |
| Sensores de rango (TFmini-S ×3) | 9–24 | Telémetros láser UART (front, left, right) |
| Radio + telemetría | 30–60 | |
| Batería, hélices, varios | 40–60 | |
| **Total estimado** | **~261–526** | Según companion computer |

---

## 8. Métricas de evaluación (capítulo de resultados)

- [ ] Error de navegación por waypoints (m).
- [ ] Deriva de la navegación sin GPS frente a verdad-terreno (m / min).
- [ ] Precisión del seguimiento (% de frames con objetivo correctamente fijado).
- [ ] Latencia del lazo visión → comando (ms).
- [ ] Coste total de la plataforma (€) y coste por capacidad.

---

## 9. Consideraciones legales

- [ ] Registro como operador en **AESA** (España) bajo categoría "abierta" de **EASA**.
- [ ] Revisar zonas geográficas y restricciones de vuelo.
- [ ] El vuelo autónomo BVLOS está muy restringido → priorizar simulación y pruebas en línea de visión.

---

## 10. Estado del arte / referencias

> Ir rellenando a medida que se investiga.

- [ ] PX4 / ArduPilot (firmware de autopiloto de código abierto).
- [ ] ROS 2 (framework de robótica).
- [ ] MAVLink / MAVSDK (protocolo y SDK de comunicación).
- [ ] VIO: VINS-Fusion u similares (navegación sin GPS).
- [ ] YOLO (detección de objetos en tiempo real).
- [ ] Doctrina de drones FPV de bajo coste (contexto y motivación).

---

## 11. Bitácora de seguimiento

> Una línea por sesión de trabajo. Lo más reciente arriba.

| Fecha | Fase/Hito | Qué hice | Próximo paso |
|-------|-----------|----------|--------------|
| 2026-06-14 | — | Creación del documento de proyecto | Empezar Hito S0: montar entorno |

---

## 12. Decisiones abiertas

- [ ] ¿PX4 o ArduPilot?
- [ ] ¿Raspberry Pi 5 o Jetson Orin Nano?
- [ ] ¿Qué clase de objeto usar como objetivo de prueba en visión?
- [ ] ¿Hasta dónde llega la validación real vs. solo simulación?
