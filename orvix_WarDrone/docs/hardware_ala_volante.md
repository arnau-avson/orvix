# Hardware Ala Volante (Flying Wing) -- WarDrone Fase 2

> Drone de ala fija tipo delta/flying wing con navegacion GPS, recuperacion en red de captura.
> Compatible con el software WarDrone (PX4 soporta fixed-wing nativo).

---

## Diferencias respecto al quadcopter (Fase 1)

| Aspecto | Quadcopter (Fase 1) | Ala volante (Fase 2) |
|---------|---------------------|----------------------|
| Motores | 4 (multirotor) | 1 (pusher trasero) |
| Control de actitud | ESCs + velocidad motores | 2 servos (elevones) |
| Despegue | Vertical (VTOL) | Lanzamiento manual o catapulta/bungee |
| Aterrizaje | Vertical | Red de captura / belly landing |
| Autonomia | ~10-15 min | ~30-45 min (mucho mas eficiente) |
| Velocidad crucero | ~30 km/h | ~60-80 km/h |
| Firmware PX4 | Airframe: Generic Quadcopter | Airframe: Generic Flying Wing |

---

## Opcion A: Componentes por separado (mas barato)

| # | Componente | Modelo | Enlace | Precio aprox. |
|---|-----------|--------|--------|---------------|
| 1 | **Airframe** | SonicModell AR Wing 900mm (EPP, pusher) | [Banggood - AR Wing 900](https://www.banggood.com/Sonicmodell-AR-Wing-900mm-Wingspan-EPP-FPV-Fly-Wing-Fixed-Wing-RC-Airplane-KIT-p-1175815.html) | ~45 EUR |
| 2 | **Motor** | 2216 1400KV Brushless (pusher config) | [Amazon - Readytosky 2216 1400KV](https://www.amazon.com/Readytosky-Brushless-Phantom-Quadcopter-Multirotor/dp/B075DD16LK) | ~12 EUR |
| 3 | **ESC** | 40A Brushless ESC con BEC 5V | [Amazon - Readytosky 40A ESC](https://www.amazon.com/Readytosky-Brushless-Electric-Controller-Quadcopter/dp/B07DJ1S525) | ~10 EUR |
| 4 | **Servos x2** | MG90S 9g metal gear (para elevones) | [Amazon - MG90S pack 4uds](https://www.amazon.com/Maxmoral-Upgraded-Digital-Vehicle-Helicopter/dp/B07NQJ1VZ2) | ~10 EUR |
| 5 | **Helice** | APC 6x4E pusher (o 7x5E) | [Amazon - APC 6x4 Pusher](https://www.amazon.com/APC-Propellers-06040EP-Electric-Propeller/dp/B002U1NWSK) | ~5 EUR |
| 6 | **Flight Controller** | Pixhawk PX4 2.4.8 (con safety switch, buzzer, SD) | [Amazon.es - LiteBee Pixhawk 2.4.8](https://www.amazon.es/Controlador-Controller-integrado-Quadcopter-LITEBEE/dp/B072FKFX3J) | ~35 EUR |
| 7 | **GPS** | M8N con brujula, compatible Pixhawk | [Amazon - HAWK'S WORK M8N GPS](https://www.amazon.com/-/es/M%C3%B3dulo-soporte-precisi%C3%B3n-posicionamiento-Multirotor/dp/B09YC5X8SF) | ~18 EUR |
| 8 | **Companion Computer** | Raspberry Pi 4 Model B 2GB | [Amazon.es - RPi 4 2GB](https://www.amazon.es/Raspberry-ARM-Cortex-A72-WLAN-ac-Bluetooth-Micro-HDMI/dp/B07TD42S27) | ~55 EUR |
| 9 | **MicroSD** | 32GB clase 10 (para RPi) | Amazon.es - cualquier marca | ~8 EUR |
| 10 | **Bateria** | LiPo 3S 11.1V 2200mAh 35C, conector XT60 | [Amazon.es - Dilwe 3S 2200mAh](https://www.amazon.es/Dilwe-Bater%C3%ADa-2200mAh-Capacidad-teledirigidos/dp/B0BNBYQ4JZ) | ~20 EUR |
| 11 | **Cargador LiPo** | IMAX B6 + adaptador de corriente 12V | [Amazon.es - SATKIT IMAX B6 + adaptador](https://www.amazon.es/SATKIT-Cargador-descargador-Adaptador-Corriente/dp/B06Y44KXYS) | ~25 EUR |
| 12 | **Radio RC** | FlySky FS-i6 6CH + receptor FS-iA6B | [Amazon - FlySky FS-i6 + iA6](https://www.amazon.com/-/es/Transmisor-Controlador-Receptor-Helic%C3%B3ptero-Quadcopter/dp/B07CWBQ2HM) | ~45 EUR |
| 13 | **BEC 5V** | UBEC 5V 3A (alimentar RPi desde bateria LiPo) | [Amazon - UBEC 5V 3A (3 uds)](https://www.amazon.com/Regulator-Quadcopter-Airplane-Robotics-Raspberry/dp/B0CB4M3DYZ) | ~8 EUR |
| 14 | **Cable UART** | Pixhawk TELEM2 a RPi GPIO (serial TTL) | Amazon - cable dupont hembra 6 pin | ~3 EUR |
| 15 | **Varios** | Cinta de embalar, velcro, bisagras para elevones, pegamento EPP | Amazon/ferreteria | ~10 EUR |
| | | | **TOTAL OPCION A** | **~309 EUR** |

### Notas Opcion A
- El SonicModell AR Wing 900mm viene en **version KIT** (solo airframe EPP, sin electronica) -- perfecto para poner nuestro propio Pixhawk
- El motor 2216 1400KV va montado en configuracion **pusher** (empuja desde atras)
- Solo necesitas **2 servos** para los elevones (mezcla elevador + aleron en cada ala)
- El mismo Pixhawk, GPS, RPi, radio y bateria de la Fase 1 se pueden **reutilizar** -- ver seccion de reutilizacion abajo
- Las helices pusher giran al reves que las normales (ojo al comprar)
- El EPP (polipropileno expandido) es muy resistente a impactos, ideal para aterrizaje en red

---

## Opcion B: Airframes alternativos

| Airframe | Envergadura | Peso | Enlace | Precio aprox. | Notas |
|----------|------------|------|--------|---------------|-------|
| **SonicModell AR Wing 900** | 900mm | ~400g | [Banggood](https://www.banggood.com/Sonicmodell-AR-Wing-900mm-Wingspan-EPP-FPV-Fly-Wing-Fixed-Wing-RC-Airplane-KIT-p-1175815.html) | ~45 EUR | Mejor relacion calidad/precio. Compacto. |
| **SonicModell AR Wing Pro** | 1000mm | ~500g | [Banggood - AR Wing Pro](https://www.banggood.com/Sonicmodell-AR-Wing-PRO-1000mm-Wingspan-EPP-FPV-Flying-Wing-RC-Airplane-KIT-or-PNP-p-1756363.html) | ~55 EUR | Version mejorada, mas estable. |
| **Skywalker X5** | 1180mm | ~600g | Buscar "Skywalker X5" en Banggood/AliExpress | ~50 EUR | Clasico, mucho espacio interior. |
| **Skywalker X8** | 2120mm | ~1.2kg | Buscar "Skywalker X8" en AliExpress | ~80 EUR | Grande, carga util alta, mas autonomia. |
| **ZOHD Dart XL** | 1000mm | ~550g | [Banggood - ZOHD Dart XL](https://www.banggood.com/ZOHD-Dart-XL-Extreme-1000mm-Wingspan-BEPP-FPV-Aircraft-RC-Airplane-KIT-p-1357653.html) | ~55 EUR | Buena velocidad, robusto. |

### Recomendacion
- **Para empezar**: SonicModell AR Wing 900mm -- barato, ligero, facil de reparar (EPP)
- **Para carga util (camara, RPi)**: Skywalker X5 o X8 -- mas espacio interior
- **Para velocidad**: ZOHD Dart XL

---

## Reutilizacion de componentes de Fase 1

Si ya tienes el hardware del quadcopter (Fase 1), puedes reutilizar:

| Componente | Reutilizable? | Notas |
|-----------|--------------|-------|
| Pixhawk 2.4.8 | Si | Solo cambiar el airframe en QGroundControl a "Flying Wing" |
| GPS M8N | Si | Montar en el fuselaje del ala |
| Raspberry Pi 4 | Si | Misma conexion UART a TELEM2 |
| Radio FlySky FS-i6 | Si | Configurar mezcla de elevones |
| Bateria 3S 2200mAh | Si | Misma bateria sirve |
| Cargador IMAX B6 | Si | Mismo cargador |
| UBEC 5V | Si | Mismo regulador para RPi |
| Cable UART | Si | Mismo cable |
| Motores 2212 920KV | No | El ala volante necesita 1 motor 2216 1400KV (mayor KV) |
| ESCs 30A x4 | Parcial | Puedes usar 1 ESC de los 4, pero mejor uno de 40A |
| Frame F450 | No | El airframe es completamente diferente |
| Helices 1045 | No | Necesitas helice pusher (ej. APC 6x4E) |

**Coste adicional si reutilizas Fase 1**: Solo necesitas el airframe (~45 EUR) + motor (~12 EUR) + ESC (~10 EUR) + servos (~10 EUR) + helice (~5 EUR) + varios (~10 EUR) = **~92 EUR**

---

## Red de captura (recovery net)

Para recuperar el ala volante sin aterrizaje convencional:

| Componente | Modelo | Enlace | Precio aprox. |
|-----------|--------|--------|---------------|
| **Red** | Red de porteria futbol 3x2m (malla 10cm) | [Amazon.es - Red porteria 3x2m](https://www.amazon.es/s?k=red+porteria+futbol+3x2m) | ~15 EUR |
| **Postes** | 4x tubo PVC 40mm, 3m largo | Ferreteria (Leroy Merlin, Bricomart) | ~20 EUR |
| **Base** | Piquetas o sacos de arena para sujetar postes | Ferreteria | ~10 EUR |
| **Amortiguacion** | Espuma o colchoneta detras de la red | Opcional, el EPP ya absorbe impacto | ~0-15 EUR |
| | | **TOTAL RED** | **~45-60 EUR** |

### Montaje de la red
```
    3m
  |------|
  |      |  2m altura
  |  RED |
  |      |
  |______|
  Postes PVC enterrados o con base
```

- El drone vuela hacia la red a velocidad reducida (~20-30 km/h)
- PX4 puede configurarse para modo "land" que reduce throttle y el drone pierde sustentacion controladamente
- El EPP del airframe absorbe el impacto sin danarse
- Recuperas el drone de la red, cambias bateria, y vuelves a lanzar

### Alternativa: belly landing
- Sin red, el ala volante puede aterrizar sobre hierba/arena ("belly landing")
- PX4 tiene modo de aterrizaje automatico para fixed-wing
- El EPP es muy resistente a este tipo de aterrizaje
- Solo necesitas un campo abierto con hierba

---

## Lanzamiento

| Metodo | Coste | Notas |
|--------|-------|-------|
| **Lanzamiento manual** | 0 EUR | Lanzas el drone como un avion de papel, con motor al maximo. Funciona bien con alas <1kg |
| **Catapulta bungee DIY** | ~20 EUR | Goma elastica de 5m + estaca + carril. Para alas mas pesadas (>1kg) |

Para el AR Wing 900mm (~600g con electronica), el **lanzamiento manual** es suficiente. Se agarra por el centro del ala y se lanza con fuerza hacia adelante mientras el motor esta a maximo throttle.

---

## Configuracion PX4 para Flying Wing

En QGroundControl:
1. **Airframe**: Seleccionar "Flying Wing" > "Generic Flying Wing"
2. **Actuators**: Configurar salidas:
   - MAIN 1: Motor (ESC)
   - MAIN 2: Servo elevon izquierdo
   - MAIN 3: Servo elevon derecho
3. **Elevon mixing**: PX4 lo hace automaticamente con el airframe "Flying Wing"
4. **Parametros clave**:
   - `FW_AIRSPD_MIN`: 8 m/s (velocidad minima)
   - `FW_AIRSPD_TRIM`: 12 m/s (velocidad crucero)
   - `FW_AIRSPD_MAX`: 20 m/s (velocidad maxima)
   - `FW_THR_CRUISE`: 0.5 (throttle crucero ~50%)
   - `NAV_ACC_RAD`: 50m (radio de aceptacion de waypoint -- las alas necesitan mas)

---

## Conexiones

```
Bateria LiPo 3S ──> ESC 40A ──> Motor 2216 1400KV (pusher)
                ──> BEC del ESC ──> Pixhawk (alimentacion)
                ──> UBEC 5V ──> Raspberry Pi 4 (GPIO 5V+GND)

Pixhawk MAIN OUT 1 ──> ESC (senal PWM motor)
Pixhawk MAIN OUT 2 ──> Servo elevon izquierdo
Pixhawk MAIN OUT 3 ──> Servo elevon derecho

Pixhawk TELEM2 ──> RPi GPIO UART (TX/RX cruzados, 3.3V)
Pixhawk RC IN ──> Receptor FlySky FS-iA6B (PPM/iBUS)
Pixhawk GPS ──> Modulo M8N GPS (dentro del fuselaje)
```

---

## Resumen de costes

| Escenario | Precio total | Notas |
|-----------|-------------|-------|
| **Desde cero (todo nuevo)** | ~309 EUR | Sin red de captura |
| **Desde cero + red** | ~355-370 EUR | Con red de captura DIY |
| **Reutilizando Fase 1** | ~92 EUR | Solo airframe + motor + ESC + servos + helice |
| **Reutilizando Fase 1 + red** | ~137-152 EUR | Lo minimo para volar fixed-wing |

---

## Checklist antes de primer vuelo

- [ ] Airframe montado, elevones con bisagras y movimiento libre
- [ ] Motor pusher montado en la parte trasera, helice correcta (pusher)
- [ ] Servos de elevones conectados y centrados
- [ ] Pixhawk montado en el centro de gravedad (CG) con amortiguadores
- [ ] GPS montado en el fuselaje (alejado del ESC/motor)
- [ ] Centro de gravedad verificado (tipico: 25-30% de la cuerda desde el borde de ataque)
- [ ] Firmware PX4 flasheado, airframe "Generic Flying Wing" seleccionado
- [ ] Calibracion: acelerometro, giroscopo, magnetometro, radio, ESC
- [ ] Movimiento de elevones correcto: pitch up/down, roll left/right
- [ ] Receptor RC vinculado con emisora FlySky
- [ ] RPi conectada por UART a TELEM2
- [ ] UBEC alimentando RPi (verificar 5V estables)
- [ ] Bateria asegurada con velcro, CG verificado con bateria puesta
- [ ] Test en banco: motor responde al throttle, elevones responden a sticks
- [ ] Red de captura montada (si se usa)
- [ ] Primer vuelo manual con RC (lanzamiento manual, volar en circulos, aterrizar en red o belly)
- [ ] Segundo vuelo con RPi conectada (telemetria pasiva)
- [ ] Tercer vuelo: test autonomo (waypoints GPS simples, ida y vuelta)
