# Hardware Fase 1 -- WarDrone Prototipo de Prueba

> Vuelo basico, telemetria, navegacion GPS, safety monitor (milestones S1-S2)
> Compatible con ROS 2 y MAVSDK. Requisito: SIN SOLDADURA.
> Precios verificados: Junio 2025/2026

---

## Opcion A: Componentes por separado (mas barato, Amazon)

| # | Componente | Modelo | Enlace | Precio aprox. | Estado |
|---|-----------|--------|--------|---------------|--------|
| 1 | **Frame** | F450 con PDB integrada y tren de aterrizaje | [Amazon.es - EVTSCAN F450](https://www.amazon.es/EVTSCAN-Quadcopter-Aircraft-Accesorio-integrada/dp/B093DJX8SR) | ~18 EUR | Disponible |
| 2 | **Motores x4 + ESCs x4** | 2212 920KV + SimonK 30A (pack de 4, con bullet connectors 3.5mm) | [Amazon - Hobbypower Motor+ESC kit](https://www.amazon.com/Hobbypower-920KV-Brushless-SimonK-Quadcopter/dp/B07LG44BJ4) | ~45 EUR | Disponible |
| 3 | **Helices** | 1045 CW/CCW (4 pares = 2 sets completos) | [Amazon - QWinOut 1045 (4 pares)](https://www.amazon.com/QWinOut-Propeller-Compatible-Multi-Copter-Quadcopter/dp/B088FH9ZPB) | ~8 EUR | Disponible |
| 4 | **Flight Controller** | Pixhawk PX4 2.4.8 (con safety switch, buzzer, I2C splitter, SD 4GB) | [Amazon - Readytosky Pixhawk 2.4.8](https://www.amazon.com/Readytosky-Pixhawk-Controller-Autopilot-Splitter/dp/B07CHQ7SZ4) | ~50 EUR | Disponible |
| 4b | **Flight Controller (alt)** | Pixhawk PX4 2.4.8 + Power Module + Vibration Damping (HAWK'S WORK, sin GPS) | [Amazon - HAWK'S WORK Pixhawk 2.4.8](https://www.amazon.com/HAWKS-WORK-Pixhawk-Controller-Absorber/dp/B0CTZTJD4J) | ~55 EUR | Disponible |
| 5 | **GPS** | M8N con brujula, compatible Pixhawk (SoloGood) | [Amazon - SoloGood M8N GPS](https://www.amazon.com/SoloGood-M8N-Compass-NEO-M8N-Controller/dp/B0BD45J3F1) | ~20 EUR | Disponible |
| 5b | **GPS (alt)** | M8N con brujula (Readytosky) | [Amazon - Readytosky M8N GPS](https://www.amazon.com/Readytosky-Compass-Protective-Standard-Controller/dp/B01KK9A8QG) | ~22 EUR | Disponible |
| 6 | **Companion Computer** | Raspberry Pi 4 Model B 2GB | [Amazon.es - RPi 4 2GB](https://www.amazon.es/Raspberry-ARM-Cortex-A72-WLAN-ac-Bluetooth-Micro-HDMI/dp/B07TD42S27) | ~55 EUR | Disponible (desde 54.27 EUR en idealo.es) |
| 6b | **Companion (alt barata)** | Orange Pi Zero 2W 2GB (Allwinner H618, WiFi, BT) | [Amazon - Orange Pi Zero 2W 2GB](https://www.amazon.com/Allwinner-Computer-Frequency-Bluetooth-onboard/dp/B0CHM6XND9) | ~22 USD (~20 EUR) | Disponible |
| 6c | **Companion (alt)** | Orange Pi 3B 2GB (Rockchip RK3566, WiFi, BT, eMMC) | [Amazon - Orange Pi 3B 2GB](https://www.amazon.com/Orange-Pi-Quad-Core-Bluetooth-Development/dp/B0CDQB1731) | ~30 USD (~28 EUR) | Disponible |
| 6d | **Companion (alt)** | Raspberry Pi Zero 2W (512MB, mas limitada pero ultra-ligera) | [Amazon - RPi Zero 2W](https://www.amazon.com/Raspberry-Zero-Bluetooth-RPi-2W/dp/B09LH5SBPS) | ~15 USD (MSRP) | Stock variable |
| 7 | **MicroSD** | 32GB clase 10 (para RPi y Pixhawk) | [Amazon - SanDisk 32GB](https://www.amazon.com/SanDisk-32GB-MicroSDHC-Memory-Card/dp/B003WGJYCY) | ~7 EUR | Disponible |
| 8 | **Bateria** | LiPo 3S 11.1V 2200mAh 50C, conector XT60 (2-pack) | [Amazon - Zeee 3S 2200mAh XT60 (2-pack)](https://www.amazon.com/Zeee-Vehicles-Airplane-Quadcopter-Helicopter/dp/B0BYNSH6Q7) | ~37 USD (~34 EUR) para 2 baterias | Disponible |
| 8b | **Bateria (alt)** | GOLDBAT 3S 2200mAh 35C XT60 (2-pack) | [Amazon - GOLDBAT 3S 2200mAh](https://www.amazon.com/GOLDBAT-2200mAh-Airplane-Quadcopter-Helicopter/dp/B07W7CW6NV) | ~30 EUR (2 uds) | Disponible |
| 9 | **Cargador LiPo** | IMAX B6 con adaptador de corriente incluido (Amazon.es) | [Amazon.es - iMax B6 + adaptador + bolsa LiPo](https://www.amazon.es/iMax-profesional-multifunci%C3%B3n-adaptador-corriente/dp/B0FD6X4XW9) | ~63 EUR | Disponible |
| 9b | **Cargador (alt)** | SATKIT IMAX B6 + adaptador corriente (mas barato) | [Amazon.es - SATKIT IMAX B6](https://www.amazon.es/SATKIT-Cargador-descargador-Adaptador-Corriente/dp/B06Y44KXYS) | ~25 EUR | Verificar stock |
| 9c | **Cargador (alt)** | Havcybin IMAX B6 80W + adaptador 12V 5A | [Amazon - Havcybin IMAX B6](https://www.amazon.com/Havcybin-Battery-Balance-Discharger-Batteries/dp/B093P2L2XC) | ~30 USD | Disponible |
| 10 | **Radio RC** | FlySky FS-i6 6CH + receptor FS-iA6 | [Amazon - FlySky FS-i6 + iA6](https://www.amazon.com/Flysky-Transmitter-Controller-Helicopter-Quadcopter/dp/B07CWBQ2HM) | ~45 EUR | Disponible |
| 10b | **Radio RC (alt)** | FlySky FS-i6X 10CH + receptor FS-iA6B (mejor, mas canales) | [Amazon - FlySky FS-i6X](https://www.amazon.com/Flysky-FS-i6X-Transmitter-FS-iA6B-Receiver/dp/B0744DPPL8) | ~55 EUR | Disponible |
| 11 | **BEC 5V** | UBEC 5V 3A (alimentar RPi desde bateria LiPo) - 3 uds | [Amazon - FPVKing UBEC 5V 3A (3 uds)](https://www.amazon.com/FPVKing-Micro-Adjustable-Quadcopter-Drone/dp/B07PLSYX9G) | ~16 USD (~15 EUR) | Disponible (Amazon's Choice) |
| 12 | **Cable UART** | Adaptador Pixhawk TELEM2 a RPi GPIO (serial TTL) | [Landmark Landing - RPi to Pixhawk Adapter](https://landmarklanding.com/products/raspberry-pi-to-pixhawk-telemetry-adapter) | ~9 USD (~8 EUR) | En stock, envia en 1 dia |
| 12b | **Cable UART (alt)** | Cable dupont hembra 6 pin (DIY) | Amazon - cable dupont hembra 6 pin | ~3 EUR | Disponible |
| 13 | **Varios** | Cables, zip ties, conectores XT60, velcro, cinta | Amazon/ferreteria | ~10 EUR |  |
| | | | **TOTAL OPCION A (con RPi 4)** | **~330-350 EUR** | |
| | | | **TOTAL OPCION A (con Orange Pi Zero 2W)** | **~295-310 EUR** | |

### Notas Opcion A -- SIN SOLDADURA
- El pack Hobbypower (item 2) incluye 4 motores + 4 ESCs con bullet connectors 3.5mm preinstalados
- Los ESCs vienen con BEC integrado de 5V/2A (suficiente para el Pixhawk, pero usar UBEC aparte para la RPi)
- **PROBLEMA de soldadura**: La PDB integrada del F450 requiere soldar los ESCs a la placa PCB
  - **Solucion sin soldar**: Usar un Power Module aparte (incluido con el Pixhawk) para la alimentacion principal, y conectar los ESCs directamente a la bateria via un cableado paralelo con conectores XT60
  - **Alternativa**: Comprar el frame HAWK'S WORK Pre-soldered ([Amazon - HAWK'S WORK F450 Pre-soldered](https://www.amazon.com/HAWKS-WORK-Quadcopter-Soldered-Version/dp/B09YQ4TM88)) donde los ESCs ya vienen soldados a la PDB
  - **Alternativa 2**: Kit Set D Pre-soldered de HAWK'S WORK ([Amazon - Set D Pre-soldered](https://www.amazon.com/HAWKS-WORK-Quadcopter-Brushless-Pre-soldered/dp/B0DPH5Q4GT)) que incluye frame + motores + ESCs + helices pre-soldado
- Verificar que el conector de la bateria sea XT60 (el mas comun) o comprar adaptadores
- El cargador SATKIT incluye el adaptador de corriente; muchos otros IMAX B6 NO lo incluyen (verificar antes de comprar)
- Comprar al menos 2 juegos de helices; se rompen facilmente
- El adaptador Landmark Landing (item 12) es la solucion mas limpia para UART Pixhawk-RPi sin soldar

### Companion Computer: Comparativa

| SBC | RAM | CPU | WiFi | Peso | Precio | ROS 2 Compatible | Nota |
|-----|-----|-----|------|------|--------|-------------------|------|
| RPi 4 2GB | 2GB | Cortex-A72 4x1.5GHz | Si | 46g | ~55 EUR | Si (Ubuntu 22.04) | Mejor rendimiento, mas documentacion |
| Orange Pi 3B 2GB | 2GB | RK3566 4x1.8GHz | Si | ~50g | ~28 EUR | Si (Ubuntu 22.04) | Buena relacion precio/rendimiento |
| Orange Pi Zero 2W 2GB | 2GB | H618 4x1.5GHz | Si | ~15g | ~20 EUR | Si (Ubuntu 22.04) | Ultraligera, forma muy compacta |
| RPi Zero 2W | 512MB | Cortex-A53 4x1GHz | Si | 10g | ~15 USD | Limitada (poca RAM) | Solo para tareas muy basicas |

> **Recomendacion**: Para presupuesto minimo con buena compatibilidad ROS 2, la **Orange Pi Zero 2W 2GB** (~20 EUR) es la mejor opcion. Si se necesita mas potencia, la **Raspberry Pi 4 2GB** (~55 EUR) es la eleccion segura.

---

## Opcion B: Kit completo HAWK'S WORK (mas facil, SIN SOLDAR)

> **Ventaja principal**: PCB pre-soldada, no necesitas soldador. Plug-and-play con bullet connectors.

### Variantes disponibles (precios de hawks-work.com, Junio 2025)

| Kit | Incluye | Enlace Amazon | Precio USD | Precio EUR aprox. |
|-----|---------|---------------|-----------|-------------------|
| **F450-E** | Frame solamente | [Amazon - Frame](https://www.amazon.com/HAWKS-WORK-Quadcopter-Soldered-version/dp/B09YQ4G4ZZ) | $75.99 | ~70 EUR |
| **F450-D** | Frame + 4x Motor + 4x ESC + Helices | [Amazon - Set D Standard](https://www.amazon.com/HAWKS-WORK-Brushless-Accessory-Original/dp/B0CDNZDVY8) | $105.99 | ~98 EUR |
| **F450-D Pre-soldered** | Frame + 4x Motor + 4x ESC + Helices (PRE-SOLDADO) | [Amazon - Set D Pre-soldered](https://www.amazon.com/HAWKS-WORK-Quadcopter-Brushless-Pre-soldered/dp/B0DPH5Q4GT) | $105.99 | ~98 EUR |
| **F450-C** | Kit D + Bateria | [Amazon - Kit C](https://www.amazon.com/HAWKS-WORK-Quadcopter-Brushless-Transmitter/dp/B0DG2QNW1H) | $195.99 | ~180 EUR |
| **F450-B** | Kit C + Pixhawk 2.4.8 + GPS M8N + Power Module + Accesorios + Guia | [Amazon - Kit B](https://www.amazon.com/HAWKS-WORK-Controller-Unassembled-Quadcopter/dp/B09SZ74YFK) | $345.99 | ~320 EUR |
| **F450-A** | Kit B + FlySky FS-i6X + receptor FS-iA6B | [Amazon - Kit A (nuevo)](https://www.amazon.com/HAWKS-WORK-Quadcopter-Brushless-Transmitter/dp/B0DG2L1TQL) | $395.99 | ~365 EUR |
| **F450-A Complete** | Kit A version completa con todo | [Amazon - Kit A Complete](https://www.amazon.com/HAWKS-WORK-Engineering-Enthusiasts-Researchers/dp/B0GG8P2651) | $399.99 | ~370 EUR |

### Estrategia optima con HAWK'S WORK (SIN SOLDAR)

**Opcion B1: Kit B + componentes sueltos** (recomendada)

| Componente | Enlace | Precio |
|-----------|--------|--------|
| HAWK'S WORK F450 Kit B (pre-soldado) | [Amazon - Kit B](https://www.amazon.com/HAWKS-WORK-Controller-Unassembled-Quadcopter/dp/B09SZ74YFK) | ~320 EUR |
| FlySky FS-i6 (comprado aparte, mas barato) | [Amazon - FS-i6](https://www.amazon.com/Flysky-Transmitter-Controller-Helicopter-Quadcopter/dp/B07CWBQ2HM) | ~45 EUR |
| Raspberry Pi 4 2GB / Orange Pi Zero 2W | [Amazon.es - RPi 4](https://www.amazon.es/Raspberry-ARM-Cortex-A72-WLAN-ac-Bluetooth-Micro-HDMI/dp/B07TD42S27) o [Orange Pi Zero 2W](https://www.amazon.com/Allwinner-Computer-Frequency-Bluetooth-onboard/dp/B0CHM6XND9) | 20-55 EUR |
| MicroSD 32GB | Amazon | ~7 EUR |
| UBEC 5V 3A | [Amazon - FPVKing UBEC](https://www.amazon.com/FPVKing-Micro-Adjustable-Quadcopter-Drone/dp/B07PLSYX9G) | ~15 EUR |
| Cable UART Pixhawk-RPi | [Landmark Landing](https://landmarklanding.com/products/raspberry-pi-to-pixhawk-telemetry-adapter) | ~8 EUR |
| | **TOTAL con RPi 4** | **~450 EUR** |
| | **TOTAL con Orange Pi Zero 2W** | **~415 EUR** |

**Opcion B2: Kit A completo + complementos** (mas caro pero todo incluido)

| Componente | Enlace | Precio |
|-----------|--------|--------|
| HAWK'S WORK F450 Kit A | [Amazon - Kit A](https://www.amazon.com/HAWKS-WORK-Engineering-Enthusiasts-Researchers/dp/B0GG8P2651) | ~370 EUR |
| Raspberry Pi 4 2GB / Orange Pi Zero 2W | Idem | 20-55 EUR |
| MicroSD 32GB + UBEC + Cable UART | Idem | ~30 EUR |
| | **TOTAL con RPi 4** | **~455 EUR** |
| | **TOTAL con Orange Pi Zero 2W** | **~420 EUR** |

### Ventajas del kit HAWK'S WORK
- PCB pre-soldada (NO necesitas soldador)
- Guia de montaje + videos paso a paso
- Garantia de 3 meses en piezas defectuosas
- Bateria de 4200mAh (mas autonomia que las de 2200mAh individuales)
- Componentes verificados como compatibles entre si
- Espacio para Raspberry Pi, camaras, sensores adicionales
- Incluye Power Module, safety switch, buzzer, SD card, cables

---

## Opcion C: AliExpress (mas barato, envio lento)

| # | Componente | Enlace | Precio aprox. |
|---|-----------|--------|---------------|
| 1 | Kit F450 completo (Pixhawk 2.4.8 + GPS M8N + BLHeli 30A ESC + 2212 Motor + Frame + Telemetria 100MW) | [AliExpress - F450 Pixhawk Kit](https://www.aliexpress.com/item/1005008659371100.html) | ~90-136 EUR |
| 1b | Kit F450 alternativo (con APM2.8 + GPS 7M + SimonK 30A ESC + FlySky) | [AliExpress - F450 APM Kit](https://www.aliexpress.com/item/32926132442.html) | ~120 EUR (30% OFF) |
| 2 | FlySky FS-i6 + receptor | [AliExpress - FS-i6](https://www.aliexpress.com/i/1005001915144332.html) | ~35-41 EUR |
| 3 | Bateria 3S 2200mAh XT60 | Buscar "3S 2200mAh XT60" en AliExpress | ~12 EUR |
| 4 | Cargador IMAX B6 + adaptador | Buscar "IMAX B6 charger adapter" en AliExpress | ~15 EUR |
| 5 | Companion Computer (RPi 4 2GB) | [Amazon.es - RPi 4 2GB](https://www.amazon.es/Raspberry-ARM-Cortex-A72-WLAN-ac-Bluetooth-Micro-HDMI/dp/B07TD42S27) (mejor en Amazon) | ~55 EUR |
| 5b | Companion Computer (Orange Pi Zero 2W 2GB) | [Amazon - Orange Pi Zero 2W](https://www.amazon.com/Allwinner-Computer-Frequency-Bluetooth-onboard/dp/B0CHM6XND9) | ~20 EUR |
| 6 | UBEC 5V 3A + cables + microSD | AliExpress/Amazon | ~15 EUR |
| | **TOTAL AliExpress (con RPi 4)** | **~230-280 EUR** |
| | **TOTAL AliExpress (con Orange Pi Zero 2W)** | **~195-245 EUR** |

> **Aviso**: AliExpress tarda 15-30 dias en llegar a Espana. La calidad es variable. La RPi/Orange Pi mejor comprarla en Amazon/tienda local.
> **Nota**: El kit AliExpress con Pixhawk puede requerir SOLDADURA de ESCs a la PDB. Verificar antes de comprar si viene pre-soldado.
> **Nota 2**: Algunos kits AliExpress usan APM2.8 en vez de Pixhawk. APM2.8 NO es compatible con PX4/ROS 2. Asegurar que sea Pixhawk 2.4.8.

---

## Opcion D: Hibrida (mejor precio/calidad SIN SOLDAR)

> Combinar lo mejor de cada fuente para conseguir el precio mas bajo SIN necesidad de soldador.

| # | Componente | Fuente | Enlace | Precio |
|---|-----------|--------|--------|--------|
| 1 | **Frame F450 Pre-soldered** (con ESCs ya soldados a PDB) | Amazon | [HAWK'S WORK Set D Pre-soldered](https://www.amazon.com/HAWKS-WORK-Quadcopter-Brushless-Pre-soldered/dp/B0DPH5Q4GT) | ~98 EUR |
| 2 | **Pixhawk 2.4.8** + safety switch + buzzer + SD | Amazon | [Readytosky Pixhawk 2.4.8](https://www.amazon.com/Readytosky-Pixhawk-Controller-Autopilot-Splitter/dp/B07CHQ7SZ4) | ~50 EUR |
| 3 | **GPS M8N** con brujula | Amazon | [SoloGood M8N GPS](https://www.amazon.com/SoloGood-M8N-Compass-NEO-M8N-Controller/dp/B0BD45J3F1) | ~20 EUR |
| 4 | **Radio RC** FlySky FS-i6 + receptor | Amazon | [FlySky FS-i6 + iA6](https://www.amazon.com/Flysky-Transmitter-Controller-Helicopter-Quadcopter/dp/B07CWBQ2HM) | ~45 EUR |
| 5 | **Bateria** LiPo 3S 2200mAh XT60 (2-pack) | Amazon | [Zeee 3S 2200mAh (2-pack)](https://www.amazon.com/Zeee-Vehicles-Airplane-Quadcopter-Helicopter/dp/B0BYNSH6Q7) | ~34 EUR |
| 6 | **Cargador** IMAX B6 + adaptador | Amazon.es | [SATKIT IMAX B6](https://www.amazon.es/SATKIT-Cargador-descargador-Adaptador-Corriente/dp/B06Y44KXYS) | ~25 EUR |
| 7 | **Companion** Orange Pi Zero 2W 2GB | Amazon | [Orange Pi Zero 2W](https://www.amazon.com/Allwinner-Computer-Frequency-Bluetooth-onboard/dp/B0CHM6XND9) | ~20 EUR |
| 8 | **MicroSD** 32GB | Amazon | [SanDisk 32GB](https://www.amazon.com/SanDisk-32GB-MicroSDHC-Memory-Card/dp/B003WGJYCY) | ~7 EUR |
| 9 | **BEC/UBEC** 5V 3A | Amazon | [FPVKing UBEC 3pcs](https://www.amazon.com/FPVKing-Micro-Adjustable-Quadcopter-Drone/dp/B07PLSYX9G) | ~15 EUR |
| 10 | **Cable UART** Pixhawk TELEM2 a RPi | Landmark Landing | [RPi-Pixhawk Adapter](https://landmarklanding.com/products/raspberry-pi-to-pixhawk-telemetry-adapter) | ~8 EUR |
| 11 | **Varios** (cables, zip ties, velcro) | Amazon/ferreteria | | ~10 EUR |
| | | | **TOTAL OPCION D** | **~332 EUR** |

> **Con Raspberry Pi 4 en lugar de Orange Pi**: total sube a ~367 EUR

---

## Resumen comparativo

| Opcion | Precio total (OPi) | Precio total (RPi 4) | Envio | Soldadura | Dificultad |
|--------|--------------------|-----------------------|-------|-----------|------------|
| **A: Por separado (Amazon)** | ~295-310 EUR | ~330-350 EUR | 2-5 dias | SI (PDB) | Media |
| **B: Kit HAWK'S WORK** | ~415-420 EUR | ~450-455 EUR | 5-10 dias | NO | Baja |
| **C: AliExpress** | ~195-245 EUR | ~230-280 EUR | 15-30 dias | Variable | Media-Alta |
| **D: Hibrida (recomendada)** | **~332 EUR** | **~367 EUR** | 3-7 dias | **NO** | **Baja-Media** |

### Recomendacion final

- **Presupuesto minimo absoluto**: Opcion C (AliExpress), ~200-250 EUR. Riesgo de calidad y largos tiempos de envio.
- **Mejor relacion precio/calidad SIN SOLDAR**: Opcion D (Hibrida), ~332-367 EUR. Usa el frame pre-soldado de HAWK'S WORK con componentes sueltos mas baratos.
- **Maxima facilidad**: Opcion B (Kit HAWK'S WORK A), ~420-455 EUR. Todo incluido, guia paso a paso, garantia.
- **Companion Computer**: Orange Pi Zero 2W 2GB es la opcion mas barata con buena compatibilidad ROS 2. RPi 4 2GB si se necesita maxima compatibilidad y documentacion.

---

## Sobre el requisito "NO SOLDAR"

### Que componentes normalmente requieren soldadura
1. **PDB del F450**: Los ESCs se sueldan directamente a la placa PCB del frame
2. **Power Module**: Algunos requieren soldar cables XT60
3. **UBEC**: Algunos vienen con cables pelados

### Soluciones sin soldadura
1. **Frame pre-soldado**: Comprar HAWK'S WORK Pre-soldered (los ESCs ya vienen soldados a la PDB)
2. **Power Module**: El que viene con Pixhawk ya tiene conectores (no requiere soldadura)
3. **UBEC**: Los FPVKing vienen con conectores listos, no requieren soldadura
4. **Motores a ESCs**: Los ESCs con bullet connectors 3.5mm se conectan directamente a los cables del motor (plug-and-play)
5. **Cable UART**: El adaptador Landmark Landing es plug-and-play (header macho para RPi GPIO + JST-GH para Pixhawk)
6. **Bateria**: Usar baterias con conector XT60 que conectan directamente al Power Module

### Lo que NO se puede evitar
- Conectar cables dupont a los pines GPIO de la RPi (pero es plug-in, no soldadura)
- Atornillar motores al frame
- Conectar cables de senal del receptor al Pixhawk RC IN

---

## Detalles de conexion UART Pixhawk TELEM2 a Raspberry Pi

```
Pixhawk TELEM2 (JST-GH 6-pin)     Raspberry Pi GPIO
Pin 1 (Red)   VCC +5V     ──>     NO CONECTAR (RPi se alimenta por UBEC)
Pin 2 (Black) UART5_TX    ──>     Pin 10 (GPIO15 RXD)
Pin 3 (Black) UART5_RX    <──     Pin 8  (GPIO14 TXD)
Pin 4 (Black) CTS         ──>     NO CONECTAR
Pin 5 (Black) RTS         ──>     NO CONECTAR
Pin 6 (Black) GND         ──>     Pin 6  (GND)
```

> Nota: TX y RX van cruzados. El Pixhawk TELEM2 opera a 3.3V, compatible directamente con los GPIO de la RPi (tambien 3.3V).
> Se recomienda el adaptador [Landmark Landing](https://landmarklanding.com/products/raspberry-pi-to-pixhawk-telemetry-adapter) ($8.99) que simplifica esta conexion. Necesita ademas un cable JST-GH 6-pin (incluido con muchos Pixhawks).

---

## Conexiones principales (para cuando llegue el hardware)

```
Bateria LiPo 3S ──> Power Module ──> Pixhawk (alimentacion)
                ──> PDB F450 ──> 4x ESC ──> 4x Motor
                ──> UBEC 5V ──> Raspberry Pi 4 / Orange Pi (GPIO 5V+GND)

Pixhawk TELEM2 ──> RPi GPIO UART (TX/RX cruzados, 3.3V)
Pixhawk RC IN ──> Receptor FlySky FS-iA6B (PPM/iBUS)
Pixhawk GPS ──> Modulo M8N GPS
```

## Checklist antes de primer vuelo

- [ ] Frame montado y motores fijados (verificar giro CW/CCW)
- [ ] ESCs conectados a PDB y a motores (3 cables, orden importa para giro)
- [ ] Pixhawk montado con amortiguadores de vibracion
- [ ] GPS montado en mastil elevado (lejos de interferencias)
- [ ] Receptor RC vinculado con emisora FlySky
- [ ] Firmware PX4 flasheado en Pixhawk (via QGroundControl)
- [ ] Calibracion: acelerometro, giroscopo, magnetometro, radio, ESCs
- [ ] RPi conectada por UART a TELEM2
- [ ] UBEC alimentando RPi (verificar 5V estables)
- [ ] Software WarDrone desplegado en RPi (docker o nativo)
- [ ] Test en banco: armar motores SIN helices, verificar telemetria
- [ ] Primer vuelo manual con RC (sin RPi, solo Pixhawk)
- [ ] Segundo vuelo con RPi conectada (modo telemetria pasivo)
- [ ] Tercer vuelo: test autonomo (waypoints simples)
