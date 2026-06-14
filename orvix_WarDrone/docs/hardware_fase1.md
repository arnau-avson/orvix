# Hardware Fase 1 -- WarDrone Prototipo de Prueba

> Vuelo basico, telemetria, navegacion GPS, safety monitor (milestones S1-S2)

---

## Opcion A: Componentes por separado (mas barato)

| # | Componente | Modelo | Enlace | Precio aprox. |
|---|-----------|--------|--------|---------------|
| 1 | **Frame** | F450 con PDB integrada y tren de aterrizaje | [Amazon.es - EVTSCAN F450](https://www.amazon.es/EVTSCAN-Quadcopter-Aircraft-Accesorio-integrada/dp/B093DJX8SR) | ~18 EUR |
| 2 | **Motores x4 + ESCs x4** | 2212 920KV + SimonK 30A (pack de 4) | [Amazon - Hobbypower Motor+ESC kit](https://www.amazon.com/Hobbypower-920KV-Brushless-SimonK-Quadcopter/dp/B07LG44BJ4) | ~45 EUR |
| 3 | **Helices** | 1045 CW/CCW (minimo 2 sets de repuesto) | [Amazon - QWinOut 1045 (4 pares)](https://www.amazon.com/QWinOut-Propeller-Compatible-Multi-Copter-Quadcopter/dp/B088FH9ZPB) | ~8 EUR |
| 4 | **Flight Controller** | Pixhawk PX4 2.4.8 (con safety switch, buzzer, SD) | [Amazon.es - LiteBee Pixhawk 2.4.8](https://www.amazon.es/Controlador-Controller-integrado-Quadcopter-LITEBEE/dp/B072FKFX3J) | ~35 EUR |
| 5 | **GPS** | M8N con brujula, compatible Pixhawk | [Amazon - HAWK'S WORK M8N GPS](https://www.amazon.com/-/es/M%C3%B3dulo-soporte-precisi%C3%B3n-posicionamiento-Multirotor/dp/B09YC5X8SF) | ~18 EUR |
| 6 | **Companion Computer** | Raspberry Pi 4 Model B 2GB | [Amazon.es - RPi 4 2GB](https://www.amazon.es/Raspberry-ARM-Cortex-A72-WLAN-ac-Bluetooth-Micro-HDMI/dp/B07TD42S27) | ~55 EUR |
| 7 | **MicroSD** | 32GB clase 10 (para RPi) | Amazon.es - cualquier marca | ~8 EUR |
| 8 | **Bateria** | LiPo 3S 11.1V 2200mAh 35C, conector XT60 | [Amazon.es - Dilwe 3S 2200mAh](https://www.amazon.es/Dilwe-Bater%C3%ADa-2200mAh-Capacidad-teledirigidos/dp/B0BNBYQ4JZ) | ~20 EUR |
| 9 | **Cargador LiPo** | IMAX B6 + adaptador de corriente 12V | [Amazon.es - SATKIT IMAX B6 + adaptador](https://www.amazon.es/SATKIT-Cargador-descargador-Adaptador-Corriente/dp/B06Y44KXYS) | ~25 EUR |
| 10 | **Radio RC** | FlySky FS-i6 6CH + receptor FS-iA6B | [Amazon - FlySky FS-i6 + iA6](https://www.amazon.com/-/es/Transmisor-Controlador-Receptor-Helic%C3%B3ptero-Quadcopter/dp/B07CWBQ2HM) | ~45 EUR |
| 11 | **BEC 5V** | UBEC 5V 3A (alimentar RPi desde bateria LiPo) | [Amazon - UBEC 5V 3A (3 uds)](https://www.amazon.com/Regulator-Quadcopter-Airplane-Robotics-Raspberry/dp/B0CB4M3DYZ) | ~8 EUR |
| 12 | **Cable UART** | Pixhawk TELEM2 a RPi GPIO (serial TTL) | Amazon - cable dupont hembra 6 pin | ~3 EUR |
| 13 | **Varios** | Cables, zip ties, conectores XT60, velcro, cinta | Amazon/ferreteria | ~10 EUR |
| | | | **TOTAL OPCION A** | **~298 EUR** |

### Notas Opcion A
- El pack Hobbypower (item 2) incluye 4 motores + 4 ESCs, es la forma mas barata
- Los ESCs vienen con BEC integrado de 5V/2A (suficiente para el Pixhawk, pero usar UBEC aparte para la RPi)
- Verificar que el conector de la bateria sea XT60 (el mas comun) o comprar adaptadores
- El cargador SATKIT incluye el adaptador de corriente; muchos otros IMAX B6 NO lo incluyen
- Comprar al menos 2 juegos de helices; se rompen facilmente

---

## Opcion B: Kit completo HAWK'S WORK (mas facil, menos soldar)

| Kit | Incluye | Enlace | Precio aprox. |
|-----|---------|--------|---------------|
| **F450 Kit B** | Frame + Pixhawk 2.4.8 + GPS M8N + Power Module + 4x ESC 20A + 4x Motor 2212 + Helices + Bateria 4200mAh + Guia + Video | [Amazon - HAWK'S WORK Kit B](https://www.amazon.com/HAWKS-WORK-Controller-Unassembled-Quadcopter/dp/B09SZ74YFK) | ~346 USD (~320 EUR) |
| **F450 Kit A** | Todo lo del Kit B + FlySky FS-i6X + receptor FS-iA6B | [Amazon - HAWK'S WORK Kit A](https://www.amazon.com/HAWKS-WORK-Controller-Unassembled-Quadcopter/dp/B09SZ7LNXB) | ~396 USD (~365 EUR) |

### Lo que falta comprar con el kit

| Componente | Enlace | Precio aprox. |
|-----------|--------|---------------|
| Raspberry Pi 4 2GB | [Amazon.es - RPi 4 2GB](https://www.amazon.es/Raspberry-ARM-Cortex-A72-WLAN-ac-Bluetooth-Micro-HDMI/dp/B07TD42S27) | ~55 EUR |
| MicroSD 32GB | Amazon.es | ~8 EUR |
| UBEC 5V 3A | [Amazon - UBEC 5V 3A](https://www.amazon.com/Regulator-Quadcopter-Airplane-Robotics-Raspberry/dp/B0CB4M3DYZ) | ~8 EUR |
| Cable UART | Amazon | ~3 EUR |
| | **TOTAL complementos** | **~74 EUR** |

**Total Opcion B (Kit A + complementos): ~439 EUR**
**Total Opcion B (Kit B + FlySky aparte + complementos): ~440 EUR**

### Ventajas del kit
- PCB pre-soldada (no necesitas soldador)
- Guia de montaje + videos
- Garantia de 3 meses en piezas
- Bateria de 4200mAh (mas autonomia que la de 2200mAh)
- Componentes verificados como compatibles entre si

---

## Alternativa AliExpress (mas barato, envio lento)

| Componente | Enlace | Precio aprox. |
|-----------|--------|---------------|
| Kit F450 completo (Pixhawk + GPS + ESC + Motor + Frame) | [AliExpress - F450 Pixhawk Kit](https://www.aliexpress.com/item/1005008659371100.html) | ~90-130 EUR |
| FlySky FS-i6 | Buscar "FlySky FS-i6" en AliExpress | ~35 EUR |
| Bateria 3S 2200mAh | Buscar "3S 2200mAh XT60" en AliExpress | ~12 EUR |
| Cargador IMAX B6 | Buscar "IMAX B6 charger" en AliExpress | ~15 EUR |
| Raspberry Pi 4 2GB | [Amazon.es](https://www.amazon.es/Raspberry-ARM-Cortex-A72-WLAN-ac-Bluetooth-Micro-HDMI/dp/B07TD42S27) (comprar en Amazon, mas seguro) | ~55 EUR |
| UBEC + cables + microSD | AliExpress/Amazon | ~15 EUR |
| | **TOTAL AliExpress** | **~230-260 EUR** |

> **Aviso**: AliExpress tarda 15-30 dias en llegar. La calidad es variable. La RPi mejor comprarla en Amazon/tienda local.

---

## Resumen comparativo

| Opcion | Precio total | Tiempo envio | Dificultad montaje |
|--------|-------------|--------------|-------------------|
| **A: Por separado (Amazon)** | ~298 EUR | 2-5 dias | Media (requiere soldar) |
| **B: Kit HAWK'S WORK (Amazon)** | ~439 EUR | 5-10 dias | Baja (pre-soldado) |
| **C: AliExpress** | ~230-260 EUR | 15-30 dias | Media-Alta |

---

## Conexiones principales (para cuando llegue el hardware)

```
Bateria LiPo 3S ──> Power Module ──> Pixhawk (alimentacion)
                ──> PDB F450 ──> 4x ESC ──> 4x Motor
                ──> UBEC 5V ──> Raspberry Pi 4 (GPIO 5V+GND)

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
