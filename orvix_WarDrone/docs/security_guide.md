# Guía de Seguridad WarDrone

## 1. ROS 2 DDS Security (SROS2)

SROS2 proporciona cifrado y autenticación a nivel de transporte para todos los topics, servicios y acciones de ROS 2.

### Generar keystore y certificados

```bash
# Crear keystore
export KEYSTORE_DIR="${HOME}/wardrone_keystore"
ros2 security create_keystore ${KEYSTORE_DIR}

# Crear enclaves para cada nodo
for node in mavsdk_bridge waypoint_navigator safety_monitor \
            mission_controller obstacle_detector obstacle_avoidance \
            flight_logger wind_estimator camera detector tracker \
            vio_bridge; do
    ros2 security create_enclave ${KEYSTORE_DIR} /wardrone/${node}
done
```

### Activar seguridad

Añadir al `.bashrc` o al launch file:

```bash
export ROS_SECURITY_KEYSTORE=${HOME}/wardrone_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce  # o "Permissive" para testing
```

### Políticas de acceso

Configurar `governance.xml` y `permissions.xml` en el keystore para restringir qué nodos pueden publicar/suscribir en cada topic. Ejemplo:

- Solo `mavsdk_bridge` puede publicar en `/wardrone/telemetry`
- Solo `mission_controller` y `obstacle_avoidance` pueden publicar en `/wardrone/cmd_velocity`
- Solo `waypoint_navigator` puede publicar en `/wardrone/cmd_goto_global`

## 2. MAVLink 2 Message Signing

PX4 soporta firma de mensajes MAVLink 2 para autenticar la comunicación entre el companion computer y el Pixhawk.

### Configurar en PX4

```
MAV_SIGNING_KEY = <clave de 32 bytes en hex>
MAV_SIGNING_ENABLE = 1
```

### Configurar en MAVSDK

MAVSDK soporta signing nativamente al conectar. Consultar la documentación de MAVSDK para habilitar `connection_with_signing()`.

## 3. Seguridad WiFi

- Usar WPA3 o WPA2-Enterprise para la red WiFi del dron
- Cambiar contraseña por defecto del hotspot
- Usar una red WiFi dedicada (no compartida)
- Considerar 4G/LTE como backup para mayor rango

## 4. HMAC para Topics Críticos (Demostrativo)

El módulo `auth_utils.py` proporciona funciones HMAC-SHA256 para firmar y verificar mensajes individuales. Es una solución académica; SROS2 es la solución de producción.

```python
from wardrone_navigation.auth_utils import sign_message, verify_message

# Firmar
signed = sign_message(b"arm", key=b"shared_secret")

# Verificar
valid, data = verify_message(signed, key=b"shared_secret")
```
