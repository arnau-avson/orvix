#!/system/bin/sh
# uninstall_root.sh - Elimina lens del sistema (requiere root)

if [ "$(id -u)" != "0" ]; then
    echo "Ejecuta como root: su -c sh $0"
    exit 1
fi

mount -o rw,remount /system
rm -f /system/bin/lens
rm -f /data/adb/service.d/99_lens.sh
rm -f /data/local/tmp/lens.pid
mount -o ro,remount /system
echo "[+] Lens desinstalado."