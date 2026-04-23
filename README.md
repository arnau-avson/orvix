# Lens

App Android (Java) que captura los cambios de texto **dentro de su propio
EditText** y los envia a un backend HTTP local. El backend los acumula en un
CSV con timestamp, accion (insert/delete) y el delta de texto.

Caso de uso legitimo: investigar patrones de tecleo del propio usuario en una
app que el usuario instala conscientemente. La app **solo** observa lo que se
escribe en su propia interfaz; no toca nada del resto del sistema, no requiere
permiso de Accesibilidad, no requiere root.

> *Renombrado desde el codigo anterior. La carpeta del proyecto sigue
> llamandose `pegasus/` por inercia; renombrala a `lens/` cuando quieras (cierra
> el IDE, mueve la carpeta, reabre).*

## Estructura

```
pegasus/
├── android/                          ← proyecto Android Studio (Gradle, Java)
│   ├── app/
│   │   ├── src/main/AndroidManifest.xml
│   │   ├── src/main/java/com/example/lens/MainActivity.java
│   │   ├── src/main/java/com/example/lens/BackendClient.java
│   │   ├── src/main/res/layout/activity_main.xml
│   │   ├── src/main/res/values/strings.xml
│   │   ├── src/main/res/values/themes.xml
│   │   └── build.gradle
│   ├── build.gradle
│   ├── settings.gradle
│   └── gradle.properties
│
├── server/                           ← backend HTTP en Python (stdlib)
│   ├── recv_http.py
│   └── keystrokes.csv               (se crea al recibir el primer evento)
│
├── start.ps1                         ← lanza todo: emulador + backend + APK
├── stop.ps1                          ← apaga el emulador
├── setup_oppo_avd.ps1                ← crea AVD del OPPO A5 Pro 5G n92u
└── README.md
```

## Setup inicial (una sola vez)

1. **Abre el proyecto Android en Android Studio:**
   `File → Open` → selecciona `c:\Users\arnau\Desktop\pegasus\android`
2. Deja que Gradle sincronice. La 1a vez descarga ~500 MB (Gradle + Android
   Gradle Plugin + dependencias). Esto crea el `gradle/wrapper/gradle-wrapper.jar`
   que necesita `start.ps1` para construir desde linea de comandos.
3. Cierra Android Studio (no hace falta tenerlo abierto despues).

## Lanzar todo de una vez

```powershell
cd C:\Users\arnau\Desktop\pegasus
.\start.ps1
```

Hace en orden:

1. Crea el AVD del OPPO si no existe (`setup_oppo_avd.ps1`).
2. **Ventana 1**: arranca el emulador.
3. Espera al boot (hasta 8 min la 1a vez con AVD nuevo).
4. **Ventana 2**: arranca el backend HTTP en `0.0.0.0:8080`.
5. Configura `adb reverse tcp:8080 tcp:8080` (la app vera el backend en
   `127.0.0.1:8080`, sin importar el estado de la wifi virtual).
6. Compila el APK con `gradlew assembleDebug` (rapido salvo la 1a vez).
7. Instala el APK con `adb install -r`.
8. Lanza la actividad con `adb shell am start`.

A partir de ahi, todo lo que escribas en el EditText de la app se envia al
backend y se anade como fila al CSV.

## Parar

```powershell
cd C:\Users\arnau\Desktop\pegasus
.\stop.ps1
```

Apaga el emulador. La ventana del backend la cierras a mano.

## Que envia la app exactamente

Cada vez que el `TextWatcher.afterTextChanged` se dispara (es decir, cada
cambio del contenido del EditText: pulsar tecla, borrar, autocompletar,
pegar...), `BackendClient.sendKeystroke` hace un POST JSON a `/keys` con:

```json
{
  "ts": 1745408201123,
  "before": "hol",
  "after": "hola"
}
```

El backend calcula el diff (insert / delete / replace + delta + texto del
cambio) y anade una fila al CSV:

| Columna           | Ejemplo                          |
|-------------------|----------------------------------|
| timestamp_iso     | `2026-04-23T13:45:01.123+00:00`  |
| timestamp_ms      | `1745408201123`                  |
| action            | `insert` / `delete` / `replace`  |
| delta_len         | `+1` / `-1` / `0`                |
| change            | `a`                              |
| after_text        | `hola`                           |

Inspeccionar el CSV en directo desde el navegador del PC:
- Status: <http://localhost:8080/>
- Descargar CSV: <http://localhost:8080/keys.csv>

## Por que esta arquitectura es legitima

- La app es **tuya**: el usuario la instala, la abre, ve un campo de texto con
  un mensaje claro de que lo que se escribe se envia al backend.
- Solo observa el `EditText` de su propia actividad — no usa
  `AccessibilityService`, no lee `/dev/input/event*`, no requiere root.
- Patron equivalente al de cualquier app de notas, formulario web con
  validacion en vivo, o test A/B de UX que mide tiempo de respuesta.

## Cambiar el nombre de "Lens" a otro

Si quieres llamarlo distinto (Aurora, Beacon, Helios...), edita estos sitios:

| Archivo                                                | Cambia                                |
|--------------------------------------------------------|---------------------------------------|
| `android/app/src/main/res/values/strings.xml`          | `app_name`, `title`                   |
| `android/app/src/main/res/values/themes.xml`           | `Theme.Lens` -> `Theme.NuevoNombre`   |
| `android/app/src/main/AndroidManifest.xml`             | referencia a `@style/Theme.Lens`      |
| `android/settings.gradle`                              | `rootProject.name = "Lens"`           |

(El nombre del paquete `com.example.lens` puedes dejarlo o renombrarlo via
Android Studio: clic derecho en el paquete -> Refactor -> Rename.)

## Notas

### El primer boot del AVD es lento

Un AVD recien creado tarda 3-8 min en su primer boot frio (debe inicializar
`userdata.img`, ejecutar `init.rc`, dexopt). Despues va mucho mas rapido.

### `adb reverse` y por que `127.0.0.1:8080` desde el movil

El emulador moderno usa `virtio_wifi`. El alias clasico `10.0.2.2` puede no ser
enrutable en condiciones especificas. `adb reverse tcp:X tcp:Y` tuneliza via la
propia conexion adb, asi que la app conecta a `127.0.0.1:8080` dentro del
emulador y termina en el puerto 8080 de tu PC. Es el metodo recomendado por
Google para desarrollo cliente-servidor en local.

### Pantalla negra al arrancar el emulador

API 35+ tiene un bug del renderer ANGLE. `start.ps1` ya lo evita lanzando con
`-gpu swiftshader_indirect`.
