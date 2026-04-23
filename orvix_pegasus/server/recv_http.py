#!/usr/bin/env python3
"""
Backend HTTP minimalista para Lens (binario nativo mobile/lens).

Endpoints:
  POST /upload   image/png (+?x=&y=&app=)  -> guarda en screenshot/shot_*.png
  GET  /                                    -> status en texto plano

Sin dependencias externas, solo stdlib (Pillow opcional para el overlay).

Uso:
    python recv_http.py                # bind 0.0.0.0:8080, datos aqui
    python recv_http.py 8080 ./datos   # otro puerto y dir de salida
"""
from __future__ import annotations

import re
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from pathlib import Path
from urllib.parse import urlparse, parse_qs

try:
    from PIL import Image, ImageDraw
    PIL_OK = True
except ImportError:
    PIL_OK = False

OUTDIR = Path(".")
SHOTS_SUBDIR = "screenshot"
SHOT_LOCK = threading.Lock()
SHOT_COUNT = 0

# El touchscreen virtio_input del emulador ranchu reporta coords en rango
# 0..32767, no en pixeles. Hay que escalar por el tamano de la imagen.
RAW_TOUCH_MAX = 32767


def shots_dir() -> Path:
    d = OUTDIR / SHOTS_SUBDIR
    d.mkdir(parents=True, exist_ok=True)
    return d


def draw_tap_overlay(png_blob: bytes, raw_x: int, raw_y: int) -> tuple[bytes, int, int]:
    """Dibuja un circulo rojo translucido en el tap. Recibe coords RAW del
    touchscreen y devuelve (png_modificado, px_x, px_y)."""
    if not PIL_OK:
        print("[recv] WARN: Pillow no instalado, no dibujo el punto. pip install Pillow")
        return png_blob, -1, -1
    try:
        img = Image.open(BytesIO(png_blob)).convert("RGBA")
        w, h = img.size
        x = int(raw_x * w / RAW_TOUCH_MAX)
        y = int(raw_y * h / RAW_TOUCH_MAX)
        overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(overlay)
        r = max(40, int(min(img.size) * 0.025))   # ~2.5% del lado menor
        d.ellipse([x - r, y - r, x + r, y + r],
                  fill=(255, 0, 0, 110),
                  outline=(255, 0, 0, 255),
                  width=6)
        img = Image.alpha_composite(img, overlay)
        out = BytesIO()
        img.convert("RGB").save(out, format="PNG")
        return out.getvalue(), x, y
    except Exception as e:
        print(f"[recv] WARN: overlay fallo ({e}), guardando original")
        return png_blob, -1, -1


_SAFE_APP_RE = re.compile(r"[^a-zA-Z0-9._-]+")


def save_screenshot(blob: bytes, tap_x: int = -1, tap_y: int = -1,
                    app: str = "") -> Path:
    """Guarda PNG en screenshot/. Si llega tap (raw), dibuja circulo rojo.
    Si llega app (package), lo anade al nombre del fichero."""
    global SHOT_COUNT
    import time as _t
    is_tap = tap_x >= 0 and tap_y >= 0
    px_x = px_y = -1
    if is_tap:
        blob, px_x, px_y = draw_tap_overlay(blob, tap_x, tap_y)
    with SHOT_LOCK:
        n = SHOT_COUNT
        SHOT_COUNT += 1
    ext = ".png" if blob.startswith(b"\x89PNG") else ".bin"
    parts = [f"shot_{int(_t.time())}_{n:05d}"]
    if app:
        parts.append(_SAFE_APP_RE.sub("_", app))
    if is_tap and px_x >= 0:
        parts.append(f"tap_{px_x}_{px_y}")
    fname = shots_dir() / ("_".join(parts) + ext)
    fname.write_bytes(blob)
    return fname


class Handler(BaseHTTPRequestHandler):

    def _send(self, code: int, body: bytes = b"", ctype: str = "text/plain; charset=utf-8") -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if body:
            self.wfile.write(body)

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        parsed = urlparse(self.path)

        if parsed.path.startswith("/upload"):
            qs = parse_qs(parsed.query)
            tap_x = int(qs["x"][0]) if "x" in qs else -1
            tap_y = int(qs["y"][0]) if "y" in qs else -1
            app = qs.get("app", [""])[0]
            fname = save_screenshot(raw, tap_x, tap_y, app)
            tag = f" TAP({tap_x},{tap_y})" if tap_x >= 0 else ""
            app_tag = f" [{app}]" if app else ""
            print(f"[recv] {self.client_address[0]} POST /upload{tag}{app_tag} "
                  f"({len(raw)//1024} KiB) -> {fname.name}  total={SHOT_COUNT}")
            self._send(204)
            return

        self._send(404, b"unknown endpoint")

    def do_GET(self) -> None:
        if self.path in ("/", "/status"):
            msg = (f"Lens backend\n"
                   f"shots received: {SHOT_COUNT}\n"
                   f"shots: {shots_dir().resolve()}\n")
            self._send(200, msg.encode())
            return
        self._send(404, b"unknown endpoint")

    def log_message(self, fmt: str, *args: object) -> None:
        return  # silenciamos el log automatico; ya logueamos en do_POST


def main(argv: list[str]) -> int:
    global OUTDIR
    port = int(argv[1]) if len(argv) > 1 else 8080
    if len(argv) > 2:
        OUTDIR = Path(argv[2])
    OUTDIR.mkdir(parents=True, exist_ok=True)

    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"[recv] Lens backend listening on http://0.0.0.0:{port}/")
    print(f"[recv] shots -> {shots_dir().resolve()}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[recv] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
