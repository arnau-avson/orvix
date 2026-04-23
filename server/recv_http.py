#!/usr/bin/env python3
"""
Backend HTTP minimalista para Lens (app Java + binario nativo).

Endpoints:
  POST /keys     JSON {ts, before, after}  -> append a keystrokes.csv
  POST /upload   image/png                 -> guarda en screenshot/shot_*.png
  GET  /                                   -> status en texto plano
  GET  /keys.csv                           -> descarga el CSV completo

Sin dependencias externas, solo stdlib.

Uso:
    python recv_http.py                # bind 0.0.0.0:8080, datos aqui
    python recv_http.py 8080 ./datos   # otro puerto y dir de salida
"""
from __future__ import annotations

import csv
import json
import sys
import threading
from datetime import datetime, timezone
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
CSV_NAME = "keystrokes.csv"
SHOTS_SUBDIR = "screenshot"
CSV_LOCK = threading.Lock()
SHOT_LOCK = threading.Lock()
KEY_COUNT = 0
SHOT_COUNT = 0


def csv_path() -> Path:
    return OUTDIR / CSV_NAME


def shots_dir() -> Path:
    d = OUTDIR / SHOTS_SUBDIR
    d.mkdir(parents=True, exist_ok=True)
    return d


def draw_tap_overlay(png_blob: bytes, x: int, y: int) -> bytes:
    """Dibuja un circulo rojo translucido en (x,y) sobre el PNG."""
    if not PIL_OK:
        print("[recv] WARN: Pillow no instalado, no dibujo el punto. pip install Pillow")
        return png_blob
    try:
        img = Image.open(BytesIO(png_blob)).convert("RGBA")
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
        return out.getvalue()
    except Exception as e:
        print(f"[recv] WARN: overlay fallo ({e}), guardando original")
        return png_blob


def save_screenshot(blob: bytes, tap_x: int = -1, tap_y: int = -1) -> Path:
    """Guarda PNG en screenshot/. Si llega tap, dibuja circulo rojo en (x,y)."""
    global SHOT_COUNT
    import time as _t
    is_tap = tap_x >= 0 and tap_y >= 0
    if is_tap:
        blob = draw_tap_overlay(blob, tap_x, tap_y)
    with SHOT_LOCK:
        n = SHOT_COUNT
        SHOT_COUNT += 1
    ext = ".png" if blob.startswith(b"\x89PNG") else ".bin"
    suffix = f"_tap_{tap_x}_{tap_y}" if is_tap else ""
    fname = shots_dir() / f"shot_{int(_t.time())}_{n:05d}{suffix}{ext}"
    fname.write_bytes(blob)
    return fname


def diff_text(before: str, after: str) -> tuple[str, int, str]:
    """Devuelve (accion, delta_len, texto_insertado_o_borrado)."""
    delta = len(after) - len(before)
    i = 0
    while i < min(len(before), len(after)) and before[i] == after[i]:
        i += 1
    if delta > 0:
        return ("insert", delta, after[i:i + delta])
    if delta < 0:
        return ("delete", delta, before[i:i + (-delta)])
    return ("replace", 0, after[i:i + 1] if i < len(after) else "")


def append_keystroke(payload: dict) -> tuple[str, int]:
    ts_ms = int(payload.get("ts", 0))
    before = str(payload.get("before", ""))
    after = str(payload.get("after", ""))
    iso = datetime.fromtimestamp(ts_ms / 1000, tz=timezone.utc).isoformat()
    action, delta_len, change = diff_text(before, after)

    with CSV_LOCK:
        path = csv_path()
        is_new = not path.exists()
        with path.open("a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if is_new:
                w.writerow(["timestamp_iso", "timestamp_ms", "action",
                            "delta_len", "change", "after_text"])
            w.writerow([iso, ts_ms, action, delta_len, change, after])
    return action, delta_len


class Handler(BaseHTTPRequestHandler):

    def _send(self, code: int, body: bytes = b"", ctype: str = "text/plain; charset=utf-8") -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if body:
            self.wfile.write(body)

    def do_POST(self) -> None:
        global KEY_COUNT
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        parsed = urlparse(self.path)

        if parsed.path.startswith("/keys"):
            try:
                payload = json.loads(raw.decode("utf-8"))
            except Exception as e:
                self._send(400, f"bad json: {e}".encode())
                return
            action, delta = append_keystroke(payload)
            KEY_COUNT += 1
            print(f"[recv] {self.client_address[0]} POST /keys   "
                  f"{action:<7} delta={delta:+d}  "
                  f"after_len={len(str(payload.get('after','')))}  total={KEY_COUNT}")
            self._send(204)
            return

        if parsed.path.startswith("/upload"):
            qs = parse_qs(parsed.query)
            tap_x = int(qs["x"][0]) if "x" in qs else -1
            tap_y = int(qs["y"][0]) if "y" in qs else -1
            fname = save_screenshot(raw, tap_x, tap_y)
            tag = f" TAP({tap_x},{tap_y})" if tap_x >= 0 else ""
            print(f"[recv] {self.client_address[0]} POST /upload{tag} "
                  f"({len(raw)//1024} KiB) -> {fname.name}  total={SHOT_COUNT}")
            self._send(204)
            return

        self._send(404, b"unknown endpoint")

    def do_GET(self) -> None:
        if self.path in ("/", "/status"):
            csv_size = csv_path().stat().st_size if csv_path().exists() else 0
            msg = (f"Lens backend\n"
                   f"keys received:  {KEY_COUNT}\n"
                   f"shots received: {SHOT_COUNT}\n"
                   f"csv:   {csv_path().resolve()} ({csv_size} bytes)\n"
                   f"shots: {shots_dir().resolve()}\n")
            self._send(200, msg.encode())
            return
        if self.path == "/keys.csv":
            if not csv_path().exists():
                self._send(404, b"csv not yet created")
                return
            data = csv_path().read_bytes()
            self._send(200, data, "text/csv; charset=utf-8")
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
    print(f"[recv] csv -> {csv_path().resolve()}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[recv] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))