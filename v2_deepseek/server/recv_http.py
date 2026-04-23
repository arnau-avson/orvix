#!/usr/bin/env python3
# recv_http.py - Recibe POST en /upload y reenvía a servidor externo HTTPS.
# Uso: python recv_http.py [puerto] [url_destino]

import requests
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import sys

DEFAULT_PORT = 8080
DEFAULT_TARGET = "https://intellapi.avson.eu/screen"  # Cambia aquí tu destino

class ForwardHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            parsed = urlparse(self.path)
            params = parse_qs(parsed.query)

            # Construir URL destino con los mismos parámetros
            query_string = "&".join(f"{k}={v[0]}" for k, v in params.items())
            target_url = self.server.target_url
            if query_string:
                target_url += ("?" if "?" not in target_url else "&") + query_string

            # Reenviar la imagen
            resp = requests.post(target_url, data=body, headers={"Content-Type": "image/png"})
            self.send_response(resp.status_code)
            self.end_headers()
            self.wfile.write(b"OK")
            print(f"Reenviado a {target_url} -> {resp.status_code}")
        except Exception as e:
            self.send_response(500)
            self.end_headers()
            print(f"Error: {e}")

    def log_message(self, format, *args):
        pass  # Silencioso

if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT
    target = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_TARGET
    server = HTTPServer(('0.0.0.0', port), ForwardHandler)
    server.target_url = target
    print(f"Proxy escuchando en http://0.0.0.0:{port}/upload")
    print(f"Reenviando a {target}")
    server.serve_forever()