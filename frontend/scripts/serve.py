#!/usr/bin/env python3
import os
import sys
from http import server

DIRECTORY = os.path.join(os.path.dirname(__file__), "../")
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080


class NoCacheHTTPRequestHandler(server.SimpleHTTPRequestHandler):
    def __init__(self, *args, directory=None, **kwargs):
        if directory is None:
            directory = DIRECTORY
        super().__init__(*args, directory=directory, **kwargs)

    def end_headers(self):
        self.send_no_cache_headers()
        super().end_headers()

    def send_no_cache_headers(self):
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")


def serve():
    print(f"Serving HTTP on 0.0.0.0 port {PORT} (http://localhost:{PORT}/) ...")
    server_address = ('', PORT)
    httpd = server.HTTPServer(server_address, NoCacheHTTPRequestHandler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, exiting.")
        sys.exit(0)


if __name__ == '__main__':
    serve()
