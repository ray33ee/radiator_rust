#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer
from datetime import datetime, timezone
import json

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/api/ip":
            now = datetime.now(timezone.utc).astimezone()
            payload = {
                "datetime": now.isoformat(),
                "timezone": str(now.tzinfo),
                "unixtime": int(now.timestamp())
            }
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            self.wfile.write(json.dumps(payload).encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()

def run(port=8080):
    with HTTPServer(("", port), Handler) as httpd:
        print(f"Mock WorldTimeAPI running at http://localhost:{port}/api/ip")
        httpd.serve_forever()

if __name__ == "__main__":
    run()