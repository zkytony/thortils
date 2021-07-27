# Serves ai2thor documentation website
# Attach .html to requests that don't have that extension

import http.server
import webbrowser
import threading
import socketserver
import os

ITHOR_DOCS = "ai2thor.allenai.org"  # Path to the directory that contains 'ithor'
os.chdir(ITHOR_DOCS)  # Set the root directory of the server
FILE = "index.html"
PORT = 8000

class RequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        """If self.path doesn't contain .html, then append it to the end"""
        if not self.path.endswith(".html"):
            self.path += ".html"
        return http.server.SimpleHTTPRequestHandler.do_GET(self)

def open_browser():
    """Start a browser after waiting for half a second."""
    def _open_browser():
        webbrowser.open('http://localhost:%s/%s' % (PORT, FILE))
    thread = threading.Timer(0.5, _open_browser)
    thread.start()

def start_server():
    """Start the server."""
    server_address = ("localhost", PORT)
    server = http.server.HTTPServer(server_address, RequestHandler)
    server.serve_forever()

if __name__ == "__main__":
    open_browser()
    start_server()
