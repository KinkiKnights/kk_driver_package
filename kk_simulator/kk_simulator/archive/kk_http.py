import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

class SimpleHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/hello":
            self.send_response(200)
            self.send_header("Content-type", "text/html; charset=utf-8")
            self.end_headers()
            html = """
            <!DOCTYPE html>
            <html>
            <head><title>Hello</title></head>
            <body>
                <h1>Hello, World!</h1>
                <p>This is a response from your class-based HTTP server.</p>
            </body>
            </html>
            """
            self.wfile.write(html.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"404 Not Found")


class Server:
    def __init__(self, host='0.0.0.0', port=8000):
        self.server_address = (host, port)
        self.httpd = HTTPServer(self.server_address, SimpleHandler)
        thread = threading.Thread(target=self.start)
        thread.start()

    def start(self):
        print(f"Starting server at http://{self.server_address[0]}:{self.server_address[1]}/")
        try:
            self.httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server.")
            self.httpd.server_close()


if __name__ == "__main__":
    server = Server(port=8000)