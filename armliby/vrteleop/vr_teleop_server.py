import os
from multiprocessing import Process

from armliby.urdf_parser import URDFParser
from flask import Flask, jsonify, render_template, send_from_directory


class VRTeleopServer:
    def __init__(
            self,
            host: str,
            app_port: int,
            wss_port: int,
            urdf_path: str,
            ssl_cert: str,
            ssl_key: str
            ):
        """
        Initialize the VR teleop server.

        Args:
            host: The host address of the server.
            app_port: The port number of the Flask app.
            wss_port: The port number of the websocket server.
            urdf_path: The path to the URDF file.
            ssl_cert: The path to the SSL certificate file.
                Create using openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes
            ssl_key: The path to the SSL key file.
        """
        self._host = host
        self._wss_port = wss_port
        self._app_port = app_port
        self._urdf_path = urdf_path
        self._ssl_cert = ssl_cert
        self._ssl_key = ssl_key

    def _setup_routes(self):
        @self._app.route('/')
        def home():
            return render_template('index.html', wss_host=self._host, wss_port=self._wss_port)

        @self._app.route('/stl/<link_name>.stl')
        def serve_stl(link_name):
            stl_mapping = self._urdf_parser.get_link_stl_map()
            stl_file_path = stl_mapping.get(link_name)
            if stl_file_path and os.path.exists(stl_file_path):
                return send_from_directory(os.path.dirname(stl_file_path), os.path.basename(stl_file_path))
            else:
                return f"STL file not found for the link name: {link_name}", 404

        @self._app.route('/bodies')
        def bodies():
            links = self._urdf_parser.get_links()
            return jsonify(links)

        @self._app.route('/<path:filename>')
        def serve_static_file(filename):
            return send_from_directory(self._app.static_folder, filename)

    def _run_app(self):
        self._urdf_parser = URDFParser(self._urdf_path)
        self._app = Flask(__name__)

        # Initialize Flask routes
        self._setup_routes()

        """Run the Flask app."""
        self._app.run(
            host=self._host,
            port=self._app_port,
            ssl_context=(self._ssl_cert, self._ssl_key)
        )

    def start(self):
        # Start the Flask app
        self._process = Process(target=self._run_app)
        self._process.start()
        print(f"Flask app started on https://{self._host}:{self._app_port} with PID {self._process.pid}")

    def stop(self):
        if self._process is not None:
            self._process.terminate()
            self._process.join()
            print("Flask app process terminated.")
