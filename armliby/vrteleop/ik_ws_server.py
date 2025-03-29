import asyncio
import ssl
from dataclasses import dataclass
from multiprocessing import Pipe, Process
from typing import Callable, Dict, List, Optional

import numpy as np
import websockets
from flask import json
from scipy.spatial.transform import Rotation as R


@dataclass
class Vec:
    array: np.ndarray

    @property
    def x(self) -> float:
        return self.array[0];

    @property
    def y(self) -> float:
        return self.array[1];

    @property
    def z(self) -> float:
        return self.array[2];


@dataclass
class Pose:
    matrix4: np.ndarray

    @property
    def x(self) -> float:
        return self.matrix4[0, 3]

    @property
    def y(self) -> float:
        return self.matrix4[1, 3]

    @property
    def z(self) -> float:
        return self.matrix4[2, 3]

    def diff_to(self, to_pose: 'Pose') -> 'Pose':
        array = np.eye(4)
        array[:3, :3] = to_pose.matrix4[:3, :3] @ self.matrix4[:3, :3].T
        array[:3,  3] = to_pose.matrix4[:3,  3] - self.matrix4[:3,  3]
        return Pose(array)
    
    def rotvec(self) -> Vec:
        return Vec(self.matrix4[:3, :3].T @ R.from_matrix(self.matrix4[:3, :3]).as_rotvec(degrees=False))


@dataclass
class Button:
    pressed: bool
    value: float


@dataclass
class Controller:
    pose: Pose
    buttons: List[Button]
    axes: List[float]
        

@dataclass
class ControllerData:
    leftController: Controller
    rightController: Controller


class VRWebsocketServer:
    def __init__(
            self,
            host: str,
            port: int,
            cert_path: str,
            key_path: str,
            ):
        """
        Initialize the VR websocket server.

        Args:
            host: The host address of the server.
            port: The port number of the server.
            cert_path: The path to the SSL certificate file. 
                Create using openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes
            key_path: The path to the SSL key file.
        """
        self._host = host
        self._port = port
        self._cert_path = cert_path
        self._key_path = key_path
        self._process = None
        self._parent_conn, self._child_conn = Pipe()

    async def _handle_connection(self, websocket):
        print("Client connected")
        try:
            async for message in websocket:

                parsed_data = json.loads(message)

                # Convert dictionary to dataclass
                controller_data = ControllerData(
                    leftController=Controller(
                        pose=Pose(np.array(parsed_data["leftController"]["pose"]).reshape(4, 4).T),
                        buttons=[Button(**btn) for btn in parsed_data["leftController"]["buttons"]],
                        axes=parsed_data["leftController"]["axes"]
                    ),
                    rightController=Controller(
                        pose=Pose(np.array(parsed_data["rightController"]["pose"]).reshape(4, 4).T),
                        buttons=[Button(**btn) for btn in parsed_data["rightController"]["buttons"]],
                        axes=parsed_data["rightController"]["axes"]
                    )
                )

                # Send the message to the main process through the pipe
                self._child_conn.send(controller_data)

                # Wait for a response from the main process
                transforms = self._child_conn.recv()  # Blocks until a value is received

                # Convert the transforms to a JSON-friendly format
                transforms_json = {key: val.tolist() for key, val in transforms.items()}

                # Send the transformation matrices back to the client
                await websocket.send(json.dumps(transforms_json))

        except websockets.exceptions.ConnectionClosed as e:
            print("Client disconnected:", e)

    async def _start_server(self):
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.load_cert_chain(
            certfile=self._cert_path,
            keyfile=self._key_path
        )

        async with websockets.serve(
            self._handle_connection, self._host, self._port, ssl=ssl_context
        ):
            print(f"WebSocket server started on wss://{self._host}:{self._port}")
            await asyncio.Future()  # Run forever

    def _run(self):
        asyncio.run(self._start_server())

    def start(self):
        """Starts the WebSocket server in a separate process."""
        self._process = Process(target=self._run)
        self._process.start()
        print(f"WebSocket server process started with PID {self._process.pid}")

    def check(self, callback: Callable[[ControllerData], Dict[str, np.ndarray]]):
        """
        Checks for messages from the WebSocket process, processes them using the callback,
        and sends the result back to the WebSocket process.
        """
        if self._parent_conn.poll():  # Check if there's a message in the pipe
            message = self._parent_conn.recv()  # Receive the message
            # Send the data back to the WebSocket process
            self._parent_conn.send(callback(message))

    def stop(self):
        """Stops the WebSocket server process."""
        if self._process is not None:
            self._process.terminate()
            self._process.join()
            print("WebSocket server process terminated.")
