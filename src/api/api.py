from dataclasses import dataclass
from multiprocessing import Pipe, Process
from multiprocessing.connection import Connection
from typing import Optional

from starlette.websockets import WebSocketState
from fastapi import FastAPI, WebSocket, WebSocketDisconnect


import json

app = FastAPI()


@dataclass
class GamePadSticks:
    leftX: int
    leftY: int
    rightX: int
    rightY: int
    padX: int
    padY: int


@dataclass
class GamePadButtons:
    A: bool
    B: bool
    X: bool
    Y: bool
    L: bool
    L2: bool
    R: bool
    R2: bool
    stickL: bool
    stickR: bool


@dataclass
class GamePad:
    connected: bool
    sticks: GamePadSticks
    buttons: GamePadButtons
    tm: int


class WS:
    socket: Optional[WebSocket] = None

    @classmethod
    async def send_feedbacks(cls):
        socket = cls.socket
        if (not socket) or socket.application_state != WebSocketState.CONNECTED:
            return
        await socket.send_text("data")


class _Server:
    def __init__(self):
        self.pipe, self.downstream = Pipe()
        self.app = app
        self.process: Optional[Process] = None

    def get_pipe(self) -> Connection:
        return self.downstream

    def set_backend(self, proc: Process):
        self.process = proc

    def get_backend(self) -> Process:
        if self.process is None:
            raise Exception("backend not created")
        return self.process


server = _Server()


@app.get("/")
def read_root():
    return {"Hello": "World"}


@app.websocket("/socket")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()

    if WS.socket:
        await WS.socket.send_text('{"error": "a new connection was opened"}')
        await WS.socket.close()

    WS.socket = websocket
    try:
        while True:
            try:
                data = await websocket.receive_text()
            except WebSocketDisconnect:
                return

            try:
                payload = json.loads(data)

                if not payload:
                    continue

                controls = GamePad(
                    connected=payload["connected"],
                    sticks=GamePadSticks(**payload["sticks"]),
                    buttons=GamePadButtons(**payload["buttons"]),
                    tm=payload["tm"])

                server.pipe.send(controls)
            except Exception as e:
                print("failed to parse "+data, e)
    finally:
        print("closing socket")
        WS.socket = None
