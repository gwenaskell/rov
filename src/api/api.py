from dataclasses import dataclass
from multiprocessing.connection import Connection
from asyncio import Queue, Task
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
        self.queue = Queue(maxsize=1)
        self.app = app
        self.backend = None

    def get_queue(self) -> Queue:
        return self.queue

    def set_backend(self, task: Task):
        self.backend = task

    def get_backend(self) -> Task:
        if self.backend is None:
            raise Exception("backend not created")
        return self.backend


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

                await server.queue.put(controls)
            except Exception as e:
                print("failed to parse "+data, e)
    finally:
        print("closing socket")
        WS.socket = None
