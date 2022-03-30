from dataclasses import dataclass
from multiprocessing.connection import Connection
from asyncio import Queue, Task
from typing import Optional

from starlette.websockets import WebSocketState
from fastapi import FastAPI, WebSocket, WebSocketDisconnect


import json

from src.api.classes import GamePad, GamePadButtons, GamePadSticks
from src.components.backend import Backend

app = FastAPI()


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

    def set_backend(self, backend: Backend):
        self.backend = backend

    def get_backend(self) -> Backend:
        if self.backend is None:
            raise RuntimeError("backend not created")
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
                    tm_ms=payload["tm_ms"])

                await server.queue.put(controls)
            except Exception as e:
                print("failed to parse "+data, e)
    finally:
        print("closing socket")
        WS.socket = None
