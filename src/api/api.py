import typing
from pydantic import BaseModel
import asyncio
from dataclasses import dataclass
import dataclasses
from multiprocessing.connection import Connection
from asyncio import Queue, Task, create_task
from typing import Optional

from starlette.websockets import WebSocketState
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware


import json

from src.api.classes import GamePad, GamePadButtons, GamePadSticks
from src.components.backend import Backend
from src.components.classes import Status

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins="*",
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

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
        self.backend: Backend = typing.cast(Backend, None)

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


class EnginesState(BaseModel):
    on: bool
    paused: bool


@app.put("/engines")
def set_engines_state(state: EnginesState):
    if state.on:
        server.backend.switch_engines(
            Status.PAUSED if state.paused else Status.RUNNING)
    else:
        server.backend.switch_engines(Status.STOPPED)


@app.websocket("/socket")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()

    if WS.socket:
        await WS.socket.send_text('{"error": "a new connection was opened"}')
        await WS.socket.close()

    WS.socket = websocket

    feedbacks_t = create_task(submit_feedbacks(websocket))
    
    print("socket opened")

    controls = GamePad(
        connected=False,
        sticks=GamePadSticks(),
        buttons=GamePadButtons(),
        tm_ms=0)
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

                controls.tm_ms = payload["tm_ms"]
                controls.connected = True
                if "sticks" in payload:
                    controls.sticks = GamePadSticks(**payload["sticks"])
                if "buttons" in payload:
                    controls.buttons = GamePadButtons(**payload["buttons"])

                await server.queue.put(controls)
            except Exception as e:
                print("failed to parse "+data, e)
    finally:
        print("closing socket")
        WS.socket = None
        await feedbacks_t


async def submit_feedbacks(websocket: WebSocket):
    while True:
        await asyncio.sleep(0.3)
        try:
            await websocket.send_json(dataclasses.asdict(server.backend.feedbacks))
        except (WebSocketDisconnect, RuntimeError):
            return
