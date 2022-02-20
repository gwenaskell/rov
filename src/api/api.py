from dataclasses import dataclass
from multiprocessing import Pipe
from multiprocessing.connection import Connection
from typing import Optional

from fastapi import FastAPI, WebSocket


import json

app = FastAPI()


@dataclass
class GamePad:
    pass


class _Server:
    def __init__(self):
        self.pipe, self.downstream = Pipe()
        self.app = app

    def setup(self) -> Connection:
        return self.downstream


server = _Server()


@app.get("/")
def read_root():
    return {"Hello": "World"}


@app.websocket("/gamepad-socket")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = await websocket.receive_text()

        try:
            controls = GamePad(**json.loads(data))
            server.pipe.send(controls)
        except:
            print("failed to parse "+data)

        await websocket.send_text(f"Message text was: {data}")
