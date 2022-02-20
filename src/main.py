from multiprocessing.dummy import Process
from .api.api import server, app
from .components.mainloop import run
import uvicorn

pipe = server.setup()


p = Process(target=run, args=(pipe,))
p.start()


def app_factory():
    return app
