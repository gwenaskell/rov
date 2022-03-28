
from multiprocessing import Process

from .api.api import server, app
from .components.mainloop import run


@app.on_event("startup")
async def startup_event():
    pipe = server.get_pipe()

    p = Process(target=run, args=(pipe,))
    server.set_backend(p)
    p.start()
    print("backend started")


@app.on_event("shutdown")
async def shutdown_event():
    print("server shutting down")
    server.get_pipe().send(None)

    server.get_backend().join(timeout=3)
    print("backend stopped")


def app_factory():
    return app
