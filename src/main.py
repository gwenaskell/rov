
from .api.api import server, app
from .components.backend import Backend


@app.on_event("startup")
async def startup_event():
    queue = server.get_queue()

    backend = Backend()

    backend.start(queue)

    server.set_backend(backend)

    print("backend started")


@app.on_event("shutdown")
async def shutdown_event():
    print("server shutting down")
    await server.get_backend().stop()

    print("backend stopped")


def app_factory():
    return app
