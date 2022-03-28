
from .api.api import server, app
from .components.mainloop import run
from asyncio import create_task


@app.on_event("startup")
async def startup_event():
    queue = server.get_queue()

    task = create_task(run(queue))

    server.set_backend(task)
    print("backend started")


@app.on_event("shutdown")
async def shutdown_event():
    print("server shutting down")
    server.get_backend().cancel()

    print("backend stopped")


def app_factory():
    return app
