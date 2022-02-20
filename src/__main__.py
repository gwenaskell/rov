import os

os.system("sudo pigpiod")

os.system("uvicorn main:app_factory")
