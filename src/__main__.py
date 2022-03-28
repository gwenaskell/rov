import os

os.system("sudo pigpiod")

os.system("uvicorn main:app_factory")

# for TU, run: uvicorn src.main:app_factory --reload
