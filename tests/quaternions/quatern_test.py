
from src.components.classes import Vec, Quaternion
from src.components.geometry import get_earth_axis_coordinates

print(Quaternion(0.707, 0, 0.707, 0).rotate(Vec(0, 0, 1)))
print(get_earth_axis_coordinates(Quaternion(0.707, 0, 0.707, 0)))