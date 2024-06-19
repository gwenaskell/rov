import numpy as np
from src.components.classes import Quaternion, Vec


def get_earth_axis_coordinates(q: Quaternion) -> Vec:
    """returns a vector representing the vertical earth axis (norm 1) 
    in the coordinates system described by the quaternion, using the
    formula q' * v * q"""
    
    # compute v * q, v = [0 0 0 1]
    v_q_prod = Quaternion(- q.z, -q.y, q.x, q.w)

    q_prime_v = q.conj().mul(v_q_prod)
    # Extract rotated vector
    return Vec(q_prime_v.x, q_prime_v.y, q_prime_v.z)