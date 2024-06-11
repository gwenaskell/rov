import numpy as np
from src.components.classes import Quaternion

def _q_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def rotate_vector_by_quaternion(v, q: Quaternion):
    # Pure quaternion from vector
    v_q = [0, v[0], v[1], v[2]]
    # Conjugate of quaternion
    q_conj = [q.w, -q.x, -q.y, -q.z]
    # Rotate vector
    v_q_prime = _q_multiply(_q_multiply([q.w, q.x, q.y, q.z], v_q), q_conj)
    # Extract rotated vector
    return v_q_prime[1:]


def get_earth_axis_coordinates(q: Quaternion):
    """returns a vector representing the vertical earth axis (norm 1) in the coordinates system described by the quaternion"""

    # Conjugate of quaternion
    q_conj = [q.w, -q.x, -q.y, -q.z]
    # Rotate vector

    q_v_prod = [- q.z, q.y, -q.x, q.w]

    v_q_prime = _q_multiply(q_v_prod, q_conj)
    # Extract rotated vector
    return v_q_prime[1:]