import numpy as np
import time
import math

# Constants
dt = 0.01  # Sampling period

# State vector: [vx, vy, vz, roll, pitch, yaw, gx, gy, gz, ax, ay, az]
n = 12
x = np.zeros(n)

# State transition matrix (A)
A = np.eye(n)
A[0, 9] = dt  # ax influence vx
A[1, 10] = dt  # ay influence vy
A[2, 11] = dt  # az influence vz
A[3, 6] = dt  # gx influence roll
A[4, 7] = dt  # gy influence pitch
A[5, 8] = dt  # gz influence yaw

# Control matrix (B)
B = np.zeros((n, 6))
B[9, 0] = dt  # ax control input
B[10, 1] = dt  # ay control input
B[11, 2] = dt  # az control input
B[6, 3] = dt  # gx control input
B[7, 4] = dt  # gy control input
B[8, 5] = dt  # gz control input

# Observation matrix (H)
H = np.zeros((9, n))
H[0, 0] = 1  # vx velocity
H[1, 1] = 1  # vy velocity
H[2, 2] = 1  # vz velocity
H[3, 3] = 1  # roll angle
H[4, 4] = 1  # pitch angle
H[5, 5] = 1  # yaw angle
H[6, 6] = 1  # gx angular velocity
H[7, 7] = 1  # gy angular velocity
H[8, 8] = 1  # gz angular velocity

# Process noise covariance (Q)
Q = np.eye(n) * 0.1

# Measurement noise covariance (R)
R = np.eye(9) * 0.5

# Estimation error covariance (P)
P = np.eye(n)

def predict(x, P, A, B, u, Q):
    x = A @ x + B @ u
    P = A @ P @ A.T + Q
    return x, P

def update(x, P, H, z, R):
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    P = (np.eye(len(P)) - K @ H) @ P
    return x, P

while True:
    # Read sensor data
    ax, ay, az = read_accelerometer()
    gx, gy, gz = read_gyroscope()
    mx, my, mz = read_magnetometer()

    # Control input (u)
    u = np.array([ax, ay, az, gx, gy, gz])

    # Prediction step
    x, P = predict(x, P, A, B, u, Q)

    # Measure angular positions from magnetometer and accelerometer
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, np.sqrt(ay**2 + az**2))
    yaw = math.atan2(-my, mx)  # Simplified calculation; use a more accurate method for real applications

    # Measurements vector (z)
    z = np.array([x[0], x[1], x[2], roll, pitch, yaw, gx, gy, gz])

    # Update step
    x, P = update(x, P, H, z, R)

    # Extract estimated states
    vel = x[:3]
    angles = x[3:6]
    ang_vel = x[6:9]
    lin_acc = x[9:12]

    # Print results
    print(f"Velocity: {vel}")
    print(f"Angles: {angles}")
    print(f"Angular velocity: {ang_vel}")
    print(f"Linear acceleration: {lin_acc}")

    # Delay to match the sampling period
    time.sleep(dt)
