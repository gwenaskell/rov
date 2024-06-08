

# class KalmanFilter:
#     def __init__(self, q_angle, q_bias, r_measure):
#         self.q_angle = q_angle
#         self.q_bias = q_bias
#         self.r_measure = r_measure

#         self.angle = 0.0  # Reset the angle
#         self.bias = 0.0   # Reset bias
#         self.rate = 0.0   # Unbiased rate

#         self.P = [[0, 0], [0, 0]]  # Error covariance matrix

#     def get_angle(self, new_angle: float, new_rate: float, dt):
#         # Predict
#         self.rate = new_rate - self.bias
#         self.angle += dt * self.rate

#         self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.q_angle)
#         self.P[0][1] -= dt * self.P[1][1]
#         self.P[1][0] -= dt * self.P[1][1]
#         self.P[1][1] += self.q_bias * dt

#         # Update
#         S = self.P[0][0] + self.r_measure
#         K = [self.P[0][0] / S, self.P[1][0] / S]

#         y = new_angle - self.angle
#         self.angle += K[0] * y
#         self.bias += K[1] * y

#         p00_temp = self.P[0][0]
#         p01_temp = self.P[0][1]

#         self.P[0][0] -= K[0] * p00_temp
#         self.P[0][1] -= K[0] * p01_temp
#         self.P[1][0] -= K[1] * p00_temp
#         self.P[1][1] -= K[1] * p01_temp

#         return self.angle

# def get_accel_angles(ax, ay, az):
#     pitch = math.atan2(ay, az) * 180 / math.pi
#     roll = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180 / math.pi
#     return pitch, roll

# def get_mag_yaw(mx, my, mz, pitch, roll):
#     pitch_rad = math.radians(pitch)
#     roll_rad = math.radians(roll)

#     # Tilt compensation
#     mx_comp = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
#     my_comp = mx * math.sin(roll_rad) * math.sin(pitch_rad) + my * math.cos(roll_rad) - mz * math.sin(roll_rad) * math.cos(pitch_rad)

#     yaw = math.atan2(-my_comp, mx_comp) * 180 / math.pi
#     return yaw


class EKF:
    def __init__(self):
        # Process noise covariance (Q)
        # When Q is large, the Kalman Filter tracks large changes in 
        # the sensor measurements more closely than for smaller Q.
        # Q is a square matrix that has the same number of rows as states.
        self.Q = np.eye(12)#*1e-2
        # Measurement noise covariance (R)
        # Has the same number of rows and columns as sensor measurements.
        # If we are sure about the measurements, R will be near zero.
        self.R = np.eye(9)*1e-2
        # Estimation error covariance (P)
        # This matrix has the same number of rows (and columns) as the 
        # number of states (i.e. 3x3 matrix). P is sometimes referred
        # to as Sigma in the literature. It represents an estimate of 
        # the accuracy of the state estimate at time k made using the
        # state transition matrix. We start off with guessed values.
        self.P = np.eye(12)
        # Vecteur d'état [vx, vy, vz, roll, pitch, yaw, gx, gy, gz, ax, ay, az]
        self.x = np.zeros(12)

        # Observation matrix (H)
        H = np.zeros((9, 12))
        # Used to convert the predicted state estimate at time k
        # into predicted sensor measurements at time k.
        # H has the same number of rows as sensor measurements
        # and same number of columns as states.
        #
        # sensor measurements vector : [roll, pitch, yaw, gx, gy, gz, ax, ay, az]
        for i in range(9):
            H[i, i+3] = 1

        self.H = H

    def predict(self, dt, accel: AccelValue):
        # State transition matrix (A)
        # number of states x number of states matrix
        # Expresses how the state of the system [x,y,yaw] changes 
        # from k-1 to k when no control command is executed.
        A = np.eye(12)
        A[0, 9] = dt  # vx += ax*dt
        A[1, 10] = dt  # vy += ay*dt
        A[2, 11] = dt  # vz += az*dt
        A[3, 6] = dt  # roll += wx*dt
        A[4, 7] = dt  # pitch += wy*dt
        A[5, 8] = dt  # yaw += wz*dt

        # Control matrix (B)
        # number of states x number of control inputs
        # Expresses how the state of the system [x,y,yaw] changes
        # from k-1 to k due to the control commands (i.e. control input).
        B = np.zeros((12, 6))
        # B[9, 0] = dt  # ax control input
        # B[10, 1] = dt  # ay control input
        # B[11, 2] = dt  # az control input
        # B[6, 3] = dt  # gx control input
        # B[7, 4] = dt  # gy control input
        # B[8, 5] = dt  # gz control input

        u = np.array([accel.ax, accel.ay, accel.az, accel.wx, accel.wy, accel.wz])

        self.x = A @ self.x + B @ u
        self.P = A @ self.P @ A.T + self.Q

    def update(self, accel: AccelValue,
               roll: float, pitch: float, yaw: float) -> RovState:
        # Measurements vector (z)
        z = np.array([roll, pitch, yaw, accel.wx, accel.wy, accel.wz, accel.ax, accel.ay, accel.az])

        # Calculate the difference between the actual sensor measurements
        # at time k minus what the measurement model predicted 
        # the sensor measurements would be for the current timestep k.
        y = z - self.H @ self.x
        # Calculate the measurement residual covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Calculate the near-optimal Kalman gain
        # We use pseudoinverse since some of the matrices might be
        # non-square or singular.
        K = self.P @ self.H.T @ np.linalg.pinv(S)
        # Calculate an updated state estimate
        self.x = self.x + K @ y
        # Update the state covariance estimate
        self.P = (np.eye(len(self.P)) - K @ self.H) @ self.P


        return RovState(
            vx=self.x[0],
            vy=self.x[1],
            vz=self.x[2],
            thx=self.x[3],
            thy=self.x[4],
            thz=self.x[5],
            wx=self.x[6],
            wy=self.x[7],
            wz=self.x[8],
            ax=self.x[9],
            ay=self.x[10],
            az=self.x[11],
        )