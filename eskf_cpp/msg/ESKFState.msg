# ESKF Custom ROS2 Message
# Full state estimate from Error-State Kalman Filter

# Header with timestamp
std_msgs/Header header

# Quaternion attitude (body to earth rotation)
geometry_msgs/Quaternion quaternion

# Relative position in earth frame [m]
geometry_msgs/Vector3 position

# Relative velocity in earth frame [m/s]
geometry_msgs/Vector3 velocity

# Normalized image features [pbar_x, pbar_y]
float64[2] image_features

# Gyroscope bias [rad/s]
geometry_msgs/Vector3 gyro_bias

# Accelerometer bias [m/s²]
geometry_msgs/Vector3 accel_bias

# Covariance diagonal (17 elements for error state)
# [dtheta(3), dpr(3), dvr(3), dpbar(2), dbgyr(3), dbacc(3)]
float64[17] covariance_diagonal

# Filter status
uint8 STATUS_OK = 0
uint8 STATUS_WAITING_FOR_IMU = 1
uint8 STATUS_HIGH_UNCERTAINTY = 2
uint8 STATUS_MEASUREMENT_REJECTED = 3
uint8 status
