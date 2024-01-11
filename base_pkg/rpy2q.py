from math import sin, cos, radians

def rollpitchyaw_to_quaternion(roll, pitch, yaw):
    roll_rad = radians(roll)
    pitch_rad = radians(pitch)
    yaw_rad = radians(yaw)

    cy = cos(yaw_rad * 0.5)
    sy = sin(yaw_rad * 0.5)
    cp = cos(pitch_rad * 0.5)
    sp = sin(pitch_rad * 0.5)
    cr = cos(roll_rad * 0.5)
    sr = sin(roll_rad * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return qw, qx, qy, qz


print(rollpitchyaw_to_quaternion(0,0,0))