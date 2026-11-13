import numpy as np


def calculate_attachment_points(radius, height, tilt_x=0, tilt_y=0):
    angles = np.radians([0, 120, 240])
    tilt_x = np.radians(tilt_x)
    tilt_y = np.radians(tilt_y)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(tilt_x), -np.sin(tilt_x)],
        [0, np.sin(tilt_x), np.cos(tilt_x)]
    ])
    Ry = np.array([
        [np.cos(tilt_y), 0, np.sin(tilt_y)],
        [0, 1, 0],
        [-np.sin(tilt_y), 0, np.cos(tilt_y)]
    ])

    points = []

    for angle in angles:
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        z = 0

        point = np.array([x, y, z])

        # Rotate around the platform center - matrix multiplication 
        rotated_point = Ry @ Rx @ point 

        # Move to the platform's center height
        rotated_point[2] += height

        points.append(rotated_point)

    return np.array(points)

def calculate_actuator_angles(points, offsets, upper, lower):
    motor_angles = []

    for i in range(3):
        x, y, z = points[i]

        r = np.hypot(x, y)
        d = offsets[i] - r
        C = (upper[i]**2 - d**2 - z**2 - lower[i]**2) / (2 * lower[i])
        B = np.arctan2(z, d)
        value = C / np.hypot(d, z)
        if abs(value) > 1:
            raise ValueError("Unreachable platform position")
        
        theta1 = -B + np.arccos(value)
        theta2 = -B - np.arccos(value)
        motor_angles.append([
            np.degrees(theta1),
            np.degrees(theta2)
        ])

    return np.array(motor_angles)

def convert_ik_to_servo_angles(ik_angles, neutral_ik, neutral_servo):
    servo_angles = []

    for i in range(3):
        angle = neutral_servo[i] - (ik_angles[i] - neutral_ik[i])
        servo_angles.append(angle)

    return servo_angles
