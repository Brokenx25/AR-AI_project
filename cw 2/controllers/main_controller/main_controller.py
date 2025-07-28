from controller import Robot, Camera
import cv2
import numpy as np
import os

# === Robot Initialization ===
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# === Motor Setup ===
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# === Distance Sensors ===
distance_sensors = []
sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    distance_sensors.append(sensor)

# === Camera Setup ===
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# === Constants and Flags ===
OBSTACLE_THRESHOLD = 80
TURN_STEPS = 25
Turn_counter = 0
is_turning = False
seen_colors = set()
image_saved = False

# === Main Loop ===
while robot.step(TIME_STEP) != -1:
    # === Save image once when camera is ready ===
    if not image_saved:
        image = camera.getImage()
        width = camera.getWidth()
        height = camera.getHeight()

        # Convert BGRA to BGR
        image_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        image_bgr = image_array[:, :, :3]
        filename = "robot_view.png"
        cv2.imwrite(filename, image_bgr)
        print(f"[INFO] Image saved to: {os.path.abspath(filename)}")
        image_saved = True

    # === Sensor Reading ===
    distances = [sensor.getValue() for sensor in distance_sensors]

    # === Get Image Data and Center Patch Color ===
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    x = width // 2
    y = height // 2

    r_total = g_total = b_total = 0
    count = 0
    for dx in range(-5, 6):
        for dy in range(-5, 6):
            xi = x + dx
            yi = y + dy
            if 0 <= xi < width and 0 <= yi < height:
                r_total += camera.imageGetRed(image, width, xi, yi)
                g_total += camera.imageGetGreen(image, width, xi, yi)
                b_total += camera.imageGetBlue(image, width, xi, yi)
                count += 1

    if count == 0:
        continue

    r = r_total // count
    g = g_total // count
    b = b_total // count

    # === Color Detection ===
    detected_color = None
    if r > 100 and g < 80 and b < 80:
        detected_color = 'red'
    elif g > 100 and r < 80 and b < 80:
        detected_color = 'green'
    elif b > 100 and r < 80 and g < 80:
        detected_color = 'blue'

    if detected_color and detected_color not in seen_colors:
        print(f"I see {detected_color}")
        seen_colors.add(detected_color)
        print("Summary: I have previously seen: " + ", ".join(seen_colors))

    # === Obstacle Avoidance Logic ===
    front_obstacle = any(sensor_value > OBSTACLE_THRESHOLD for sensor_value in distances[:2] + distances[5:7])

    if is_turning:
        Turn_counter += 1
        left_motor.setVelocity(4.0)
        right_motor.setVelocity(-4.0)
        if Turn_counter >= TURN_STEPS:
            is_turning = False
            Turn_counter = 0
    elif front_obstacle:
        print("Obstacle detected!!")
        is_turning = True
        Turn_counter = 0
        left_motor.setVelocity(4.0)
        right_motor.setVelocity(-4.0)
    else:
        left_motor.setVelocity(5.0)
        right_motor.setVelocity(5.0)
