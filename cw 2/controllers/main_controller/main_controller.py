from controller import Robot, Camera
import numpy as np
import os
import cv2
import time

# === SUPPRESS TensorFlow Logs ===
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
import logging
logging.getLogger('tensorflow').setLevel(logging.FATAL)

# === LOAD MODELS ===
model_adam = tf.keras.models.load_model('cnn_adam_model.h5')
model_sgd = tf.keras.models.load_model('cnn_sgd_model.h5')

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
    #print(r,g,b)
    # === Image Capture ONLY IF Center is White ===
    if not image_saved and r > 100 and g > 100 and b > 100:
        image_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        image_rgb = image_array[:, :, :3]
        filename = "robot_view.png"
        cv2.imwrite(filename, image_rgb)
        print(f"[INFO] White detected! Image saved to: {os.path.abspath(filename)}")

        # === Preprocess for CNN model ===
        resized = cv2.resize(image_rgb, (32, 32))
        normalized = resized.astype('float32') / 255.0
        input_image = np.expand_dims(normalized, axis=0)

        # === Prediction using Adam and SGD models ===
        prediction_adam = model_adam.predict(input_image, verbose=0)
        prediction_sgd = model_sgd.predict(input_image, verbose=0)

        class_names = ['airplane', 'automobile', 'bird', 'cat', 'deer',
                       'dog', 'frog', 'horse', 'ship', 'truck']

        label_adam = class_names[np.argmax(prediction_adam)]
        label_sgd = class_names[np.argmax(prediction_sgd)]

        print(f"[CNN Adam] Prediction: {label_adam}")
        print(f"[CNN SGD ] Prediction: {label_sgd}")

        # === STOP and wait 3 seconds ===
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        pause_steps = int(3000 / TIME_STEP)
        for _ in range(pause_steps):
            robot.step(TIME_STEP)

        image_saved = True

    # === Color Detection ===
    detected_color = None
    if r > 200 and g < 40 and b < 45:
        detected_color = 'red'
    elif g > 200 and r < 45 and b < 45:
        detected_color = 'green'
    elif b > 200 and r < 45 and g < 45:
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
