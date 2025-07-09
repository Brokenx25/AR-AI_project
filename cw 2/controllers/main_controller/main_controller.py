from controller import Robot

# Create a robot instance 
robot = Robot()

# Time step (how often the robot updates)
TIME_STEP = int(robot.getBasicTimeStep())

# Motor setup
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Distance sensors
distance_sensors = []
sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    distance_sensors.append(sensor)

# Camera
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# Constants
OBSTACLE_THRESHOLD = 80
TURN_STEPS = 25
Turn_counter = 0
is_turning = False

seen_colors = set()  # Track detected colors

# Main loop
while robot.step(TIME_STEP) != -1:
    distances = [sensor.getValue() for sensor in distance_sensors]
    
    # Get camera image
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    x = width // 2
    y = height // 2

    # Average color over small region around center
    r_total = g_total = b_total = 0
    count = 0
    for dx in range(-5, 6):  # 11x11 patch
        for dy in range(-5, 6):
            xi = x + dx
            yi = y + dy
            if 0 <= xi < width and 0 <= yi < height:
                r_total += camera.imageGetRed(image, width, xi, yi)
                g_total += camera.imageGetGreen(image, width, xi, yi)
                b_total += camera.imageGetBlue(image, width, xi, yi)
                count += 1
    if count == 0:
        continue  # avoid division by zero
    r = r_total // count
    g = g_total // count
    b = b_total // count

    # Debug: Show color values


    # Determine color
    detected_color = None
    if r > 100 and g < 80 and b < 80:
        detected_color = 'red'
    elif g > 100 and r < 80 and b < 80:
        detected_color = 'green'
    elif b > 100 and r < 80 and g < 80:
        detected_color = 'blue'


    # feedback if new colour found
    if detected_color and detected_color not in seen_colors:
        print(f"I see {detected_color}")
        seen_colors.add(detected_color)
        print("Summary: I have previously seen: " + ", ".join(seen_colors))

    # Obstacle avoidance and turning logic
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
