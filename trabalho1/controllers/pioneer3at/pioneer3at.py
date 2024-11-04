from controller import Robot, DistanceSensor,Motor, Lidar
import math

# Constants
TIME_STEP = 32
MAX_SPEED = 6.4
CRUISING_SPEED = 5.0
OBSTACLE_THRESHOLD = 0.1 # Lidar
DECREASE_FACTOR = 0.9
BACK_SLOWDOWN = 0.9

N_SONAR_SENSORS = 16
MIN_DISTANCE = 1.0 # Sonar
WHEEL_WEIGHT_THRESHOLD = 100 # Sonar

robot = Robot()

# Get wheels
front_left_wheel = robot.getDevice("front left wheel")
front_right_wheel = robot.getDevice("front right wheel")
back_left_wheel = robot.getDevice("back left wheel")
back_right_wheel = robot.getDevice("back right wheel")

# Initialize wheels
for wheel in [front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel]:
    wheel.setPosition(float('inf'))

# Initialize speeds for each wheel
back_left_speed = back_right_speed = 0.0
front_left_speed = front_right_speed = 0.0

class SonarSensor:
    def __init__(self, device_tag, wheel_weight):
        self.device_tag = device_tag
        self.wheel_weight = wheel_weight

# Configure Sonar Sensors
sonar_sensors = []
for i in range(N_SONAR_SENSORS):
    sensor_name = f"so{i}"
    sensor_device = robot.getDevice(sensor_name)
    sensor_device.enable(TIME_STEP)
    # Define o peso de cada sensor para a direção
    if i < 8:
        wheel_weight = [150 + 50 * i, 0] if i < 4 else [0, 150 + 50 * (i - 4)]
    else:
        wheel_weight = [0, 0]
    sonar_sensors.append(SonarSensor(sensor_device, wheel_weight))

# Initialize lidar
lms291 = robot.getDevice("Sick LMS 291")
lms291.enable(TIME_STEP)
lms291.enablePointCloud()

lms291_width = lms291.getHorizontalResolution()
half_width = lms291_width // 2
max_range = lms291.getMaxRange()
range_threshold = max_range / 20.0

# Gaussian function
def gaussian(x, mu, sigma):
    return (1.0 / (sigma * math.sqrt(2.0 * math.pi))) * math.exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma))

# Initialize Braitenberg coefficients
braitenberg_coefficients = [gaussian(i, half_width, lms291_width / 5) for i in range(lms291_width)]

def rotate_90():
    front_left_wheel.setVelocity(0.5 * MAX_SPEED)
    front_right_wheel.setVelocity(-0.5 * MAX_SPEED)
    back_left_wheel.setVelocity(0.5 * MAX_SPEED)
    back_right_wheel.setVelocity(-0.5 * MAX_SPEED)
    robot.step(TIME_STEP * 25)  # Ajuste o tempo conforme necessário

# Control loop
while robot.step(TIME_STEP) != -1:
    wheel_weight_total = [0.0, 0.0]

    # Atualiza as leituras dos sensores e calcula o peso nas rodas
    for sensor in sonar_sensors:
        sensor_value = sensor.device_tag.getValue()
        if sensor_value == 0.0:
            speed_modifier = 0.0
        else:
            distance = 5.0 * (1.0 - (sensor_value / N_SONAR_SENSORS))
            speed_modifier = 1 - (distance / MIN_DISTANCE) if distance < MIN_DISTANCE else 0.0

        # Ajusta o peso total nas rodas com base nos sensores
        for j in range(2):
            wheel_weight_total[j] += sensor.wheel_weight[j] * speed_modifier

    # Get lidar values
    lms291_values = lms291.getRangeImage()
    left_obstacle = right_obstacle = 0.0

    # Apply the Braitenberg coefficients
    for i in range(half_width):
        if lms291_values[i] < range_threshold:
            left_obstacle += braitenberg_coefficients[i] * (1.0 - lms291_values[i] / max_range)

        j = lms291_width - i - 1
        if lms291_values[j] < range_threshold:
            right_obstacle += braitenberg_coefficients[i] * (1.0 - lms291_values[j] / max_range)

    # Overall front obstacle
    obstacle = left_obstacle + right_obstacle

    # Ajusta a velocidade com base no LIDAR
    if obstacle > OBSTACLE_THRESHOLD:
        print('Usando Lidar')
        speed_factor = (1.0 - DECREASE_FACTOR * obstacle) * MAX_SPEED / obstacle
        front_left_speed = speed_factor * left_obstacle
        front_right_speed = speed_factor * right_obstacle
        back_left_speed = BACK_SLOWDOWN * front_left_speed
        back_right_speed = BACK_SLOWDOWN * front_right_speed
    else:
        # Se o Lidar falhar usa o Sonar
        if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD:  # Objeto à direita
            print('Usando Sonar')
            front_left_speed = 0.7 * MAX_SPEED
            front_right_speed = -0.7 * MAX_SPEED
            back_left_speed = BACK_SLOWDOWN * front_left_speed
            back_right_speed = BACK_SLOWDOWN * front_right_speed
        elif wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:  # Objeto à esquerda
            print('Usando Sonar')
            front_left_speed = -0.7 * MAX_SPEED
            front_right_speed = 0.7 * MAX_SPEED
            back_left_speed = BACK_SLOWDOWN * front_left_speed
            back_right_speed = BACK_SLOWDOWN * front_right_speed
        else:
            # Sem obstaculos
            back_left_speed = back_right_speed = CRUISING_SPEED
            front_left_speed = front_right_speed = CRUISING_SPEED

    # Set actuator velocities
    front_left_wheel.setVelocity(front_left_speed)
    front_right_wheel.setVelocity(front_right_speed)
    back_left_wheel.setVelocity(back_left_speed)
    back_right_wheel.setVelocity(back_right_speed)
