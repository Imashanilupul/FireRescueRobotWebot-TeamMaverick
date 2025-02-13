from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
import time

# Initialize robot
TIME_STEP = 32
THRESHOLD = 900  # Distance sensor threshold
robot = Robot()

# Motors
motor_names = ["motor_1", "motor_2", "motor_3", "motor_4"]
motors = []
for name in motor_names:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))  # Set to velocity control mode
    motor.setVelocity(0.0)
    motors.append(motor)

# Distance sensors
sensors = []
sensor_names = ["ds_1", "ds_2", "ds_3", "ds_4", "ds_5", "ds_6", "ds_7", "ds_8"]
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    sensors.append(sensor)

# Position sensors (Encoders)
ps_names = ["ps_1", "ps_2", "ps_3", "ps_4"]
position_sensors = []
for name in ps_names:
    ps = robot.getDevice(name)
    ps.enable(TIME_STEP)
    position_sensors.append(ps)

def reset_encoders():
    for ps in position_sensors:
        ps.getValue()

def get_average_distance():
    values = [ps.getValue() for ps in position_sensors]
    return sum(values) / len(values)

# Adjusted Sensor Mapping
front_sensors = [sensors[2], sensors[6]]  # ds_3, ds_7 (front-left, front-right)
left_sensors = [sensors[0], sensors[4]]   # ds_1, ds_5 (left side)
right_sensors = [sensors[3], sensors[7]]  # ds_4, ds_8 (right side)
rear_sensors = [sensors[1], sensors[5]]   # ds_2, ds_6 (rear)

# Define grid parameters
cell_size = 0.25  # Each cell is 25cm x 25cm
maze_size = (20, 20)  # 5m x 5m maze (20x20 grid)
maze = np.zeros(maze_size, dtype=int)  # 0 = unexplored, 1 = path, -1 = wall

# Robot's initial position in the grid
start_x, start_y = 10, 0
position = (start_x, start_y)
maze[start_x][start_y] = 1

# Fire Pit and Survivor Detection
fire_zones = {}  # Dictionary to store fire pit locations
survivors = {}   # Dictionary to store survivor locations

def detect_fire_pit(x, y):
    # Simulate fire pit detection based on robot's position
    return (x, y) in fire_zones

def detect_survivor(x, y):
    return (x, y) in survivors

def rescue_survivor():
    print("Rescuing survivor...")
    time.sleep(3)  # Simulate rescue delay

# Visualization setup
plt.ion()
fig, ax = plt.subplots()
def update_visualization():
    ax.clear()
    ax.imshow(maze, cmap='gray_r', origin='upper')
    ax.set_title("Maze Mapping")
    plt.draw()
    plt.pause(0.05)

# Movement functions
def move_forward(distance=0.25):
    reset_encoders()
    while get_average_distance() < distance:
        for motor in motors:
            motor.setVelocity(10.0)
        robot.step(TIME_STEP)
    stop_motors()

def turn_left():
    motors[0].setVelocity(-5.0)
    motors[1].setVelocity(-5.0)
    motors[2].setVelocity(5.0)
    motors[3].setVelocity(5.0)
    robot.step(int(500 / TIME_STEP))
    stop_motors()

def turn_right():
    motors[0].setVelocity(5.0)
    motors[1].setVelocity(5.0)
    motors[2].setVelocity(-5.0)
    motors[3].setVelocity(-5.0)
    robot.step(int(500 / TIME_STEP))
    stop_motors()

def stop_motors():
    for motor in motors:
        motor.setVelocity(0.0)
    robot.step(TIME_STEP)

def is_wall(sensor_group):
    return any(sensor.getValue() > THRESHOLD for sensor in sensor_group)

# Wall-following and mapping logic
visited = set()
stack = [position]

directions = {
    (0, 1): front_sensors,
    (1, 0): right_sensors,
    (0, -1): rear_sensors,
    (-1, 0): left_sensors
}

while robot.step(TIME_STEP) != -1:
    if not stack:
        break  # If no more cells to explore, stop
    
    current_pos = stack.pop()
    x, y = current_pos
    visited.add(current_pos)
    
    if detect_survivor(x, y):
        rescue_survivor()
    
    if detect_fire_pit(x, y):
        print("Fire detected! Changing route.")
        continue  # Skip moving into fire pit
    
    # Check available directions (right, forward, left, backward)
    possible_directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    available_directions = []
    
    for direction in possible_directions:
        new_x, new_y = x + direction[0], y + direction[1]
        if (0 <= new_x < maze_size[0] and 0 <= new_y < maze_size[1]
                and (new_x, new_y) not in visited and not detect_fire_pit(new_x, new_y)):
            if not is_wall(directions[direction]):
                available_directions.append((new_x, new_y, direction))
    
    if available_directions:
        new_x, new_y, _ = available_directions[0]
        maze[new_x][new_y] = 1  # Mark as path
        stack.append((new_x, new_y))
        move_forward()
    else:
        for direction in possible_directions:
            new_x, new_y = x + direction[0], y + direction[1]
            if (0 <= new_x < maze_size[0] and 0 <= new_y < maze_size[1]
                    and maze[new_x][new_y] == 0):
                maze[new_x][new_y] = -1  # Mark as wall
    
    update_visualization()

plt.ioff()
plt.show()
print("Mapping complete:")
print(maze)