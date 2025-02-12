from controller import Robot, GPS
import numpy as np
# import matplotlib.pyplot as plt
import random
import math

# Initialize robot
robot = Robot()
time_step = int(robot.getBasicTimeStep())

# Initialize sensors and motors
distance_sensors = [robot.getDevice(f'ps{i}') for i in range(8)]
for sensor in distance_sensors:
    sensor.enable(time_step)
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Initialize GPS
gps = robot.getDevice('gps')
gps.enable(time_step)

def wait_for_gps():
    while robot.step(time_step) != -1:
        x, y, _ = gps.getValues()
        if not math.isnan(x) and not math.isnan(y):
            break

wait_for_gps()

# Maze and mapping setup
cell_size = 0.25
maze_size = (10, 10)
maze_map = np.zeros(maze_size)  # 0 = unvisited, 1 = visited, 2 = wall

# Convert GPS position to grid index

def get_grid_position():
    x, y, _ = gps.getValues()
    if math.isnan(x) or math.isnan(y):
        return None  # Handle NaN values
    row = max(0, min(maze_size[0] - 1, int((y + 2.5) / cell_size))) - 5 
    col = max(0, min(maze_size[1] - 1, int((x + 2.5) / cell_size))) - 5 
    return row, col


# Movement settings
speed = 2.8
directions = [(0, 1), (-1, 0), (0, -1), (1, 0)]  # Right, Up, Left, Down
current_direction = 0

# Ensure initial position is valid
while True:
    current_position = get_grid_position()
    if current_position is not None:
        break
    robot.step(time_step)

maze_map[current_position] = 1

def move_forward():
    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)
    robot.step(int(1000 * cell_size / speed))
    update_position()

def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    robot.step(time_step)

def turn(degrees):
    rotation_time = int((abs(degrees) / 90) * 800)
    if degrees > 0:
        left_motor.setVelocity(speed)
        right_motor.setVelocity(-speed)
    else:
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(speed)
    robot.step(rotation_time)
    stop()
    update_direction(degrees)

def update_position():
    global current_position
    new_position = get_grid_position()
    if new_position is not None:
        current_position = new_position
        maze_map[current_position] = 1

def update_direction(degrees):
    global current_direction
    if degrees > 0:
        current_direction = (current_direction + 1) % 4
    else:
        current_direction = (current_direction - 1) % 4

def detect_walls():
    if distance_sensors[0].getValue() > 80 or distance_sensors[7].getValue() > 80:
        dx, dy = directions[current_direction]
        wall_x = current_position[0] + dx
        wall_y = current_position[1] + dy
        if 0 <= wall_x < maze_size[0] and 0 <= wall_y < maze_size[1]:
            maze_map[wall_x, wall_y] = 2  # Mark as wall

def choose_new_direction():
    global current_direction
    for i in range(4):
        new_dir = (current_direction + i) % 4
        dx, dy = directions[new_dir]
        new_x = current_position[0] + dx
        new_y = current_position[1] + dy
        if 0 <= new_x < maze_size[0] and 0 <= new_y < maze_size[1]:
            if maze_map[new_x, new_y] == 0:
                turn(90 * i)
                return
    turn(180)  # If no new direction, turn around

# Main loop
while robot.step(time_step) != -1:
    detect_walls()
    front_obstacle = distance_sensors[0].getValue() > 80 or distance_sensors[7].getValue() > 80
    if front_obstacle:
        choose_new_direction()
    else:
        move_forward()
    
    # Real-time visualization (optional)
    # plt.imshow(maze_map, cmap='gray_r', origin='lower')  # Ensure correct orientation
    # plt.pause(0.1)

plt.show()
