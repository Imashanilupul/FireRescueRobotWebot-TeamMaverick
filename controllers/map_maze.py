from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
import time

# Initialize robot
time_step = 32
robot = Robot()

duration = 0.5

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

sensors = []
sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(time_step)
    sensors.append(sensor)

# Define grid parameters
cell_size = 0.25  # Each cell is 25cm x 25cm
maze_size = (20, 20)  # 5m x 5m maze (20x20 grid)
maze = np.zeros(maze_size, dtype=int)  # 0 = unexplored, 1 = path, -1 = wall

# Robot's initial position in the grid
start_x, start_y = 10, 10
position = (start_x, start_y)
maze[start_x][start_y] = 1

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
def move_forward():
    left_motor.setVelocity(5.0)
    right_motor.setVelocity(5.0)
    robot.step(int(duration * 1000 / time_step))

def turn_left():
    left_motor.setVelocity(-5.0)
    right_motor.setVelocity(5.0)
    robot.step(int(duration * 500 / time_step))

def turn_right():
    left_motor.setVelocity(5.0)
    right_motor.setVelocity(-5.0)
    robot.step(int(duration * 500 / time_step))

visited = set()
stack = [position]

def is_wall():
    return sensors[0].getValue() > 80

while robot.step(time_step) != -1:
    if not stack:
        break  # If no more cells to explore, stop
    
    current_pos = stack.pop()
    x, y = current_pos
    visited.add(current_pos)
    
    for direction in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # Right, Down, Left, Up
        new_x, new_y = x + direction[0], y + direction[1]
        if (0 <= new_x < maze_size[0] and 0 <= new_y < maze_size[1]
                and (new_x, new_y) not in visited):
            if not is_wall():
                maze[new_x][new_y] = 1  # Mark as path
                stack.append((new_x, new_y))
                move_forward()
            else:
                maze[new_x][new_y] = -1  # Mark as wall
                turn_right()
    
    update_visualization()

plt.ioff()
plt.show()
print("Mapping complete:")
print(maze)
