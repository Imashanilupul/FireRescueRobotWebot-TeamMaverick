from controller import Robot

# Time step (should match the simulation timestep in Webots)
TIME_STEP = 32

# Create the Robot instance
robot = Robot()

# Get the motors
wheel_names = ["motor_1", "motor_2", "motor_3", "motor_4"]
motors = []
for name in wheel_names:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))  # Set to velocity control mode
    motors.append(motor)

# Set the wheel speeds for forward movement
speed = 10.0  # Adjust as needed
motors[0].setVelocity(speed)
motors[1].setVelocity(speed)
motors[2].setVelocity(speed)
motors[3].setVelocity(speed)

# Main loop
while robot.step(TIME_STEP) != -1:
    pass  # The robot will keep moving forward
