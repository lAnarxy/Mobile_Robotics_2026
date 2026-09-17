import os
os.chdir("../../..")

"""ExperimentSupervisor controller."""
from realm_tools.robot_lib.my_robot import MyRobot

maze_file = 'simulation/worlds/environments/labs/maze1.xml'

# create the robot/supervisor instance.
robot = MyRobot()

# Loads the environment from the maze file
robot.load_environment(maze_file)

# Show basic robot/supervisor functions
robot.move_to_start()

# Shape Vars
L = 3
W = 1
R1 = 1.5
R2 = 0.5
S = 1

turn_speed = 2
circle_speed = 10
line_speed = 12
# Rectangle
robot.drive_straight(L/2, line_speed)
robot.turn_in_place(-90, turn_speed)
robot.drive_straight(W, line_speed)
robot.turn_in_place(-90, turn_speed)
robot.drive_straight(L, line_speed)
robot.turn_in_place(-90, turn_speed)
robot.drive_straight(W, line_speed)
robot.turn_in_place(-90, turn_speed)
robot.drive_straight(L/2, line_speed)

# Circle 1
robot.turn_on_circle(360, circle_speed, R1)

# Circle 2
robot.turn_on_circle(-360, circle_speed, R2)

# Triangle
robot.drive_straight(S, line_speed)
robot.turn_in_place(120, turn_speed)
robot.drive_straight(S, line_speed)
robot.turn_in_place(120, turn_speed)
robot.drive_straight(S, line_speed)
robot.turn_in_place(120, turn_speed)

