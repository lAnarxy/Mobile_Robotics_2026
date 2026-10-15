import os
os.chdir("../../..")

"""ExperimentSupervisor controller."""
from realm_tools.robot_lib.my_robot import MyRobot

maze_file = 'simulation/worlds/environments/labs/maze3.xml'

# create the robot/supervisor instance.
robot = MyRobot()

# Loads the environment from the maze file
robot.load_environment(maze_file)

# Show basic robot/supervisor functions
robot.move_to_start()

