import os
os.chdir("../../..")

"""ExperimentSupervisor controller."""
from realm_tools.robot_lib.my_robot import MyRobot

maze_file = 'simulation/worlds/environments/labs/maze2.xml'

# create the robot/supervisor instance.
robot = MyRobot()

# Loads the environment from the maze file
robot.load_environment(maze_file)

# Show basic robot/supervisor functions
robot.move_to_start()

D1 = 1
D2 = 0.5
D3 = 2.5

# Drive parameters
r = 1.2
Kp = 10.0 * r
Ki = 0.5 * r
Kd = 1 * r

# Turn ratio for PID compass turns
robot.move_pid_lidar(D1, Kp, Ki, Kd)
robot.move_pid_encoder(D2, Kp, Ki, Kd)
robot.move_pid_lidar(D1, Kp, Ki, Kd)
robot.turn_pid_compass(-180, Kp, Ki, Kd)
robot.move_pid_lidar(D1, Kp, Ki, Kd)
robot.move_pid_encoder(D2, Kp, Ki, Kd)
robot.move_pid_lidar(D1, Kp, Ki, Kd)
robot.turn_pid_compass(180, Kp, Ki, Kd)
robot.move_pid_encoder(D3, Kp, Ki, Kd)

