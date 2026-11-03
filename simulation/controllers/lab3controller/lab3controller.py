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

wall = 0 #Left
prev_state = "MTG"
state = "MTG"
prev_error = 0
while robot.experiment_supervisor.step(robot.timestep) != -1:
    state = robot.bug_0(state)
    print(state)
    if state == "MTG":
        prev_state = state
        phi_left, phi_right = robot.find_goal()
        robot.set_left_motor_velocity(phi_left)
        robot.set_right_motor_velocity(phi_right)
    elif state == "WF":
        if (min(robot.get_lidar_range_image()[175:185]) > 0.5 and prev_state == "MTG"):
            phi_left = phi_right = robot.pid_calc(min(robot.get_lidar_range_image()[175:185]) - 0.3, 30, label = "Approach Wall")[0]
        else:
            prev_state = state
            phi_left, phi_right, prev_error = robot.wall_follow_pid(wall, prev_error = prev_error)
        robot.set_left_motor_velocity(phi_left)
        robot.set_right_motor_velocity(phi_right)
    else:
        robot.stop()
    
