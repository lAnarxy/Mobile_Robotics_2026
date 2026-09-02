import os
os.chdir("../../..")

from realm_tools.robot_lib.my_robot import MyRobot

VELOCITY = 15.0
TURN_VELOCITY = 3.0

robot = MyRobot()
robot.load_environment('simulation/worlds/environments/samples/octagon.xml')

keyboard = robot.experiment_supervisor.getKeyboard()
keyboard.enable(robot.timestep)

print("Keyboard control active:")
print("  Arrow Up    -> forward")
print("  Arrow Down  -> backward")
print("  Arrow Left  -> turn left")
print("  Arrow Right -> turn right")
print("  Space       -> stop")

while robot.experiment_supervisor.step(robot.timestep) != -1:
    key = keyboard.getKey()

    if key == keyboard.UP:
        robot.set_left_motor_velocity(VELOCITY)
        robot.set_right_motor_velocity(VELOCITY)
    elif key == keyboard.DOWN:
        robot.set_left_motor_velocity(-VELOCITY)
        robot.set_right_motor_velocity(-VELOCITY)
    elif key == keyboard.LEFT:
        robot.set_left_motor_velocity(-TURN_VELOCITY)
        robot.set_right_motor_velocity(TURN_VELOCITY)
    elif key == keyboard.RIGHT:
        robot.set_left_motor_velocity(TURN_VELOCITY)
        robot.set_right_motor_velocity(-TURN_VELOCITY)
    else:
        robot.stop()
