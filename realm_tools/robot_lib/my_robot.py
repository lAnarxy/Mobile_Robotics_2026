from realm_tools.robot_lib.hambot import HamBot
import math

class MyRobot(HamBot):
    def __init__(self):
        HamBot.__init__(self)

    def drive_straight(self, distance, speed):
        # Caps the robots speed to max velocity
        if speed > self.max_motor_velocity:
            speed = self.max_motor_velocity
        # Records the initial time and encoder readings of the robot's wheels
        time_initial = self.experiment_supervisor.getTime()
        left_motor_initial_angle = self.get_left_motor_encoder_reading()
        right_motor_initial_angle = self.get_right_motor_encoder_reading()
        while self.experiment_supervisor.step(self.timestep) != -1:

            print("Max rotational motor velocity: ", self.max_motor_velocity)

            # Reads and Prints Robot's Encoder Readings
            print("Motor Encoder Readings: ", self.get_encoder_readings())

            # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
            print("Lidar Front Reading", self.get_lidar_range_image()[180])
            print("Lidar Left Reading", self.get_lidar_range_image()[90])
            print("Lidar Rear Reading", self.get_lidar_range_image()[0])
            print("Lidar Right Reading", self.get_lidar_range_image()[270])
            print("Simulation Time", self.experiment_supervisor.getTime())
            print("Compass Reading", self.get_compass_reading())

            # Sets the robot's motor velocity to 18 rad/sec
            self.set_right_motor_velocity(speed)
            self.set_left_motor_velocity(speed)

            # Calculates distance the wheel has turned since beginning of simulation
            distance_left_wheel_traveled = self.wheel_radius * (self.get_left_motor_encoder_reading() - left_motor_initial_angle)
            distance_right_wheel_traveled = self.wheel_radius * (self.get_right_motor_encoder_reading() - right_motor_initial_angle)
            travel_time = self.experiment_supervisor.getTime() - time_initial

            wheel_distance_average = (distance_left_wheel_traveled + distance_right_wheel_traveled) / 2

            # Error region based on speed
            error_region = 0.03 * speed * self.wheel_radius
            # Stops the robot after the robot moves a distance of 1.5 meters
            if distance + error_region > wheel_distance_average > distance - error_region:
                print(f"Time to travel {distance} meters:{travel_time:.2f}")
                self.stop()
                break

    #Turns the robot in place by a specified angle (in degrees) at a specified speed (in rad/sec)
    def turn_in_place(self, angle, speed):
        # Caps the robots speed to max velocity
        if speed > self.max_motor_velocity:
            speed = self.max_motor_velocity
        # Records the initial time and encoder readings of the robot's wheels
        time_initial = self.experiment_supervisor.getTime()
        left_motor_initial_angle = self.get_left_motor_encoder_reading()
        right_motor_initial_angle = self.get_right_motor_encoder_reading()
        # 1 = left -1 = right
        direction = 1 if angle > 0 else -1
        while self.experiment_supervisor.step(self.timestep) != -1:

            print("Max rotational motor velocity: ", self.max_motor_velocity)

            # Reads and Prints Robot's Encoder Readings
            print("Motor Encoder Readings: ", self.get_encoder_readings())

            # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
            print("Lidar Front Reading", self.get_lidar_range_image()[180])
            print("Lidar Left Reading", self.get_lidar_range_image()[90])
            print("Lidar Rear Reading", self.get_lidar_range_image()[0])
            print("Lidar Right Reading", self.get_lidar_range_image()[270])
            print("Simulation Time", self.experiment_supervisor.getTime())

            # Sets the robot's motor velocity to 18 rad/sec
            self.set_left_motor_velocity(direction * speed * -1)
            self.set_right_motor_velocity(direction * speed)

            # Calculates distance the wheel has turned since beginning of simulation
            distance_left_wheel_traveled = self.wheel_radius * (self.get_left_motor_encoder_reading() - left_motor_initial_angle)
            distance_right_wheel_traveled = self.wheel_radius * (self.get_right_motor_encoder_reading() - right_motor_initial_angle)
            travel_time = self.experiment_supervisor.getTime() - time_initial

            wheel_distance_average = (direction * distance_left_wheel_traveled * -1 + direction * distance_right_wheel_traveled) / 2
            distance_goal = direction * (self.axel_length * math.pi * angle) / 360  # distance traveled by the wheel to turn desired angle.
            print("Compass Reading", self.get_compass_reading())
            print("Distance Goal", distance_goal)
            print("Travel Distance", wheel_distance_average)

            # Error region based on speed
            error_region = 0.09 * speed * self.wheel_radius
            # Stops the robot after the robot moves a distance of 1.5 meters
            if distance_goal + error_region > wheel_distance_average > distance_goal - error_region:
                print(f"Time to turn {angle} degrees:{travel_time:.2f}")
                self.stop()
                break

    def turn_on_circle(self, angle, speed, radius):
        # 1 = left -1 = right
        direction = 1 if angle > 0 else -1
        w = speed / radius  # angular velocity of robot
        left_speed = w * (radius + direction * -1 * self.axel_length / 2)  # speed of left wheel
        right_speed = w * (radius + direction * self.axel_length / 2)  # speed of right wheel
        distance_goal = direction * radius * math.pi * angle / 180  # distance goal for robot

        # Caps the robots speed to max velocity, and leaves wiggle room for outer wheel to be faster
        if left_speed > self.max_motor_velocity:
            ratio = self.max_motor_velocity / left_speed
            left_speed *= ratio
            right_speed *= ratio
        if right_speed > self.max_motor_velocity:
            ratio = self.max_motor_velocity / right_speed
            left_speed *= ratio
            right_speed *= ratio
        
        # Records the initial time and encoder readings of the robot's wheels
        time_initial = self.experiment_supervisor.getTime()
        left_motor_initial_angle = self.get_left_motor_encoder_reading()
        right_motor_initial_angle = self.get_right_motor_encoder_reading()
        
        while self.experiment_supervisor.step(self.timestep) != -1:

            print("Max rotational motor velocity: ", self.max_motor_velocity)

            # Reads and Prints Robot's Encoder Readings
            print("Motor Encoder Readings: ", self.get_encoder_readings())

            # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
            print("Lidar Front Reading", self.get_lidar_range_image()[180])
            print("Lidar Left Reading", self.get_lidar_range_image()[90])
            print("Lidar Rear Reading", self.get_lidar_range_image()[0])
            print("Lidar Right Reading", self.get_lidar_range_image()[270])
            print("Simulation Time", self.experiment_supervisor.getTime())

            # Sets the robot's motor velocity
            self.set_left_motor_velocity(left_speed)
            self.set_right_motor_velocity(right_speed)

            # Calculates distance the wheel has turned since beginning of simulation
            distance_left_wheel_traveled = self.wheel_radius * (self.get_left_motor_encoder_reading() - left_motor_initial_angle)
            distance_right_wheel_traveled = self.wheel_radius * (self.get_right_motor_encoder_reading() - right_motor_initial_angle)
            travel_time = self.experiment_supervisor.getTime() - time_initial

            wheel_distance_average = (distance_left_wheel_traveled + distance_right_wheel_traveled) / 2
            print("Compass Reading", self.get_compass_reading())
            print("Distance Goal", distance_goal)
            print("Travel Distance", wheel_distance_average)

            # Error region based on speed
            error_region = 0.08 * speed * self.wheel_radius
            # Stops the robot after the robot moves a distance of 1.5 meters
            if distance_goal + error_region > wheel_distance_average > distance_goal - error_region:
                print(f"Time to turn {angle} degrees:{travel_time:.2f}")
                self.stop()
                break
