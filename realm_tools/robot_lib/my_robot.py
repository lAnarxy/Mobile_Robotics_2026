from realm_tools.robot_lib.hambot import HamBot
import math

class MyRobot(HamBot):
    def __init__(self):
        HamBot.__init__(self)

    # Saturation function for motor speed
    def saturation(self, speed):
        P = 0.8
        if speed > P * self.max_motor_velocity:
            speed = P * self.max_motor_velocity
        elif speed < P * -self.max_motor_velocity:
            speed = P * -self.max_motor_velocity
        return speed

    # Drives the robot in a straight line for a specified distance at a specified speed (in rad/sec)
    def drive_straight(self, distance, speed):
        # Caps the robots speed to max velocity
        if speed > self.max_motor_velocity:
            speed = self.max_motor_velocity
        # Records the initial time and encoder readings of the robot's wheels
        time_initial = self.experiment_supervisor.getTime()
        left_motor_initial_angle = self.get_left_motor_encoder_reading()
        right_motor_initial_angle = self.get_right_motor_encoder_reading()
        while self.experiment_supervisor.step(self.timestep) != -1:

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

    # Turns the robot in place by a specified angle (in degrees) at a specified speed (in rad/sec)
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

    # Turns the robot on a circle of a specified radius by a specified angle (in degrees) at a specified speed (in rad/sec)
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

    # Moves the robot towards a specified distance using a PID controller based on the lidar reading
    def move_pid_lidar(self, distance = 1, Kp = 10.0, Ki = 0.1, Kd = 0.5, tolerance = 0.001):
        I = 0
        prev_error = 0
        while self.experiment_supervisor.step(self.timestep) != -1:
            error = self.get_lidar_range_image()[180] - distance

            print("Moving with PID Lidar:")
            print(f"Current Lidar Reading: {self.get_lidar_range_image()[180]:.4f}, Error: {error:.4f}")

            P = Kp * error
            I = Ki * (I + error * self.timestep)
            D = Kd * (error - prev_error) / self.timestep
            prev_error = error
            phi = self.saturation(P + I + D)

            print(f"P: {P:.4f}, I: {I:.4f}, D: {D:.4f}")
            print(f"PID output (phi): {phi:.4f}")

            self.set_left_motor_velocity(phi)
            self.set_right_motor_velocity(phi)
            if abs(error) < tolerance :
                self.stop()
                break

    # Moves the robot towards a specified distance using a PID controller based on the encoder reading
    def move_pid_encoder(self, distance = 1, Kp = 10.0, Ki = 0.1, Kd = 0.5, tolerance = 0.001):
        I = 0
        prev_error = 0
        angle_delta = 0
        initial = (self.get_left_motor_encoder_reading() + self.get_right_motor_encoder_reading()) / 2
        while self.experiment_supervisor.step(self.timestep) != -1:
            current = (self.get_left_motor_encoder_reading() + self.get_right_motor_encoder_reading()) / 2
            error = distance - (current - initial) * self.wheel_radius

            print("Moving with PID Encoder:")
            print(f"Moving {'forward' if error > 0 else 'backward'}")
            print(f"Traveled Distance: {(current - initial) * self.wheel_radius:.4f}, Error: {error:.4f}")

            P = Kp * error
            I = Ki * (I + error * self.timestep)
            D = Kd * (error - prev_error) / self.timestep
            prev_error = error
            phi = self.saturation(P + I + D)

            print(f"P: {P:.4f}, I: {I:.4f}, D: {D:.4f}")
            print(f"PID output (phi): {phi:.4f}")

            self.set_left_motor_velocity(phi)
            self.set_right_motor_velocity(phi)
            if abs(error) < tolerance :
                self.stop()
                break

    # Turns the robot to a specified angle using a PID controller based on the compass reading
    # (+degrees = counterclockwise, -degrees = clockwise)
    def turn_pid_compass(self, angle = 180, Kp = 1.0, Ki = 0.01, Kd = 0.05, tolerance = 0.001):
        I = 0
        prev_error = 0
        net_delta = 0
        prev_compass = self.get_compass_reading()
        while self.experiment_supervisor.step(self.timestep) != -1:
            delta = self.get_compass_reading() - prev_compass
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360
            net_delta += delta
            prev_compass = self.get_compass_reading()
            error = angle - net_delta

            print("Turning PID Compass:")
            print(f"Turning {'counterclockwise' if angle > 0 else 'clockwise'}")
            print(f"Current net delta: {net_delta:.4f}, Error: {error:.4f}")

            # Recalibrates Kp/i/d to account for wheel distance traveled
            # for easy compatibility with other PID functions
            r = (math.pi / 180) * self.robot_radius # Distance per compass degree turned
            P = Kp * r * error
            I = Ki * r * (I + error * self.timestep)
            D = Kd * r * (error - prev_error) / self.timestep
            prev_error = error
            phi = self.saturation(P + I + D)

            print(f"P: {P:.4f}, I: {I:.4f}, D: {D:.4f}")
            print(f"PID output (phi): {phi:.4f}")
            print(f"Left wheel speed: {-1 * phi:.4f}, Right wheel speed: {phi:.4f}")

            self.set_left_motor_velocity(-1 * phi)
            self.set_right_motor_velocity(phi)
            if abs(error) < tolerance :
                self.stop()
                break
    
    # Follows a wall on its left (side = 0) or right (side = 1) using a PID controller based on the lidar reading
    def wall_follow_pid(self, side = 0, Df = 1, Kpf = 10.0, Ds = 0.5, Kps = 5.0, tolerance = 0.001):
        while self.experiment_supervisor.step(self.timestep) != -1:
            error = self.get_lidar_range_image()[180] - Df
            phi = self.saturation(Kpf * error)
            side_distance = min(self.get_lidar_range_image()[80 + 180*side:100 + 180*side])
            side_error = side_distance - Ds
            if side_error < 0:
                delta_phi_l = Kps * side_error
                delta_phi_r = -Kps * side_error
            if side_error > 0:
                delta_phi_l = -Kps * side_error
                delta_phi_r = Kps * side_error
            else:
                delta_phi_l = 0
                delta_phi_r = 0
            self.set_left_motor_velocity(phi + delta_phi_l)
            self.set_right_motor_velocity(phi + delta_phi_r)
