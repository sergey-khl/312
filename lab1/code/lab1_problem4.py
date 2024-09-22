#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: 2024/09/22
 
Brick Number: G2

Lab Number: 1

Problem Number: 4
 
Brief Program/Problem Description: 

    Need to perform dead reckoning. After running ./lab1_problem4.py add the commands
    in groups of 3 for the input, and add "done" when finished adding commands.
    If no commands are sent then the default commands from the lab are used.

Brief Solution Summary:

	used MoveTank and used the provided notes to perform the dead reckoning
    We measured the max wheel rotation speed using the encoders and a ruler and hard coded the value here
    We also hard coded the integration steps at 10. More info can be found in the report.

Used Resources/Collaborators:
	https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html
    https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/sensors.html
    http://ugweb.cs.ualberta.ca/~vis/courses/robotics/assign/a1MobRob/lab1_notes.jpg

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

from time import sleep
from math import cos, sin, pi

from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveTank

class Robot(MoveTank):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.max_rot_sec = 2.13
        self.d = 99
        self.integration_steps = 10
        self.radius = 35
        self.gyro = GyroSensor()
        self.previous_degrees = 0
        self.recalibrate()

    def logEncodings(self):
        print("angle", self.gyro.angle)
        print("left motor distance traveled", str(self.left_motor.rotations * 2 * self.radius * pi))
        print("right motor distance traveled", str(self.right_motor.rotations * 2 * self.radius * pi))

    def findDistance(self, velocity, angular_velocity, seconds, prev_x, prev_y, prev_orientation):
        x = prev_x
        y = prev_y
        orientation_1 = 0
        orientation_2 = 0
        t_1 = 0
        t_2 = 0
        delta_angular_velocity = angular_velocity / self.integration_steps
        delta_sec = seconds / self.integration_steps
        
        for i in range(self.integration_steps):
            if(angular_velocity):
                orientation_1 = prev_orientation + delta_angular_velocity * i
                orientation_2 = prev_orientation + delta_angular_velocity * (i+1)
            else:
                orientation_1 = prev_orientation
                orientation_2 = prev_orientation
            t_1 = delta_sec * i
            t_2 = delta_sec * (i+1)
            x += t_2 * velocity * cos(orientation_2) - t_1 * velocity * cos(orientation_1)
            y += t_2 * velocity * sin(orientation_2) - t_1 * velocity * sin(orientation_1)
            
            # print(f"x: {x}, y: {y}, orientation: {orientation_2}")

        return x, y, orientation_2

    def logPosition(self, speed_left, speed_right, seconds, prev_x, prev_y, prev_orientation):
        # find velocities of each wheel
        v_left = speed_left / 100 * self.radius * 2 * pi * self.max_rot_sec
        v_right = speed_right / 100 * self.radius * 2 * pi * self.max_rot_sec

        # turning on angle
        if v_left != v_right:
            angular_velocity = (v_right - v_left) / (2 * self.d)
            rotation_radius = self.d * (v_right + v_left) / (v_right - v_left)

            velocity = rotation_radius * angular_velocity
        # straight line
        else:
            velocity = v_left
            angular_velocity = 0
            rotation_radius = None


        distance_x, distance_y, orientation = self.findDistance(
            velocity, angular_velocity, seconds, prev_x, prev_y, prev_orientation
        )

        # print(f"Distance X: {distance_x}, Distance Y: {distance_y}, Orientation: {orientation}")
        print("ESTIMATED:", "v_left", v_left, "v_right", v_right, "velocity", velocity, "rotation_radius", rotation_radius, "angular_velocity", angular_velocity, "distance x", distance_x, "distance y", distance_y, "orientation", orientation)

        return distance_x, distance_y, orientation

    def go(self, speed_left, speed_right, seconds):
        # print(f"Going: Left Speed {speed_left}, Right Speed {speed_right}, for {seconds} seconds")
        print("BEFORE: going left speed", speed_left, "right speed", speed_right, "for", seconds)
        self.logEncodings()
        self.on_for_seconds(speed_left, speed_right, seconds)
        print("AFTER: going left speed", speed_left, "right speed", speed_right, "for", seconds)
        self.logEncodings()

    def moveDeadReckoning(self, commands):
        if commands == []:
            # default commands
            commands = [
                [80, 60, 2],
                [60, 60, 1],
                [-50, 80, 2],
            ]
        prev_x = 0
        prev_y = 0
        prev_orientation = 0
        for command in commands:
            self.go(*command)
            distance_x, distance_y, orientation = self.logPosition(*command, prev_x, prev_y, prev_orientation)
            prev_x = distance_x
            prev_y = distance_y
            prev_orientation = orientation

    def recalibrate(self):
        self.gyro.reset()
        self.gyro.calibrate()
        sleep(1)

if __name__ == "__main__":
    robo = Robot(OUTPUT_B, OUTPUT_A)
    # dead reckoning commands
    commands = []
    while True:
        user_input = input("Enter 3 space-separated numbers (or type 'done' to exit): ")
        
        if user_input.lower() == "done":
            print("running commands")
            break

        numbers = user_input.split()

        if len(numbers) != 3:
            print("Error: Please enter exactly 3 numbers.")
            continue

        commands.append([numbers])
    robo.moveDeadReckoning(commands)