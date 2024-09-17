#!/usr/bin/env python3

from time import sleep
from math import cos, sin

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, SpeedRPM, SpeedPercent, MoveTank

class Robo(MoveTank):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.max_rot_sec = 2.13
        self.d = 9.75
        self.integration_steps = 10

    def logEncodings(self):
        # print("angle and rate", self.gyro.angle_and_rate)
        # print("angle", self.gyro.angle)
        # print("circle angle", self.gyro.circle_angle())
        # print("previous deg", self.previous_degrees)
        print("left motor position", str(self.left_motor.rotations))
        print("right motor position", str(self.right_motor.rotations))

    def findDistance(self, velocity, angular_velocity, seconds):
        total_x = 0
        total_y = 0
        orientation = 0
        delta_angular_velocity = angular_velocity / self.integration_steps
        t_1= 0
        delta_sec = seconds / self.integration_steps
        for i in range(self.integration_steps):
            orientation += delta_angular_velocity 
            t_2 = t_1 + delta_sec
            x = t_2 *velocity * cos(delta_angular_velocity) - t_1*velocity*cos()
            total_y += velocity * sin(orientation)

        return total

    def logPosition(self, speed_left, speed_right, seconds):
        # find velocities of each wheel
        v_left = speed_left/100 * self.max_rot_sec
        v_right = speed_right/100 * self.max_rot_sec

        # turning on angle
        if (v_left != v_right):
            angular_velocity = (v_right - v_left)/(2*self.d)
            rotation_radius = self.d*(v_right + v_left)/(v_right - v_left)

            velocity = rotation_radius * angular_velocity
        # straight line
        else:
            velocity = v_left
            angular_velocity = 0

        distance_x, distance_y, orientation = self.findDistance(velocity, angular_velocity, seconds)

        print(distance_x, distance_y, orientation)
        # print(distance_x, distance_y, )

    def go(self, speed_left, speed_right, seconds):
        print("going straight")
        self.on_for_seconds(speed_left, speed_right, seconds)

    def moveDeadReckoning(self, commands):
        for command in commands:
            self.go(*command)
            self.logPosition(*command)

if __name__ == "__main__":
    robo = Robo(OUTPUT_B, OUTPUT_A)
    # dead reckoning commands
    commands = [
        [80, 60, 2],
        [60, 60, 1],
        [-50, 80, 2],
    ]
    robo.logEncodings()
    robo.moveDeadReckoning(commands)
    robo.logEncodings()
