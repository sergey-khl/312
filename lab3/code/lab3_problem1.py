#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: XXX
 
Brick Number: G2

Lab Number: 3

Problem Number: 1
 
Brief Program/Problem Description: 

    XXX

Brief Solution Summary:

    XXX

Used Resources/Collaborators:
	https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html
    https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/sensors.html

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

import sys
from time import sleep
from math import pi, cos, sin, sqrt, atan, acos, atan2, copysign
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B

class ArmPart(LargeMotor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

class Arm():
    def __init__(self, default_speed = 15, stop_action = "hold"):
        self.lower_arm = ArmPart(OUTPUT_A)
        self.upper_arm = ArmPart(OUTPUT_B)
        self.setStopAction(stop_action)
        self.setSpeed(0, 0)

        # calibrated config
        # assume we manually put in midpoint each time
        self.lower_arm.midpoint = self.lower_arm.position
        self.upper_arm.midpoint = self.upper_arm.position
        self.lower_arm.default_duty = default_speed
        self.upper_arm.default_duty = default_speed

        # arm lengths in mm
        self.lower_arm.length = 160
        self.upper_arm.length = 85
    
    def __del__(self):
        self.setStopAction("coast")
        self.stop()

    def setStopAction(self, stop_action):
        self.lower_arm.stop_action = stop_action
        self.upper_arm.stop_action = stop_action

    def setSpeed(self, speed_lower, speed_upper):
        self.lower_arm.duty_cycle_sp = speed_lower
        self.upper_arm.duty_cycle_sp = speed_upper

    def getRadFromDeg(self, deg):
        return deg*pi/180

    def getAngleOfArm(self, arm, radians=False):
        angle = arm.position - arm.midpoint
        return self.getRadFromDeg(angle) if radians else angle
    
    # of end effector
    def getPosition(self):
        lower_arm_angle = self.getAngleOfArm(self.lower_arm, True)
        upper_arm_angle = self.getAngleOfArm(self.upper_arm, True)
        x = self.lower_arm.length * cos(lower_arm_angle) + self.upper_arm.length * cos(lower_arm_angle + upper_arm_angle)
        y = self.lower_arm.length * sin(lower_arm_angle) + self.upper_arm.length * sin(lower_arm_angle + upper_arm_angle)
        return x, y

    def getPositionWithKnownAngles(self, theta_1, theta_2):
        x = self.lower_arm.length * cos(theta_1) + self.upper_arm.length * cos(theta_1 + theta_2)
        y = self.lower_arm.length * sin(theta_1) + self.upper_arm.length * sin(theta_1 + theta_2)
        return x, y
    
    def stop(self):
        self.lower_arm.stop()
        self.upper_arm.stop()


    def createIntermediatePoints(self, init_x, init_y, end_x, end_y, num_points):
        delta_x = (end_x - init_x)/(num_points)
        delta_y = (end_y - init_y)/(num_points)

        points = [None] * num_points
        for i in range(1, num_points + 1):
            points[i - 1] = [init_x+delta_x*i, init_y+delta_y*i]
            
        return points

    def euclideanDistance(self, init_x, init_y, end_x, end_y):
        return sqrt((end_x - init_x) ** 2 + (end_y - init_y) ** 2)
    
    def analyticalSolve(self, points):
        curr_theta_2 = self.getAngleOfArm(self.upper_arm, True)
        for i, (x, y) in enumerate(points):
            d = (x ** 2 + y ** 2 - self.lower_arm.length ** 2 - self.upper_arm.length ** 2)/(2*self.lower_arm.length*self.upper_arm.length)
            theta_2_pos = atan2(sqrt(1-d ** 2), d)
            theta_1_pos = atan2(y, x) - atan2(self.upper_arm.length*sin(theta_2_pos), self.lower_arm.length + self.upper_arm.length*cos(theta_2_pos))
            theta_2_neg = atan2(-sqrt(1-d ** 2), d)
            theta_1_neg = atan2(y, x) - atan2(self.upper_arm.length*sin(theta_2_neg), self.lower_arm.length + self.upper_arm.length*cos(theta_2_neg))
            if (abs(curr_theta_2 - theta_2_pos) <= abs(curr_theta_2 - theta_2_neg)):
                curr_theta_2 = theta_2_pos
                points[i] = [theta_1_pos, theta_2_pos]
            else:
                curr_theta_2 = theta_2_neg
                points[i] = [theta_1_neg, theta_2_neg]

        # new angles
        return points
    
    def moveStraight(self, end_x, end_y):
        init_x, init_y = self.getPosition()
        points = self.createIntermediatePoints(init_x, init_y, end_x, end_y,  17)
        print(points)
        print("============================")
        angles = self.analyticalSolve(points)

        curr_theta_1 = self.getAngleOfArm(self.lower_arm, True)
        curr_theta_2 = self.getAngleOfArm(self.upper_arm, True)
        self.lower_arm.run_direct()
        self.upper_arm.run_direct()
        for i, (theta_1, theta_2) in enumerate(angles):
            theta_1_delta = theta_1 - curr_theta_1
            theta_2_delta = theta_2 - curr_theta_2

            lower_speed = self.lower_arm.default_duty * copysign(1, theta_1_delta)
            upper_speed = self.upper_arm.default_duty * copysign(1, theta_2_delta)
            self.setSpeed(lower_speed, upper_speed)

            count = 0
            while self.euclideanDistance(*self.getPosition(), *points[i]) > 5 and (self.lower_arm.duty_cycle_sp != 0 or self.upper_arm.duty_cycle_sp != 0):
                if copysign(1, theta_1_delta) == copysign(1, self.getAngleOfArm(self.lower_arm, True) - theta_1):
                    lower_speed = 0
                else:
                    # make more powerful if not getting to correct position
                    lower_speed = (self.lower_arm.default_duty + count // 5) * copysign(1, theta_1_delta)
                if copysign(1, theta_2_delta) == copysign(1, self.getAngleOfArm(self.upper_arm, True) - theta_2):
                    upper_speed = 0
                else:
                    # make more powerful if not getting to correct position
                    upper_speed = (self.upper_arm.default_duty + count // 5) * copysign(1, theta_2_delta)
                count += 1
                self.setSpeed(lower_speed, upper_speed)

            curr_theta_1 = theta_1
            curr_theta_2 = theta_2
            print(self.getPosition())

        self.stop()

if __name__ == "__main__":
    arm = Arm()

    poses = input("enter space separated desired x and y: ")
    x, y = map(int, poses.split())
    arm.moveStraight(x, y)
