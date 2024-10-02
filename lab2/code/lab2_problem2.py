#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: XXX
 
Brick Number: G2

Lab Number: 2

Problem Number: 2
 
Brief Program/Problem Description: 

	Forward kinematics to find position from joint angles

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
from math import pi, cos, sin, sqrt, atan
import time
from ev3dev2.motor import LargeMotor, SpeedPercent, OUTPUT_A, OUTPUT_B
from ev3dev2.sensor.lego import TouchSensor

class ArmPart(LargeMotor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

class Arm():
    def __init__(self, speed = 60, stop_action = "coast"):
        self.lower_arm = ArmPart(OUTPUT_A)
        self.upper_arm = ArmPart(OUTPUT_B)
        # self.lower_arm = TempMotor()
        # self.upper_arm = TempMotor()
        self.lower_arm.speed_sp = speed
        self.lower_arm.stop_action = stop_action
        self.upper_arm.speed_sp = speed
        self.upper_arm.stop_action =stop_action

        # calibrated config
        self.lower_arm.lower_bound = -157
        self.lower_arm.upper_bound = 40
        self.lower_arm.midpoint = -59
        self.upper_arm.lower_bound = -123
        self.upper_arm.upper_bound = 83
        self.upper_arm.midpoint = -20

        # arm lengths in mm
        self.lower_arm.length = 160
        self.upper_arm.length = 110
        self.touchSensor = TouchSensor()

        # move to initial position
        self.moveArmsAbsolute(self.lower_arm.midpoint, self.upper_arm.midpoint)
    
    def __del__(self):
        self.stop()

    def getRadFromDeg(self, deg):
        return deg*pi/180

    def getDegFromRad(self, rad):
        return rad*180/pi

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
    
    def getLowerArm(self):
        return self.lower_arm

    def getUpperArm(self):
        return self.upper_arm

    def stop(self):
        self.lower_arm.stop()
        self.upper_arm.stop()

    def moveArmsAbsolute(self, lower_pos, upper_pos):
        self.lower_arm.position_sp = lower_pos
        self.upper_arm.position_sp = upper_pos
        self.lower_arm.run_to_abs_pos()
        self.upper_arm.run_to_abs_pos()
        self.lower_arm.wait_while("running")
        self.upper_arm.wait_while("running")
        
    def moveWithTheta(self, lower_angle, upper_angle):
        print(self.getPosition())
        self.moveArmsAbsolute(self.lower_arm.midpoint + lower_angle, self.upper_arm.midpoint + upper_angle)
        print(self.getPosition())
    
    def euclideanDistance(self, init_x, init_y, end_x, end_y):
        return sqrt((end_x - init_x) ** 2 + (end_y - init_y) ** 2)

    def recordLength(self):
        while not self.touchSensor.is_pressed:
            print("move the arm and press button")
            time.sleep(1)
            
        x, y = self.getPosition()
        print("got position:", x, y)
        
        return x, y

    def findDistanceBetweenPoints(self):
        # TODO: change this to touch sensor entering points
        first_x, first_y = self.recordLength()

        time.sleep(2)

        second_x, second_y = self.recordLength()

        distance = self.euclideanDistance(first_x, first_y, second_x, second_y)
        print("distance between points is:" , distance, "mm")

    def findAngleBetweenPoints(self):
        intersect_x, intersect_y = self.recordLength()

        first_x, first_y = self.recordLength()

        second_x,second_y = self.recordLength()

        try:
            first_slope = (first_y - intersect_y)/(first_x - intersect_x)
            print("first", first_slope)
            second_slope = (second_y - intersect_y)/(second_x - intersect_x)
            print("second", second_slope)
            angle = atan(abs(first_slope - second_slope)/(1+first_slope*second_slope))
        except:
            print("division by zero. try some other points")
        print("angle between lines is:" , angle, "radians or ", self.getDegFromRad(angle), "degrees")


if __name__ == "__main__":
    arm = Arm()
    if len(sys.argv) != 2:
        print("Error: Exactly one argument (b/c_dist/c_angle) is required.")
        sys.exit(1)
    
    input_arg = sys.argv[1]

    if input_arg not in ["b", "c_dist", "c_angle"]:
        print("Error: Argument must be 'b' or 'c_dist' or 'c_angle'.")
        sys.exit(1)

    if input_arg == "b":
        arm.moveWithTheta(-10, 35)
    elif input_arg == "c_dist":
        arm.findDistanceBetweenPoints()
    elif input_arg == "c_angle":
        arm.findAngleBetweenPoints()