#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: 2024-10-06
 
Brick Number: G2

Lab Number: 2

Problem Number: 2
 
Brief Program/Problem Description: 

	Forward kinematics to find position from joint angles.
    Use this to move joints with theta to find the end effector position.
    Then record points and find the distance between them. Then record points
    and find angle between them.

Brief Solution Summary:

    Used simple forward kinematics equations found in the notes. Always position the robot
    before each run to set that as the origin. This way, when we move the motors we will know how
    far in radians it has moved from the center point. This will give us the position.

    Implementing part c was done with a button and distance was calculated with euclidean method
    and angle was found using the tangent of the slopes of the 2 lines.

Used Resources/Collaborators:
	https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html
    https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/sensors.html
    http://ugweb.cs.ualberta.ca/~vis/courses/robotics/assign/a2Arm/lab2Notes.pdf

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

import sys
from math import pi, cos, sin, sqrt, atan2
from time import sleep
from ev3dev2.motor import LargeMotor, SpeedPercent, OUTPUT_A, OUTPUT_B
from ev3dev2.sensor.lego import TouchSensor

class ArmPart(LargeMotor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

class Arm():
    def __init__(self, speed = 60, stop_action = "hold"):
        self.lower_arm = ArmPart(OUTPUT_A)
        self.upper_arm = ArmPart(OUTPUT_B)
        self.setStopAction(stop_action)
        self.setSpeed(speed)

        # calibrated config
        self.lower_arm.lower_bound = -127
        self.lower_arm.upper_bound = 75
        # assume we manually put in midpoint each time
        self.lower_arm.midpoint = self.lower_arm.position
        self.upper_arm.lower_bound = -43
        self.upper_arm.upper_bound = 209
        self.upper_arm.midpoint = self.upper_arm.position

        # arm lengths in mm
        self.lower_arm.length = 160
        self.upper_arm.length = 130
        self.touchSensor = TouchSensor()

        # move to initial position
        self.moveArmsAbsolute(self.lower_arm.midpoint, self.upper_arm.midpoint)
    
    def __del__(self):
        self.setStopAction("coast")
        self.stop()
    
    def setStopAction(self, stop_action):
        self.lower_arm.stop_action = stop_action
        self.upper_arm.stop_action = stop_action

    def setSpeed(self, speed):
        self.lower_arm.speed_sp = speed
        self.upper_arm.speed_sp = speed

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
        print(lower_arm_angle, upper_arm_angle)
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
        self.lower_arm.wait_while("running", 360000/self.lower_arm.speed_sp)
        self.upper_arm.wait_while("running", 360000/self.upper_arm.speed_sp)
        
    def moveWithTheta(self, lower_angle, upper_angle):
        print(self.getPosition())
        print(self.getAngleOfArm(self.lower_arm), self.getAngleOfArm(self.upper_arm))
        self.moveArmsAbsolute(self.lower_arm.midpoint + lower_angle, self.upper_arm.midpoint + upper_angle)
        print(self.getPosition())
        print(self.getAngleOfArm(self.lower_arm), self.getAngleOfArm(self.upper_arm))
    
    def euclideanDistance(self, init_x, init_y, end_x, end_y):
        return sqrt((end_x - init_x) ** 2 + (end_y - init_y) ** 2)

    def recordLength(self):
        self.setStopAction("coast")
        self.stop()
        while not self.touchSensor.is_pressed:
            print("move the arm and press button")
            sleep(1)
            
        self.setStopAction("hold")
        self.stop()
        x, y = self.getPosition()
        print("got position:", x, y)
        
        sleep(2)

        return x, y

    def findDistanceBetweenPoints(self):
        first_x, first_y = self.recordLength()

        second_x, second_y = self.recordLength()

        distance = self.euclideanDistance(first_x, first_y, second_x, second_y)
        print("distance between points is:" , distance, "mm")

    def findAngleBetweenPoints(self):
        intersect_x, intersect_y = self.recordLength()

        first_x, first_y = self.recordLength()

        second_x,second_y = self.recordLength()

        try:
            first_x_diff = first_x - intersect_x
            first_y_diff = first_y - intersect_y
            second_x_diff = second_x - intersect_x
            second_y_diff = second_y - intersect_y

            angle = atan2(second_y_diff*first_x_diff-second_x_diff*first_y_diff, second_x_diff*first_x_diff+second_y_diff*first_y_diff)
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
        x, y = map(int, input("enter theta_lower_arm theta_upper_arm space separated to move to: ").split())
        arm.moveWithTheta(x, y)
    elif input_arg == "c_dist":
        arm.findDistanceBetweenPoints()
    elif input_arg == "c_angle":
        arm.findAngleBetweenPoints()