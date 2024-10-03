#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: XXX
 
Brick Number: G2

Lab Number: 2

Problem Number: 2
 
Brief Program/Problem Description: 

	inverse kinematics to move end effector to position

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
from math import pi, cos, sin, sqrt, atan, acos
# from ev3dev2.motor import LargeMotor, SpeedPercent, OUTPUT_A, OUTPUT_B

class TempArm():
    pass

# class ArmPart(LargeMotor):
#     def __init__(self, *args, **kwargs):
#         super().__init__(*args, **kwargs)
class TempMotor():
    lower_arm = TempArm()
    upper_arm = TempArm()

    def stop(self):
        pass

    def run_to_abs_pos(self):
        self.position = self.position_sp

    def wait_while(self, m):
        pass

class Arm():
    def __init__(self, speed = 60, stop_action = "hold"):
        # self.lower_arm = ArmPart(OUTPUT_A)
        # self.upper_arm = ArmPart(OUTPUT_B)
        self.lower_arm = TempMotor()
        self.upper_arm = TempMotor()
        self.setStopAction(stop_action)
        self.setSpeed(speed)

        # calibrated config
        self.lower_arm.lower_bound = 41316
        self.lower_arm.upper_bound = 41534
        self.lower_arm.midpoint = 41425

        self.upper_arm.lower_bound = 41316
        self.upper_arm.upper_bound = 41534
        self.upper_arm.midpoint = 41425

        # arm lengths in mm
        self.lower_arm.length = 160
        self.upper_arm.length = 110

        # inverse config
        self.newton_error = 0.01

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

    # will convert given theta from radians to a pos in degrees
    def getAbsPosFromTheta(self, arm, theta):
        return arm.midpoint + self.getDegFromRad(theta)
    
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
        
        return x, y

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
            # theta_2 = atan(sqrt(1-d ** 2) / d)
            theta_2 = acos(d)
            if (abs(curr_theta_2 - theta_2) < abs(curr_theta_2 + theta_2)):
                curr_theta_2 = theta_2
            else:
                curr_theta_2 = -theta_2
            
            points[i] = [atan(y/x) - atan((self.upper_arm.length*sin(curr_theta_2))/(self.lower_arm.length + self.upper_arm.length*cos(curr_theta_2))), curr_theta_2]
        
        return points
    
    def moveToPos(self, type_arg, end_x, end_y):
        init_x, init_y = self.getPosition()
        print(self.getPosition())

        points = self.createIntermediatePoints(init_x, init_y, end_x, end_y, max(1, int(self.euclideanDistance(init_x, init_y, end_x, end_y) // 10)))
        if type_arg == "anal":
            angles = self.analyticalSolve(points)

        for theta_1, theta_2 in angles:
            self.moveArmsAbsolute(self.getAbsPosFromTheta(self.lower_arm, theta_1), self.getAbsPosFromTheta(self.upper_arm, theta_2))
        print(self.getPosition())

    def moveToMid(self, type_arg):
        pass
        

if __name__ == "__main__":
    arm = Arm()
    if len(sys.argv) != 3:
        print("Error: Exactly two arguments (pos/mid) and (anal/num) are required.")
        sys.exit(1)
    
    prog_arg = sys.argv[1]
    type_arg = sys.argv[2]

    if prog_arg not in ["pos", "mid"]:
        print("Error: Argument must be 'pos' or 'mid'.")
        sys.exit(1)

    if type_arg not in ["anal", "num"]:
        print("Error: Argument must be 'anal' or 'num'.")
        sys.exit(1)

    if prog_arg == "pos":
        poses = input("enter space separated desired x and y: ")
        x, y = map(int, poses.split())
        arm.moveToPos(type_arg, x, y)
    else:
        arm.moveToMid(type_arg)