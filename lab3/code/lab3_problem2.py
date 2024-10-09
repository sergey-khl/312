#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: XXX
 
Brick Number: G2

Lab Number: 3

Problem Number: 2
 
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
from ev3dev2.motor import LargeMotor, SpeedPercent, OUTPUT_A, OUTPUT_B
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sensor.port import LegoPort
from ev3dev2.sensor import Sensor

class ArmPart(LargeMotor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

class Arm():
    def __init__(self, speed = 60, stop_action = "hold"):
        self.lower_arm = ArmPart(OUTPUT_A)
        self.upper_arm = ArmPart(OUTPUT_B)
        self.setStopAction(stop_action)
        self.setSpeed(speed)

        self.cam = LegoPort()
        self.cam.mode = 'auto'

        sleep(2)

        # calibrated config
        # assume we manually put in midpoint each time
        self.lower_arm.midpoint = self.lower_arm.position
        self.upper_arm.midpoint = self.upper_arm.position
        self.lower_arm.duty_cycle_sp = 0
        self.upper_arm.duty_cycle_sp = 0
        self.default_duty = 25

        # arm lengths in mm
        self.lower_arm.length = 160
        self.upper_arm.length = 130
        # self.touchSensor = TouchSensor()

        # inverse config
        self.newton_error = 1
        self.max_iter =  200
    
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

    def getPositionWithKnownAngles(self, theta_1, theta_2):
        x = self.lower_arm.length * cos(theta_1) + self.upper_arm.length * cos(theta_1 + theta_2)
        y = self.lower_arm.length * sin(theta_1) + self.upper_arm.length * sin(theta_1 + theta_2)
        return x, y
    
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
            print("d: ", d)
            theta_2_pos = atan(sqrt(1-d ** 2) / d)
            theta_1_pos = atan2(y, x) - atan((self.upper_arm.length*sin(theta_2_pos))/(self.lower_arm.length + self.upper_arm.length*cos(theta_2_pos)))
            theta_2_neg = pi - atan(-sqrt(1-d ** 2) / d)
            theta_1_neg = atan2(y, x) - atan((self.upper_arm.length*sin(theta_2_neg))/(self.lower_arm.length + self.upper_arm.length*cos(theta_2_neg)))
            if (abs(curr_theta_2 - theta_2_pos) <= abs(curr_theta_2 - theta_2_neg)):
                curr_theta_2 = theta_2_pos
                points[i] = [theta_1_pos, theta_2_pos]
                print(self.euclideanDistance(*self.getPositionWithKnownAngles(theta_1_pos, theta_2_pos), -100, -100), self.getPositionWithKnownAngles(theta_1_pos, theta_2_pos))
            else:
                curr_theta_2 = theta_2_neg
                points[i] = [theta_1_neg, theta_2_neg]
                print(self.euclideanDistance(*self.getPositionWithKnownAngles(theta_1_neg, theta_2_neg), -100, -100), self.getPositionWithKnownAngles(theta_1_neg, theta_2_neg))
            print("curr theta: ", curr_theta_2, "pos", theta_2_pos, "neg", theta_2_neg)

        # new angles
        return points
    
    def moveStraight(self, end_x, end_y):
        init_x, init_y = self.getPosition()
        print(init_x, init_y)
        points = self.createIntermediatePoints(init_x, init_y, end_x, end_y,  17)
        angles = self.analyticalSolve(points)

        self.lower_arm.run_direct()
        self.upper_arm.run_direct()
        curr_theta_1 = self.getAngleOfArm(self.lower_arm, True)
        curr_theta_2 = self.getAngleOfArm(self.upper_arm, True)
        for i, (theta_1, theta_2) in enumerate(angles):
            print("NEW ANGLE ", theta_1, theta_2)
            print("OLD ANGLE ", curr_theta_1, curr_theta_2)
            theta_1_slope = theta_1 - curr_theta_1
            theta_2_slope = theta_2 - curr_theta_2
            rate = theta_2_slope/theta_1_slope

            rate = max(self.default_duty/10, min(25/self.default_duty, abs(rate)))

            lower_speed = self.default_duty * copysign(1, theta_1_slope)
            upper_speed = self.default_duty * rate * copysign(1, theta_2_slope)
            print("rate", rate, "1slope", theta_1_slope, "2slope", theta_2_slope, "lower speed", lower_speed, "upper speed", upper_speed)
            self.lower_arm.duty_cycle_sp = lower_speed
            self.upper_arm.duty_cycle_sp = upper_speed

            while self.euclideanDistance(*self.getPosition(), *points[i]) > 5 and (self.lower_arm.duty_cycle != 0 or self.upper_arm.duty_cycle_sp != 0):
                print(self.euclideanDistance(*self.getPosition(), *points[i]), (self.getAngleOfArm(self.lower_arm, True), theta_1), (self.getAngleOfArm(self.upper_arm, True), theta_2))

                if abs(self.getAngleOfArm(self.lower_arm, True) - theta_1) < 0.1:
                    self.lower_arm.duty_cycle_sp = 0
                if abs(self.getAngleOfArm(self.upper_arm, True) - theta_2) < 0.1:
                    self.upper_arm.duty_cycle_sp = 0


            curr_theta_1 = theta_1
            curr_theta_2 = theta_2

        self.upper_arm.wait_while("running", 360000/self.upper_arm.speed_sp)
        self.lower_arm.wait_while("running", 360000/self.lower_arm.speed_sp)


    def velocity_kinematics(self, theta_1, theta_2):
        j_11 = self.upper_arm.length * cos(theta_1 + theta_2)
        j_12 = self.upper_arm.length * sin(theta_1 + theta_2)
        j_21 =  -self.lower_arm.length * cos(theta_1) - self.upper_arm.length * cos(theta_1 + theta_2)
        j_22 = - self.lower_arm.length * sin(theta_1) - self.upper_arm.length * sin(theta_1 + theta_2)
        determinant = self.lower_arm.length * self.upper_arm.length * sin(theta_2)
        

        if (sin(theta_2) == 0):
          ## what to do when singular configuration met, use a super small value ex: 1e-6
            determinant = 1e-2
            
        determinant_inverse = 1/determinant
        #perform matrix to get jacobian
        vel_kin = [
            [determinant_inverse * j_11, determinant_inverse * j_12],
            [determinant_inverse * j_21, determinant_inverse * j_22]
            ]

        return vel_kin

    def newtonApproach(self, target_x, target_y):
        theta_1 = self.getAngleOfArm(self.lower_arm, True)
        theta_2 = self.getAngleOfArm(self.upper_arm, True)

        angles = [] #initialize empty arrays

        for i in range(0, self.max_iter): ## or should we be slowly incrementing the x,y till we get to the desired location
            
            curr_x, curr_y = self.getPositionWithKnownAngles(theta_1, theta_2) # forward kinematics 
            print("round", i, "current location x", curr_x, ", current location y", curr_y)
            error_position = [[float(target_x - curr_x)],[float(target_y - curr_y)]]
            
            delta_pos = self.euclideanDistance(curr_x, curr_y, target_x, target_y)

            if(delta_pos < self.newton_error):
                break

            vel_kin = self.velocity_kinematics(theta_1, theta_2)
            print("error of the position:", error_position, "distance to end point", delta_pos)
            delta_theta = [
                vel_kin[0][0] * error_position[0][0] + vel_kin[0][1] * error_position[1][0],
                vel_kin[1][0] * error_position[0][0] + vel_kin[1][1] * error_position[1][0]
            ]

            max_change = 0.5

            if (delta_theta[0] > max_change):
                delta_theta[0] = max_change
            elif (delta_theta[0] < -max_change):
                delta_theta[0] = -max_change
                

            if (delta_theta[1] > max_change):
                delta_theta[1] = max_change
            elif (delta_theta[1] < -max_change):
                delta_theta[1] = -max_change


            theta_1 += delta_theta[0]
            theta_2 += delta_theta[1]
            
            angles.append([theta_1, theta_2])

            print("angles:", theta_1, ',', theta_2)
        return angles
        

if __name__ == "__main__":
    arm = Arm()

    poses = input("enter space separated desired x and y: ")
    x, y = map(int, poses.split())
    arm.moveStraight(x, y)
