#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: XXX
 
Brick Number: G2

Lab Number: 2

Problem Number: 1
 
Brief Program/Problem Description: 

	Need to draw out full workspace of the robot

Brief Solution Summary:

    XXX

Used Resources/Collaborators:
	https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html
    https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/sensors.html

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

from ev3dev2.motor import LargeMotor, SpeedPercent, OUTPUT_A, OUTPUT_B

class Arm():
    def __init__(self):
        self.lower_arm = LargeMotor(OUTPUT_A)
        self.upper_arm = LargeMotor(OUTPUT_B)
        self.lower_arm.speed_sp = 15
        self.lower_arm.stop_action = "brake"
        self.upper_arm.speed_sp = 15
        self.upper_arm.stop_action = "brake"

        # calibrated ends
        self.lower_arm_lower_bound = 41316
        self.lower_arm_upper_bound = 41534
        self.lower_arm_midpoint = 41425
    
    def logPositions(self):
        print("lower angle", self.getAngle(self.lower_arm))
        print("upper angle", self.getAngle(self.upper_arm))
    
    def getAngle(self, arm):
        return arm.position % arm.count_per_rot

    def getPosition(self, arm):
        return arm.position
    
    def getLowerArm(self):
        return self.lower_arm

    def getUpperArm(self):
        return self.upper_arm

    def stop(self):
        self.lower_arm.stop()
        self.upper_arm.stop()

    def moveArm(self, arm, speed_percent):
        arm.on(SpeedPercent(speed_percent))
        return self.getPosition(arm)

    # not necessarily true midpoint just in current context
    # will move the motor to the middle
    def findBounds(self, arm, length):
        # find upper bound
        for i in range(length):
            upper_bound = self.moveArm(arm, 15)

        # find lower bound
        for i in range(length):
            lower_bound = self.moveArm(arm, -15)

        midpoint = (lower_bound + upper_bound) // 2
        arm.position_sp = midpoint

        # move to the middle
        arm.run_to_abs_pos()
        arm.wait_while("running")

        return lower_bound, upper_bound, midpoint

    def calibrate(self):
        self.findBounds(self.upper_arm, 350)

        lower_bound_lower_arm, upper_bound_lower_arm, midpoint_lower_arm = self.findBounds(self.lower_arm, 350)

        lower_bound_lower_arm += 15
        upper_bound_lower_arm -= 68
        midpoint_lower_arm = (lower_bound_lower_arm + upper_bound_lower_arm) // 2

        lower_bound_upper_arm, upper_bound_upper_arm, midpoint_upper_arm = self.findBounds(self.upper_arm, 350)

        self.stop()

    def workspace(self):
        self.lower_arm.position_sp = self.lower_arm_lower_bound

        # move to the middle
        self.lower_arm.run_to_abs_pos()
        self.lower_arm.wait_while("running")

        self.lower_arm.position_sp = self.lower_arm_upper_bound

        # move to the middle
        self.lower_arm.run_to_abs_pos()
        self.lower_arm.wait_while("running")

        self.lower_arm.position_sp = self.lower_arm_midpoint

        # move to the middle
        self.lower_arm.run_to_abs_pos()
        self.lower_arm.wait_while("running")

if __name__ == "__main__":
    arm = Arm()
    arm.workspace()