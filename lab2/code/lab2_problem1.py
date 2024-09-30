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
    def __init__(self, speed = 15, stop_action = "brake"):
        self.lower_arm = LargeMotor(OUTPUT_A)
        self.upper_arm = LargeMotor(OUTPUT_B)
        self.lower_arm.speed_sp = speed
        self.lower_arm.stop_action = stop_action
        self.upper_arm.speed_sp = speed
        self.upper_arm.stop_action =stop_action

        # calibrated config
        self.lower_arm_lower_bound = 41316
        self.lower_arm_upper_bound = 41534
        self.lower_arm_midpoint = 41425

        self.upper_arm_lower_bound = 41316
        self.upper_arm_upper_bound = 41534
        self.upper_arm_midpoint = 41425
    
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

    # this is blocking due to the wait while
    def moveArmRelative(self, arm, pos):
        arm.position_sp = pos
        # move to the middle
        arm.run_to_rel_pos()
        arm.wait_while("running")
        return self.getPosition(arm)

    # this is blocking due to the wait while
    def moveArmAbsolute(self, arm, pos):
        arm.position_sp = pos
        # move to the middle
        arm.run_to_abs_pos()
        arm.wait_while("running")
        return self.getPosition(arm)

    # not necessarily true midpoint just in current context
    # will move the motor to the middle
    def findBounds(self, arm, length):
        # find upper bound
        upper_bound = self.moveArmRelative(arm, 90)

        # find lower bound
        lower_bound = self.moveArmRelative(arm, -90)

        midpoint = (lower_bound + upper_bound) // 2

        # move to the middle
        self.moveArmAbsolute(arm, midpoint)

        return lower_bound, upper_bound, midpoint

    def calibrate(self):
        self.findBounds(self.upper_arm)

        lower_bound_lower_arm, upper_bound_lower_arm, midpoint_lower_arm = self.findBounds(self.lower_arm)

        lower_bound_lower_arm += 15
        upper_bound_lower_arm -= 68
        midpoint_lower_arm = (lower_bound_lower_arm + upper_bound_lower_arm) // 2

        lower_bound_upper_arm, upper_bound_upper_arm, midpoint_upper_arm = self.findBounds(self.upper_arm)

        self.stop()
        self.lower_arm_lower_bound = lower_bound_lower_arm
        self.lower_arm_upper_bound = upper_bound_lower_arm
        self.lower_arm_midpoint = midpoint_lower_arm

        self.upper_arm_lower_bound = lower_bound_upper_arm
        self.upper_arm_upper_bound = upper_bound_upper_arm
        self.upper_arm_midpoint = midpoint_upper_arm

        print("lower arm lower bound", self.lower_arm_lower_bound)
        print("lower arm upper bound", self.lower_arm_upper_bound)
        print("lower arm midpoint", self.lower_arm_midpoint)

        print("upper arm lower bound", self.upper_arm_lower_bound)
        print("upper arm upper bound", self.upper_arm_upper_bound)
        print("upper arm midpoint", self.upper_arm_midpoint)

    def workspace(self):
        # move to the middle first
        self.moveArmAbsolute(self.lower_arm, self.lower_arm_midpoint)
        self.moveArmAbsolute(self.upper_arm, self.upper_arm_midpoint)

        # go to lower bound
        self.moveArmAbsolute(self.lower_arm, self.lower_arm_lower_bound)
        self.moveArmAbsolute(self.upper_arm, self.upper_arm_lower_bound)

        self.moveArmAbsolute(self.upper_arm, self.upper_arm_midpoint)

        # go to upper bound
        self.moveArmAbsolute(self.lower_arm, self.lower_arm_upper_bound)
        self.moveArmAbsolute(self.upper_arm, self.upper_arm_upper_bound)

if __name__ == "__main__":
    arm = Arm()
    arm.calibrate()
    arm.workspace()