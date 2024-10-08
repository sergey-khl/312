#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: 2024-10-06
 
Brick Number: G2

Lab Number: 2

Problem Number: 1
 
Brief Program/Problem Description: 

	Need to draw out full workspace of the robot.

Brief Solution Summary:

    First created a calibration function to find the upper and lower bounds of the robot. This was adjusted a little bit
    to make sure the robot is able to use its full range of motion. Then a workspace is drawn out to see where the robot can draw.

Used Resources/Collaborators:
	https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

import sys
from ev3dev2.motor import LargeMotor, SpeedPercent, OUTPUT_A, OUTPUT_B

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
        self.lower_arm.lower_bound = -24
        self.lower_arm.upper_bound = 173
        self.lower_arm.midpoint = 93
        self.upper_arm.lower_bound = -97
        self.upper_arm.upper_bound = 161
        self.upper_arm.midpoint = 12
    
    def __del__(self):
        self.setStopAction("coast")
        self.stop()

    def setStopAction(self, stop_action):
        self.lower_arm.stop_action = stop_action
        self.upper_arm.stop_action = stop_action

    def setSpeed(self, speed):
        self.lower_arm.speed_sp = speed
        self.upper_arm.speed_sp = speed
    
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
    def moveArmAbsolute(self, arm, pos):
        arm.position_sp = pos
        # move to the middle
        arm.run_to_abs_pos()
        arm.wait_while("running", 360000/arm.speed_sp)
        return self.getPosition(arm)

    # not necessarily true midpoint just in current context
    # will move the motor to the middle
    def findBounds(self, arm, adj=True):
        # find upper bound
        upper_bound = self.moveArmAbsolute(arm, arm.position + 360)
        if adj:
            upper_adj = int(input("how many deg do you want to adjust the upper bound (enter 0 to stop): "))
            while upper_adj != 0:
                self.moveArmAbsolute(arm, self.getPosition(arm) - upper_adj)
                upper_adj = int(input("how many deg do you want to adjust the upper bound (enter 0 to stop): "))
            # find new upper bound
            upper_bound = self.getPosition(arm)

        # find lower bound
        lower_bound = self.moveArmAbsolute(arm, arm.position - 360)
        if adj:
            lower_adj = int(input("how many deg do you want to adjust the lower bound(enter 0 to stop): "))
            while lower_adj != 0:
                self.moveArmAbsolute(arm, self.getPosition(arm) + lower_adj)
                lower_adj = int(input("how many deg do you want to adjust the lower bound(enter 0 to stop): "))
            # total how much we moved for lower bound
            lower_bound = self.getPosition(arm)

        midpoint = (lower_bound + upper_bound) // 2

        # move to the middle
        self.moveArmAbsolute(arm, midpoint)
        if adj:
            mid_adj = int(input("how many deg do you want to adjust the midpoint (enter 0 to stop): "))
            while mid_adj != 0:
                self.moveArmAbsolute(arm, self.getPosition(arm) - mid_adj)
                mid_adj = int(input("how many deg do you want to adjust the midpoint (enter 0 to stop): "))
            # find new midpoint
            midpoint = self.getPosition(arm)

        return lower_bound, upper_bound, midpoint

    def calibrate(self):
        lower_bound_lower_arm, upper_bound_lower_arm, midpoint_lower_arm = self.findBounds(self.lower_arm)

        lower_bound_upper_arm, upper_bound_upper_arm, midpoint_upper_arm = self.findBounds(self.upper_arm)

        self.stop()
        print("self.lower_arm.lower_bound =", lower_bound_lower_arm)
        print("self.lower_arm.upper_bound =", upper_bound_lower_arm)
        print("self.lower_arm.midpoint =", midpoint_lower_arm)

        print("self.upper_arm.lower_bound =", lower_bound_upper_arm)
        print("self.upper_arm.upper_bound =", upper_bound_upper_arm)
        print("self.upper_arm.midpoint =", midpoint_upper_arm)

    def workspace(self):
        # go to middle
        self.moveArmAbsolute(self.upper_arm, self.upper_arm.midpoint)
        self.moveArmAbsolute(self.lower_arm, self.lower_arm.midpoint)

        # go to lower bound
        self.moveArmAbsolute(self.lower_arm, self.lower_arm.lower_bound)
        self.moveArmAbsolute(self.upper_arm, self.upper_arm.lower_bound)

        # go to upper bound
        self.moveArmAbsolute(self.lower_arm, self.lower_arm.upper_bound)
        self.moveArmAbsolute(self.upper_arm, self.upper_arm.upper_bound)

        # go to lower bound
        self.moveArmAbsolute(self.lower_arm, self.lower_arm.lower_bound)

        # go to middle again
        self.moveArmAbsolute(self.upper_arm, self.upper_arm.midpoint)
        self.moveArmAbsolute(self.lower_arm, self.lower_arm.midpoint)

if __name__ == "__main__":
    arm = Arm()
    if len(sys.argv) != 2:
        print("Error: Exactly one argument (cal/work) is required.")
        sys.exit(1)
    
    input_arg = sys.argv[1]

    if input_arg not in ["cal", "work"]:
        print("Error: Argument must be 'cal' or 'work'.")
        sys.exit(1)

    if input_arg == "cal":
        arm.calibrate()
    else:
        arm.workspace()