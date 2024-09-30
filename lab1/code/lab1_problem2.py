#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: 2024/09/22
 
Brick Number: G2

Lab Number: 1

Problem Number: 2
 
Brief Program/Problem Description: 

	Find the error of going in a straight line and while rotating as a function of power.

Brief Solution Summary:

	Either call the program with line or rot to indicate type of error we are collecting.
    For line we compare actual distance traveled with wheel encoding distance traveled as well as what we put in as a command.
    For rotation we compare actual rotation to encoding estimation and gyro estimation.
    We used distance traveled by wheels to calculate these values as detailed in the report.

Used Resources/Collaborators:
	https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html
    https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/sensors.html

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

import sys
from time import sleep

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveDifferential, SpeedRPM, SpeedPercent
from ev3dev2.wheel import EV3Tire
from ev3dev2.sensor.lego import GyroSensor

class Robo(MoveDifferential):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.gyro = GyroSensor()
        self.recalibrate()

    def logEncodings(self):
        print("angle", self.gyro.angle)
        print("left motor distance traveled", str(self.left_motor.rotations * 216))
        print("right motor distance traveled", str(self.right_motor.rotations * 216))

    def goDegreeTurn(self, speed_percent, deg):
        self.turn_degrees(
            speed=SpeedPercent(speed_percent),
            degrees= deg,
            error_margin=2,
            use_gyro=True,
        )

    def goStraight(self, speed_percent, mm):
        print("going straight")
        self.on_for_distance(SpeedPercent(speed_percent), mm)

    def recalibrate(self):
        self.gyro.reset()
        self.gyro.calibrate()
        sleep(1)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Error: Exactly one argument (line/rot) is required.")
        sys.exit(1)
    
    input_arg = sys.argv[1]

    if input_arg not in ["line", "rot"]:
        print("Error: Argument must be 'line' or 'rot'.")
        sys.exit(1)

    if input_arg == "line":
        robo = Robo(OUTPUT_B, OUTPUT_A, EV3Tire, 198)
        robo.logEncodings()
        robo.goStraight(25, 150)
        robo.logEncodings()
    else:
        robo = Robo(OUTPUT_B, OUTPUT_A, EV3Tire, 120)
        robo.logEncodings()
        robo.goDegreeTurn(25, 90)
        robo.logEncodings()
