#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: 2024/09/22
 
Brick Number: G2

Lab Number: 1

Problem Number: 3
 
Brief Program/Problem Description: 

	We need to move 3 times in a rectangle or lemniscate (figure 8).
    To do so call the program with ./lab1_problem3 [lemniscate/rectangle]

Brief Solution Summary:

	used the MoveDifferential to perform all movement. Encodings from the wheels
    and gyro are logged after each movement. We simply applied the methods from the 
    MoveDifferential class and tweaked the parameters until we were happy with
    the results.

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
        # Initialize the tank's gyro sensor
        self.gyro = GyroSensor()
        self.previous_degrees = 0
        self.recalibrate()

    def logEncodings(self, custom_message):
        print(custom_message)
        print("angle", self.gyro.angle)
        print("left motor distance traveled", str(self.left_motor.rotations * 216))
        print("right motor distance traveled", str(self.right_motor.rotations * 216))

    def goDegreeTurn(self, deg, compensate=False):
        self.turn_degrees(
            speed=SpeedPercent(15),
            degrees= deg + self.previous_degrees if compensate else deg,
            error_margin=2,
            use_gyro=True,
        )
        # compensate for next turn
        self.previous_degrees = 90 - self.gyro.angle

    def goArcTurnRight(self, r, deg):
        print("arcing right")
        while self.gyro.angle <= deg:
            print(self.gyro.angle)
            self.on_arc_right(
                speed=SpeedRPM(30),
                radius_mm=r,
                distance_mm=30,
            )

    def goArcTurnLeft(self, r, deg):
        print("arcing left")
        while self.gyro.angle >= deg:
            print(self.gyro.angle)
            self.on_arc_left(
                speed=SpeedRPM(30),
                radius_mm=r,
                distance_mm=30,
            )

    def goStraight(self, mm):
        print("going straight")
        self.on_for_distance(SpeedPercent(60), mm)

    def moveRectangle(self, loops=1):
        rectangle_loop = [
            [self.goStraight, [150]],
            [self.logEncodings, ["after straight"]],
            [self.goDegreeTurn, [90, False]],
            [self.logEncodings, ["after turn"]],
        ]

        for i in range(loops):
            for j in range(4):
                print("iteration:", i + 1, "rectangle part:", j + 1)
                for func, args in rectangle_loop:
                    func(*args)

    def moveLemniscate(self, loops=1):
        lemniscate_loop = [
            [self.goArcTurnRight, [200, 350]],
            [self.logEncodings, ["after top half"]],
            [sleep, [5]],
            [self.goArcTurnLeft, [200, 10]],
            [self.logEncodings, ["after bottom half"]],
        ]
        for i in range(loops):
            print("iteration:", i + 1)
            for func, args in lemniscate_loop:
                func(*args)

    def recalibrate(self):
        self.gyro.reset()
        self.gyro.calibrate()
        sleep(1)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Error: Exactly one argument (lemniscate/rectangle) is required.")
        sys.exit(1)
    
    input_arg = sys.argv[1]

    if input_arg not in ["rectangle", "lemniscate"]:
        print("Error: Argument must be 'lemniscate' or 'rectangle'.")
        sys.exit(1)

    if input_arg == "rectangle":
        robo = Robo(OUTPUT_B, OUTPUT_A, EV3Tire, 120)
        robo.moveRectangle(3)
    else:
        robo = Robo(OUTPUT_B, OUTPUT_A, EV3Tire, 198)
        robo.moveLemniscate(3)
