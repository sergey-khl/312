#!/usr/bin/env python3

from time import sleep
from math import pi

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveDifferential, SpeedRPM, SpeedPercent, follow_for_ms
from ev3dev2.wheel import EV3Tire
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1


class Robo(MoveDifferential):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Initialize the tank's gyro sensor
        self.gyro = GyroSensor()
        self.previous_degrees = 0

    def logEncodings(self):
        # print("angle and rate", self.gyro.angle_and_rate)
        print("angle", self.gyro.angle)
        print("circle angle", self.gyro.circle_angle())
        print("previous deg", self.previous_degrees)
        # print("left motor position", str(self.left_motor.position))
        # print("right motor position", str(self.right_motor.position))

    def goDegreeTurn(self, deg, compensate=False):
        print("turning")
        self.turn_degrees(
            speed=SpeedRPM(70),
            degrees=deg + self.previous_degrees if compensate else deg,
            error_margin=2,
            use_gyro=True,
        )
        # compensate for next turn
        self.previous_degrees = 90 - self.gyro.angle if compensate else 0

    def goDegreeTurnBetter(self, deg):
        print("turning")
        print(self.gyro.angle)
        self.turn_degrees(
            speed=SpeedRPM(15),
            degrees=deg,
            error_margin=2,
            use_gyro=True,
        )
        print(self.gyro.angle)
    
    def goArcTurnRight(self, r, deg):
        print("arcing right")
        while self.gyro.angle <= deg:
            self.on_arc_right(
                speed=SpeedRPM(30),
                radius_mm=r,
                distance_mm=30,
            )

    def goArcTurnLeft(self, r, deg):
        print("arcing left")
        while self.gyro.angle >= deg:
            self.on_arc_left(
                speed=SpeedRPM(30),
                radius_mm=r,
                distance_mm=30,
            )
        # compensate for next turn
        # self.previous_degrees = 90 - self.gyro.angle

    def goStraight(self, mm):
        print("going straight")
        self.on_for_distance(SpeedRPM(60), mm)

    def moveRectangle(self, loops=1):
        rectangle_loop = [
            [self.recalibrate, []],
            [self.goStraight, [150]],
            [self.goDegreeTurn, [90, True]],
            [self.logEncodings, []],
        ]

        for i in range(loops):
            for j in range(4):
                for func, args in rectangle_loop:
                    func(*args)

    def moveLemniscate(self, loops=1):
        lemniscate_loop = [
            [self.recalibrate, []],
            [self.goStraight, [200]],
            [self.goDegreeTurnBetter, [30]],
            [self.goArcTurnRight, [50, 180]],
            [self.goDegreeTurnBetter, [30]],
            # [self.recalibrate, []],
            [self.goStraight, [200]],
            [self.goDegreeTurnBetter, [-30]],
            [self.goArcTurnLeft, [50, 0]],
            [self.goDegreeTurnBetter, [-30]],
        ]
        for i in range(loops):
            for func, args in lemniscate_loop:
                func(*args)

    def recalibrate(self):
        self.gyro.reset()
        self.gyro.calibrate()
        sleep(1)


if __name__ == "__main__":
    robo = Robo(OUTPUT_B, OUTPUT_A, EV3Tire, 100)
    robo.moveLemniscate(3)
    robo.moveRectangle(3)
