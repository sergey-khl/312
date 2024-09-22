#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: 2024/09/22
 
Brick Number: G2

Lab Number: 1

Problem Number: 5
 
Brief Program/Problem Description: 

    Bot must either act cowardly (move to the side unless light right in front, in which case charge),
    or act aggressive (always charge the light)

Brief Solution Summary:

	used 2 light sensors on either side of the robot to tell when the light is closer to one of the sensors
    used this difference to either go away or charge at the robot. The distance to the light was used to
    perform a charging maneuver. We used the self.on function to constantly move.

Used Resources/Collaborators:
    https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/sensors.html
    http://ugweb.cs.ualberta.ca/~vis/courses/robotics/assign/a1MobRob/lab1_notes.jpg

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

import sys
from time import sleep

from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_2, INPUT_4
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveTank


class Robot(MoveTank):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.left_light = ColorSensor(INPUT_4)
        self.right_light = ColorSensor(INPUT_2)
        self.margin = 1/7
        self.min_speed = 10
        self.max_speed = 30
    
    def doCowardice(self):
        left_light = self.left_light.ambient_light_intensity
        right_light = self.right_light.ambient_light_intensity
        scaled_margin = self.margin * (left_light + right_light)
        if (left_light + scaled_margin < right_light):
            self.on(self.min_speed, self.max_speed)
        elif (left_light > right_light + scaled_margin):
            self.on(self.max_speed, self.min_speed)
        else:
            self.on(min(100, self.min_speed * scaled_margin), min(100, self.min_speed * scaled_margin))

    def doAggression(self):
        left_light = self.left_light.ambient_light_intensity
        right_light = self.right_light.ambient_light_intensity
        left_speed = min(100, self.max_speed * right_light * self.margin)
        right_speed = min(100, self.max_speed * left_light * self.margin)
        self.on(left_speed, right_speed)

    def startEmotion(self, emotion):
        while True:
            if emotion == "cowardice":
                self.doCowardice()
            elif emotion == "aggression":
                self.doAggression()
            else:
                print("unknown emotion")
                return


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Error: Exactly one argument (aggression/cowardice) is required.")
        sys.exit(1)
    
    input_arg = sys.argv[1]

    if input_arg not in ["aggression", "cowardice"]:
        print("Error: Argument must be 'aggression' or 'cowardice'.")
        sys.exit(1)

    robo = Robot(OUTPUT_B, OUTPUT_A)
    if input_arg == "aggression":
        robo.startEmotion("aggression")
    else:
        robo.startEmotion("cowardice")
