#!/usr/bin/env python3

from time import sleep

from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_2, INPUT_4
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, SpeedRPM, SpeedPercent, MoveTank


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

    def startEmotion(self, emotion="cowardice"):
        while True:
            if emotion == "cowardice":
                self.doCowardice()
            elif emotion == "aggression":
                self.doAggression()
            else:
                print("unknown emotion")
                return


if __name__ == "__main__":
    robo = Robot(OUTPUT_B, OUTPUT_A)
    # dead reckoning commands
    robo.startEmotion("aggression")
