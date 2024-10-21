#!/usr/bin/env python3
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: 2024-10-20
 
Brick Number: G2

Lab Number: 3

Problem Number: 2
 
Brief Program/Problem Description: 

    need to move a blue point at the end effector to a red circle somewhere else in the robots workspace

Brief Solution Summary:

    For the client part we mostly just read angles from the server and execute. The client also
    works to initialize the jacobian by moving the lower and upper arm separately and telling the
    server to record the pixels of the end effector.

Used Resources/Collaborators:
	https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html
    https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/sensors.html
    we used the client and color tracking code found on eclass: https://eclass.srv.ualberta.ca/mod/resource/view.php?id=8123643

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B
import socket

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
        # assume we manually put in midpoint each time
        self.lower_arm.midpoint = self.lower_arm.position
        self.upper_arm.midpoint = self.upper_arm.position

        # arm lengths in mm
        self.lower_arm.length = 160
        self.upper_arm.length = 45

        # inverse config
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

    def stop(self):
        self.lower_arm.stop()
        self.upper_arm.stop()

    def getAngleOfArm(self, arm):
        return arm.position - arm.midpoint

    def moveArmsAbsolute(self, lower_pos, upper_pos):
        self.lower_arm.position_sp = lower_pos
        self.upper_arm.position_sp = upper_pos
        self.lower_arm.run_to_abs_pos()
        self.upper_arm.run_to_abs_pos()
        out_l = self.lower_arm.wait_while("running", 360000/self.lower_arm.speed_sp)
        out_u = self.upper_arm.wait_while("running", 360000/self.upper_arm.speed_sp)
        return out_l and out_u
    
    def initializeJacobian(self, client):
        perturbation_lower = 20
        perturbation_upper = 20
        if self.getAngleOfArm(self.lower_arm) > 0:
            perturbation_lower *= -1
        if self.getAngleOfArm(self.upper_arm) > 0:
            perturbation_upper *= -1

        # send next tells the server to pickup the data from the camera
        print("moving lower", self.lower_arm.position, self.upper_arm.position)
        self.moveArmsAbsolute(self.lower_arm.position + perturbation_lower, self.upper_arm.position)
        client.sendRecordPoint(perturbation_lower)

        print("moving lower", self.lower_arm.position, self.upper_arm.position)
        self.moveArmsAbsolute(self.lower_arm.position - perturbation_lower, self.upper_arm.position)
        client.sendRecordPoint()

        print("moving upper", self.lower_arm.position, self.upper_arm.position)
        self.moveArmsAbsolute(self.lower_arm.position, self.upper_arm.position + perturbation_upper)
        client.sendRecordPoint(perturbation_upper)

        print("moving upper", self.lower_arm.position, self.upper_arm.position)
        self.moveArmsAbsolute(self.lower_arm.position, self.upper_arm.position - perturbation_upper)
        client.sendRecordPoint()

# This class handles the client side of communication. It has a set of predefined messages to send to the server as well as functionality to poll and decode data.
class Client:
    def __init__(self, host, port):
        # We need to use the ipv4 address that shows up in ipconfig in the computer for the USB. Ethernet adapter handling the connection to the EV3
        print("Setting up client\nAddress: " + host + "\nPort: " + str(port))
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.s.connect((host, port))                               
    
    def __del__(self):
        self.s.close()
        
    # Block until a message from the server is received. When the message is received it will be decoded and returned as a string.
    # Output: UTF-8 decoded string containing the instructions from server.
    def pollData(self):
        print("Waiting for Data")
        data = self.s.recv(128).decode("UTF-8")
        print("Data Received")
        return data

    # Sends a message to the server letting it know that it should record with the camera. wait for the ok to continue
    def sendRecordPoint(self, delta_theta = "RECORD"):
        self.s.send(str(delta_theta).encode("UTF-8"))
        print(self.s.recv(128).decode("UTF-8"))

    # Sends a message to the server letting it know that it is ready for another angle
    def sendNext(self):
        self.s.send("NEXT".encode("UTF-8"))

    # Sends a message to the server letting it know that the movement of the motors was executed without any inconvenience.
    def sendDone(self):
        print("GOT TO POINT")
        self.s.send("DONE".encode("UTF-8"))

    # Sends a message to the server letting it know that there was an isse during the execution of the movement (obstacle avoided) and that the initial jacobian should be recomputed (Visual servoing started from scratch)
    def sendReset(self):
        self.s.send("RESET".encode("UTF-8"))


if __name__ == "__main__":
    arm = Arm()

    host = "169.254.128.117"
    port = 9999
    client = Client(host, port)
    i = 0
    while True:
        angles = client.pollData()
        # this means we must initialize the jacobian
        if angles == "INIT":
            arm.initializeJacobian(client)
            continue
        if angles == "EXIT":
            break
        theta_1, theta_2 = map(float, angles.split(","))
        succeeded = arm.moveArmsAbsolute(arm.lower_arm.position + theta_1, arm.upper_arm.position + theta_2)

        if succeeded:
            client.sendNext()
        else:
            client.sendReset()
