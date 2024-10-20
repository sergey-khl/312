#!/usr/bin/python
"""
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: XXX
 
Brick Number: G2

Lab Number: 3

Problem Number: 22
 
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

import socket
import time
from queue import Queue
import cv2
import numpy as np
import threading
import json
from math import atan2, sin, cos, sqrt, pi, copysign

#####HSV Colour Ranges#################
#If the ball is red (0-10) or (170-180)
redLowMask = (0,25, 220)
redHighMask = (179, 255, 255)

#If the ball is blue
blueLowMask = (80, 25, 100)
blueHighMask = (160, 255, 220)
########################################

# This class handles the Server side of the comunication between the laptop and the brick.
class Server:
    def __init__(self, host, port):
       # setup server socket
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        # We need to use the ip address that shows up in ipconfig for the usb ethernet adapter that handles the comunication between the PC and the brick
        print("Setting up Server\nAddress: " + host + "\nPort: " + str(port))
        
        self.serversocket.bind((host, port))
        # queue up to 5 requests
        self.serversocket.listen(5) 
        self.cs, addr = self.serversocket.accept()
        print ("Connected to: " + str(addr))
    
    def __del__(self):
        self.cs.close()
        self.serversocket.close()

    # Sends set of angles to the brick via TCP.
    # Input: base_angle [Float]: The angle by which we want the base to move
    #        joint_angle [Float]: The angle by which we want to joint to move
    #        queue [Thread-safe Queue]: Mutable data structure to store (and return) the messages received from the client
    def sendAngles(self, theta_1, theta_2):
        # Format in which the client expects the data: "angle1,angle2"
        data = str(theta_1) + "," + str(theta_2)
        self.cs.send(data.encode("UTF-8"))
        # Waiting for the client (ev3 brick) to let the server know that it is done moving
        reply = self.cs.recv(128).decode("UTF-8")
        return reply

    # Sends a termination message to the client. This will cause the client to exit "cleanly", after stopping the motors.
    def sendTermination(self):
        self.cs.send("EXIT".encode("UTF-8"))

    # Lets the client know that it should enable safety mode on its end
    def sendEnableSafetyMode(self):
        self.cs.send("SAFETY_ON".encode("UTF-8"))
    
    # Lets the client know that it should disable safety mode on its end
    def sendDisableSafetyMode(self):
        self.cs.send("SAFETY_OFF".encode("UTF-8"))

class Tracker:
    def __init__(self, pointColor, goalColor):
        self.point = (0,0,0)
        self.goal = (0,0,0)
        thread = threading.Thread(target=self.TrackerThread, args=(pointColor, goalColor), daemon=True)
        thread.start()

    def TrackerThread(self, pointColor, goalColor):
        print("Tracker Started")
        # Get the camera
        vc = cv2.VideoCapture(1)
        if vc.isOpened(): # try to get the first frame
            rval, frame = vc.read()
        else:
            rval = False
        while rval:
            # Handle current frame
            rval, frame = vc.read()
            circlesPoint = self.GetLocation(frame, pointColor)
            circlesGoal = self.GetLocation(frame, goalColor)
            self.DrawCircles(frame, circlesPoint, (255, 0, 0))
            self.DrawCircles(frame, circlesGoal, (0, 0, 255))

            if circlesPoint is not None:
                self.point = circlesPoint[0][0]
            
            if circlesGoal is not None:
                self.goal = circlesGoal[0][0]

            # Shows the original image with the detected circles drawn.
            cv2.imshow("Result", frame)

            # check if esc key pressed
            key = cv2.waitKey(20)
            if key == 27:
                break
        
        vc.release()
        cv2.destroyAllWindows()
        print("Tracker Ended")

    def GetLocation(self, frame, color):
        # Uncomment for gaussian blur
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        # blurred = cv2.medianBlur(frame,11)
        blurred = frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if color == 'r':
            # Red Tracking
            mask = cv2.inRange(hsv, redLowMask, redHighMask)
        if color == 'b':
            # Blue Tracking
            mask = cv2.inRange(hsv, blueLowMask, blueHighMask)
        # Perform erosion and dilation in the image (in 11x11 pixels squares) in order to reduce the "blips" on the mask
        mask = cv2.erode(mask, np.ones((11, 11),np.uint8), iterations=2)
        mask = cv2.dilate(mask, np.ones((11, 11),np.uint8), iterations=5)
        # Mask the blurred image so that we only consider the areas with the desired colour
        masked_blurred = cv2.bitwise_and(blurred,blurred, mask= mask)
        # masked_blurred = cv2.bitwise_and(frame,frame, mask= mask)
        # Convert the masked image to gray scale (Required by HoughCircles routine)
        result = cv2.cvtColor(masked_blurred, cv2.COLOR_BGR2GRAY)
        # Detect circles in the image using Canny edge and Hough transform
        return cv2.HoughCircles(result, cv2.HOUGH_GRADIENT, 1.5, 300, param1=100, param2=20, minRadius=20, maxRadius=200)
            
    def DrawCircles(self, frame, circles, dotColor):
        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                #print("Circle: " + "("+str(x)+","+str(y)+")")
                # draw the circle in the output image, then draw a rectangle corresponding to the center of the circle
                # The circles and rectangles are drawn on the original image.
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), dotColor, -1)

class Arm():
    def __init__(self):
        self.lower_arm = type('ArmPart', (), {})()
        self.upper_arm = type('ArmPart', (), {})()

        # arm lengths in mm
        self.lower_arm.length = 160
        self.upper_arm.length = 45

        self.error_tol = 30
        self.max_iter = 1000
        self.prev_y = None
        self.prev_theta_1 = None
        self.prev_theta_2 = None

        #initialize jacobian
        # found using method in report
        # self.jacobian = np.array([[120, 42.9], [24, 8.6]])
        # self.jacobian = np.eye(2)
        self.jacobian = None

    def getPositionWithKnownAngles(self, theta_1, theta_2):
        x = self.lower_arm.length * cos(theta_1) + self.upper_arm.length * cos(theta_1 + theta_2)
        y = self.lower_arm.length * sin(theta_1) + self.upper_arm.length * sin(theta_1 + theta_2)
        return x, y
    
    def createIntermediatePoints(self, init_x, init_y, end_x, end_y, num_points):
        delta_x = (end_x - init_x)/(num_points)
        delta_y = (end_y - init_y)/(num_points)

        points = [None] * num_points
        for i in range(1, num_points + 1):
            points[i - 1] = [init_x+delta_x*i, init_y+delta_y*i]
            
        return points

    def euclideanDistance(self, init_x, init_y, end_x, end_y):
        return sqrt((end_x - init_x) ** 2 + (end_y - init_y) ** 2)

    def initializeJacobian(self, theta_1, theta_2):
        self.prev_y = np.array([[tracker.point[0]], [tracker.point[1]]])

        delta_q = server.sendAngles(theta_1, theta_2)
        theta_1, theta_2 = map(float, delta_q.split(","))
        delta_q = np.array([[theta_1], [theta_2]])

        new_y = np.array([[tracker.point[0]], [tracker.point[1]]])
        delta_y = new_y - self.prev_y
        self.jacobian = np.array([[delta_y[0][0]/delta_q[0][0], delta_y[0][0]/delta_q[1][0]], [delta_y[1][0]/delta_q[0][0], delta_y[1][0]/delta_q[1][0]]])

    def findNextAngles(self, curr_u, curr_v, target_u, target_v):
        error_distance = self.euclideanDistance(curr_u,curr_v, target_u, target_v)
        print(error_distance)
        if error_distance < self.error_tol:
            return False

        delta_y = np.subtract(np.array([[curr_u], [curr_v]]), self.prev_y)
        self.prev_y = np.array([[curr_u], [curr_v]])
        error = np.subtract(np.array([[target_u], [target_v]]), self.prev_y)

        print("DELTA Y ", delta_y)
        print("JACOBIAN", np.linalg.cond(self.jacobian))
        delta_q = np.linalg.pinv(self.jacobian) @ error
        print("DELTA Q ", delta_q)
        delta_q = np.clip(delta_q, -25, 25)

        # find delta_q from the amount actually moved
        delta_q = server.sendAngles(delta_q[0][0], delta_q[1][0])

        if delta_q == "RESET":
            out = input("enter space separated perturbation of theta_1 theta_2")
            arm.initializeJacobian(*map(int, out.split(" ")))
        else:
            theta_1, theta_2 = map(float, delta_q.split(","))
            delta_q = np.array([[theta_1], [theta_2]])
            alpha = min(400, error_distance) / 400
            self.jacobian = self.jacobian + alpha * np.divide((delta_y - self.jacobian@delta_q), np.transpose(delta_q) @ delta_q) @ np.transpose(delta_q)


        return True

if __name__ == "__main__":
    arm = Arm()

    tracker = Tracker('b', 'r')

    host = "169.254.128.117"
    port = 9999
    server = Server(host, port)

    while True:
        out = input("enter space separated initial perturbation of theta_1 theta_2 to init or q to quit")
        if out == "q":
            break
        print(*tracker.point, *tracker.goal)
        if all(i == 0 for i in tracker.point) or all(i == 0 for i in tracker.goal):
            print("could not find point or goal, try again...")
            continue
        arm.initializeJacobian(*map(int, out.split(" ")))
        go = True
        while go:
            go = arm.findNextAngles(*tracker.point[:2], *tracker.goal[:2])

        server.sendTermination()
        