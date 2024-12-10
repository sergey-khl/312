#!/usr/bin/env python

import crtk
import sensable_phantom
import math
import sys
import rospy
import time
import PyKDL
import argparse
from sensor_msgs.msg import Joy
import threading
import socket


# example of application using arm.py
class DaVinci:

    # configuration
    def __init__(self, ral, robot_name, expected_interval, ip, port):
        self.expected_interval = expected_interval
        self.arm = sensable_phantom.arm(ral = ral,
                                        arm_name = robot_name,
                                        expected_interval = expected_interval)
        self.address = ("0.0.0.0", int(port))
        self.sock = self.createUdpSocket()

        self.remote = (ip, int(port))

        self.button_event = threading.Event()

        udp_thread = threading.Thread(target=self.receiveCollisions)
        udp_thread.daemon = True
        udp_thread.start()

        rospy.Subscriber(robot_name + 'button1',
                         Joy, self.buttonEvent1)
        rospy.Subscriber(robot_name + 'button2',
                         Joy, self.buttonEvent2)

        print('checking connections')
        self.arm.ral().check_connections()
        print('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')

    def __del__(self):
        wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm.servo_cf(wrench)

    def buttonEvent1(self, data):
        self.sendData("1," + str(data.buttons[0]))

    def buttonEvent2(self, data):
        self.sendData("2,"+ str(data.buttons[0]))

    def createUdpSocket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(20)
        sock.bind(self.address)
        return sock

    def sendData(self, message):
        self.sock.sendto(message.encode(), self.remote)

    def receiveData(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            return data.decode()
        except socket.timeout:
            return None
        
    def receiveCollisions(self):
        while not rospy.is_shutdown():
            data = self.receiveData()
            try:
                direction_str, magnitude_str = data.split('_')  # Split at the underscore
                # convert to direction (array of floats) and magnitude (float)
                direction_str = direction_str.strip('()')

                # to find the wrench multiply the direction by the magnitude by -1 to get the force in the opposite direction
                wrench_force = [-float(i) * float(magnitude_str) for i in direction_str.split(',')]

                # scale the wrench values to a max absolute value of 2
                max_wrench = max(abs(force) for force in wrench_force)
                if max_wrench > 2:
                    scale_factor = 2 / max_wrench
                    wrench_force = [force * scale_factor for force in wrench_force]

                wrench = [*wrench_force, 0.0, 0.0, 0.0]

                self.arm.servo_cf(wrench)
            except:
                print(f"Error processing the data: {data}")

if __name__ == '__main__':
    argv = crtk.ral.parse_argv(sys.argv[1:])

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace') 
    parser.add_argument('-u', '--ip', type=str, required=True,
                        help = 'ip of socket endpoint')
    parser.add_argument('-p', '--port', type=str, required=True,
                        help = 'port of socket endpoint')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv)

    ral = crtk.ral('sensable_phantom_da_vinci')
    application = DaVinci(ral, args.arm, args.interval, args.ip, args.port)
    rospy.spin()
