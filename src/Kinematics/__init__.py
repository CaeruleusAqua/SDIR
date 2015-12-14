#!/usr/bin/python2.7
from openravepy import *
import Kinematics as kin
import kinematics_geom as kin_base
import MotionFunctions as mf
import numpy as np
import sys
import socket
import math


def dataTransfer():
    UDP_IP = "localhost"
    UDP_PORT = 54321
      
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind((UDP_IP, UDP_PORT))
        while True:
            data, addr = sock.recvfrom(2048)
            send = handleData(data) 
            sock.sendto(send, addr)
    finally:
        sock.close()
    
def serializeDOF(arr):
    return ';'.join(map(lambda x: str(math.degrees(x)), arr)) + '#'

# handles the data received from the GUI and sets up data for sending
def handleData(data):
    # split data string
    data_arr = data.split("#")
    
    # check if GUI requests the current robot axis values as well as current orientation and position 
    if data_arr[0] == 'GET':
        # prefix for parsing
        prefix = "VAL#"
        # get Axis values
        axis_arr = robot.GetDOFValues()
        # convert to string
        axis_values = serializeDOF(axis_arr)
        # adding dummy values for orientation and position (you need to compute the values)
        cart_values = "0;0;0;0;0;0"
        return prefix+axis_values+cart_values
    
    # check if the robot should be moved 
    elif data_arr[0] == 'MOV':
        # get values
        values = data_arr[1].split(';')
        # convert from string to float and save in numpy array
        target = np.array([math.radians(float(values[0])), math.radians(float(values[1])), math.radians(float(values[2])), math.radians(float(values[3])), math.radians(float(values[4])), math.radians(float(values[5]))])

        # get the motion type
        motion_type = data_arr[2]
        
        # get trajectory
        trajectory = mf.PTPtoConfiguration(robot.GetDOFValues(), target, motion_type)
        # move robot
        mf.Move(robot, trajectory)
        
        # send new information about the robot's axis values, position and orientation to the GUI for updating purpose
        # prefix for parsing
        prefix = "VAL#"
        # get Axis values
        axis_arr = robot.GetDOFValues()
        # convert to string
        axis_values = serializeDOF(axis_arr)
        # adding dummy values for orientation and position (you need to compute the values)

        kin = np.round(kin_base.Kinematics_geom().direct_kin_to_wrist(target),3)

        cart_values = str(kin[0]) + ";" + str(kin[1]) + ";" +  str(kin[2]) + ";0;0;0"
        return prefix+axis_values+cart_values
    
    # check if inverse kinematics should be calculated
    if data_arr[0] == "CAL":
        # get string with values
        values = data_arr[1].split(';')
        
        # calculate inverse kinematic solution
        
        # send the (multiple) solutions to the GUI
        # prefix for parsing
        prefix = "INK#"
        # adding dummy values (you need to replace them with the solutions)
        ik_values = "0;0;0;0;0;0"
        return prefix+ik_values
    
    
if __name__ == "__main__":
    env = Environment() # create openrave environment
    env.SetViewer('qtcoin') # attach viewer (optional)
    env.Load('../../MyData/MyEnvironment/MyEnv.xml') # load a simple scene
    robot = env.GetRobots()[0] # get the first robot
    dataTransfer()
