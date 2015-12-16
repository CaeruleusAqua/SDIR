#!/usr/bin/python2.7
from openravepy import *
import Kinematics as kin
import kinematics_geom as kin_base
import MotionFunctions as mf
from LIN import LIN
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
    return ';'.join(map(lambda x: str(round(math.degrees(x),4)), arr)) + '#'

def deserializeDOF(s):
    return np.array(map(lambda x: math.radians(float(x)), s.split(';')))

# handles the data received from the GUI and sets up data for sending
def handleData(data):
    # split data string
    data_arr = data.split("#")
    
    # check if the robot should be moved 
    if data_arr[0] == 'MOV':
        target = deserializeDOF(data_arr[1])

        # get the motion type
        motion_type = data_arr[2]
        
        # get trajectory
        trajectory = mf.PTPtoConfiguration(robot.GetDOFValues(), target, motion_type)
        # move robot
        mf.Move(robot, trajectory)
        
        # Simulate GET-request to deduplicate code
        data_arr[0] = 'GET'
    elif data_arr[0] == 'LIN':
        start = kin_base.Kinematics_geom().direct_kin_to_wrist(robot.GetDOFValues())
        target = map(lambda x: float(x), data_arr[1].split(';'))
        
        lin = LIN(kin_base.Kinematics_geom())
        
        trajectory = lin.calculate(start, target[:3])
        
        if trajectory is not None:
            mf.Move(robot, trajectory)
        
        # Simulate GET-request to deduplicate code
        data_arr[0] = 'GET'
        
        
    
    # check if GUI requests the current robot axis values as well as current orientation and position 
    if data_arr[0] == 'GET':
        # prefix for parsing
        prefix = "VAL#"
        # get Axis values
        axis_arr = robot.GetDOFValues()
        print axis_arr
        # convert to string
        axis_values = serializeDOF(axis_arr)
        d=kin_base.Kinematics_geom().direct_kin(axis_arr) #np.round(kin_base.Kinematics_geom().direct_kin(axis_arr),3)
        kin= np.round(d[0])

        cart_values = str(kin[0]) + ";" + str(kin[1]) + ";" +  str(kin[2]) + ";" +  str(d[1]) + ";" +  str(d[2]) + ";" +  str(d[3])
        
        return prefix+axis_values+cart_values
    
    # check if inverse kinematics should be calculated
    if data_arr[0] == "CAL":
        # get string with values
        values = data_arr[1].split(';')
        print values
        
        floats = []
        
        for item in values:
            floats.append(float(item))
            
        print floats
        
        # calculate inverse kinematic solution
        solution = kin_base.Kinematics_geom().inverse_kin([floats[0],floats[1],floats[2]])
        print solution
        
        ik_values = ""
        
        # find the valid solutions and serialize
        for item in solution:
            if kin_base.Kinematics_geom().isSolutionValid(item,[floats[0],floats[1],floats[2]]) == True:
                ik_values = ik_values + serializeDOF([item[0],item[1],item[2],0,0,0])[:-1] + "\n\n"
                
        if ik_values=="":
            ik_values="No Solution found!"
            
            
        print ik_values
        
        # send the (multiple) solutions to the GUI
        # prefix for parsing
        prefix = "INK#"
        # adding dummy values (you need to replace them with the solutions)
        return prefix+ik_values
    
    
if __name__ == "__main__":
    env = Environment() # create openrave environment
    env.SetViewer('qtcoin') # attach viewer (optional)
    env.Load('../../MyData/MyEnvironment/MyEnv.xml') # load a simple scene
    robot = env.GetRobots()[0] # get the first robot
    dataTransfer()
