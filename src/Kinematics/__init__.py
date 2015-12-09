#!/usr/bin/python2.7
from openravepy import *
import Kinematics as kin
import kinematics_base as kin_base
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
        axis_values = str(axis_arr[0])+";"+str(axis_arr[1])+";"+str(axis_arr[2])+";"+str(axis_arr[3])+";"+str(axis_arr[4])+";"+str(axis_arr[5])+'#'
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
        axis_values = str(axis_arr[0])+";"+str(axis_arr[1])+";"+str(axis_arr[2])+";"+str(axis_arr[3])+";"+str(axis_arr[4])+";"+str(axis_arr[5])+'#'
        # adding dummy values for orientation and position (you need to compute the values)
        cart_values = "0;0;0;0;0;0"     
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
    # setting up the operave environment
    kin=kin_base.Kinematics_base()
    wp = kin.direct_kin_to_wrist([0.0 , math.radians(45.0) , math.radians(-1.0) , 0.0])
    print "TCP:"
    print np.round(wp, 3)

    theta_0 = np.empty([2])
    theta_0[0]=np.arctan2(wp[1], wp[0])                 # turn robot arm into wrist point plane

    if theta_0[0] < 0:
        theta_0[1] = theta_0[0] + np.pi
    else:
        theta_0[1] = theta_0[0] - np.pi

    print "theta0"
    print np.round(theta_0,3)

    shoulder=np.round(kin.direct_kin_to_shoulder([theta_0[0]]),3)
    #print "shoulder"
    #print shoulder

    X_zp=math.sqrt( (shoulder[0] - wp[0])**2  +  (shoulder[1] - wp[1])**2 )
    Z_zp=wp[2]-shoulder[2]
    beta1 = math.atan2(Z_zp,X_zp)
    #print math.degrees(beta1)                           ## looks good

    R=np.linalg.norm(wp-shoulder)
    print R                                             ## looks good

    a=abs(kin.dh[3]['a'])
    b=abs(kin.dh[4]['d'])
    d=math.sqrt(a**2 + b**2)
    e=abs(kin.dh[2]['a'])


    beta2=math.acos((d**2-e**2-R**2)/(-2*e*R))
    #print math.degrees(beta2)


    theta_1 = -math.pi/2+beta1+beta2

    print "theta1"
    print math.degrees(theta_1)

    beta1 = math.acos((-R**2+d**2+e**2)/(2*d*e))
    beta2 = math.asin(b/d)

    print "theta2"
    print math.degrees(beta1+beta2-math.pi)


    #env = Environment() # create openrave environment
    #env.SetViewer('qtcoin') # attach viewer (optional)
    #env.Load('../../MyData/MyEnvironment/MyEnv.xml') # load a simple scene
    #robot = env.GetRobots()[0] # get the first robot
    #dataTransfer()

    #### x:  0.205
    #### y:  -0.0
    #### z:  3.718