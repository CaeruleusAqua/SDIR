#!/usr/bin/python2.7
from openravepy import *
import Kinematics as kin
import kinematics_geom as kin_base
import MotionFunctions as mf
from LIN import LIN
from BSplineMotion import BSplineMotion
from CSplineMotion import CSplineMotion
from CMSplineMotion import CMSplineMotion
import numpy as np
import sys
import socket
import math

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    theta = np.asarray(theta)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def dataTransfer():
    UDP_IP = "localhost"
    UDP_PORT = 54321
      
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind((UDP_IP, UDP_PORT))
        while True:
            data, addr = sock.recvfrom(2048)
            send=''
            try:
                send = handleData(data) 
            except Exception, e:
                print e
                import traceback
                traceback.print_exc()
            sock.sendto(send, addr)
    finally:
        sock.close()
    
def serializeDOF(arr):
    return ';'.join(map(lambda x: str(round(math.degrees(x),4)), arr)) + '#'

def deserializeDOF(s):
    return np.array(map(lambda x: math.radians(float(x)), s.split(';')))

def deserializePoints(s):
    raw_points = s.split('\n')
    
    return np.array(map(lambda a: map(lambda x: float(x), a.split(';')), raw_points))

handles = []
axis = None

# handles the data received from the GUI and sets up data for sending
def handleData(data):
    # split data string
    data_arr = data.split("#")
    print data_arr
    
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
        print "DOF: ",robot.GetDOFValues()
        start = kin_base.Kinematics_geom().direct_kin_to_wrist(robot.GetDOFValues())
        target = map(lambda x: float(x), data_arr[1].split(';'))
        
        lin = LIN(kin_base.Kinematics_geom())
        
        trajectory = lin.calculate(start, target)
        
        if trajectory is not None:
            handles.append(env.drawlinestrip(points=np.array([[start[0], start[1], start[2]],[target[0], target[1], target[2]]]),
                           linewidth=2.0,
                           colors=np.array([[0,1,0],[0,1,0]])))
                    
            mf.Move(robot, trajectory)
            
        
        # Simulate GET-request to deduplicate code
        data_arr[0] = 'GET'
    elif data_arr[0] == 'SPL':
        spline = None
        color = None
        if data_arr[1] == 'B':
            spline = BSplineMotion(kin_base.Kinematics_geom())
            color = [0, 1, 0]
        elif data_arr[1] == 'C':
            spline = CSplineMotion(kin_base.Kinematics_geom())
            color = [1, 0, 0]
        elif data_arr[1] == 'CM':
            spline = CMSplineMotion(kin_base.Kinematics_geom())
            color = [1, 0, 0]
        
        dps = deserializePoints(data_arr[2])
        
        print dps
        (path, trajectory) = spline.move(dps, 0.01, np.array([0, 0, 0]))

        print ">Path"
        print path
        print "<"
        
        if trajectory is not None:
            handles.append(env.drawlinestrip(points=path,
                           linewidth=2.0,
                           colors=color))
                    
            mf.Move(robot, trajectory)
        
        data_arr[0] = 'GET'
    
    # check if GUI requests the current robot axis values as well as current orientation and position 
    if data_arr[0] == 'GET':
        # prefix for parsing
        prefix = "VAL#"
        # get Axis values
        axis_arr = robot.GetDOFValues()
        # convert to string
        axis_values = serializeDOF(axis_arr)
        kin = kin_base.Kinematics_geom().direct_kin_to_wrist(axis_arr)
        d=kin_base.Kinematics_geom().direct_kin(axis_arr)
        #d= np.round(d,3)

        cart_values = str(np.round(kin[0],3)) + ";" + str(np.round(kin[1],3)) + ";" +  str(np.round(kin[2],3)) + ";" +  str(np.round(d[1],3)) + ";" +  str(np.round(d[2],3)) + ";" +  str(np.round(d[3],3))
        
        
        point = kin_base.Kinematics_geom().direct_kin_to_wrist(robot.GetDOFValues())
        d = kin_base.Kinematics_geom().direct_kin(robot.GetDOFValues())

        temp = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        
        temp = np.dot(rotation_matrix(np.array([0, 0, 1]), d[1]), temp)
        temp = np.dot(rotation_matrix(np.array([0, 1, 0]), d[2]), temp)
        temp = np.dot(rotation_matrix(np.array([1, 0, 0]), d[3]), temp)
        
        axis_m = np.array([
                         [temp[0][0], temp[0][1], temp[0][2], point[0]],
                         [temp[1][0], temp[1][1], temp[1][2], point[1]],
                         [temp[2][0], temp[2][1], temp[2][2], point[2]],
                         [0, 0, 0, 1]
                         ])
        
        print axis_m
        handles.append(misc.DrawAxes(env, axis_m, 0.2, 2))
        
        return prefix+axis_values+cart_values
    
    # check if inverse kinematics should be calculated
    if data_arr[0] == "CAL":
        # get string with values
        values = data_arr[1].split(';')
        
        floats = []
        
        for item in values:
            floats.append(float(item))

        
        # calculate inverse kinematic solution
        solution = kin_base.Kinematics_geom().inverse_kin([floats[0],floats[1],floats[2]],[floats[3],floats[4],floats[5]])
        
        ik_values = ""
        
        # find the valid solutions and serialize
        for item in solution:
            if kin_base.Kinematics_geom().isSolutionValid(item,[floats[0],floats[1],floats[2]]) == True:
                ik_values = ik_values + serializeDOF([item[0],item[1],item[2],item[3],item[4],item[5]])[:-1] + "\n\n"
                
        if ik_values=="":
            ik_values="No Solution found!"

        
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
