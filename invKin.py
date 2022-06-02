#!/usr/bin/env python3


"""
Allows to use the service dynamixel_command 
"""
from numpy import array
from sqlalchemy import true
import rospy
import time
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand


# keyboard
import termios, sys, os

#matrix
import numpy as np

#inverse kinematics
from getInvKin import *

TERMIOS = termios

def getKey(): #Captura la letra que ingresa del teclado
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    c = str(c).replace('b', "").replace('\'', "")
    return c

def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def deg2raw(input_list: list = [0,0,0,0], min_deg: int = -150, max_deg: int = 150)->list:
    out_list = [0,0,0,0]
    for i in range(len(input_list)):
        out_list[i] = int( ((input_list[i] - min_deg)*1023)/(max_deg-min_deg) )
    return out_list


class Movement:
    def __init__(self, name , step):
         self.name = name
         self.step = step

def moveRobot(T, l, movement, direction):
    codo = 1 #0: down, 1: up    
    previous_T = T.copy()
    T = changeMatrix(T, movement, direction)
    try:
        q = np.degrees(getInvKin(T, l))
        goal_position_raw = deg2raw(q[codo,:])
        moveJoints(goal_position_raw, q[codo, :])
    except:        
        print("Exception...\n")
        T = previous_T.copy()
    return T




def changeMatrix(T, movement, direction):
    if movement.name == "rot":
        print("Rotation")
        angle = math.radians(movement.step)*direction
        c = math.cos(angle)
        s = math.sin(angle)
        rot_y = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        T[0:3, 0:3] = np.dot(T[0:3, 0:3],rot_y)
        
    else:
        print("Traslation")
        if(movement.name == "trax"): row = 0
        elif(movement.name == "tray"): row = 1
        elif(movement.name == "traz"): row = 2

        T[row, 3] = T[row, 3] + movement.step * direction  
    return T



def moveJoints(goal_position_raw, q):
    motors_ids = [1,2,3,4]
    for i in reversed(range(len(motors_ids))):
        jointCommand('', motors_ids[i], 'Goal_Position', goal_position_raw[i], 0.5)
        print("Moving ID:", motors_ids[i], " Angle: ", q[i], " Raw: ", goal_position_raw[i])

def adjustTorque():
    motors_ids = [1,2,3,4]
    torque = [500, 800, 500, 500]
    for i in range(len(motors_ids)):
        jointCommand('', motors_ids[i], 'Torque_Limit', torque[i], 0)

def enableTorque():
    motors_ids = [1,2,3,4]
    for i in range(len(motors_ids)):
        jointCommand('', motors_ids[i], 'Torque_Enable', 1, 0)


def main(step: list = [1, 1 , 1, 10]):
    codo = 1
    l = [14.5, 10.7, 10.7, 9]    

    T = np.array([[1.0,   0.0,    0.0,   0.0],
                  [0.0,   1.0,    0.0,   0.0],
                  [0.0,   0.0,    1.0,   44.9], 
                  [0.0, 0.0, 0.0, 1.0]])

    TRAX = Movement("trax", step[0])
    TRAY = Movement("tray", step[1])
    TRAZ = Movement("traz", step[2])
    ROT  = Movement("rot",  step[3])    
    movements = [TRAX, TRAY, TRAZ, ROT]

    print("--- Inverse Kinematics with Python and ROS ---")

    #Enable torque

    enableTorque()
    #Torque adjustment
    adjustTorque()    

    # For initial position
    print("Initial Position: ", T[:,3])
    q = np.degrees(getInvKin(T, l))
    goal_position_raw = deg2raw(q[codo,:])
    moveJoints(goal_position_raw, q[codo, :])
    
    i = 0
    
    while(True):
        print("T now:\n", T)
        print("Enter key: ")
        key = getKey()
        
        if(key == "w"):
            i = (i+1)%4
            print("Current movement is: ", movements[i].name, "\n")

        elif(key == "s"):
            i = (i-1)%4
            print("Current movement is: ", movements[i].name, "\n")

        elif(key == "d"):
            print("Movement: ", movements[i].name, " of: ", movements[i].step)
            T = moveRobot(T, l, movements[i], 1)
        elif(key == "a"):
            print("Movement: ", movements[i].name, " of: ", -1*movements[i].step)
            T = moveRobot(T, l, movements[i], -1)

if __name__ == '__main__':
    try:
        main()        
    except rospy.ROSInterruptException:
        pass
