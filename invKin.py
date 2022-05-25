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
        out_list[i] = int( ((input_list[i] - min_deg)*1024)/(max_deg-min_deg) )
    return out_list


class Movement:
    def __init__(self, name , step):
         self.name = name
         self.step = step

def moveRobot(T, l, movement, direction):
    codo = 0 #0: down, 1: up
    print("Before: ", T)
    changeMatrix(T, movement, direction)
    print("After: ", T)
    print("Position: ", T[:,3])
    q = np.degrees(getInvKin(T, l))
    goal_position_raw = deg2raw(q[codo,:])
    moveJoints(goal_position_raw, q[codo, :])
    
    #primero a change matrix- dependiendo de si es ROT se hace algo y si no se hace lo otro
    #luego a getInv
    #luego a move joints
    pass

def changeMatrix(T, movement, direction):
    if movement.name == "rot":
        T[0:3, 0:3]
    else:
        if(movement.name == "trax"): row = 0
        elif(movement.name == "tray"): row = 1
        elif(movement.name == "traz"): row = 2

        T[row, 3] = T[row, 3] + movement.step * direction  


def moveJoints(goal_position_raw, q):
    motors_ids = [6,7,8,9]
    for i in range(len(motors_ids)):
        #jointCommand('', motors_ids[i], 'Goal_Position', goal_position_raw[i], 0.5)
        print("Moving ID:", motors_ids[i], " Angle: ", q[i], " Raw: ", goal_position_raw[i])

def main(step: list = [2,20 ,2,2]):
    l = [14.5, 10.7, 10.7, 9]
    

    T = np.array([[0.4096,   -0.8660,    0.2868,   11.7068],
                  [0.7094,    0.5000,    0.4967,   20.2768],
                  [ -0.5736,   -0.0000,    0.8192,   32.4098], 
                  [0, 0, 0, 1]])

    TRAX = Movement("trax", step[0])
    TRAY = Movement("tray", step[1])
    TRAZ = Movement("traz", step[2])
    ROT  = Movement("rot",  step[3])    

    print("--- Inverse Kinematics with Python and ROS ---")
    # For initial position
    print("Initial Position: ", T[:,3])
    q = np.degrees(getInvKin(T, l))
    goal_position_raw = deg2raw(q[0,:])
    moveJoints(goal_position_raw, q[0, :])
    moveRobot(T, l, TRAY, -1)


    # movements = [TRAX, TRAY, TRAZ, ROT]
    # i = 3
    
    # changeMatrix(T, movements[i], -1)
    # print("After:\n", T)

    # while(True):
    #     print("Enter key: ")
    #     key = getKey()
        
    #     if(key == "w"):
    #         i = (i+1)%4s
    #         print("Current movement is: ", movements[i].name, "\n")

    #     elif(key == "s"):
    #         i = (i-1)%4
    #         print("Current movement is: ", movements[i].name, "\n")

    #     elif(key == "d"):
    #         print("Movement: ", movements[i].name, " to: ", movements[i].step)
    #         #moveRobot
    #     elif(key == "a"):
    #         print("Movement: ", movements[i].name, " to: ", -1*movements[i].step)




if __name__ == '__main__':
    try:
        main()        
    except rospy.ROSInterruptException:
        pass
