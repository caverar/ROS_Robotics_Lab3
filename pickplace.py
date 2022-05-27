#!/usr/bin/env python3

# ROS libraries
#import rospy
#from std_msgs.msg import String
#from dynamixel_workbench_msgs.srv import DynamixelCommand

import numpy as np

import time

#inverse kinematics
from getInvKin import *

# import petercorke toolbox for visualization
from roboticstoolbox import *
from spatialmath.pose3d import *

motors_ids = [6,7,8,9,10]


phantomX_l = [14.5, 10.63, 10.65, 8.97] # Length of links
phantomX_q = [0,0,0,0]

phantomX = DHRobot(
    [
        RevoluteDH(alpha =np.pi/2, a=0, d=phantomX_l[0], offset=0, qlim=[-3*np.pi/4, 3*np.pi/4]),
        RevoluteDH(alpha =0, a=phantomX_l[1], d=0, offset=np.pi/2, qlim=[-3*np.pi/4, 3*np.pi/4]),
        RevoluteDH(alpha =0, a=phantomX_l[2], d=0, offset=0, qlim=[-3*np.pi/4, 3*np.pi/4]),
        RevoluteDH(alpha =0, a=0, d=0, offset=0, qlim = [-3*np.pi/4, 3*np.pi/4]),
    ], 
    name="PhantomX",
    tool = SE3(np.array([[ 0,  0, 1, phantomX_l[3]], 
                         [-1,  0, 0, 0], 
                         [ 0, -1, 0, 0], 
                         [ 0,  0, 0, 1]]))
)


def signedRads2raw(input_list: list = [0,0,0,0], 
                   min_rad: float = -3*np.pi/4, 
                   max_rad: float = 3*np.pi/4)->list:
    out_list = [0,0,0,0]
    for i in range(len(input_list)):
        out_list[i] = int( ((input_list[i] - min_rad)*1023)/(max_rad-min_rad) )
    return out_list

# def jointCommand(command, id_num, addr_name, value, time):
    
#     """
#     Make a request to a the "dynamixel_command" ros service to modify a  
#     parameter such as position or torque of the motor specified by the "id_num" 
#     parameter.
#     """
#     rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
#     try:        
#         dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
#         result = dynamixel_command(command,id_num,addr_name,value)
#         rospy.sleep(time)
#         return result.comm_result
#     except rospy.ServiceException as exc:
#         print(str(exc))




def getQ(x:float=4, y:float=4, z:float=5, phi:float=-np.pi)->np.array:
    
    MTH = SE3(x,y,z)*SE3.Ry(phi)
    q = getInvKin(MTH.data[0], phantomX_l)
    return q[0]

def setFullPostion(q:np.array = np.array([0, 0, 0, 0]) )->None:
    
    phantomX.plot(q)
    q_raw=signedRads2raw(q)
    print(q)

    # jointCommand('', motors_ids[0], 'Goal_Position', q_raw[0], 0)
    # jointCommand('', motors_ids[1], 'Goal_Position', q_raw[1], 0)
    # jointCommand('', motors_ids[2], 'Goal_Position', q_raw[2], 0)
    # jointCommand('', motors_ids[3], 'Goal_Position', q_raw[3], 0)

def setGripperPosition(servoValue:int=0)->None:
    print("TODO: "+str(servoValue))
    #jointCommand('', motors_ids[4], 'Goal_Position', value, 0)

def main():
    
    home_x_position = 18
    home_y_position =  0

    base_x_position = 0
    base_y_position =-18

    load_x_position = 0
    load_y_position = 18



    free_movement_safe_height = 6       # pending to test
    base_gripper_height = 4             # pending to test
    load_gripper_height = 1.5           # pending to test
    load_over_base_height = 2           # pending to test
    
    base_gripper_closed_value = 300     # pending to test
    load_gripper_closed_value = 300     # pending to test
    gripper_open_value = 0              # pending to test

    pick_up_base_down_steps = 5
    pick_up_base_up_steps = 5
    place_base_down_steps = 5
    home_up_steps = 5
    pick_up_load_down_steps = 5
    pick_up_load_up_steps = 5
    place_load_down_steps = 5

    # Data generation
    q_vector1 = np.zeros((2 + pick_up_base_down_steps,4))
    q_vector2 = np.zeros((1 + pick_up_base_up_steps + place_base_down_steps,4))
    q_vector3 = np.zeros((1 + home_up_steps + pick_up_load_down_steps,4))
    q_vector4 = np.zeros((1 + pick_up_load_up_steps + place_load_down_steps,4))


    # 1-homming
    q_vector1[0] = getQ(x = home_x_position,
                        y = home_y_position,
                        z = free_movement_safe_height
                   )

    # 1-home->base_position
    q_vector1[1] = getQ(x = base_x_position,
                        y = base_y_position,
                        z = free_movement_safe_height
                   )
    # 2 pick-up-base_down

    for i in range(pick_up_base_down_steps):
        q_vector1[i+2] = getQ(x=base_x_position,
                              y=base_y_position,
                              z=free_movement_safe_height-
                              (((free_movement_safe_height-base_gripper_height)
                             *(i+1))/pick_up_base_down_steps)
                         )
    # 3 pick-up-base_up

    for i in range(pick_up_base_up_steps):
        q_vector2[i] = getQ(x=base_x_position,
                            y=base_y_position,
                            z=base_gripper_height+
                            (((free_movement_safe_height-base_gripper_height)
                            *(i+1))/pick_up_base_up_steps)
                       )

    # 4-base_position->home
    q_vector2[pick_up_base_up_steps] = getQ(x = home_x_position,
                                            y = home_y_position,
                                            z = free_movement_safe_height
                                       )

    # 5-place-base_down

    for i in range(place_base_down_steps):
        q_vector2[i+pick_up_base_up_steps+1] = getQ(x=home_x_position,
                                                    y=home_y_position,
                                                    z=free_movement_safe_height-
                                                    (((free_movement_safe_height-base_gripper_height)
                                                    *(i+1))/place_base_down_steps)
                                               )

    # 6-home-up

    for i in range(home_up_steps):
        q_vector3[i] = getQ(x=home_x_position,
                            y=home_y_position,
                            z=base_gripper_height+
                            (((free_movement_safe_height-base_gripper_height)
                            *(i+1))/home_up_steps)
                       )
    
    # 7-home->load
    q_vector3[home_up_steps] = getQ(x = load_x_position,
                                    y = load_y_position,
                                    z = free_movement_safe_height
                                )
    # 8-pick-up-load_down
    for i in range(pick_up_load_down_steps):
        q_vector3[i+home_up_steps+1] = getQ(x=load_x_position,
                                            y=load_y_position,
                                            z=free_movement_safe_height-
                                            (((free_movement_safe_height-load_gripper_height)
                                            *(i+1))/pick_up_load_down_steps)
                                       )

    #9-pick-up-load_up
    for i in range(pick_up_load_up_steps):
        q_vector4[i] = getQ(x=load_x_position,
                            y=load_y_position,
                            z=load_gripper_height+
                            (((free_movement_safe_height-load_gripper_height)
                            *(i+1))/pick_up_load_up_steps)
                       )
    
    #10 load->home
    q_vector4[pick_up_load_up_steps] = getQ(x = home_x_position,
                                            y = home_y_position,
                                            z = free_movement_safe_height
                                       )

    #11 place-load-down

    for i in range(place_load_down_steps):
        q_vector4[i+pick_up_base_up_steps+1] = getQ(x=home_x_position,
                                                    y=home_y_position,
                                                    z=free_movement_safe_height-
                                                    (((free_movement_safe_height-load_over_base_height)
                                                    *(i+1))/place_load_down_steps)
                                               )


    q_vector = np.concatenate((q_vector1,q_vector2,q_vector3,q_vector4),axis=0)
    print(q_vector)

    for i in q_vector:
        setFullPostion(q_vector)
        time.sleep(0.1)
        

def tets():
    phantomX.plot([0,0,0,0],jointaxes = True, backend="pyplot")
    pos = [13,13,4]
    for i in range(100):
        setFullPostion(pos[0], pos[1], pos[2], phi=-np.pi)
        time.sleep(0.1)
        pos[0]-=0.1
        pos[1]-=0.1
        #pos[2]+=0.1
    
    x = input()


if __name__ == '__main__':
    #try:
        main()
    # except rospy.ROSInterruptException:
    #     pass