# Lab 3: Inverse Kinematics with Phantom X

This repository contains MATLAB and Python scripts to develop a model of inverse kinematics for the robot Phantom X. This requieres the definition of the robot's workspace and the implementation of a function to calculate the inverse kinematics given an specific position and orientation for the end effector. To check the proper operation of the inverse kinematics, an application of pick and place and motion in the robot's workspace was carried out.

## Authors

* Maria Alejandra Arias Frontanilla
* Camilo Andres Vera Ruiz

## Inverse Kinematics of Phantom X

In previous labs, we have already seen how with a certain configuration _q_, which gives us the corresponding values of the joints, we obtain a certain position and orientation of the end effector. That is what we call forward kinematics. That was pretty good and interesting but now it gets even better and more sophisticated because we are going to do the inverse process. Instead of choosing the values of the joints, we will now choose a certain position and orientation for the end effector and then we will figure out which values of joints, i.e. which configuration, we must have to achieve it.

As you can imagine, this process is a little bit more complicated than the one of forward kinematics, but don't worry, we will understand it step by step.

### First Joint (The easiest one)

Ok! First things first. Let's begin with the first joint. In order to calculate the value of each one of the joints, it's important to know the kind of information that we already have. In our case, we know the position and orientation of the end effector which can be given in a MTH of the tool like the following one:

![Matrix tool](images/matrixT.png)  

Having the position in terms of _xc, yc_ and _zc_, it's easy to find the value for the first joint.

![im1](images/firstJoint.png)  

As we can see in the image above, the value of θ1 for the first joint can be calculated as follows:

θ1 = atan2(yc, xc)

Great! We already have the first joint calculated. Let's go for the other 3.

### Kinematic Decoupling

For us to calculate the second and third joint, we will use something called kinematic decoupling in the wrist of our Phantom X. Wait a moment... Does the Phantom have a wrist? Yes! It does. And it has an elbow too.

The wrist is located in the fourth joint and we will have to specify which position corresponds to it.

With the MTH of the tool we have access to the values of the vector _|a|_ which in the previous image corresponds to the axis z on the tool tip. Our wrist is located close to the end point (_xc, yc, zc_) but with a negative displacement of value _L4_ (which corresponds to the length of the fourth link)in the same direction as the vector |a|. And that's exactly the way we will calculate it.

![im2](images/PosW.png)

The position of W, Phantom X's wrist, is then calculated as follows:

![im3](images/eq_w.png)

At this point you may be wondering why we calculated _W_. Well, as everything in life, this also has a purpose and it's to allow us to find the values of the second and third joint seeing it as a 2R mechanism.

### Finding Second and Third Joint seeing it as a 2R

Don't you see it yet? No problem. The image below will help you to identify with ease what we mean with a 2R mechanism

![im4](images/2_R.png)

If we take just the second and third joint with the end effector as the wrist, we have now a much simpler problem which can be solved using trigonometry.

With the triangle that is formed and using the cosine law and getting help of our old dear friend Pythagoras, we get to the following values for θ2 and θ3, corresponding to the second and third joint, respectively.

![im6](triangle_2R.png)

The values for θ2 and θ3 are shown below.

![im7](images/theta_2.png)

![im8](images/theta_3.png)

### Fourth Joint (we are almost done)

Now we are only missing the fourth joint, the last one. In this case we will use again our vector _|a|_. We will call φ the angle between _|a|_ and the plain formed by axes _x_ and _y_. If we see it closely, we can see that φ is equal to the sum of the angles θ2, θ3 and θ4. In this way we can find θ4 as follows:

θ4 = φ - θ2 - θ3

Now, we just have to figure out how to measure that φ.

Since we have all the components of our vector _|a|_, we can calculate φ using again Pythagoras.

And we are finally done :D

This whole process of inverse kinematics is implemented in MATLAB in the file _invKinPhantom.m_ and in Python _getInvKin.py_.

__Note__: The values of θ2 and θ4 are not directly the values that correspond to _q2_ and _q4_ since in the DH Parameters exists an offset that we have to take into account.

## Workspace of Phantom X and matlab inverse kinematics

A robot workspace is defined as the reachable space for the robot tool, and calculate it by an analytically method can be very difficult, for that reason a numeric method is used bellow using MATLAB and Peter corke toolbox.

A Monte Carlo method approach is used to do the calculation, the main idea is to get the position of the tool for random positions of the articulations using forward kinematics with the peter corke toolbox. The code showed bellow is used to create the robot instance with the DH parameters and the MTH for the tool using NOA convention.

```matlab
% Robot definition
l = [14.5, 10.63, 10.65, 8.97]; % Length of links
L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');

%Tool orientation following NAO Convention
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];
```

The next step is to generate 4 vectors with random position values for each joint (within the limits of the joints), and calculate all the final positions for each value set by forward kinematics, and save the x,y,z coordinates from the the MTH into vectors.

Finally MATLAB boundary function is used to get the specific boundary points that contain the rest and plot them in a 3d representation. The plot and the code to achieve that is showed bellow.

```matlab
%% 3d workspace
rng('default')
samples = 1000000;
q1 = ((3*pi/2)*rand(samples,1))-((3*pi/4)*(ones(samples,1)));
q2 = ((3*pi/2)*rand(samples,1))-((3*pi/4)*(ones(samples,1)));
q3 = ((3*pi/2)*rand(samples,1))-((3*pi/4)*(ones(samples,1)));
q4 = ((3*pi/2)*rand(samples,1))-((3*pi/4)*(ones(samples,1)));

x_pos = zeros(samples,1);
y_pos = zeros(samples,1);
z_pos = zeros(samples,1);
disp(x_pos(1))

for i = 1:1:samples
    MTH = PhantomX.fkine([q1(i) q2(i) q3(i) q4(i)]);
    x_pos(i) = MTH(1,4);
    y_pos(i) = MTH(2,4);
    z_pos(i) = MTH(3,4);
end

%% 3d boundary
bound=boundary(x_pos,y_pos,z_pos);
trisurf(bound,x_pos,y_pos,z_pos)
```

![im9](images/wsplot.png)

The plot as expected showed some kind of sphere with a little bit of deformation at the bottom due to the joint range limited to a range less than 360 degrees.

Another way to do inverse kinematics is by using Peter corke toolbox, in the [documentation](https://www.petercorke.com/RTB/r9/html/SerialLink.html) some methods shown below are explained.

1. `kine6s`: inverse kinematics for 6-axis spherical wrist revolute robot
2. `ikine`:	inverse kinematics using iterative numerical method
3. `ikunc`:	inverse kinematics using optimization
4. `ikcon`:	inverse kinematics using optimization with joint limits
5. `ikine_sym`: analytic inverse kinematics obtained symbolically

This methods relies more in a numerical method approach instead of an analytical one making them more suitable to a different robots, instead of having to manually analyze every robot to be able to do inverse kinematics. 

## Analysis questions

_Sabiendo que el robot Phantom X posee 4 GDL, de los cuales 3 corresponden a posición, el GDL restante proporciona una medida independiente para un ángulo de orientación (asuma orientación en ángulos fijos). ¿De qué ángulo de orientación se trata?_

The Phantom X robot, is only capable of move _pitch_ orientation because all links are aligned and share the same plane in all configurations, this can be verified looking the python program in the section `Motion in robot workspace`.

_¿Cuántas soluciones posibles existen para la cinemática inversa del manipulador Phantom X?_

The analysis showed before about inverse kinematics, identifies two posible solution corresponding to up-elbow and down-elbow.

_Consulte en qué consiste el espacio diestro de un manipulador._

The configuration space is the subregion of the robot workspace that is reachable by all the orientations controlable by the robot, that is for 6DoF anthropomorphic robot all orientations, and for PhantomX, pitch orientation.

## Pick and Place Application

To put in practice inverse kinematics a pick and place application is developed.
The idea is to move the 3d printed parts showed bellow. The two start at a different position, the one is moved to a defined position, and then the other one is inserted in the first one.

![im20](images/parts.png)

First we prepare all the required libraries, in this case we use the peter corke toolbox for python to generate MTH at each position, plus ROS to communicate with the robot and numpy to do all the math. A custom library develop by us called _getInvKin_ is used to calculate inverse kinematics.

```python
#!/usr/bin/env python3

import numpy as np
import time

# ROS libraries
import rospy
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand


#import petercorke toolbox for visualization
from roboticstoolbox import *
from spatialmath.pose3d import *

#inverse kinematics library
from getInvKin import *
```

The next two functions used in the previous labs are used too, `deg2raw` allow us to convert an array of joints positions in degrees to 12 bits values from 0 to 1023 that are used to control the motors. `jointCommand` is used to send data to the robot using a ROS service.

```python
def deg2raw(input_list: list = [0,0,0,0], min_deg: int = -150, max_deg: int = 150)->list:
    """
    Convert degrees array to 10 bit motor control value.
    """

    out_list = [0,0,0,0]
    for i in range(len(input_list)):
        out_list[i] = int( ((input_list[i] - min_deg)*1024)/(max_deg-min_deg) )
    return out_list



def jointCommand(command, id_num, addr_name, value, time):
    
    """
    Make a request to a the "dynamixel_command" ros service to modify a  
    parameter such as position or torque of the motor specified by the "id_num" 
    parameter.
    """
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))
```

Three new functions are created, the first one `getQ` takes x, y, z position and a pitch orientation, then a MTH is generated using peter corke toolbox, to finally use the inverse kinematic library to get a vector of joints angles at up-elbow configuration.

```python

def getQ(x:float=4, y:float=4, z:float=5, phi:float=-np.pi)->np.array:
    """
    Generates the MTH matrix to execute inverse kinematics and returns
    an array with the joint position.
    """
    
    MTH = SE3(x,y,z)*SE3.Ry(phi)
    print(MTH)
    q = getInvKin(MTH.data[0], phantomX_l)
    return q[1]
```
`setFullPostion` is used to send a full array of joints positions to the robot, first converting them 12 bit values and then using the `jointCommand` function to send all the data.

```python
def setFullPostion(q:np.array = np.array([0, 0, 0, 0]) )->None:
    """
    Set all the joints of the robot to the position specified by the input array
    """
    #phantomX.plot(q)
    q_raw=deg2raw(np.degrees(q))
    print(q_raw)
    jointCommand('', motors_ids[0], 'Goal_Position', q_raw[0], 0)
    jointCommand('', motors_ids[1], 'Goal_Position', q_raw[1], 0)
    jointCommand('', motors_ids[2], 'Goal_Position', q_raw[2], 0)
    jointCommand('', motors_ids[3], 'Goal_Position', q_raw[3], 0)
```

`setGripperPosition` sends a specific 12 bit value to the gripper to allow grab the parts.

```python
def setGripperPosition(servoValue:int=0)->None:
    """
    Set the gripper position to the specified 0 to 1023 value
    """

    jointCommand('', motors_ids[4], 'Goal_Position', servoValue, 0)
```

To do the the movement first we need to define a trayectory. the image bellow describe the main idea, the robot first move to the base part location and put it in the location at the end of the fourth movement, and then moves the circular part from its position at the end of the sixth movement to finally iserting over the base part.

![im21](images/tray.png)

With the robotics knowledge at this point, the only way to do that is define some intermediate points and separate the trayectory in steps,
taking into account that is not posible to garantee the position of the robot between the points, making the adjusment of the number points critical.

Resume the coding, the main function define some parameters to manually adjust the trayectory, and meeting precision requirements by increasing the resolution of the movement, plus the proper gripper values to be able to grab the parts.

```python

def main():
    
    # Movement parameters 
    home_x_position = 10
    home_y_position =  0

    home_to_base_x_postion = 10
    home_to_base_y_postion = -12

    base_x_position = 0
    base_y_position =-12

    home_to_load_x_postion = 10
    home_to_load_y_postion = 12

    load_x_position = 0
    load_y_position = 12

    free_movement_safe_height = 12
    base_gripper_height = 4
    load_gripper_height = 4.5
    load_over_base_height = 7

    # Gripper values
    base_gripper_closed_value = 348
    load_gripper_closed_value = 348
    gripper_open_value = 512


    # Number of steps of each movement phase
    home_to_base_x_steps = 10
    home_to_base_y_steps = 10
    home_to_load_x_steps = 10
    home_to_load_y_steps = 10

    pick_up_base_down_steps = 20
    pick_up_base_up_steps = 20
    place_base_down_steps = 20
    home_up_steps = 20
    pick_up_load_down_steps = 20
    pick_up_load_up_steps = 20
    place_load_down_steps = 20

```

Here comes the long complex part, due to the necesity of several parameter to properly adjust the movement, and the limitation of python to be an interpreted programing language slow at runtime, each step of the movement must be precalculated to avoid calculation during the movement and make the robot slow (This is more caution rather that a proven fact).

All the calculation of each step of the movement are showed in the code bellow, all of them are linear movements intepolated by a number of defined steps in the parameter list.

```python
  # Trajectory generation:


    q_vector1 = np.zeros((1 + pick_up_base_down_steps+home_to_base_y_steps+home_to_base_x_steps,4))
    q_vector2 = np.zeros((pick_up_base_up_steps + place_base_down_steps+home_to_base_x_steps+home_to_base_y_steps,4))
    q_vector3 = np.zeros((home_up_steps + pick_up_load_down_steps+home_to_load_y_steps+home_to_load_x_steps,4))
    q_vector4 = np.zeros((pick_up_load_up_steps +home_to_load_x_steps+home_to_load_y_steps+ place_load_down_steps,4))

    # 1-homming
    q_vector1[0] = getQ(x = home_x_position,
                        y = home_y_position,
                        z = free_movement_safe_height
                   )

    # 1-home->base_position

    for i in range(home_to_base_y_steps):
        q_vector1[i+1] = getQ(x=home_x_position,
                              y=home_y_position+
                              (((home_to_base_y_postion-home_y_position)
                              *(i+1))/home_to_base_y_steps),
                              z = free_movement_safe_height
                         )
    for i in range(home_to_base_x_steps):
        q_vector1[i+home_to_base_y_steps+1] = getQ(x=home_to_base_x_postion+
                                                 (((base_x_position-home_to_base_x_postion)
                                                 *(i+1))/home_to_base_x_steps),
                                                 y = home_to_base_y_postion,
                                                 z = free_movement_safe_height
                                            )
    # 2 pick-up-base_down

    for i in range(pick_up_base_down_steps):
        q_vector1[i+home_to_base_y_steps + home_to_base_x_steps+1] = getQ(x=base_x_position,
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

    for i in range(home_to_base_x_steps):
        q_vector2[i+pick_up_base_up_steps] = getQ(x=base_x_position+
                                                 (((home_to_base_x_postion-base_x_position)
                                                 *(i+1))/home_to_base_x_steps),
                                                 y = base_y_position,
                                                 z = free_movement_safe_height
                                            )
    for i in range(home_to_base_y_steps):
        q_vector2[pick_up_base_up_steps+home_to_base_x_steps+i] = getQ(x=home_to_base_x_postion,
                                                                       y=home_to_base_y_postion+
                                                                       (((home_y_position-home_to_base_y_postion)
                                                                       *(i+1))/home_to_base_y_steps),
                                                                       z = free_movement_safe_height
                                                                   ) 
    # 5-place-base_down

    for i in range(place_base_down_steps):
        q_vector2[i+pick_up_base_up_steps+home_to_base_x_steps+home_to_base_x_steps] = getQ(x=home_x_position,
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

    for i in range(home_to_load_y_steps):
        q_vector3[home_up_steps+i] = getQ(x=home_x_position,
                                          y=home_y_position+
                                          (((home_to_load_y_postion-home_y_position)
                                          *(i+1))/home_to_load_y_steps),
                                          z = free_movement_safe_height
                                      )
    for i in range(home_to_base_x_steps):
        q_vector3[i+home_to_load_y_steps+home_up_steps] = getQ(x=home_to_load_x_postion+
                                                               (((load_x_position-home_to_load_x_postion)
                                                               *(i+1))/home_to_base_x_steps),
                                                               y = home_to_load_y_postion,
                                                               z = free_movement_safe_height
                                                           )

    # 8-pick-up-load_down
    for i in range(pick_up_load_down_steps):
        q_vector3[i+home_to_load_y_steps+home_to_base_x_steps+home_up_steps] = getQ(x=load_x_position,
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

    for i in range(home_to_load_x_steps):
        q_vector4[i+pick_up_load_up_steps] = getQ(x=load_x_position+
                                                 (((home_to_load_x_postion-load_x_position)
                                                 *(i+1))/home_to_load_x_steps),
                                                 y = load_y_position,
                                                 z = free_movement_safe_height
                                            )
    for i in range(home_to_load_y_steps):
        q_vector4[pick_up_load_up_steps+home_to_load_x_steps+i] = getQ(x=home_to_load_x_postion,
                                                                       y=home_to_load_y_postion+
                                                                       (((home_y_position-home_to_load_y_postion)
                                                                       *(i+1))/home_to_load_y_steps),
                                                                       z = free_movement_safe_height
                                                                   ) 


    #11 place-load-down

    for i in range(place_load_down_steps):
        q_vector4[i+pick_up_base_up_steps+home_to_load_x_steps+home_to_load_y_steps] = getQ(x=home_x_position,
                                                                                              y=home_y_position,
                                                                                              z=free_movement_safe_height-
                                                                                              (((free_movement_safe_height-load_over_base_height)
                                                                                              *(i+1))/place_load_down_steps)
                                                                                        )

```

finally all the points calculated are sent to the robot to execute the movement, adding a short pause between steps to make the robot more stable due to vibration generated assembly looseness.

```python
  
    # Motors configuration

    jointCommand('', motors_ids[0], 'Torque_Limit', 300, 0)
    jointCommand('', motors_ids[1], 'Torque_Limit', 500, 0)
    jointCommand('', motors_ids[2], 'Torque_Limit', 300, 0)
    jointCommand('', motors_ids[3], 'Torque_Limit', 300, 0)
    jointCommand('', motors_ids[4], 'Torque_Limit', 300, 0)
    
    jointCommand('', motors_ids[0], 'Torque_Enable', 1, 0)
    jointCommand('', motors_ids[1], 'Torque_Enable', 1, 0)
    jointCommand('', motors_ids[2], 'Torque_Enable', 1, 0)
    jointCommand('', motors_ids[3], 'Torque_Enable', 1, 0)
    jointCommand('', motors_ids[4], 'Torque_Enable', 1, 0)

    # Initial gripper open position
    setGripperPosition(gripper_open_value)

    # Movement
    time.sleep(1)
    for i in q_vector1:
        setFullPostion(i)    
    setGripperPosition(base_gripper_closed_value)

    time.sleep(0.8)         # Stabnilization delay
    for i in q_vector2:
        setFullPostion(i)
    setGripperPosition(gripper_open_value)
    time.sleep(0.8)
    for i in q_vector3:
        setFullPostion(i)
    setGripperPosition(load_gripper_closed_value)
    time.sleep(0.8)
    for i in q_vector4:
        setFullPostion(i)
    setGripperPosition(gripper_open_value)
    time.sleep(0.5)
```

The result can be found [here](https://www.youtube.com/watch?v=NGFBQhBNVEc).

## Motion in robot's workspace

The last application that we developed for this lab relates to the whole robot's workspace. Using keyboard inputs, we will control the position of the end effector.

To create the movements that will allows us to control the PhantomX, the class _Movement_ was created in the file _invKin.py_.

```python
class Movement:
    def __init__(self, name , step):
         self.name = name
         self.step = step
```

In this class we define the name of the movement and the step, which refers to the change of value that we will have.

For our application we have 4 different movements:

* TRAX = Axis-x translation with a step of 1 cm.
* TRAY = Axis-y translation with a step of 1 cm.
* TRAZ = Axis-z translation with a step of 1 cm.
* ROT  = Rotation with a step of 10 degrees.

All of these movements are created and added to a list called movements

```python
TRAX = Movement("trax", step[0])
    TRAY = Movement("tray", step[1])
    TRAZ = Movement("traz", step[2])
    ROT  = Movement("rot",  step[3])    
    movements = [TRAX, TRAY, TRAZ, ROT]
```

The creation of this list will help us to iterate through all the movements depending on the keyboard input. If we get a _w_, we will move to the next movement. If we get an _s_, we will move to the previous movement. In both cases we will print the name of the movement we are in. The exciting part comes when we get a _d_ or an _a_. For those cases, we will move a positive or negative step, respectively, in the movement we are in.

```python
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
```

As you can see from the function above, all the magic occurs in the function _moveRobot_. This function receives as parameters the MTH of the end effector, _T_, the length of the links saved in the list _l_, the corresponding movement and a 1 if the step will be positive or a -1 in case it is negative.

### How do we move our Phantom X?

Well, the interesting part comes when we try to understand how the function _moveRobot_ works.

```python
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
```

The first thing that we can see is the choice of the elbow. We can have elbow up or down for our 2R mechanism explained in the section of inverse kinematics. After this, we copy the MTH of the end effector in the variable _previous T_ just in case we reach a position that is outside the robot's workspace. After this, we call the function _changeMatrix_, (we will see it later, don't worry), which updates the MTH. Having the MTH, we can now call the inverse kinematics function called _getInvKin_ and transform the answer that we get from radians to degrees and save it in the variable _q_. This last variable contains the values of the joints in both configurations: elbow up and elbow down. That's why we have to choose which one we want to use.

With the value of _q_ we can now call our mapping function that converts degrees to raw data which is directly received by the joints of our Phantom X (-150 to 150 degrees is now 0 to 1023). And with this information we can now call our function _moveJoints_ and see the magic happening. But before doing that, let's explain the functions that we are missing.

### Change matrix function

With this function, we modify the values of the MTH that we get. If we have the movement of rotation, we multiply the rotation matrix of the MTH by a rotation in the y-axis on an angle equal to step, taking into account if it's positive or negative. (Do you remember the 1 and -1? Yeah, it's exactly here where we use it).

On the other hand, if we get a translation movement, we will change the component of the vector of position depending on the axis that we are interested in and that's it :)

```python
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
```

### Moving the joints

Finally with the function, we call the service dynamixel command and pass the corresponding parameters for each joint using the raw data that we calculated.

```python
def moveJoints(goal_position_raw, q):
    motors_ids = [1,2,3,4]
    for i in reversed(range(len(motors_ids))):
        jointCommand('', motors_ids[i], 'Goal_Position', goal_position_raw[i], 0.5)
        print("Moving ID:", motors_ids[i], " Angle: ", q[i], " Raw: ", goal_position_raw[i])
```

Now we have all the ingredients ready for this great recipe. Let's see how it tastes in the following link: [Video - Motion in robot's workspace](https://www.youtube.com/watch?v=eJQ3HbZY1dU).


## Conclusions

* At pick and place application with PhantomX robot, some problems were identified in the orientation that were initially attributed to code, but with a more detailed analysis, some physical problems with the robot were identified, such as joint backslash due to looseness in assembly and possibly servomotors gearbox wear, that modified severely the robot orientation thanks to the links weight, especially when the center of gravity was further from the base, so is it highly recommended to advise to the next students using this robots to take in account these problems that can lead to wrong conclusions with the code.

* Learning to use tools may require a little bit of time but after a while you will figure out how it works. We, as engineering students, shouldn't worry too much about this. The thing that really matters and that can make a big difference is the understanding of the core concepts on which the whole system is based. In this case, a good understanding of the inverse kinematics allowed us to do our own implementation which was fundamental to implement the solution of both applications.
