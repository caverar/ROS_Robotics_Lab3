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

![](images/firstJoint.png)  

As we can see in the image above, the value of θ1 for the first joint can be calculated as follows:

θ1 = atan2(yc, xc)

Great! We already have the first joint calculated. Let's go for the other 3.

### Kinematic Decoupling

For us to calculate the second and third joint, we will use something called kinematic decoupling in the wrist of our Phantom X. Wait a moment... Does the Phantom have a wrist? Yes! It does. And it has an elbow too. 

The wrist is located in the fourth joint and we will have to specify which position corresponds to it. 

With the MTH of the tool we have access to the values of the vector _|a|_ which in the previous image corresponds to the axis z on the tool tip. Our wrist is located close to the end point (_xc, yc, zc_) but with a negative displacement of value _L4_ (which corresponds to the length of the fourth link)in the same direction as the vector |a|. And that's exactly the way we will calculate it.

![](images/PosW.png)  

The position of W, Phantom X's wrist, is then calculated as follows:

![](images/eq_w.png)  

At this point you may be wondering why we calculated _W_. Well, as everything in life, this also has a purpose and it's to allow us to find the values of the second and third joint seeing it as a 2R mechanism.

### Finding Second and Third Joint seeing it as a 2R

Don't you see it yet? No problem. The image below will help you to identify with ease what we mean with a 2R mechanism

![](images/2_R.png)  

If we take just the second and third joint with the end effector as the wrist, we have now a much simpler problem which can be solved using trigonometry. 

With the triangle that is formed and using the cosine law and getting help of our old dear friend Pythagoras, we get to the following values for θ2 and θ3, corresponding to the second and third joint, respectively. 

![](triangle_2R.png)  

The values for θ2 and θ3 are shown below.

![](images/theta_2.png)  

![](images/theta_3.png)  

### Fourth Joint (we are almost done)

Now we are only missing the fourth joint, the last one. In this case we will use again our vector _|a|_. We will call φ the angle between _|a|_ and the plain formed by axes _x_ and _y_. If we see it closely, we can see that φ is equal to the sum of the angles θ2, θ3 and θ4. In this way we can find θ4 as follows:

θ4 = φ - θ2 - θ3 

Now, we just have to figure out how to measure that φ. 

Since we have all the components of our vector _|a|_, we can calculate φ using again Pythagoras. 

And we are finally done :D

This whole process of inverse kinematics is implemented in MATLAB in the file _invKinPhantom.m_ and in Python _getInvKin.py_. 

__Note__: The values of θ2 and θ4 are not directly the values that correspond to _q2_ and _q4_ since in the DH Parameters exists an offset that we have to take into account.


## Workspace of Phantom X and...
Esboce el espacio de trabajo del robot Phantom X.
Consulte los métodos disponibles en el toolbox para determinar la cinemática inversa de un manipulador.
Análisis:
Sabiendo que el robot Phantom X posee 4 GDL, de los cuales 3 corresponden a posición, el GDL restante
proporciona una medida independiente para un ángulo de orientación (asuma orientación en ángulos fijos).
¿De qué ángulo de orientación se trata?
¿Cuántas soluciones posibles existen para la cinemática inversa del manipulador Phantom X ?
Consulte en qué consiste el espacio diestro de un manipulador.

## Pick and Place Application

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

As you can see from the function above, all the magic occurs in the fuction _moveRobot_. This function receives as parameters the MTH of the end effector, _T_, the length of the links saved in the list _l_, the corresponding movement and a 1 if the step will be positive or a -1 in case it is negative.

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

Now we have all the ingredients ready for this great recipe. Let's see how it tastes in the following link: [Video - Motion in robot's workspace ](https://www.youtube.com/watch?v=eJQ3HbZY1dU).

## Conclusions

Learning to use tools may require a little bit of time but after a while you will figure out how it works. We, as engineering students, shouldn't worry too much about this. The thing that really matters and that can make a big difference is the understanding of the core concepts on which the whole system is based. In this case, a good understanding of the inverse kinematics allowed us to do our own implementation which was fundamental to implement the solution of both applications. 