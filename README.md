Udacity Kinematics Project Writeup
=

##Kuka Arm Diagram##

The first step is to diagram the Kuka Arm configuration using the kr210.urdf file:
<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Kuka%20Arm%20Base%20Diagram.jpg?raw=true" alt="Kuka Arm URDF Layout" width="640px">

The above Kuka Arm layout has new reference frames defined to allow for  the use of Denavit-Hartenberg transformation matrices. From the new layout we derive the DH parameters:

<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Kuka%20DH%20Parameters.jpg?raw=true" alt="Kuka Arm DH parameters" width="640px">

Going forward as a simplification d7 combines the distance from DH6 to the end effector and the grip length.

##Forward Kinematics##
There are a total of seven DH transformation matrices from the base to the end effector(EE), with the resulting final matrix represented as:

**T0\_EE = T0\_1 x T1\_2 x T2\_3 x T3\_4 x T4\_5 x T5\_6 x T6_EE**

One more transformation is required to rotate the end effector orientation back in line with the world orientation:
<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Corrective%20Rotation.jpg?raw=true" alt="Corrective Rotation" width="320px">

This results in a corrective (corr) matrix of: 

**R\_corr = R\_z\_corr x R\_y_corr**
##Inverse Kinematics##
For the inverse kinematics, we first solve for the wrist center. The simulation gives us the end effector position and Euler angles for roll-pitch-yaw. We can use that information to solve for the wrist center. 

To move from the wrist center to the end effector, the wrist center undergoes a roll-pitch-yaw rotation and linear transform the distance from the wrist to the end effector. The roll-pitch-yaw transformation matrix is an intrisic rotation of:

**Ryrp = R\_z(yaw) X R\_y(roll) X R\_x(pitch)**

As the wrist and end effector translation axis are in line with the end effector X axis, the linear transform is a vector with the distance being the X displacement:

**T\_EE\_WC = [X, 0, 0]**

With that, the end effector position can be described as:

**EE\_xyz = Wrist\_xyz + Ryrp x T\_EE\_WC**

Rearranged to solve for the wrist coordinates:

**Wrist\_xyz = EE\_xyz  - Ryrp x T\_EE\_WC**

Now we can start solving for the Theta values for the DH matrix.

##Solving for Theta##

All joints in the arm from the base to the wrist center are coplanar with the plane always being normal to the XY plane.

First, we solve for Thetas 1-3:

**Theta 1**

<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Theta%201%20Calculation.jpg?raw=true" alt="Theta1 Calculation" width="240px">

**Theta 2**

<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Theta%202%20Calculation.jpg?raw=true" alt="Theta2 Calculation" width="640px">

**Theta 3**

<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Theta%203%20Calculation.jpg?raw=true" alt="Theta3 Calculation" width="320px">

Keeping in mind that the roll-pitch-yaw transform is equal to the transforms from joint 1 to joint 6, adjusted for the corrective matrix:

**R0\_6 = Ryrp x R\_corr**

With the first three angles solved for, we can isolate the matrix for the last 3 angles by multiplying both sides by the inverse of the combined rotations from joint 1 to joint 3:

**R3\_6 = R0\_3<sup>-1</sup> x Ryrp x R\_corr**


**Theta 4-6**

<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Theta%204%20-%206.jpg?raw=true" alt="Theta 4-6 calculations" width="640px">

##Code Implementation##

Code used the Sympy matrix methods detailed in the Kinematics classes and labs. For a complete breakdown see comments in **IK\_server.py**

Implementation required several additional additions or modifications. Unless otherwise stated, all changes are in **IK\_server.py**

**Global Variables**

While the project code initially put the definition of matrix terms and variables inside the handle\_calculate\_IK function, during testing the simulation showed greatly improved stability if as much of the matrix terms were defined as globals, presumably so they would not have to be recalculated after each call.

**Transpose vs Inverse Matrix**

While the logic for leading up to solving for Theta 4-6 used an inverse matrix, using the Sympy inverse function, both with and without lower-upper (LU) decomposition, made the simulation unstable and/or inaccurate. Given that **R0\_3** is a normal matrix, the transpose function could also be used, and provided greater stability and accuracy during testing.

**Theta 4-6 Atan2**

Atan2 was used over Atan due to the quadrant limitations of Atan. In addition, a check using sin(theta5) was used to determine an internal consistent  logic for the y and x signs in atan2 when solving for theta4 and theta6.

            
            if sin(theta5) > 0:
                theta4 = atan2(r33,-r13)
                theta6 = atan2(-r22, r21)
            
            else: #sin(theta5) =< 0
                theta4 = atan2(-r33, r13)
                theta6 = atan2(r22, -r21)

**Random Drop-off Position**

To prevent object stacking, randomization was added to the drop-off coordinates in **target\_spawn.py**     

    x_bin = (randint(-2,2)/float(100)) + 0.000 
    z_bin = (randint(-2,2)/float(100)) + 0.000
    rospy.set_param('target_drop_location', {'x': x_bin, 'y': 2.500, 'z': z_bin})

**Gripper Time**

From the Common Questions page, the **trajectory\_sampler.cpp** file had a sleep added to help with cylinder gripping in line 327:

    ros::Duration(6.0).sleep();

##Results##

Several iterations had 8 out of 10 retrievals, with one case going into 12 straight successes. For final submission the following images show 9 straight retrievals from start:

**Two Objects**

<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/2%20objects.png?raw=true" alt="2 objects retrieved" width="640px">

**Six Objects**

<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/6%20objects.png?raw=true" alt="6 objects retrieved" width="640px">

**Nine Objects**

<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/9%20objects.png?raw=true" alt="9 objects retrieved" width="640px">

##Future Improvements##

At this time thoughts for future improvements include a more robust code for optimizing the arm movement, as there were several edge cases where the arm took large movements. Making LU matrix inversion work properly in theory would make for a more mathematically true setup. 