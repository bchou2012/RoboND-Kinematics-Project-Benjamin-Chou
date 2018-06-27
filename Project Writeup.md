Udacity Kinematics Project Writeup
=

##Kuka Arm Diagram##

The first step is to diagram the Kuka Arm configuration using the kr210.urdf file:
<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Kuka%20Arm%20Base%20Diagram.jpg?raw=true" alt="Kuka Arm URDF Layout" width="640px">

The above Kuka Arm layout is reconfigured to allow for  the use of Denavit-Hartenberg transformation matrices. From the new layout we derive the DH parameters:

<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Kuka%20DH%20Parameters.jpg?raw=true" alt="Kuka Arm DH parameters" width="640px">

##Forward Kinematics##
There are a total of seven DH transformation matrices from the base to the end effector(EE), with the resulting final matrix represented as:

**T0\_EE = T0\_1 x T1\_2 x T2\_3 x T3\_4 x T4\_5 x T5\_6 x T6_EE**

One more transformation is required to rotate the end effector orientation back in line with the world orientation:
<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Corrective%20Rotation.jpg?raw=true" alt="Corrective Rotation" width="480px">

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

**Theta 1**

All joints in the arm from the base to the wrist center are coplanar with the plane always being normal to the XY plane. 

For Joint 1, that means it is always a straight line projected onto the XY plane to the wrist center. Theta 1 solves as:

<img src="https://github.com/bchou2012/RoboND-Kinematics-Project-Benjamin-Chou/blob/master/images/Theta%201%20Calculation.jpg?raw=true" alt="Theta1 Calculation" width="480px">

**Theta 2**

<img src="" alt="" width="640px">
<img src="" alt="" width="640px">