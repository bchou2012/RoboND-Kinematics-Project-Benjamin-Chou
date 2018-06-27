from time import time
from mpmath import *
from sympy import *
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''


test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}
              
###FK Setup as globals
th1, th2, th3, th4, th5, th6, th7 = symbols('th1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
al0, al1, al2, al3, al4, al5, al6 = symbols('al0:7')
roll, pitch, yaw = symbols('roll, pitch, yaw')
q1, q2, q3 = symbols('q1:4')

deg_to_rad = pi/180
rad_to_deg = 180/pi

origin = Matrix([[0],[0],[0],[1]])
#DH Parameters
s = {al0:0, a0: 0, d1: 0.75,
    al1: -pi/2, a1: 0.35, d2: 0, th2: th2-(pi/2),
    al2: 0, a2: 1.25, d3: 0,
    al3: -pi/2, a3: -0.054, d4: 1.5,
    al4: pi/2, a4: 0, d5: 0,
    al5: -pi/2, a5: 0, d6: 0,
    al6: 0, a6: 0, d7: 0.303, th7: 0}
    
#DH to RVis Rotation Parameters
Corr_var = {q2: -pi/2, q3: pi}      
 
#Joint Rotation Limits:
th1_lim = {'max': float(185* deg_to_rad), 'min': float(-185*deg_to_rad)}
th2_lim = {'max': float(85* deg_to_rad), 'min': float(-45*deg_to_rad)}
th3_lim = {'max': float(155* deg_to_rad), 'min': float(-210*deg_to_rad)}
th4_lim = {'max': float(350* deg_to_rad), 'min': float(-350*deg_to_rad)}
th5_lim = {'max': 125* deg_to_rad.evalf(), 'min': -125*deg_to_rad.evalf()}
th6_lim = {'max': float(350* deg_to_rad), 'min': float(-350*deg_to_rad)}

#XYZ Rotation Matrices
R_x = Matrix([[ 1,              0,        0],
  		[0,        cos(q1), -sin(q1)],
  		[0,        sin(q1),  cos(q1)]])

R_y = Matrix([[ cos(q2),        0,  sin(q2)],
              [       0,        1,        0],
              [-sin(q2),        0,  cos(q2)]])

R_z = Matrix([[ cos(q3), -sin(q3),        0],
              [ sin(q3),  cos(q3),        0],
              [ 0,              0,        1]])
#XYZ Rotation+Translation Matrices
Rt_x = Matrix([[ 1,              0,        0, 0],
  		[0,        cos(q1), -sin(q1), 0],
  		[0,        sin(q1),  cos(q1), 0],
  		[0, 0, 0, 1]])

Rt_y = Matrix([[ cos(q2),        0,  sin(q2), 0],
              [       0,        1,        0, 0],
              [-sin(q2),        0,  cos(q2), 0],
              [0,0,0,1]])

Rt_z = Matrix([[ cos(q3), -sin(q3),        0, 0],
              [ sin(q3),  cos(q3),        0, 0],
              [ 0,              0,        1, 0],
              [0, 0, 0, 1]])

#DH Rotation Matrices
T0_1 = Matrix([[cos(th1), -sin(th1), 0, a0],
          [sin(th1)*cos(al0), cos(th1)*cos(al0), -sin(al0), -sin(al0)*d1],
          [sin(th1)*sin(al0), cos(th1)*sin(al0), cos(al0), cos(al0)*d1],
          [0,0,0,1]])

T1_2 = Matrix([[cos(th2), -sin(th2), 0, a1],
          [sin(th2)*cos(al1), cos(th2)*cos(al1), -sin(al1), -sin(al1)*d2],
          [sin(th2)*sin(al1), cos(th2)*sin(al1), cos(al1), cos(al1)*d2],
          [0,0,0,1]])

T2_3 = Matrix([[cos(th3), -sin(th3), 0, a2],
          [sin(th3)*cos(al2), cos(th3)*cos(al2), -sin(al2), -sin(al2)*d3],
          [sin(th3)*sin(al2), cos(th3)*sin(al2), cos(al2), cos(al2)*d3],
          [0,0,0,1]])

T3_4 = Matrix([[cos(th4), -sin(th4), 0, a3],
          [sin(th4)*cos(al3), cos(th4)*cos(al3), -sin(al3), -sin(al3)*d4],
          [sin(th4)*sin(al3), cos(th4)*sin(al3), cos(al3), cos(al3)*d4],
          [0,0,0,1]])

T4_5 = Matrix([[cos(th5), -sin(th5), 0, a4],
          [sin(th5)*cos(al4), cos(th5)*cos(al4), -sin(al4), -sin(al4)*d5],
          [sin(th5)*sin(al4), cos(th5)*sin(al4), cos(al4), cos(al4)*d5],
          [0,0,0,1]])

T5_6 = Matrix([[cos(th6), -sin(th6), 0, a5],
          [sin(th6)*cos(al5), cos(th6)*cos(al5), -sin(al5), -sin(al5)*d6],
          [sin(th6)*sin(al5), cos(th6)*sin(al5), cos(al5), cos(al5)*d6],
          [0,0,0,1]])

T6_EE = Matrix([[cos(th7), -sin(th7), 0, a6],
          [sin(th7)*cos(al6), cos(th7)*cos(al6), -sin(al6), -sin(al6)*d7],
          [sin(th7)*sin(al6), cos(th7)*sin(al6), cos(al6), cos(al6)*d7],
          [0,0,0,1]])

#Matrix Evaluations
Rzyx = R_z*R_y*R_x #Intrinsic RollPitchYaw
R_corr = R_z*R_y #DH-Rviz corrective rotation
R_corr = R_corr.subs(Corr_var)
Rt_corr = Rt_z*Rt_y #DH-Rviz corrective rotation-translation
Rt_corr = Rt_corr.subs(Corr_var)

#Joint value initialization
T0_1 = T0_1.subs(s)
T1_2 = T1_2.subs(s)
T2_3 = T2_3.subs(s)
T3_4 = T3_4.subs(s)
T4_5 = T4_5.subs(s)
T5_6 = T5_6.subs(s)
T6_EE = T6_EE.subs(s)
T0_3 = T0_1*T1_2*T2_3 #Transform evaluation from base to joint 3
T0_EE = T0_3*T3_4*T4_5*T5_6*T6_EE #Transform evaltuation from joint 3 to end effector

#Rotation evaluation from base to end effector
R0_EE = T0_EE[0:3,0:3]

#Rotation evaluation from base to joint 3
R0_3 = T0_3[0:3, 0:3]
#R0_3_Trans = Transpose(R0_3)
R0_3_Inv_normal = R0_3.inv()
#R0_3_Inv = R0_3.inv()
R0_3_Inv = R0_3.T
####

#Cosine Law
def cosine_law(a,b,c):
    theta = acos((b**2 + c**2 - a**2)/(2*b*c))
    return theta

#Atan with joint angle check
def atan2_limits(y, x, limits):
    
    angle = atan2(y,x)
    #print("Theta:", angle)
    #print("Max and min:", limits['max'], " ", limits['min'])
    if angle > limits['max'] or angle < limits['min']:
        angle = atan2(-y, -x)
        #print("Other Atan2")
    return angle
#Return hypotenuse
def hypot(x,y):
    return sqrt(x**2 + y**2)
    
def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    #####################
    ##### Setup
    ####################
    
    
   
    
    #print("Normal R0_3 Inverse:", R0_3_Inv_normal)
    #print("LU R0_3 Inverse", R0_3_Inv)
    
    #Euler from Quaternion
    #yaw, pitch, roll = tf.transformations.euler_from_quaternion([req.poses[x].orientation.x, req.poses[x].orientation.y, req.poses[x].orientation.z, req.poses[x].orientation.w], 'rzyx')
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
    r_rpy = {q1:roll, q2: pitch, q3:yaw}
   
    #End Effector Position Vector
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z
    ee_xyz = Matrix([[px],[py],[pz]])
    
    Ryrp = Rzyx.subs(r_rpy)
    #Translation vector from wrist center to end effector
    d7_solved = d7.subs(s) #0.303
    T_wc_ee = Matrix([[d7_solved],[0],[0]])
    
    #Solve for wrist center
    W_sol = ee_xyz - Ryrp*T_wc_ee
    wrist = {'x': W_sol[0,0], 'y': W_sol[1,0], 'z': W_sol[2,0]}
    ########################################################################################
    ## 

    ## Insert IK code here!
    
    theta1 = atan2(wrist['y'], wrist['x'])
    #print('Theta1: ', theta1)
    #theta2 evaluations
    a1_solved = a1.subs(s)#0.35
    x2 = wrist['x'] - a1_solved*cos(theta1)
    y2 = wrist['y'] - a1_solved*sin(theta1)
    
    d1_solved = d1.subs(s)#0.75
    x2p = hypot(x2, y2)
    z2 = wrist['z'] - d1_solved
    
    a2_solved = a2.subs(s)
    a2_solved = float(a2_solved)
    d3_solved = d3.subs(s)
    d3_solved = float(d3_solved)
    a3_solved = a3.subs(s)
    a3_solved = float(a3_solved)
    d4_solved = d4.subs(s)
    d4_solved = float(d4_solved)


    a_l = a2_solved
    b_l = float(hypot(a3_solved, d4_solved))
    c_l = float(hypot(x2p, z2))
    alpha_l = cosine_law(a_l, b_l, c_l)
    beta_l = cosine_law(b_l, a_l, c_l)
    gamma_l = cosine_law(c_l, a_l, b_l)
    delta_l = atan2(z2, x2p)
    
    theta2 = pi/2 - beta_l - delta_l

    #theta3 evaluations

    z2_3 = float(cos(theta2)*a2_solved)
    z3 = float(d1_solved + z2_3)
    x_p2_3 = float(sin(theta2)*a2_solved)
    x3 = float(cos(theta1)*(a1_solved+x_p2_3))
    y3 = float(sin(theta1)*(a1_solved+x_p2_3))
    x2_3 = hypot(x3,y3)
    eta = atan2(abs(a3_solved),d4_solved)
   
    #x3_w = float(wrist['x'] - x3)
    #y3_w = float(wrist['y'] - y3)
    #z3_w = float(wrist['z'] - z3)
    
    theta3 = pi/2 - eta - gamma_l
    #print('Theta3: ', float(theta3))
    
    #theta1-3 symbols
    TH1_3 = {th1:float(theta1), th2:float(theta2), th3:float(theta3)}
    #print("TH1-3: ", TH1_3)
    #theta4-6 evaluations
    
    #Transform from base to Joint 3 evaluated:
    #T0_3_eval = T0_3.evalf(subs={th1:float(theta1), th2:float(theta2), th3:float(theta3)})
    #Rotation from joint 3 to joint 6 matrix:
    R0_3_Inv_Eval = R0_3_Inv.subs(TH1_3)
    R3_6 =  R0_3_Inv_Eval * Ryrp * R_corr
    #Evaluation of rotation matrix terms to solve for theta 4-6
    r11 = R3_6[0,0]
    #r11 = r11.evalf()
    r12 = R3_6[0,1]
    r13 = R3_6[0,2]
    r21 = R3_6[1,0]
    r22 = R3_6[1,1]
    r23 = R3_6[1,2]
    r31 = R3_6[2,0]
    r32 = R3_6[2,1]
    r33 = R3_6[2,2]
    
    #print(type(r11))
    #theta5
    theta5 = atan2_limits(sqrt(r13**2 + r33**2), r23, th5_lim)
    #print('Theta5: ', float(theta5))
    if sin(theta5) > 0.0:
        theta4 = atan2(r33,-r13)
        theta6 = atan2(-r22, r21)
    
    else: #sin(theta5) < 0
        theta4 = atan2(-r33, r13)
        theta6 = atan2(r22, -r21)


    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################
    th_sol = {th1: theta1.evalf(), th2: theta2.evalf(), th3: theta3.evalf(), th4:theta4.evalf(), th5: theta5.evalf(), th6:theta6.evalf()}
    #print("Final angles: ", th_sol)
    #End Effector position
    #print("Test case EE: ", test_case[0][0][0], " ", test_case[0][0][1], " ", test_case[0][0][2])
    #Wrist Position
    T0_WC = T0_3*T3_4*T4_5
    T0_WC_eval = T0_WC.subs(th_sol)*origin
    FF_EE = T0_WC.subs(th_sol)*T5_6.subs(th_sol)*T6_EE.subs(th_sol)*origin
    #print("Calculated end effector: ", FF_EE)
    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [T0_WC_eval[0],T0_WC_eval[1],T0_WC_eval[2]] # <--- Load your calculated WC values in this array
    your_ee = [FF_EE[0],FF_EE[1],FF_EE[2]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1
    test_code(test_cases[test_case_number])
    test_case_number = 2
    test_code(test_cases[test_case_number])
    test_case_number = 3
    test_code(test_cases[test_case_number])

