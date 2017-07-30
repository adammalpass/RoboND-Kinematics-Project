from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generate test cases can be added to the test_cases dictionary
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
    ########################################################################################
    ## Insert IK code here starting at: Define DH parameter symbols

    ## YOUR CODE HERE!

    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    # Define DH param symbols

    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angles
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link lengths
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offsets
    
    
    # Joint angle symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # 'theta' joint angles


    # Modified DH params
    #q2 = q2 - pi / 2.
    #q7 = 0

    
    # Define Modified DH Transformation matrix
    s = {alpha0: 0,     a0:   0,    d1: 0.75, 
         alpha1: -pi/2, a1: 0.35,   d2: 0,       q2: q2 - pi/2,  
         alpha2: 0,     a2: 1.25,   d3: 0,
         alpha3: -pi/2, a3: -0.054, d4: 1.5,
         alpha4: pi/2,  a4: 0,      d5: 0,
         alpha5: -pi/2, a5: 0,      d6: 0,
         alpha6: 0,     a6: 0,      d7: 0.303,   q7: 0}



    # Create individual transformation matrices
    T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
       [                   0,                   0,            0,               1]])
    
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
       [                   0,                   0,            0,               1]])
    
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
       [                   0,                   0,            0,               1]])
    
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
       [                   0,                   0,            0,               1]])
    
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
       [                   0,                   0,            0,               1]])
    
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
       [                   0,                   0,            0,               1]])
    
    T5_6 = T5_6.subs(s)

    T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
       [                   0,                   0,            0,               1]])
    
    R3_6 = simplify(T3_4*T4_5*T5_6)[:3,:3]
    print R3_6

    T6_7 = T6_7.subs(s)

    # Transform from base link to WC
    T0_5 = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5)
    # Transform from base link to end effector
    #T0_7 = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7)


    #Correction for orientation difference between defintion of gripper link URDF v DH
    
    #first rotate around z-axis by pi
    R_z = Matrix([[             cos(pi),            -sin(pi),            0,              0],
       [                        sin(pi),            cos(pi),             0,              0],
       [                        0,                  0,                   1,              0],
       [                        0,                  0,                   0,              1]])

    #then rotate around y-axis by -pi/2
    R_y = Matrix([[             cos(-pi/2),         0,                   sin(-pi/2),     0],
       [                        0,                  1,                   0,              0],
       [                        -sin(-pi/2),        0,                   cos(-pi/2),     0],
       [                        0,                  0,                   0,              1]])

    #calculate total correction factor
    R_corr = simplify(R_z * R_y)

    #calculate corrected transform from base to end effector
    #T_total = simplify(T0_7 * R_corr)

    T0_3 = simplify(T0_1 * T1_2 * T2_3)
    T0_7_corr = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7 * R_corr)

    # Extract rotational component of transform matrix
    R0_3 = T0_3[0:3, 0:3]


    r, p, ya = symbols('r p ya')

    R_roll = Matrix([[ 1,              0,        0],
                  [ 0,        cos(r), -sin(r)],
                  [ 0,        sin(r),  cos(r)]])

    R_pitch = Matrix([[ cos(p),        0,  sin(p)],
                  [       0,        1,        0],
                  [-sin(p),        0,  cos(p)]])

    R_yaw = Matrix([[ cos(ya), -sin(ya),        0],
                  [ sin(ya),  cos(ya),        0],
                  [ 0,              0,        1]])

    #R0_6 = simplify(R_roll * R_pitch * R_yaw)
    R0_6 = simplify(R_yaw * R_pitch * R_roll)
    R0_6 = simplify(R0_6 * R_corr[0:3, 0:3])

    # Calculate joint angles using Geometric IK method

    J5__0 = [1.85, 0, 1.946]

    ########################## 1) First calculate Wrist Centre (WC) #################################

    ############ 1.a) Construct R0_6 based on target roll, pitch and yaw angles of end-effector ######

    #print("R0_6")
    #print(R0_6.evalf(subs={roll:0, yaw:0, pitch:0}))
    R0_6_num = R0_6.evalf(subs={r:roll, ya:yaw, p:pitch})


    #############   1.b) Calculate wrist centre (WC) based on ###################################
    #############   translation along z-axis from EE location ####################################

    P_EE = Matrix([[px],[py],[pz]])
    #P_EE = Matrix([[2.153],[0],[1.946]])
    #P_EE = Matrix([[-0.18685],[2.1447],[1.9465]])

    P_WC = P_EE - 0.303 * R0_6_num * Matrix([[0],[0],[1]])
    #print("P_WC")
    #print(P_WC.evalf(subs={roll:0, yaw:0, pitch:0}))

    
    J5 = P_WC
    #J5 = [1.79505, 1.84825, 0.3094]   #q1 = 0.8, q2 = 1.1, q3 = -0.4

    ################################### 1.c) Calculate theta1  ######################################

    theta1 = atan2(J5[1], J5[0])
    #print("theta1",theta1)

    J2__0 = [0.35, 0, 0.75]
    J3__0 = [0.35, 0, 2]
    J5__0 = [1.85, 0, 1.946]

    J2 = [J2__0[0] * cos(theta1), J2__0[0] * sin(theta1), J2__0[2]]
    #print("J2", J2)

    L2_5_X = J5[0] - J2[0]
    L2_5_Y = J5[1] - J2[1]
    L2_5_Z = J5[2] - J2[2]
    L2_5 = sqrt(L2_5_X**2 + L2_5_Y**2 + L2_5_Z**2)

    L2_3__0 = 1.25

    L3_5_X__0 = J5__0[0] - J3__0[0]
    L3_5_Z__0 = J5__0[2] - J3__0[2]
    L3_5__0 = sqrt(L3_5_X__0**2 + L3_5_Z__0**2)

    #print("L2_5", L2_5)
    #print("L3_5", L3_5__0)


    ############################### 1.d) Calculate theta 3 ###################################

    #D = cos(theta)
    D = (L2_5**2 - L2_3__0**2 - L3_5__0**2) / -(2 * L2_3__0 * L3_5__0)
    #print("D", D)

    theta3_internal = atan2(sqrt(1-D**2), D)
    theta3 = pi / 2 - theta3_internal + atan2(L3_5_Z__0, L3_5_X__0)
    print("atan2", atan2(L3_5_Z__0, L3_5_X__0))
    #theta3_2 = pi / 2 - (atan2(-sqrt(1-D**2), D) - atan2(L3_5_Z__0, L3_5_X__0))
    #q3_1 = atan2(sqrt(1-D**2), D)
    #q3_2 = atan2(-sqrt(1-D**2), D) 
    #print("theta3", theta3.evalf())
    #print("q3_2", q3_2.evalf())

    ########################## 1.e) Calculate theta 2 #####################################

    #q2 = atan2(L2_5_Z, L2_5_X) - atan2(L3_5__0 * sin(pi - q3_internal), L2_3__0 + L3_5__0 * cos(pi - q3_internal))
    m1 = L3_5__0 * sin(theta3_internal)
    m2 = L2_3__0 - L3_5__0 * cos(theta3_internal)
    b2 = atan2(m1,m2)
    b1 = atan2(J5[2]-J2[2], sqrt((J5[0]-J2[0])**2 + (J5[1]-J2[1])**2))
    theta2 = pi / 2 - b2 - b1

    ######################  2) Calculate EE orientation ######################################

    ######################  2.a) Calculate R0_3  ############################################

    # Evaluate with calculated q1, q2 & q3
    R0_3_num = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
    #rospy.loginfo("R0_3_num", R0_3_num)

    #calculate inverse of R0_3
    #R0_3_num_inv = R0_3_num ** -1
    R0_3_num_inv = R0_3_num.inv("LU")

    R3_6 = R0_3_num_inv * R0_6_num

    #alpha, beta, gamma = tf.transformations.euler_from_matrix(R3_6, axes = 'ryzx')
    #theta4 = alpha
    #theta5 = beta - pi/2
    #theta6 = gamma - pi/2

    #theta4 = atan2(R3_6[1,0],R3_6[0,0]) # rotation about Z-axis
    #theta5 = atan2(-R3_6[2,0], sqrt(R3_6[0,0]*R3_6[0,0]+R3_6[1,0]*R3_6[1,0])) # rotation about Y-axis
    #theta6 = atan2(R3_6[2,1],R3_6[2,2]) # rotation about X-axis

    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    ## Ending at: Populate response for the IK request
    ########################################################################################
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    T0_5_num = T0_5.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})
    T0_7_corr_num = T0_7_corr.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})
    #print("Forward Kinematics T0_7_num")
    #print(T0_7_num)
    #print(T0_7_num.col(-1).row(0)[0])

    #print("Forward Kinematics T0_5_num")
    #print(T0_5_num)
    #print(T0_5_num.col(-1).row(0)[0])

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    #your_wc = [1,1,1] # <--- Load your calculated WC values in this array
    #your_wc = [T0_5_num.col(-1).row(0)[0], T0_5_num.col(-1).row(1)[0], T0_5_num.col(-1).row(2)[0]]
    your_wc = [P_WC[0], P_WC[1], P_WC[2]]
    #your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    your_ee = [T0_7_corr_num.col(-1).row(0)[0], T0_7_corr_num.col(-1).row(1)[0], T0_7_corr_num.col(-1).row(2)[0]]
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
           \nthat the arm can have muliple posisiotns. It is best to add your forward kinmeatics to \
           \nlook at the confirm wether your code is working or not**")
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
    test_case_number = 3

    test_code(test_cases[test_case_number])
