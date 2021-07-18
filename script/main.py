#!/usr/bin/env python

from logging import shutdown
import rospy
from rospy.timer import sleep
from std_msgs.msg import String, Float64MultiArray, Float32
from math import *
import time
import numpy as np

import PyKDL as kdl
from mpc_controller import MPC_controller
from ur5_kinematic import *
import matplotlib.pyplot as plt
from data import*


def joint_2_np(array, len):
    res = np.array([])
    for  i in range(0, len):
        res = np.append(res, array[i])
    return res
def np_2_jointArr(array, len):
    res = kdl.JntArray(len)
    for i in range(0, len):
        res[i] = array[i]
    return res
def kdl_2_mat(data):
        mat =  np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i,j] = data[i,j]
        return mat

def hermite(p0, p1, m0, m1, t):
    P = (2*t**3-3*t**2+1)*p0+(t**3-2*t**2+t)*m0+(-2*t**3+3*t**2)*p1+(t**3-t**2)*m1
    return P   

FREQ = 100
dt = 0.01
n = 6
p = 5
m = 5


q = np.zeros(n)
q_prev = q
Integral = np.zeros(n*p)
dq = q

torques = kdl.JntArray(n) 
q_dest = kdl.JntArray(n)
J = kdl.Jacobian(n)      

def position_callback(msg):
    global q 
    #q = np.array(msg.data)
    
def torque_callback(msg):
    torques = msg.data
    #rospy.loginfo(msg.data)

if __name__ == '__main__':
    try:
        rospy.init_node("ur5_control")

        rate = rospy.Rate(FREQ)

        position_pub = rospy.Publisher('UR5/control/position', Float64MultiArray, queue_size=1, latch=True)
        #torque_pub = rospy.Publisher('UR5/control/torque', Float64MultiArray, queue_size=1, latch=True)
        position_sub = rospy.Subscriber('UR5/feedback/position', Float64MultiArray, position_callback)
        torque_sub = rospy.Subscriber('UR5/feedback/position', Float64MultiArray, torque_callback)
        #velocity_sub = rospy.Subscriber('UR5/feedback/velocity', Float64MultiArray, velocity_callback)



        mpc = MPC_controller(p, m, n, dt)

        #DH UR5 parametrs
        a2 = -0.425
        a3 = -0.39225

        alpha1 = pi / 2
        alpha4 = pi / 2
        alpha5 = -pi / 2

        d1 = 0.089159
        d4 = 0.10915
        d5 = 0.09465
        d6 = 0.0823


        a = [-0, 0.425, -0.39225, 0, 0, 0]
        d = [0.089159, 0, 0, 0.1333, 0.0997, 0.0996]
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
        th = [0,0,0,0,0,0]


        
        # rotation1 =  kdl.Rotation(1, 0, 0, 0, cos(alpha1), -sin(alpha1), 0, sin(alpha1), cos(alpha1))
        # rotation2 =  kdl.Rotation(1, 0, 0, 0, cos(alpha4), -sin(alpha4), 0, sin(alpha4), cos(alpha4))
        # rotation3 =  kdl.Rotation(1, 0, 0, 0, cos(alpha5), -sin(alpha5), 0, sin(alpha5), cos(alpha5))

        #kdl_frame = kdl.Frame()
        #create chain
        #chain = kdl.Chain()
        # chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.None), kdl.Frame(kdl.Vector(0.0, 0.0, d1))))
        # chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), kdl.Frame(rotation1)))
        # chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), kdl.Frame(kdl.Vector(a2, 0.0, 0.0))))
        # chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), kdl.Frame(kdl.Vector(a3, 0.0, d4))))
        # chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), kdl.Frame(rotation2)*kdl.Frame(kdl.Vector(0.0, 0.0, d5))))
        # chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), kdl.Frame(rotation3)*kdl.Frame(kdl.Vector(0.0, 0.0, d6))))
        # chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), kdl.Frame(kdl.Vector(0.0, 0.0, 0.0))))

        # frame = kdl.Frame()
        # for i in range(n):
        #     frame = kdl_frame.DH(a[i], alpha[i], d[i], th[i])
        #     chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), frame))
         
    
        # jac_solver = kdl.ChainJntToJacSolver(chain)
        # fk_solver_pos = kdl.ChainFkSolverPos_recursive(chain)
        # solver_vel = kdl.ChainIkSolverVel_pinv(chain)
        # ik_solver_pos = kdl.ChainIkSolverPos_LMA(chain)
        # #ik_solver_pos = kdl.ChainIkSolverPos_NR(chain, fk_solver_pos, solver_vel, 1000, 1e-4)
        # jac_solv = kdl.ChainJntToJacSolver(chain)

        t_len = 50
        path_len = 150
        counter = 0
        counter_2 = 0
        x_0 = 0.4
        x_1 = 0.7
        y_0 = 0
        y_1 = 0
        z_0 = 0.2
        z_1 = 0.4

        t = 0

        x_arr = np.zeros((1, int(path_len/2)))
        y_arr = np.zeros((1, int(path_len/2)))
        z_arr = np.zeros((1, int(path_len/2)))

        th = 2./path_len
        for i in range(int(path_len/2)):

            x_arr[0][i] = hermite(x_0, x_1, 0, 0, t)
            y_arr[0][i] = hermite(y_0, y_1, 0, 0, t)
            z_arr[0][i] = hermite(z_0, z_1, 0, 0, t)
            t += th

        trajectory_1 = np.concatenate(((x_arr), (y_arr), (z_arr)), axis = 0)
        trajectory_2 = np.concatenate((np.flip(x_arr), np.flip(y_arr), np.flip(z_arr)), axis = 0)
        trajectory = np.concatenate((trajectory_1, trajectory_2), axis = 1)
        rospy.loginfo(n)

        du = np.zeros(n)
        u = np.zeros(n)

        Q_d = np.array([])
        
        for i in range(0, path_len):
            T06 = mat([[0, 0, 1, trajectory[0][i]],[0, 1, 0, trajectory[1][i]],[-1, 0,  0, trajectory[2][i]],[0, 0, 0, 1]], copy=False)
            q_d_np = np.matrix(invKine(T06))  
            q_d = np.transpose(q_d_np).tolist()[1]
            Q_d = np.append(Q_d, q_d)
            # frame.p[0] = trajectory[0][i]
            # frame.p[1] = trajectory[1][i]
            # frame.p[2] = trajectory[2][i]  
            # frame.M = kdl.Rotation.RotY(pi/2)
            
            # ret = ik_solver_pos.CartToJnt(q, frame, q_dest)
            # for k in range(0, n):
            #     Q_d = np.append(Q_d, q_dest[k])
        # x_t = np.linspace(0, path_len*dt, num = path_len)
        # fig, axis = plt.subplots()
        # y_t = np.array([])
        # y_t1 = np.array([])
        # o = 5
        # for i in range(path_len):
        #     y_t = np.append(y_t, Q_d[i*n+o])
        #     y_t1 = np.append(y_t1, Q_d1[i*n+o])
        # axis.plot(x_t, y_t, label = "q6_CS")
        # axis.plot(x_t, y_t1, label = "q6_emul")
        # plt.legend()
        # plt.show()
        print(np.shape(Q_d))
        Q1 = np.array([])
        Q2 = np.array([])
        Q3 = np.array([])
        Q4 = np.array([])
        Q5 = np.array([])
        Q6 = np.array([])

        q = np.array([pi/2, pi/2, 0, pi/2, 0, 0])
        t = np.array([])
        T_1 = 0
        while not rospy.is_shutdown():

            global Integral, dq, q_prev

            pos = Float64MultiArray()

            # frame.p[0] = 0.6
            # frame.p[1] = 0.0
            # frame.p[2] = 0.2
            #frame.M = kdl.Rotation.RotY(0)
            
            #res = kdl.Frame()
            #ret = fk_solver_pos.JntToCart(q, res)
            # ret = ik_solver_pos.CartToJnt(q, frame, q_dest)
            # jac_solv.JntToJac(q, J)
            # J_np = kdl_2_mat(J)
            #tau = np.transpose(J_np).dot(joint_2_np(torques, n))
            # T06 = mat([[1, 0, 0, 0.6],[0, 1, 0, 0.2],[0, 0,  1,   0.5],[0, 0, 0, 1]], copy=False)
            # q_d = np.matrix(invKine(T06))   
            # q_dest = np.transpose(q_d[:, 0])
            
            #rospy.loginfo(np.shape(np.transpose(q_d).tolist()[0]))
            #rospy.loginfo(np.shape(Q_d))
            #pos.data.extend([q_d1[0], q_d1[0], q_d1[0], q_d1[0], q_d1[0], q_d1[0]])


            
            #rospy.loginfo(rospy.get_time())
           
            Q_k = np.array([])  
            for i in range(mpc.p):
                Q_k = np.append(Q_k, q)   
            
            Integral += (Qd_1[n*counter: n*counter+n*mpc.p] - Q_k)*dt
            
            mpc.Ek_p = Qd_1[n*counter: n*counter+n*mpc.p] - mpc.Ia.dot(q) - mpc.Ib.dot(dq) + Integral.dot(mpc.Ki)
            mpc.w =  mpc.Kmpc.dot(mpc.Ek_p)
            #rospy.loginfo(q)
            #err = Q_d1[counter*n:(counter+1)*n] - np.transpose(q)
            # print(np.shape(err))
            # Integral += ki*err*dt
            # val = kp*err + Integral 
            du += mpc.w*dt
            u += du*dt
            q = Q_d[n*counter: n*counter+mpc.p+1] + u
  

            pos.data.extend(q)
            position_pub.publish(pos)
         

            counter_2 += 1
            if (counter_2 == 15):
                t = np.append(t, T_1)
                # Q1 = np.append(Q1, Q_d1[n*counter] )
                # Q2 = np.append(Q2, Q_d1[n*counter+1])
                # Q3 = np.append(Q3, Q_d1[n*counter+2] )
                # Q4 = np.append(Q4, Q_d1[n*counter+3] )
                # Q5 = np.append(Q5, Q_d1[n*counter+4] )
                # Q6 = np.append(Q6, Q_d1[n*counter+5] )
                Q1 = np.append(Q1, q[0] )
                Q2 = np.append(Q2, q[1] )
                Q3 = np.append(Q3, q[2] )
                Q4 = np.append(Q4, q[3] )
                Q5 = np.append(Q5, q[4] )
                Q6 = np.append(Q6, q[5] )
                dq = (q - q_prev)*FREQ/p
                q_prev = q
                counter_2 = 0
                T_1 += dt
                #print(u)
                counter += 1
            if (counter == path_len - p - 1):
                counter = 0
            rate.sleep()

        fig, axis = plt.subplots()   
        axis.plot(t, Q1, label = "q1")
        axis.plot(t, Q2, label = "q2")
        axis.plot(t, Q3, label = "q3")
        axis.plot(t, Q4, label = "q4")
        axis.plot(t, Q5, label = "q5")
        axis.plot(t, Q6, label = "q6")

        plt.legend()
        plt.show()
    except rospy.ROSInterruptException as e:
        rospy.loginfo('Robot: ' + e)