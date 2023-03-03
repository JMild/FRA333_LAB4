#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
from sensor_msgs.msg import JointState
import numpy as np
import math

class Kinematics_Server(Node):
    def __init__(self):
        super().__init__('kinematics_server')

        # self.publisher_ = self.create_publisher(JointState, 'topic', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.a = 0
        self.q_dot = [0.,0.,0.]
        self.endPos = [0.,0.,0.]
        self.q = [0.,0.,0.]    # initial ่joint angles
        self.p = [0.,0.,0.]
        self.n = []
        
        #FK
        if sys.argv[1] == 'FK':
            self.subscribe = self.create_subscription(JointState,'/joint_states',self.joint_states_callback,10)    # subscribes joint_state topic
            self.publisher = self.create_publisher(JointState,'/end_effector_state',10)

        # #IK 
        if sys.argv[1] == 'IK':
            self.publisher = self.create_publisher(JointState,'/joint_states',10)    # subscribes joint_state topic
            self.subscribe = self.create_subscription(JointState,'/end_effector_state',self.end_effector_state_callback,10)     # publishes end_effector_state topic
            
    def tx (self,t_x = float):  #function translation matrix about x axis
        return np.array([[1,0,0,t_x], 
                        [0,1,0,0], 
                        [0,0,1,0],
                        [0,0,0,1]])
    def tz (self,t_z = float):  #function translation matrix about z axis
        return np.array([[1,0,0,0], 
                        [0,1,0,0], 
                        [0,0,1,t_z],
                        [0,0,0,1]])
    def rx (self,r_x = float):  #function rotation matrix about x axis
        return np.array([[1,0,0,0], 
                        [0,np.cos(r_x),-np.sin(r_x),0], 
                        [0,np.sin(r_x),np.cos(r_x),0],
                        [0,0,0,1]])
    def rz (self,r_z = float):  #function rotation matrix about z axis
        return np.array([[np.cos(r_z),-np.sin(r_z),0,0], 
                        [np.sin(r_z),np.cos(r_z),0,0], 
                        [0,0,1,0],
                        [0,0,0,1]])

    def joint_states_callback(self,msg:JointState):  # Joint Angle >> End Effector Pose
        jointsAng = msg.position  # sub joints position 
        jointsVel = msg.velocity  # sub joints velocity 

        type_joint = ([1,1,1])    #0: prismatics joint | 1 :revolute joint
        n = 3   # DOF
        H = np.identity(4)          #indentity 4x4
        DH = np.array([[0,0,0.03,0], [0.025,np.pi/2,0,0],[0.07,0,0,0]]) #DH Table
        H3e = np.array([[0,1,0,0.04], [0,0,1,0],[1,0,0,0],[0,0,0,1]])
        for i in range (n):
            if type_joint[i] == 1:       # check type joint if it is revolute joint
                Hj = self.rz(jointsAng[i])      # let Hj is rotation matrix about z axis
            else:                        # check type joint if it is prismatics joint
                Hj = self.tz(jointsAng[i])      # let Hj is translation matrix about z axis
            H = H.dot(self.tx(DH[i][0])).dot(self.rx(DH[i][1])).dot(self.tz(DH[i][2])).dot(self.rz(DH[i][3])).dot(Hj) 
        H0e = H.dot(H3e)                 # pose of end-effector
        position_x = H0e[0][3]
        position_y = H0e[1][3]
        position_z = H0e[2][3]
        self.p = [position_x,position_y,position_z]   # sub End Effector Pose
    
    def end_effector_state_callback(self,msg:JointState):  # End Effector Pose >> Joint Angle
        # self.endPos = msg.position
        # endVel = msg.velocity

        # endeffector_position = msg.position
        p1 = [0,2]   # endeffector_position_x
        p2 = [0,2]   # endeffector_position_y
        p3 = [0,2]   # endeffector_position_z
        t = 1
        Joint_vel = []
        for i in range (len(p1)-1):
            v = (p1[i+1] - p1[i])/t
            Joint_vel.append(v)
            v = (p2[i+1] - p2[i])/t
            Joint_vel.append(v)
            v = (p3[i+1] - p3[i])/t
            Joint_vel.append(v)
            Joint_vel = np.array([[Joint_vel[0]],[Joint_vel[1]],[Joint_vel[2]]]) 

        l1 = 1      # lengh of link1
        l2 = 1      # lengh of link2
        l3 = 0.8    # lengh of link3

        a1 = -l2*np.sin(p1[1]+p2[1])-l1*np.sin(p1[1])-l3*np.sin(p1[1]+p2[1]+p3[1])
        a2 = -l2*np.sin(p1[1]+p2[1])-l3*np.sin(p1[1]+p2[1]+p3[1])
        a3 = -l3*np.sin(p1[1]+p2[1]+p3[1])
        a4 = l2*np.cos(p1[1]+p2[1])+l1*np.cos(p1[1])+l3*np.cos(p1[1]+p2[1]+p3[1])
        a5 = l2*np.cos(p1[1]+p2[1])+l3*np.cos(p1[1]+p2[1]+p3[1])
        a6 = l3*np.cos(p1[1]+p2[1]+p3[1])

        jacobian_matrix = np.array([[a1,a2,a3],[a4,a5,a6],[1,1,1]])   # size 6x6
        self.q_dot = jacobian_matrix.dot(Joint_vel)

        #################################

        end_effector_x = p1[0]
        end_effector_y = p2[0]
        end_effector_z = p3[0]

        gramma = [-1,1]  #arm config
        r = math.sqrt(end_effector_x**2 + end_effector_y**2)*gramma[0]     
        c2 = (r**2+(end_effector_z-l1**2)-l2**2-l3**2)/(2*l2*l3)
        s2 = math.sqrt(1-c2**2)*gramma[1]

        theta_1 = np.arctan2(end_effector_y/gramma[0],end_effector_x/gramma[0])
        theta_2 = np.arctan2(end_effector_z-l1,gramma[1]) - np.arctan2(l3*s2,l2+(l3*c2))
        theta_3 = np.arctan2(s2, c2)
        self.q = [theta_1, theta_2, theta_3]

    def timer_callback(self):

        msg = JointState() 
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']

        if sys.argv[1] == 'FK':
            msg.position = [0.0,0.0,0.0]      
            # msg.velocity = [1.,1.,1.]
        if sys.argv[1] == 'IK':
            # msg.position = [1.0,1.0,1.0] 
            msg.position = self.q    
            msg.velocity = self.q_dot
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    kinematics_server = Kinematics_Server()
    rclpy.spin(kinematics_server)
    kinematics_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()