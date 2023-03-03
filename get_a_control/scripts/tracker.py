#!/usr/bin/python3
import rclpy
from rclpy.node import Node 
from get_a_interfaces.srv import EnableTracker
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml
import numpy as np
import matplotlib.pyplot as plt

class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')
        self.timer_period = 0.1 #second
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        self.enable = False
        self.srv = self.create_service(EnableTracker,'enableTracker',self.enable_callback) #create service server set_join
        self.publisher = self.create_publisher(Float64MultiArray,'/velocity_controllers/commands',10)
        self.a = 0
        self.to = self.get_time()
        self.n = []
        self.pi_x = 0
        self.pi_y = 0
        self.pi_z = 0

        # self.subscribe_1 = self.create_subscription(Float64,'/velocity_controllers/commands',self.vel_sub_callback,10)
        self.cmd_vel = 0
        
        # subscribe /joint_states from kinematics_server
        self.subscribe_2 = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,10)
        self.sub_joint_states = [0.0,0.0,0.0]
        
        # subscribe /joint_states (ref position & ref velocity)
        self.subscribe_3 = self.create_subscription(JointState,'/joint_trajectory_point',self.joint_traject_callback,10)
        self.ref_position = [0.0,0.0,0.0]
        self.ref_velocity = [0.0,0.0,0.0]
 
    def enable_callback (self,request:EnableTracker.Request,response:EnableTracker.Response):
        self.enable = request.enable
        return response
    
    def joint_state_callback(self,msg:JointState):
        qr = msg.position
        qr_dot = msg.velocity

    def joint_traject_callback(self,msg:JointState):
        self.ref_position = msg.position              # q ref
        self.ref_velocity = msg.velocity              # q dot ref

    # def vel_sub_callback(self,msg:Float64):
    #     self.cmd_vel = msg.data

    def get_time(self):
        time = self.get_clock().now().to_msg()
        return time.sec + time.nanosec*(10**-9)

    def timer_callback(self):

        Amp = 0.1  # 180 degree
        f = 0.25
        time = self.get_time()-self.to
            
        qr = Amp*np.sin(2*np.pi*f*time)
        qr_dot = 2*np.pi*f*Amp*np.cos(2*np.pi*f*time)

        # move 
        with open('/home/onnalin/ROS2_Directory/Homework_ws/get_a_controller/get_a_control/config/tracker_config.yaml') as fh:
            read_data = yaml.load(fh, Loader=yaml.FullLoader)
        
        self.pi_x = qr_dot + read_data['config node']['Kp']* (qr - self.sub_joint_states[0]) + read_data['config node']['Ki'] * 0
        self.pi_y = qr_dot + read_data['config node']['Kp']* (qr - self.sub_joint_states[1]) + read_data['config node']['Ki'] * 0
        self.pi_z = qr_dot + read_data['config node']['Kp']* (qr - self.sub_joint_states[2]) + read_data['config node']['Ki'] * 0
        #
        
        msg = Float64MultiArray()

        if self.enable == True:
            self.get_logger().info(f'{qr - self.sub_joint_states[1]}')
            msg.data = [self.pi_x,self.pi_y,self.pi_z]
            self.publisher.publish(msg)
            self.get_logger().info('sent')
            self.a = 1
        if self.enable == False:
            if self.a == 1:
                msg.data = [0.0,0.0,0.0]
                self.publisher.publish(msg)
                self.get_logger().info('stop')
                self.a = 0

def main(args=None):
    rclpy.init(args=args)
    tracker = Tracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()



