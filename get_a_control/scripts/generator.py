#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class Generator(Node):
    def __init__(self):
        super().__init__('generator')
        # pub end_effector_state to kinematics_server
        self.publisher = self.create_publisher(JointState,'/ref',10)
        self.rate = 5.0 #Hz
        timer_period = 1/self.rate #second
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.ref_position = [1.0,2.0,3.0]  # initial reference position
        self.ref_velocity = [6.0,5.0,4.0]  # initial velocity position

        # sub initial, final via point, duration from scheduler
        self.subscribe = self.create_subscription(JointTrajectory,'/scheduler',self.sub_scheduler_callback,10)

    def sub_scheduler_callback(self,msg:JointTrajectory):
        self.get_logger().info('Publishing: "%s"' % msg.points)

    def timer_callback(self):
        # publish the reference position and velocity in taskspace
        msg = JointState()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = self.ref_position 
        msg.velocity = self.ref_velocity
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    generator = Generator()
    rclpy.spin(generator)
    generator.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()