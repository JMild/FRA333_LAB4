#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

class Scheduler(Node):
    def __init__(self):
        super().__init__('Scheduler')
        # pub scheduler to Tracker
        self.publisher = self.create_publisher(JointTrajectory,'/scheduler',10)
        timer_period = 1 #second
        self.timer = self.create_timer(timer_period,self.timer_callback)

        # with open('/home/onnalin/ROS2_Directory/Homework_ws/get_a_controller/get_a_control/config/list_viapoint.yaml') as fh:
        #     read_data = yaml.load(fh, Loader=yaml.FullLoader)
        #     read_data['config node']['position']

        # self.subscribe = self.create_subscription(JointTrajectory,'/sub_scheduler',self.sub_callback,10)

    def timer_callback(self):
        P_i = [0.0,0.0,0.0]     # start_positions 
        P_f = [0.5,0.5,0.5]    # goal_positions

        tra_msg = JointTrajectory()
        tra_msg.joint_names = ['joint_1','joint_2','joint_3']
        ## creating a point
        point1 = JointTrajectoryPoint()
        point1.positions = P_i
        point1.time_from_start = Duration(sec=1)
        ## creating a point
        point2 = JointTrajectoryPoint()
        point2.positions = P_f
        point2.time_from_start = Duration(sec=8)

        ## adding newly created point into trajectory message
        tra_msg.points.append(point1)
        tra_msg.points.append(point2)
        self.publisher.publish(tra_msg)

def main(args=None):
    rclpy.init(args=args)
    scheduler = Scheduler()
    rclpy.spin(scheduler)
    scheduler.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()



# class FSM:
#     def __init__(self):
#         self.state = "OFF"

#     def trun_on(self):
#         if self.state == "OFF":
#             self.state = "ON"
#             print('Truning on')
#         else:
#             print("Already on")
    
#     def trun_off(self):
#         if self.state == "ON":
#             self.state = "OFF"
#             print('Truning off')
#         else:
#             print("Already off")
    