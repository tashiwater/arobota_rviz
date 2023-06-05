#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

# import dynamic_reconfigure.client
from arobota_rviz import se3_operations
from arobota_rviz import ros_utility


class RigidBodyMotion(object):
    def __init__(self):
        clock = rospy.get_param("/rigid_body_simulation_clock")
        input_topic = rospy.get_param("~input_topic", default="cmd_input")
        output_topic = rospy.get_param("~output_topic", default="posestamped")

        self.g_dot = self.g_dot_old = np.zeros((4, 4), dtype=np.float64)
        self.dt = 1.0 / clock
        self.g = se3_operations.point_rpy_to_gmatrix(
            rospy.get_param("initial_pose/x", 0),
            rospy.get_param("initial_pose/y", 0),
            rospy.get_param("initial_pose/z", 0),
            rospy.get_param("initial_pose/R_deg", 0) / 180.0 * 3.14159265358979,
            rospy.get_param("initial_pose/P_deg", 0) / 180.0 * 3.14159265358979,
            rospy.get_param("initial_pose/Y_deg", 0) / 180.0 * 3.14159265358979,
            axes="rxyz",
        )

        self.pose_pub = rospy.Publisher(output_topic, PoseStamped, queue_size=1)
        rospy.Subscriber(input_topic, Twist, self.twist_callback)
        rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

    def timer_callback(self, event):
        self.move()
        pub_msg = ros_utility.g_to_posestamped(self.g)
        self.pose_pub.publish(pub_msg)

    def twist_callback(self, msg_data):
        twist_vec = np.c_[
            np.array(
                [
                    msg_data.linear.x,
                    msg_data.linear.y,
                    msg_data.linear.z,
                    msg_data.angular.x,
                    msg_data.angular.y,
                    msg_data.angular.z,
                ]
            )
        ]
        Vb = se3_operations.vec6_to_mat44(twist_vec)
        self.g_dot = self.g.dot(Vb)

    def move(self):
        dt = self.dt
        ##### tagged integrator(台形積分)
        g = self.g + (self.g_dot + self.g_dot_old) * dt / 2.0
        self.g = se3_operations.g_reprojection(g.astype(np.float64))
        self.g_dot_old = self.g_dot


if __name__ == "__main__":
    rospy.init_node("rigid_body_motion", anonymous=True)
    rbm = RigidBodyMotion()
    rospy.spin()
