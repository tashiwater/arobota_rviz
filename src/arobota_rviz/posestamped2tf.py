#!/usr/bin/env python
# -*- coding: utf-8 -*-
import warnings

warnings.filterwarnings("ignore")  ##ignore the warning due to the python2

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped



class PoseStamped2Tf:
    def __init__(self):
        self.WORLD_TF = rospy.get_param("~world", default="world")
        input_topic = rospy.get_param("~posestamped")
        self.AGENT_TF = rospy.get_param("~tfname")
        self._br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(input_topic, PoseStamped, self.callback)

    def callback(self, msg):
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = msg.header.stamp
        tf_stamped.header.frame_id = self.WORLD_TF
        tf_stamped.child_frame_id = self.AGENT_TF
        tf_stamped.transform.translation.x = msg.pose.position.x
        tf_stamped.transform.translation.y = msg.pose.position.y
        tf_stamped.transform.translation.z = msg.pose.position.z
        tf_stamped.transform.rotation.x = msg.pose.orientation.x
        tf_stamped.transform.rotation.y = msg.pose.orientation.y
        tf_stamped.transform.rotation.z = msg.pose.orientation.z
        tf_stamped.transform.rotation.w = msg.pose.orientation.w
        self._br.sendTransform(tf_stamped)


if __name__ == "__main__":
    rospy.init_node("posestamped2tf", anonymous=True)
    node = PoseStamped2Tf()
    rospy.spin()
