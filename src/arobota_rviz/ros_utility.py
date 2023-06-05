#!/usr/bin/env python
# -*- coding: utf-8 -*-
import warnings

warnings.filterwarnings("ignore")  ##ignore the warning due to the python2

import numpy as np
import rospy
from geometry_msgs.msg import Twist, TwistStamped, Pose, PoseStamped
import tf


def velocity_saturator(vel6, linear_max=3, angular_max=3):
    linear_norm = np.linalg.norm(vel6[0:3], ord=2)
    angular_norm = np.linalg.norm(vel6[3:6], ord=2)
    if linear_norm > linear_max:
        vel6[0:3] = vel6[0:3] * (linear_max / linear_norm)
    if angular_norm > angular_max:
        vel6[3:6] = vel6[3:6] * (angular_max / angular_norm)
    return vel6


def twist_saturator(twist, linear_max=3, angular_max=3):
    return vec6_to_twist(
        velocity_saturator(twist_to_vec6(twist), linear_max, angular_max)
    )


def vec6_to_twiststamped(vec6, time):
    msg = TwistStamped()
    msg.header.stamp = time
    msg.twist.linear.x = vec6[0]
    msg.twist.linear.y = vec6[1]
    msg.twist.linear.z = vec6[2]
    msg.twist.angular.x = vec6[3]
    msg.twist.angular.y = vec6[4]
    msg.twist.angular.z = vec6[5]
    return msg


def vec6_to_twist(vec6):
    msg = Twist()
    msg.linear.x = vec6[0]
    msg.linear.y = vec6[1]
    msg.linear.z = vec6[2]
    msg.angular.x = vec6[3]
    msg.angular.y = vec6[4]
    msg.angular.z = vec6[5]
    return msg


def twist_to_vec6(twist):
    vec6 = np.zeros((6, 1))
    vec6[0] = twist.linear.x
    vec6[1] = twist.linear.y
    vec6[2] = twist.linear.z
    vec6[3] = twist.angular.x
    vec6[4] = twist.angular.y
    vec6[5] = twist.angular.z
    return vec6

def R_to_quaternion(R):
    R_mat = np.eye(4)
    R_mat[0:3, 0:3] = R
    orientation = tf.transformations.quaternion_from_matrix(R_mat)
    ### 正規化しないと小数点の誤差でエラーが出る
    orientation = orientation / np.linalg.norm(orientation)
    return orientation

def g_to_pose(g_mat):
    pose_msg = Pose()
    pos_vec = g_mat[0:3, 3:4].T.tolist()[0]
    
    orientation = R_to_quaternion(g_mat[:3, :3])
    pose_msg.position.x = pos_vec[0]
    pose_msg.position.y = pos_vec[1]
    pose_msg.position.z = pos_vec[2]
    pose_msg.orientation.x = orientation[0]
    pose_msg.orientation.y = orientation[1]
    pose_msg.orientation.z = orientation[2]
    pose_msg.orientation.w = orientation[3]
    return pose_msg



def g_to_posestamped(g_mat):
    posestamped_msg = PoseStamped()
    posestamped_msg.pose = g_to_pose(g_mat)
    posestamped_msg.header.stamp = rospy.Time.now()
    return posestamped_msg


def pose_to_g(pose_msg):
    pos = pose_msg.position
    ori = pose_msg.orientation
    vec3 = np.c_[np.array([pos.x, pos.y, pos.z])]
    R44 = tf.transformations.quaternion_matrix([ori.x, ori.y, ori.z, ori.w])
    R44[0:3, 3:4] = vec3
    return R44


def tf_to_gmat(tf_listner, parent, child):
    try:
        (trans, rot) = tf_listner.lookupTransform(child, parent, rospy.Time())
        gmat = np.eye(4)
        gmat = tf.transformations.quaternion_matrix(rot)
        gmat[0:3, 3:4] = np.c_[np.array(trans)]
        return gmat
    except (
        tf.LookupException,
        tf.ConnectivityException,
        tf.ExtrapolationException,
    ) as e:
        rospy.logwarn(e)
        return None

def Rt_to_g(self, xyz, R):
    """並進と回転行列から同次変換行列の生成

    Args:
        xyz (ndarray): 並進
        R (ndarray): 回転行列

    Returns:
        ndarray: 同次変換行列
    """    
    g = np.eye(4)
    xyz_vec = xyz.reshape(3,1)
    g[:3, :3] = R
    g[3, :3] = xyz_vec
    return g

