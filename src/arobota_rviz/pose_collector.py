#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseArray


class Collector:
    def __init__(self, agentName):
        self.ready = False

        topicName = rospy.get_param("~posestampedTopic", default="posestamped")
        preTopicName = rospy.get_param("~preTopicName", "/")
        subTopic = preTopicName + agentName + "/" + topicName
        rospy.loginfo("topicName:" + subTopic)
        # subscriber for each agent's region
        rospy.Subscriber(subTopic, PoseStamped, self.poseStampedCallback, queue_size=1)
        # initialze with zeros
        self.pose = Pose()

    def poseStampedCallback(self, msg_data):
        if self.ready == False:
            self.ready = True
        self.pose = msg_data.pose

    def getPose(self):
        return self.pose

    def getReady(self):
        return self.ready


class poseCollector:
    def __init__(self):
        # Number of Agents
        output_topic = rospy.get_param("~output_topic", default="/allPose")
        self._agentNum = rospy.get_param("/agentNum")
        self._clock = rospy.get_param("pose_collector/clock")

        self.Collectors = []
        # create [Agent's number] subscriber
        for agentID in range(self._agentNum):
            agentName = "agent10" + str(agentID + 1)
            collector = Collector(agentName)
            self.Collectors.append(collector)

        self.pub_allPose = rospy.Publisher(output_topic, PoseArray, queue_size=1)
        # node freq
        self.rate = rospy.Rate(self._clock)

    def spin(self):

        while not rospy.is_shutdown():
            poselist = []
            ready = True
            for agentID in range(self._agentNum):
                pose = self.Collectors[agentID].getPose()
                poselist.append(pose)
                ready = ready * self.Collectors[agentID].getReady()

            if ready:
                self.pub_allPose.publish(PoseArray(poses=poselist))

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("poseCollector", anonymous=True)
    posecollector = poseCollector()
    posecollector.spin()
