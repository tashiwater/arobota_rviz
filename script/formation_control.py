#!/usr/bin/env python

from xbox_button import XBoxButton
import tf
import math
import numpy as np
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist

class AgentManagerExample:
    def __init__(self):
        rospy.init_node("robot1_formation_velocity")
        self.clock= 20
        self._main_start = False
        self.myid = int(rospy.get_param("agentID", default=-1))
        self.rate = rospy.Rate(10)
        self._uh = Twist()
        self.finalvelo= Twist()
        self._my_zeta = np.zeros((1,3))
        self._zetas = np.zeros((3,3))

        self._pub_zeta = rospy.Publisher("zeta", PoseStamped, queue_size=1)
        rospy.Subscriber("/allPose", PoseArray, self.poseArrayCallback, queue_size=1)
        self.pubvel = rospy.Publisher('cmd_input',Twist, queue_size=10)
        rospy.Subscriber('/joy',Joy,self.joy_callback) #joy

    def joy_callback(self,msg):
        if msg.buttons[XBoxButton.X]: # X button
            rospy.loginfo("main start")
            self._main_start = True

        joy_ux = msg.axes[XBoxButton.LX]
        joy_uy = msg.axes[XBoxButton.LY]
        joy_omega = msg.axes[XBoxButton.RX]
        self._uh.linear.x = joy_ux/2 #(-1,1)
        self._uh.linear.y = joy_uy/2 #(-1,1)
        self._uh.angular.z = joy_omega/2 #(-1,1)

    def poseArrayCallback(self, msg):
        arraynum = len(msg.poses)
        self.positions = np.zeros((3,3))
        self.zetas = np.zeros((3,1))

        for i in range(arraynum):
            pos = [
                msg.poses[i].position.x,
                msg.poses[i].position.y,
                msg.poses[i].position.z,
                ]
            quat = [
                msg.poses[i].orientation.x,
                msg.poses[i].orientation.y,
                msg.poses[i].orientation.z,
                msg.poses[i].orientation.w
                ]
            quat = np.array(quat)
            angle = tf.transformations.euler_from_quaternion(quat)
            angle = angle[2]  # angle about the z-axis
            self.positions[i]=pos
            self.zetas[i]= angle
        
            # print(self.positions)
            
    def main_control(self):
        ### you can use drone's position to design the input ###
        all_positions = self.positions

        #triangular    
        d1 = np.array([(-0.25)*math.cos(math.pi/6),(-0.25)*math.sin(math.pi/6),0])  
        d2 = np.array([0,0.25,0])
        d3 = np.array([0.25*math.cos(math.pi/6),-0.25*math.sin(math.pi/6),0])

        q1= all_positions[0] + d1
        q2= all_positions[1] + d2
        q3= all_positions[2] + d3
        # zeta1 = self.zetas[0] #0 degree
        # zeta2 = self.zetas[1] + ((2*math.pi)/3) #120 degree
        # zeta3 = self.zetas[2] + ((4*math.pi)        self.myid = 1/3) #360 degree
        self.velocity = 0
        if self.myid==1:
            self.velocity = 0.2*((q2-q1)+ (q3-q1)) 
        if self.myid==2:
            self.velocity = 0.2*((q1-q2) + (q3-q2))
        if self.myid==3:
            self.velocity = 0.2*((q2-q3)+ (q1-q3))

        zeta_dot = self.velocity
        dt = 1.0 / self.clock
        self._my_zeta += zeta_dot * dt
        self.publish_zeta(self._my_zeta)
        zeta1 = self._zetas[0]
        zeta2 = self._zetas[1]
        zeta3 = self._zetas[2]
        if self.myid==1:
            self.PIvelocity = 0.2* ((zeta1-zeta2)+(zeta1-zeta3))
        if self.myid==2:
            self.PIvelocity = 0.2 *((zeta2-zeta1)+(zeta2-zeta3))
        if self.myid==3:
            self.PIvelocity = 0.2 * ((zeta3-zeta1)+(zeta3-zeta2))
        self.velocity_x, self.velocity_y = float(self.velocity[0]+self.PIvelocity[0]+self._uh.linear.x),float(self.velocity[1]+self.PIvelocity[1]+self._uh.linear.y)
        self.finalvelo.linear.x = self.velocity_x
        self.finalvelo.linear.y = self.velocity_y
        self.finalvelo.angular.z = self._uh.angular.z
        self.pubvel.publish(self.finalvelo)

    def publish_zeta(self, zeta):
        msg = PoseStamped()
        msg.pose.position.x = zeta[0][0]
        msg.pose.position.y = zeta[0][1]
        msg.pose.position.z = zeta[0][2]
        self._pub_zeta.publish(msg)

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            if self._main_start:
                self.main_control()
                self.rate.sleep()

if __name__=='__main__':
    try:
        agent = AgentManagerExample()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
