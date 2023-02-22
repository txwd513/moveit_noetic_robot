#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将发布/person_info话题，自定义消息类型learning_topic::Person

import rospy
import roslib
import actionlib
import sensor_msgs.msg
import control_msgs.msg


class FollowJointTrajectoryActionServer:
    def __init__(self):
        self.FollowJointTrajectory  = control_msgs.msg.FollowJointTrajectoryAction()
        self.RecSucceeded = False
        self.server = actionlib.SimpleActionServer('/robot_control/follow_joint_trajectory',control_msgs.msg.FollowJointTrajectoryAction,self.execute,False)
        self.server.start()
        
        print("FollowJointTrajectoryActionServer start succeeded\n")

    def execute(self,Trajectory ):
        # print(goal.trajectory.points)
        self.FollowJointTrajectory = Trajectory 
        self.RecSucceeded = True
        self.server.set_succeeded()


if __name__ == '__main__':
   
    try:
        # ROS节点初始化
        rospy.init_node('joint_state_publisher', anonymous=True)
        # 初始化FollowJointTrajectoryActionServer
        server = FollowJointTrajectoryActionServer()
        # 初始化joint_states话题
        joint_states_pub = rospy.Publisher('move_group/ros_controller_joint_states', sensor_msgs.msg.JointState, queue_size=10)
        joint_states_msg=sensor_msgs.msg.JointState()
        ##joint_states_msg.header.frame_id="base_link"
        joint_states_msg.name.append('Joint1')
        joint_states_msg.name.append('Joint2')
        joint_states_msg.name.append('Joint3')
        joint_states_msg.name.append('Joint4')
        joint_states_msg.name.append('Joint5')


        while True:
            if server.RecSucceeded:
                server.RecSucceeded = False
                for goal_joint in server.FollowJointTrajectory.trajectory.points :
                    t = rospy.get_rostime()
                    joint_states_msg.header.stamp.secs = t.secs
                    joint_states_msg.header.stamp.nsecs = t.nsecs
                    joint_states_msg.position = goal_joint.positions
                    joint_states_msg.velocity = goal_joint.velocities
                    joint_states_msg.effort = goal_joint.effort
                    # 发布消息
                    joint_states_pub.publish(joint_states_msg)
                    print(goal_joint)
                    rospy.sleep(0.1)

        # joint_states_publisher()
    except rospy.ROSInterruptException:
        pass

