#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    rospy.sleep(1)

    grasps = []
    grasp = Grasp() 
    grasp.id = "right_grasp" 
    grasp.grasp_pose.header.frame_id = "base_link" 
    grasp.grasp_pose.header.stamp = rospy.Time.now() 
    grasp.grasp_pose.pose.position.x = 0.127233849387
    grasp.grasp_pose.pose.position.y = -0.288138890182
    grasp.grasp_pose.pose.position.z = 0.127521900721
    grasp.grasp_pose.pose.orientation.w = -0.0267003256568
    grasp.grasp_pose.pose.orientation.x = -0.0163927540977
    grasp.grasp_pose.pose.orientation.y = -0.681901741856
    grasp.grasp_pose.pose.orientation.z = 0.730772457525
    grasps.append(grasp)

    # clean the scene
    scene.remove_world_object("part")
    
    part_x = 0.01
    part_y = 0.01
    part_z = 0.1
    
    part_x_offset = 0.1
    part_y_offset = 0.02
    part_z_offset = 0.06
    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.orientation.w = 1.0
    p.pose.position.x = grasp.grasp_pose.pose.position.x + part_x + part_x_offset
    p.pose.position.y = grasp.grasp_pose.pose.position.y + part_y_offset
    p.pose.position.z = part_z/2 + part_z_offset
    scene.add_box("part", p, (part_x, part_y, part_z))

    rospy.sleep(1)

    # pick an object
    robot.right_arm.allow_replanning(True)
    robot.right_arm.allow_looking(True)
    robot.right_arm.set_goal_tolerance(0.05)
    robot.right_arm.set_planning_time(60)
    ret_val = -1
    tries = 0
    while ret_val == -1:
        ret_val = robot.right_arm.pick("part", grasps)
        tries = tries + 1
        print "Attempt: " + str(tries) + ", Response: " + str(ret_val)

    rospy.spin()
    roscpp_shutdown()
