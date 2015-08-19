#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 5 10:31:00 2013
@author: Sam Pfeiffer
"""
from moveit_commander import RobotCommander, PlanningSceneInterface
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult
import copy

def createGrasp(grasp_pose):
    grasp = Grasp()
    grasp.id = "grasp_test"
    grasp.grasp_pose = grasp_pose
    
    grasp.pre_grasp_posture.header.frame_id = "base_link"
    grasp.pre_grasp_posture.header.stamp = rospy.Time.now() 
    grasp.pre_grasp_posture.joint_names = ["right_gripper_joint"]
    pos = JointTrajectoryPoint() # pre-grasp with thumb down and fingers open
    pos.positions.append(0.0)
    grasp.pre_grasp_posture.points.append(pos)
    
    grasp.grasp_posture.header.frame_id = "base_link"
    grasp.grasp_posture.header.stamp = rospy.Time.now() 
    grasp.grasp_posture.joint_names = ["right_gripper_joint"]
    pos = JointTrajectoryPoint() # grasp with all closed
    pos.positions.append(.008)
    grasp.grasp_posture.points.append(pos)
    
    grasp.max_contact_force = 0
    
    return grasp

def createPickupGoal(group="right_arm", target="part", grasp_pose=PoseStamped()):
    pug = PickupGoal()
    pug.target_name = target
    pug.group_name = group
    # Create grasp to append
    grsp = createGrasp(grasp_pose)
    pug.allowed_planning_time = 5.0
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.allowed_touch_objects.append("all")
    pug.allowed_touch_objects.append("part")
    
    return pug


if __name__=='__main__':
    rospy.init_node("grasp_test_as")
    
    rospy.loginfo("Connecting to pickup AS")
    pickup_ac = actionlib.SimpleActionClient('/pickup', PickupAction)
    pickup_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    scene = PlanningSceneInterface()
    
    rospy.sleep(1)   
    
    rospy.loginfo("Cleaning world objects")
    # clean the scene
    scene.remove_world_object("part")
    
    # publish a demo scene
    part_x = 0.01
    part_y = 0.01
    part_z = 0.1
    
    part_x_offset = 0.22
    part_y_offset = -0.275
    part_z_offset = 0.035
    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = "base_link"
    p.pose.orientation.w = 1.0
    p.pose.position.x = part_x + part_x_offset
    p.pose.position.y = part_y_offset
    p.pose.position.z = part_z/2 + part_z_offset
    scene.add_box("part", p, (part_x, part_y, part_z))
    
    rospy.loginfo("Added object to world")
    
    rospy.sleep(1)
    
    p.pose.position.x = 0.127233849387
    p.pose.position.y = -0.288138890182
    p.pose.position.z = 0.127521900721
    p.pose.orientation.w = -0.0267003256568
    p.pose.orientation.x = -0.0163927540977
    p.pose.orientation.y = -0.681901741856
    p.pose.orientation.z = 0.730772457525
    
    pose_grasp = copy.deepcopy(p)
    goal = createPickupGoal("right_arm", "part", pose_grasp)
    rospy.loginfo("Sending goal")
    pickup_ac.send_goal(goal)
    rospy.loginfo("Waiting for result")
    pickup_ac.wait_for_result()
    result = pickup_ac.get_result()
    rospy.loginfo("Result is:")
    print result