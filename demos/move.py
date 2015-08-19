#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    robot = RobotCommander()
    rospy.sleep(1)
    
    p_offset_x = 0.01
    p_offset_y = 0.01
    p_offset_z = 0.01
    
    p = robot.right_arm.get_current_pose()
    p_target = [p.pose.position.x + p_offset_x, p.pose.position.y + p_offset_y, p.pose.position.z + p_offset_z]
    o_target = [p.pose.orientation.x + p_offset_x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
    
    p = PoseStamped()
    p.header.frame_id = "up1_footprint"
    p.pose.position.x = 0.12792118579
    p.pose.position.y = -0.285290879999
    p.pose.position.z = 0.120301181892
    
    p.pose.orientation.x = 0.0
    p.pose.orientation.y = 0.0
    p.pose.orientation.z = -0.706825181105
    p.pose.orientation.w = 0.707388269167

    robot.right_arm.set_pose_target(p.pose)

    # pick an object
    robot.right_arm.allow_replanning(True)
    robot.right_arm.allow_looking(True)
    robot.right_arm.set_goal_tolerance(0.05)
    robot.right_arm.set_planning_time(60)
    #robot.right_arm.set_position_target(p_target)
    #robot.right_arm.set_orientation_target(o_target)
    success = 0
    attempt = 0
    while not success:
        p_plan = robot.right_arm.plan()
        attempt = attempt + 1
        print "Planning attempt: " + str(attempt)
        if p_plan.joint_trajectory.points != []:
            success = 1
    robot.right_arm.execute(p_plan)

    rospy.spin()
    roscpp_shutdown()
