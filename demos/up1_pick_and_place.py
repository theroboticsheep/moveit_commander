#!/usr/bin/env python

import sys, rospy
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

GRIPPER_OPEN = [0,0]
GRIPPER_CLOSED = [0.017,0.017]
GRIPPER_NEUTRAL = [0.009,0.009]
GRIPPER_GRASP = [0.012,0.012]

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('up1_pick_and_place')
    
    robot = RobotCommander()
    robot.right_arm.allow_replanning(True)
    robot.right_arm.allow_looking(True)
    robot.right_arm.set_goal_tolerance(0.05)
    robot.right_arm.set_planning_time(60)
    rospy.sleep(1)
    robot.right_arm.set_named_target('right_arm_rest')
    robot.right_arm.go()
    robot.right_gripper.set_joint_value_target(GRIPPER_OPEN)
    robot.right_gripper.go()
    rospy.sleep(5)

    # pick pose
    p = PoseStamped()
    p.header.frame_id = "up1_footprint"
    p.pose.position.x = 0.12792118579
    p.pose.position.y = -0.285290879999
    p.pose.position.z = 0.120301181892
    roll = 0
    pitch = 0
    yaw = -1.57 #pi/2 radians
    
    q = quaternion_from_euler(roll, pitch, yaw)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]

    robot.right_arm.set_pose_target(p.pose)

    max_pick_attempts = 5
    success = 0
    attempt = 0
    while success != 1 and attempt < max_pick_attempts:
        p_plan = robot.right_arm.plan()
        attempt = attempt + 1
        print "Planning attempt: " + str(attempt)
        if p_plan.joint_trajectory.points != []:
            success = 1
    
    if success == 1:
        print "Executing pick"
        robot.right_arm.execute(p_plan)
        rospy.sleep(5)
        robot.right_gripper.set_joint_value_target(GRIPPER_GRASP)
        robot.right_gripper.go()
        rospy.sleep(5)
        print "Executing place"
        robot.right_arm.set_named_target('right_arm_up')
        robot.right_arm.go()
        rospy.sleep(5)
        robot.right_arm.set_named_target('right_arm_pick')
        robot.right_arm.go()
        robot.right_gripper.set_joint_value_target(GRIPPER_OPEN)
        robot.right_gripper.go()
        rospy.sleep(5)
        robot.right_arm.set_named_target('right_arm_up')
        robot.right_arm.go()
        rospy.sleep(5)
        robot.right_arm.set_named_target('right_arm_rest')
        robot.right_arm.go()
    else:
        print "Unable to plan pick"

    roscpp_shutdown()
