#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    robot = RobotCommander()
    rospy.sleep(1)
    
    print "Right Arm Current Pose"
    print robot.right_arm.get_current_pose()
    print "Left Arm Current Pose"
    print robot.left_arm.get_current_pose()
    
    print "Right Gripper Current Joint Values"
    print robot.right_gripper.get_current_joint_values()
    
    print "Right Gripper Current Variable Values"
    print robot.right_gripper.get_joints()

    rospy.spin()
    roscpp_shutdown()
