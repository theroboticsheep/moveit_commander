#!/usr/bin/env python

import sys, rospy, copy
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose

GRIPPER_OPEN = [0,0]
GRIPPER_CLOSED = [0.017,0.017]
GRIPPER_NEUTRAL = [0.009,0.009]
GRIPPER_GRASP = [0.012,0.012]

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('up1_cartesian')
    
    robot = RobotCommander()
    robot.right_arm.allow_replanning(True)
    robot.right_arm.allow_looking(True)
    robot.right_arm.set_goal_tolerance(0.05)
    robot.right_arm.set_planning_time(60)
    rospy.sleep(1)
    robot.right_arm.set_named_target('right_arm_rest')
    robot.right_arm.go()
    robot.right_gripper.set_joint_value_target(GRIPPER_CLOSED)
    robot.right_gripper.go()
    rospy.sleep(5)

    waypoints = []

    # start with the current pose
    waypoints.append(robot.right_arm.get_current_pose().pose)
    
    wpose = Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    
    wpose.position.z += 0.02
    waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.z -= 0.02
    waypoints.append(copy.deepcopy(wpose))
    
    #We want the cartesian path to be interpolated at a resolution of 1 cm 
    #which is why we will specify 0.01 as the eef_step in cartesian translation. 
    
    print "Planning cartesian path"
    (cartesian_plan, fraction) = robot.right_arm.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.005,       # eef_step
                                 0.01,        # jump_threshold
                                 True)        # allow collisions

    print "Path followed: " + str(fraction)
    print "Trajectory:"
    print cartesian_plan
    print "Executing cartesian path"
    robot.right_arm.execute(cartesian_plan)
    rospy.sleep(5)

    print "Cartesian path complete"
    robot.right_arm.set_named_target('right_arm_rest')
    robot.right_arm.go()

    roscpp_shutdown()
