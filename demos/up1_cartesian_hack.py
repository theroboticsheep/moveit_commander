#!/usr/bin/env python

import sys, rospy, copy
from math import floor
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose

GRIPPER_OPEN = [0,0]
GRIPPER_CLOSED = [0.017,0.017]
GRIPPER_NEUTRAL = [0.009,0.009]
GRIPPER_GRASP = [0.012,0.012]

RESOLUTION = 0.05

class Up1Cartesian():
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('up1_cartesian')
        
        left_arm = MoveGroupCommander('left_arm')
        right_arm = MoveGroupCommander('right_arm')
        left_gripper = MoveGroupCommander('left_gripper')
        right_gripper = MoveGroupCommander('right_gripper')
        self.initArms(left_arm, right_arm, left_gripper, right_gripper)
        
        print "Planning cartesian path for right arm"
        
        arm = right_arm
        gripper = right_gripper
        
        arm.set_named_target('right_arm_rest')
        arm.go()
        gripper.set_joint_value_target(GRIPPER_CLOSED)
        gripper.go()
        rospy.sleep(5)
    
        wpose = arm.get_current_pose().pose
        
        wpose.position.z += 0.2
        print "New pose target: z + 0.2"
        waypoints = self.createWaypoints(arm, wpose, RESOLUTION)
        for waypoint in waypoints:
            self.planPath(arm, waypoint)
        rospy.sleep(5)
        
        wpose.position.z -= 0.2
        print "New pose target: z - 0.2"
        waypoints = self.createWaypoints(arm, wpose, RESOLUTION)
        for waypoint in waypoints:
            self.planPath(arm, waypoint)
        rospy.sleep(5)
         
        wpose.position.z += 0.2
        print "New pose target: z + 0.2"
        waypoints = self.createWaypoints(arm, wpose, RESOLUTION)
        for waypoint in waypoints:
            self.planPath(arm, waypoint)
        rospy.sleep(5)
        
        wpose.position.z -= 0.2
        print "New pose target: z - 0.2"
        waypoints = self.createWaypoints(arm, wpose, RESOLUTION)
        for waypoint in waypoints:
            self.planPath(arm, waypoint)
        rospy.sleep(5)
        
        print "Cartesian demo complete"
        arm.set_named_target('right_arm_rest')
        arm.go()
        
        roscpp_shutdown()
    
    def createWaypoints(self, arm, wpose, resolution):
        current_pose = arm.get_current_pose()
        
        diff_x = wpose.position.x - current_pose.pose.position.x
        diff_y = wpose.position.y - current_pose.pose.position.y
        diff_z = wpose.position.z - current_pose.pose.position.z
        
        temp = abs(diff_x) if abs(diff_x) > abs(diff_y) else abs(diff_y)
        diff = temp if temp > abs(diff_z) else abs(diff_z)
        num_waypoints = int(floor(diff/resolution))
        
        step_x = diff_x/num_waypoints
        step_y = diff_y/num_waypoints
        step_z = diff_z/num_waypoints
        
        waypoints = []

        for idx in range(0, num_waypoints):
            current_pose.pose.position.x += step_x
            current_pose.pose.position.y += step_y
            current_pose.pose.position.z += step_z
            
            waypoints.append(copy.deepcopy(current_pose))
        
        return waypoints
    
    def planPath(self, arm, wpose):
        arm.set_pose_target(wpose)
        max_pick_attempts = 5
        success = False
        attempt = 0
        while success != True and attempt < max_pick_attempts:
            p_plan = arm.plan()
            attempt = attempt + 1
            print "Planning attempt: " + str(attempt)
            if p_plan.joint_trajectory.points != []:
                success = True
                print "Executing cartesian waypoint"
                arm.execute(p_plan)
        
        if success == False:
            print "Failed to plan waypoint"
    
    def initArms(self, left_arm, right_arm, left_gripper, right_gripper):
        right_arm.allow_replanning(True)
        right_arm.allow_looking(True)
        right_arm.set_goal_tolerance(0.05)
        right_arm.set_planning_time(60)
        left_arm.allow_replanning(True)
        left_arm.allow_looking(True)
        left_arm.set_goal_tolerance(0.05)
        left_arm.set_planning_time(60)
        rospy.sleep(1)
        right_arm.set_named_target('right_arm_up')
        right_arm.go()
        right_gripper.set_joint_value_target(GRIPPER_CLOSED)
        right_gripper.go()
        left_arm.set_named_target('left_arm_up')
        left_arm.go()
        left_gripper.set_joint_value_target(GRIPPER_CLOSED)
        left_gripper.go()
        rospy.sleep(5)

if __name__=='__main__':
    up1Cartesian = Up1Cartesian()