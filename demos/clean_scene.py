#!/usr/bin/env python

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor


GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper'

#GRIPPER_FRAME = 'right_finger1'
GRIPPER_FRAME = 'up1_footprint'


REFERENCE_FRAME = 'up1_footprint'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
                        
                
        # Give the scene a chance to catch up
        rospy.sleep(2)

        # Give each of the scene objects a unique name        
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'
        target_id = 'target'
        tool_id = 'tool'
                
        # Remove leftover objects from a previous run
        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        scene.remove_world_object(target_id)
        scene.remove_world_object(tool_id)
        
        # Remove any attached objects from a previous session
        scene.remove_attached_object(GRIPPER_FRAME, target_id)
        
        # Give the scene a chance to catch up    
        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()