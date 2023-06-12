#!/usr/bin/env python2

# Imports
from niryo_robot_python_ros_wrapper import *
import rospy
import time

import geometry_msgs.msg

from niryo_pick_and_place.srv import PoseEstimationService, PoseEstimationServiceResponse

# Initializing ROS node
rospy.init_node('PickAndPlaceNode')

can_positions = [
    [-0.015, -0.34, 0.2], #Red
    [0.065, -0.2, 0.2],  #Blue
    [-0.08, -0.2, 0.2] #Green
]

robot_default_pose = [0,0,0,0,0,0] #NEED TO DEFINE

def main():
    global can_positions

    rospy.wait_for_service('pose_estimation_srv')
    pe_service = rospy.ServiceProxy('pose_estimation_srv', PoseEstimationService) 
    # Connecting to the ROS Wrapper & calibrating if needed
    niryo_robot = NiryoRosWrapper()
    print("Wrapper")
    niryo_robot.calibrate_auto()
    print("Calibrate")
    while True:

        try:
   
            resp = pe_service()
            cube_pos = resp.estimated_position
            label = resp.estimated_class
        except rospy.ServiceException as e:
            print("Pose estimation service call failed: %s"%e)
            raise

        if label == 0:
            print("No piece found. Exiting.")
            time.sleep(1)
            continue

        #print("CUBE POS: ", cube_pos)

        
        # Updating tool
        # niryo_robot.update_tool()
        #print("Update")
        # Opening Gripper/Pushing Air
        niryo_robot.release_with_tool()
        print("Opening")
        # Going to pick pose
        try:
            niryo_robot.move_pose(cube_pos.x, cube_pos.y, cube_pos.z+0.05, 0.0, 1.57, 0)
            niryo_robot.move_pose(cube_pos.x, cube_pos.y, cube_pos.z, 0.0, 1.57, 0)
            # Picking
            niryo_robot.grasp_with_tool()
            print("Picking")
            #Go up again
            niryo_robot.move_pose(cube_pos.x, cube_pos.y, cube_pos.z+0.13, 0.0, 1.57, 0)

            # Moving to place pose
            niryo_robot.move_pose(can_positions[label-1][0],can_positions[label-1][1],can_positions[label-1][2], 0.0, 1.57, 0)
            print("Move")
            # Placing !
            niryo_robot.release_with_tool()
            print("Place")
            #Go default position
            # niryo_robot.move_pose(*)
        except:
            print("Cube not reachable.")
            

        time.sleep(1)
    

if __name__ == '__main__':
    main()
   
