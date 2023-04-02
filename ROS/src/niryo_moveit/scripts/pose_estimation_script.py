#!/usr/bin/env python3

from niryo_moveit.setup_and_run_model import *

import rospy
import io
import os
import math

from niryo_moveit.srv import PoseEstimationService, PoseEstimationServiceResponse
from PIL import Image, ImageOps
from geometry_msgs.msg import Point, Quaternion, Pose
from scipy.spatial.transform import Rotation as R

NODE_NAME = "PoseEstimationNode"
PACKAGE_LOCATION = os.path.dirname(os.path.realpath(__file__))[:-(len("/scripts"))] # remove "/scripts"
MODEL_PATH = PACKAGE_LOCATION + "/models/UR3_single_cube_model.tar"

count = 0


def _save_image(req):
    """  convert raw image data to a png and save it
    Args:
        req (PoseEstimationService msg): service request that contains the image data
     Returns:
        image_path (str): location of saved png file
    """
    global count
    count += 1

    image_height = req.image.width
    image_width = req.image.height
    #image = Image.frombytes('RGBA', (image_width,image_height), req.image.data)
    image = Image.open(io.BytesIO(req.image.data))
    #image = ImageOps.flip(image)
    image_name = "Input" + str(count) + ".png"
    if not os.path.exists(PACKAGE_LOCATION + "/images/"):
        os.makedirs(PACKAGE_LOCATION + "/images/")
    image_path = PACKAGE_LOCATION + "/images/" + image_name
    image.save(image_path)
    return image_path


def _run_model(image_path):
    """ run the model and return the estimated position/quaternion
    Args:
        image_path (str): location of saved png file
     Returns:
        position (float[]): object estimated x,y,z
        quaternion (float[]): object estimated w,x,y,z
    """
    output = run_model_main(image_path, MODEL_PATH)
    positionV = output[0].flatten()
    quaternionV = output[1].flatten()
    scaleYV = output[2].flatten()
    return positionV, quaternionV, scaleYV


def _format_response(est_position, est_rotation, est_scaleY):
    """ format the computed estimated position/rotation as a service response
       Args:
           est_position (float[]): object estimated x,y,z
           est_rotation (float[]): object estimated Euler angles
        Returns:
           response (PoseEstimationServiceResponse): service response object
       """

    position1 = Point()
    position1.x = est_position[0]
    position1.y = est_position[1]
    position1.z = est_position[2]

    position2 = Point()
    position2.x = est_position[3]
    position2.y = est_position[4]
    position2.z = est_position[5]

    position3 = Point()
    position3.x = est_position[6]
    position3.y = est_position[7]
    position3.z = est_position[8]

    rotation1 = Quaternion()
    rotation1.x = est_rotation[0]
    rotation1.y = est_rotation[1]
    rotation1.z = est_rotation[2]
    rotation1.w = est_rotation[3]
    
    rotation2 = Quaternion()
    rotation2.x = est_rotation[4]
    rotation2.y = est_rotation[5]
    rotation2.z = est_rotation[6]
    rotation2.w = est_rotation[7]

    rotation3 = Quaternion()
    rotation3.x = est_rotation[8]
    rotation3.y = est_rotation[9]
    rotation3.z = est_rotation[10]
    rotation3.w = est_rotation[11]
    
    scaleY1 = est_scaleY[0]
    scaleY2 = est_scaleY[1]
    scaleY3 = est_scaleY[2]
    
    pose1 = Pose()
    pose1.position = position1
    pose1.orientation = rotation1
    
    pose2 = Pose()
    pose2.position = position2
    pose2.orientation = rotation2

    pose3 = Pose()
    pose3.position = position3
    pose3.orientation = rotation3

    response = PoseEstimationServiceResponse()
    response.estimated_pose1 = pose1
    response.estimated_pose2 = pose2
    response.estimated_pose3 = pose3
    response.estimated_scaleY1 = scaleY1
    response.estimated_scaleY2 = scaleY2
    response.estimated_scaleY3 = scaleY3
    return response


def pose_estimation_main(req):
    """  main callback for pose est. service.
    Args:
        req (PoseEstimationService msg): service request that contains the image data
     Returns:
       response (PoseEstimationServiceResponse): service response object
    """
    print("Started estimation pipeline")
    image_path = _save_image(req)
    print("Predicting from screenshot " + image_path)
    est_position, est_rotation, est_scaleY = _run_model(image_path)
    response = _format_response(est_position, est_rotation, est_scaleY)
    print("Finished estimation pipeline\n")
    return response


def main():
    """
     The function to run the pose estimation service
     """
    rospy.init_node(NODE_NAME)
    s = rospy.Service('pose_estimation_srv', PoseEstimationService, pose_estimation_main)
    print("Ready to estimate pose!")
    rospy.spin()


if __name__ == "__main__":
    main()
