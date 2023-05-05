#!/usr/bin/env python3

from niryo_moveit.setup_and_run_model import *
from niryo_moveit.setup_and_run_model_class import run_model_main_class

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
MODEL_PATH = PACKAGE_LOCATION + "/models/Niryo_1by1_model_translation_ep52.tar"
MODEL_CLASS_PATH = PACKAGE_LOCATION + "/models/Niryo_1by1_model_class_ep64.tar"

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
    scaleYV = output[1].flatten()
    output = run_model_main_class(image_path, MODEL_CLASS_PATH)
    label = output.flatten()

    return positionV, scaleYV, label


def _format_response(est_position, est_scaleY, est_class):
    """ format the computed estimated position/rotation as a service response
       Args:
           est_position (float[]): object estimated x,y,z
           est_rotation (float[]): object estimated Euler angles
        Returns:
           response (PoseEstimationServiceResponse): service response object
       """

    position = Point()
    position.x = est_position[0]/100
    position.y = est_position[1]/100
    position.z = est_position[2]/100
    
    scaleY = est_scaleY[0]/100

    label = est_class[0]
    
    response = PoseEstimationServiceResponse()
    response.estimated_position = position
    response.estimated_scaleY = scaleY
    response.estimated_class = label

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
    est_position, est_scaleY, est_class = _run_model(image_path)
    response = _format_response(est_position, est_scaleY, est_class)
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
