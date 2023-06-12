#!/usr/bin/env python2

import rospy
import io
import os
import math
import numpy
import cv2
import open3d
import pickle

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, Quaternion, Pose, PointStamped
from std_srvs.srv import Empty
import time
from niryo_pick_and_place.srv import PoseEstimationService, PoseEstimationServiceResponse
from niryo_pick_and_place.srv import BlobSegmentService, BlobSegmentServiceResponse

from scipy.spatial.transform import Rotation as R

import tf2_ros
import tf2_geometry_msgs 

from libpcd import *


#from niryo_robot_python_ros_wrapper import *

NODE_NAME = "PoseEstimationNode"

bridge = None
count = 0
use_sam = False

rgb_image = 0
depth_image = 0
point_clouds = []
queue_size = 3

label2color = {
    1: "Red",
    2: "Blue",
    3: "Yellow"
}

def callback_rgb(data):
    global rgb_image
    rgb_image = data
    

def callback_d(data):
    global depth_image
    depth_image = data

def callback_pc(data):
    global point_cloud_queue, queue_size
    point_clouds.append(data)
    if(len(point_clouds)>queue_size):
        point_clouds.pop(0)

def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def pose_estimation_main(req):
    """  main callback for pose est. service.
    Args:
        req (PoseEstimationService msg): service request that contains the image data
     Returns:
       response (PoseEstimationServiceResponse): service response object
    """
    global point_clouds, queue_size, label2color, use_sam

    print("Started estimation pipeline")

    if use_sam:
        cv2.imwrite("/home/bee/Desktop/sensor_image.jpg", bridge.imgmsg_to_cv2(rgb_image, "bgr8"))

        while not os.path.exists("/home/bee/Desktop/centroid.pkl"):
            time.sleep(1)
        
        

        with open("/home/bee/Desktop/centroid.pkl") as f:
            resx, resy, label = pickle.load(f)
            print("Centroid has been read")
        
        os.remove("/home/bee/Desktop/centroid.pkl")
    
    else:
        rospy.wait_for_service('segment_blob_srv')
        try:
            sb_service = rospy.ServiceProxy('segment_blob_srv', BlobSegmentService)
            resp = sb_service(rgb_image)
            resx = resp.resx
            resy = resp.resy
            label = resp.label 
        except rospy.ServiceException as e:
            print("Blob segment service call failed: %s"%e)
            raise

    if label==0:
        print("No piece found.")
        response = PoseEstimationServiceResponse()
        response.estimated_position = Point()
        response.estimated_class = label

        return response

    pcd = convertCloudFromRosToOpen3d(point_clouds[0])
        
    points = np.asarray(pcd.points)   
    numcols = point_clouds[0].width
    xyz_cam = points[numcols*resy+resx]

    for i in range(1,queue_size):
        pcd = convertCloudFromRosToOpen3d(point_clouds[i])
        
        points = np.asarray(pcd.points)
        #print(points[numcols*resy+resx])
        xyz_cam[2] = max(xyz_cam[2],points[numcols*resy+resx][2])
        #xyz_cam = xyz_cam + (points[numcols*resy+resx]-xyz_cam)/(i+1) #Incremental mean
        

    #print(xyz_cam)

    cam_pose = Pose()
    cam_pose.position.x = xyz_cam[0]
    cam_pose.position.y = xyz_cam[1]
    cam_pose.position.z = xyz_cam[2]
    cam_pose.orientation.x = 0
    cam_pose.orientation.y = 0.0
    cam_pose.orientation.z = 0.0
    cam_pose.orientation.w = 0.0



    world_pose = transform_pose(cam_pose, "camera_link", "world")
    center = rgb_image.width/2

    if resx<center-40:
        #print("a",float(center-resx)/20000)
        #world_pose.position.z = world_pose.position.z - float(center-resx)/20000
        world_pose.position.z = world_pose.position.z - float(center-150)/20000
    # elif resx>center+50:
    #     world_pose.position.z = world_pose.position.z - float(resx-(center+50))/20000
    #     print("b",float(resx-(center+50))/20000)
    # listener = 
    # world_pose = listener.transformPose(target_frame, pose_msg)
    response = PoseEstimationServiceResponse()
    response.estimated_position = world_pose.position
    response.estimated_class = label

    print("Cube World Pose:")
    print(response.estimated_position)

    print("Label: " + label2color[label])
    print("Finished estimation pipeline\n")
    return response


def main():
    """
     The function to run the pose estimation service
     """
    global bridge
    bridge = CvBridge()
    rospy.init_node(NODE_NAME)
    rospy.Subscriber("/camera/color/image_raw", Image , callback_rgb)
    #rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback_d)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback_pc)
    s = rospy.Service('pose_estimation_srv', PoseEstimationService, pose_estimation_main)

    if os.path.exists("/home/bee/Desktop/centroid.pkl"):
        os.remove("/home/bee/Desktop/centroid.pkl")
    
    print("Ready to estimate pose!")
    rospy.spin()


if __name__ == "__main__":
    main()