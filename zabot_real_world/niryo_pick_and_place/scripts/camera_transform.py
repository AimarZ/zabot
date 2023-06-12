#!/usr/bin/env python2

# Imports
import rospy

import tf
import tf2_ros
import geometry_msgs.msg

from tf.transformations import quaternion_from_euler


class CameraTransform:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "parent")
        self.child_frame_id = rospy.get_param("~child_frame_id", "child")

        self.trans_x = rospy.get_param("~trans_x", 0.0)
        self.trans_y = rospy.get_param("~trans_y", 0.0)
        self.trans_z = rospy.get_param("~trans_z", 0.0)
        self.rot_roll = rospy.get_param("~rot_roll", 0.0)
        self.rot_pitch = rospy.get_param("~rot_pitch", 0.0)
        self.rot_yaw = rospy.get_param("~rot_yaw", 0.0)
    
    def print_parameters(self):
        rospy.loginfo("frame_id: %s child_frame_id: %s", self.frame_id, self.child_frame_id)
        rospy.loginfo("Translation: %f %f %f", self.trans_x, self.trans_y, self.trans_z)
        rospy.loginfo("Rotation: %f %f %f", self.rot_roll, self.rot_pitch, self.rot_yaw)

    def run(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = self.frame_id
        static_transformStamped.child_frame_id = self.child_frame_id

        static_transformStamped.transform.translation.x = self.trans_x
        static_transformStamped.transform.translation.y = self.trans_y
        static_transformStamped.transform.translation.z = self.trans_z

        quat = tf.transformations.quaternion_from_euler(self.rot_roll, self.rot_pitch, self.rot_yaw)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("camera_transform")
    camera_transform = CameraTransform()
    camera_transform.print_parameters()
    camera_transform.run()