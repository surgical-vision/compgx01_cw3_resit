#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject, ObjectType
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import PyKDL
import tf2_kdl
from tf_conversions import posemath


class ObjectTracker(object):
    def __init__(self):
        self.object_names = rospy.get_param('object_list', ['duplo_2x2x1', 'duplo_2x4x1'])
        self.noise_params = rospy.get_param('sensor_noise', '')
        self.camera_link_name = rospy.get_param('camera_link_name', 'camera_link')
        self.gazebo_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.calback_gazebo_state,
                                                  queue_size=30)

        self.object_publisher = rospy.Publisher('/recognized_object_array', RecognizedObjectArray, queue_size=10)

        # Get camera link in reference to the world through tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        cam_trans_msg = self.tf_buffer.lookup_transform(self.camera_link_name, 'world', rospy.Time.now())
        self.cam_trans = tf2_kdl.transform_to_kdl(cam_trans_msg)

    def calback_gazebo_state(self, msg):
        # Find the indicies for the object list
        matched_object_list = []
        object_array = RecognizedObjectArray()
        object_array.header.stamp = rospy.Time.now()
        for object in self.object_names:
            try:
                index = msg.name.index(object)
                matched_object_list.append([msg.pose[index], msg.name[index]])
            except ValueError:
                continue

        for object in matched_object_list:
            object_msg = RecognizedObject()
            object_pose = PoseWithCovarianceStamped()
            object_pose.pose.pose = posemath.toMsg(self.cam_trans.Inverse() * posemath.fromMsg(object[0]))
            object_msg.pose = object_pose
            object_msg.type.key = object[1]
            object_array.objects.append(object_msg)

        self.object_publisher.publish(object_array)


if __name__ == "__main__":
    rospy.init_node('object_tracker')
    track = ObjectTracker()
    rospy.spin()