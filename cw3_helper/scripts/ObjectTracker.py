#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject, ObjectType
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
import PyKDL
import tf2_kdl
from tf_conversions import posemath
import numpy as np
import threading


class ObjectTracker(object):
    def __init__(self):
        self.object_names = rospy.get_param('object_list', ['RedCylinder', 'BlueCuboid', 'GreenCube'])
        self.trans_noise = rospy.get_param('trans_noise', 0.0)
        self.rotation_noise = rospy.get_param('rotation_noise', 0.0)
        self.camera_link_name = rospy.get_param('camera_link_name', 'camera_link')

        self.object_publisher = rospy.Publisher('/recognized_objects_array', RecognizedObjectArray, queue_size=10)

        # Get camera link in reference to the world through tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # tf publisher
        self.tf_pub = tf2_ros.TransformBroadcaster()

        self.matched_object_list = []
        self.matched_object_lock = threading.Lock()

        rospy.sleep(5)

        cam_trans_msg = None
        while cam_trans_msg is None:
            try:
                cam_trans_msg = self.tf_buffer.lookup_transform(self.camera_link_name, 'world', rospy.Time.now())
            finally:
                if cam_trans_msg is not None:
                    break

        self.cam_trans = tf2_kdl.transform_to_kdl(cam_trans_msg)
        self.gazebo_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.calback_gazebo_state,
                                                  queue_size=30)

    def generate_noise_trans(self):
        '''
        Generates a noisy transfrom to apply to the true transform of the model.
        :return: Noisy transform
        :rtype: PyKDL.Frame
        '''

        if self.trans_noise > 0.0:
            noise_t = np.random.normal(np.zeros(3), self.trans_noise * np.ones(3))
        else:
            noise_t = np.zeros(3)

        if self.rotation_noise > 0.0:
            noise_r = np.random.normal(np.zeros(3), self.rotation_noise * np.ones(3))
        else:
            noise_r = np.zeros(3)

        noise_vec = PyKDL.Vector(noise_t[0], noise_t[1], noise_t[2])
        noise_rot = PyKDL.Rotation().EulerZYX(noise_r[0], noise_r[1], noise_r[2])
        noise_trans = PyKDL.Frame(noise_rot, noise_vec)
        return noise_trans

    def calback_gazebo_state(self, msg):
        '''
        Callback function for getting the model states from gazebo. Function gets all the model states then matches the
        names to the object list.
        :param msg: gazebo model states
        :return:
        '''

        try:
            self.matched_object_lock.acquire()
            self.matched_object_list = []
            object_array = RecognizedObjectArray()
            object_array.header.stamp = rospy.Time.now()
            object_array.header.frame_id = self.camera_link_name
            for object_name in self.object_names:
                try:
                    index = msg.name.index(object_name)
                    self.matched_object_list.append([msg.pose[index], msg.name[index]])
                except ValueError:
                    continue

        finally:
            self.matched_object_lock.release()

    def run(self):
        '''
        Main run function that constructs the recognised object array message from the matched object list and noise
        params.
        :return:
        '''

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            try:
                self.matched_object_lock.acquire()

                object_array = RecognizedObjectArray()
                object_array.header.stamp = rospy.Time.now()
                object_array.header.frame_id = self.camera_link_name

                for object in self.matched_object_list:
                    object_msg = RecognizedObject()
                    object_pose = PoseWithCovarianceStamped()
                    object_pose.header.stamp = rospy.Time.now()
                    object_pose.header.frame_id = self.camera_link_name
                    object_pose.pose.pose = posemath.toMsg(self.cam_trans * posemath.fromMsg(object[0]) * \
                                            self.generate_noise_trans())
                    object_msg.pose = object_pose
                    object_msg.type.key = object[1]
                    object_array.objects.append(object_msg)

                    # publish tf
                    trans = TransformStamped()
                    trans.header.stamp = rospy.Time.now()
                    trans.header.frame_id = self.camera_link_name
                    trans.child_frame_id = object[1]
                    trans.transform.translation = object_pose.pose.pose.position
                    trans.transform.rotation = object_pose.pose.pose.orientation
                    self.tf_pub.sendTransform(trans)

                self.object_publisher.publish(object_array)

            except rospy.exceptions.ROSInterruptException:
                break

            finally:
                self.matched_object_lock.release()
                rate.sleep()


if __name__ == "__main__":
    rospy.init_node('object_tracker')
    try:
        track = ObjectTracker()
        track.run()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo('Object Tracker shutting down.')


