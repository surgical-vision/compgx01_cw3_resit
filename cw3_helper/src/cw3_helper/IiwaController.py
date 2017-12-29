#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from math import pi
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cw3_helper.IiwaState import IiwaState


class IiwaController(object):
    def __init__(self):
        # ROS Params for possible variables
        param_joint_topic = rospy.get_param('joint_topic', '/iiwa/joint_states')
        param_trajectory_controller = rospy.get_param('trajectory_topic',
                                                     '/iiwa/EffortJointInterface_trajectory_controller')

        # Setup the subscribers for the joint states
        self.subscriber_joint_state = rospy.Subscriber(param_joint_topic, JointState, self.joint_state_callback,
                                                        queue_size=5)
        # TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # TF2 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Trajectory Publisher
        self.traj_publisher = rospy.Publisher(param_trajectory_controller + '/command', JointTrajectory,
                                         queue_size=3)
        self.joint_names = rospy.get_param(param_trajectory_controller + '/joints')

        self.current_state = IiwaState()

    def joint_state_callback(self, msg):
        """
        Joint state callback function given to the joint state subscriber, copies the current value to the iiwa state class.
        :param msg: Joint State Message
        :type msg: JointState
        :return:
        """
        self.current_state.set_position(msg.position)
        self.current_state.set_velocity(msg.velocity)
        self.current_state.set_torque(msg.effort)

    def get_joint_position(self):
        return self.current_state.get_position()

    def get_velocity(self):
        return self.current_state.get_velocity()

    def get_torque(self):
        return self.current_state.get_torque()

    def get_pose(self):
        raise NotImplementedError()

    def get_jacobian(self, joint):
        raise NotImplementedError()

    def inverse_kinematics(self, desired_pose, current_joint):
        raise NotImplementedError()

    def publish_joint_trajectory(self, joint_trajectory, time_from_start=537230041):
        # Most simplistic implementation possible
        # Takes a single array of joint values for each joint

        # Recommend reimplementing this
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()

        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_trajectory
        point.time_from_start.nsecs = time_from_start

        msg.points.append(point)

        self.traj_publisher.publish(msg)
