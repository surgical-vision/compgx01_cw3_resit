#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from cw3_helper.srv import ChangeCollisionObject
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import numpy as np
from sensor_msgs.msg import JointState


def cw3_example_script():
    """
    This script will go through the main aspects of moveit and the components you will need to complete the coursework.
    You can find more information on
    """
    # Initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cw3_example_script')

    # rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    # rospy.spin()

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Robot contains the entire state of the robot (iiwa and shadow hand)
    robot = moveit_commander.RobotCommander()

    # We can get a list of all the groups in the robot
    print('============ Robot Groups:')
    print('{}\n\n'.format(robot.get_current_state()))

    iiwa_group = moveit_commander.MoveGroupCommander('object_iiwa')
    link_0 = iiwa_group.get_current_pose('object_iiwa_link_0')
    link_1 = iiwa_group.get_current_pose('object_iiwa_link_1')
    link_2 = iiwa_group.get_current_pose('object_iiwa_link_2')
    link_3 = iiwa_group.get_current_pose('object_iiwa_link_3')
    link_4 = iiwa_group.get_current_pose('object_iiwa_link_4')
    link_5 = iiwa_group.get_current_pose('object_iiwa_link_5')
    link_6 = iiwa_group.get_current_pose('object_iiwa_link_6')
    link_7 = iiwa_group.get_current_pose('object_iiwa_link_7')
    link_ee = iiwa_group.get_current_pose('object_iiwa_link_ee')
    link_obj = iiwa_group.get_current_pose('object_link_0')

    link_mass = [5, 4, 4, 3, 2.7, 1.7, 1.8, 0.3]
    link_posiiton = [[-0.1, 0, 0.07], [0, 0, 0.1575], [0, 0, 0.2025], [0, 0.2045, 0], [0, 0, 0.2155], [0, 0.1845, 0],
                     [0, 0, 0.2155], [0, 0.081, 0], [0, 0, 0]]
    g = 9.81

    positions = [[]] * 9
    positions[0] = [link_0.pose.position.x, link_0.pose.position.y, link_0.pose.position.z]
    positions[1] = [link_1.pose.position.x, link_1.pose.position.y, link_1.pose.position.z]
    positions[2] = [link_2.pose.position.x, link_2.pose.position.y, link_2.pose.position.z]
    positions[3] = [link_3.pose.position.x, link_3.pose.position.y, link_3.pose.position.z]
    positions[4] = [link_4.pose.position.x, link_4.pose.position.y, link_4.pose.position.z]
    positions[5] = [link_5.pose.position.x, link_5.pose.position.y, link_5.pose.position.z]
    positions[6] = [link_6.pose.position.x, link_6.pose.position.y, link_6.pose.position.z]
    positions[7] = [link_7.pose.position.x, link_7.pose.position.y, link_7.pose.position.z]
    positions[8] = [link_ee.pose.position.x, link_ee.pose.position.y, link_ee.pose.position.z]
    # positions[9] = [link_obj.pose.position.x, link_obj.pose.position.y, link_obj.pose.position.z]

    orientations = [[]] * 9
    orientations[0] = [link_0.pose.orientation.x, link_0.pose.orientation.y, link_0.pose.orientation.z,
                       link_0.pose.orientation.w]
    orientations[1] = [link_1.pose.orientation.x, link_1.pose.orientation.y, link_1.pose.orientation.z,
                       link_1.pose.orientation.w]
    orientations[2] = [link_2.pose.orientation.x, link_2.pose.orientation.y, link_2.pose.orientation.z,
                       link_2.pose.orientation.w]
    orientations[3] = [link_3.pose.orientation.x, link_3.pose.orientation.y, link_3.pose.orientation.z,
                       link_3.pose.orientation.w]
    orientations[4] = [link_4.pose.orientation.x, link_4.pose.orientation.y, link_4.pose.orientation.z,
                       link_4.pose.orientation.w]
    orientations[5] = [link_5.pose.orientation.x, link_5.pose.orientation.y, link_5.pose.orientation.z,
                       link_5.pose.orientation.w]
    orientations[6] = [link_6.pose.orientation.x, link_6.pose.orientation.y, link_6.pose.orientation.z,
                       link_6.pose.orientation.w]
    orientations[7] = [link_7.pose.orientation.x, link_7.pose.orientation.y, link_7.pose.orientation.z,
                       link_7.pose.orientation.w]
    # orientations[8] = [link_ee.pose.orientation.x, link_ee.pose.orientation.y, link_ee.pose.orientation.z,
    #                    link_ee.pose.orientation.w]
    orientations[8] = [link_obj.pose.orientation.x, link_obj.pose.orientation.y, link_obj.pose.orientation.z,
                       link_obj.pose.orientation.w]

    t = [[]] * 9
    z = [[]] * 8
    o = [[]] * 9

    for i in range(0, 9):
        t[i] = quaternion_matrix(orientations[i])
        t[i][0][[3]] = positions[i][0]
        t[i][1][[3]] = positions[i][1]
        t[i][2][[3]] = positions[i][2]

    z[0] = [0, 0, 1]
    z[1] = [t[2][0][2], t[2][1][2], t[2][2][2]]
    z[2] = [t[3][0][2], t[3][1][2], t[3][2][2]]
    z[3] = [t[4][0][2], t[4][1][2], t[4][2][2]]
    z[4] = [t[5][0][2], t[5][1][2], t[5][2][2]]
    z[5] = [t[6][0][2], t[6][1][2], t[6][2][2]]
    z[6] = [t[7][0][2], t[7][1][2], t[7][2][2]]
    z[7] = [t[8][0][2], t[8][1][2], t[8][2][2]]
    # z[8] = [t[9][0][2], t[9][1][2], t[9][2][2]]

    jw = [[z[0][0], z[1][0], z[2][0], z[3][0], z[4][0], z[5][0], z[6][0], z[7][0]],
          [z[0][1], z[1][1], z[2][1], z[3][1], z[4][1], z[5][1], z[6][1], z[7][1]],
          [z[0][2], z[1][2], z[2][2], z[3][2], z[4][2], z[5][2], z[6][2], z[7][2]]
          ]

    o[0] = [0, 0, 0, 0]
    o[1] = [t[1][0][3], t[1][1][3], t[1][2][3]]
    o[2] = [t[2][0][3], t[2][1][3], t[2][2][3]]
    o[3] = [t[3][0][3], t[3][1][3], t[3][2][3]]
    o[4] = [t[4][0][3], t[4][1][3], t[4][2][3]]
    o[5] = [t[5][0][3], t[5][1][3], t[5][2][3]]
    o[6] = [t[6][0][3], t[6][1][3], t[6][2][3]]
    o[7] = [t[7][0][3], t[7][1][3], t[7][2][3]]
    o[8] = [t[8][0][3], t[8][1][3], t[8][2][3]]
    # o[9] = [t[9][0][3], t[9][1][3], t[9][2][3]]

    o8_0 = [o[8][0] - o[0][0], o[8][1] - o[0][1], o[8][2] - o[0][2]]
    o8_1 = [o[8][0] - o[1][0], o[8][1] - o[1][1], o[8][2] - o[1][2]]
    o8_2 = [o[8][0] - o[2][0], o[8][1] - o[2][1], o[8][2] - o[2][2]]
    o8_3 = [o[8][0] - o[3][0], o[8][1] - o[3][1], o[8][2] - o[3][2]]
    o8_4 = [o[8][0] - o[4][0], o[8][1] - o[4][1], o[8][2] - o[4][2]]
    o8_5 = [o[8][0] - o[5][0], o[8][1] - o[5][1], o[8][2] - o[5][2]]
    o8_6 = [o[8][0] - o[6][0], o[8][1] - o[6][1], o[8][2] - o[6][2]]
    o8_7 = [o[8][0] - o[7][0], o[8][1] - o[7][1], o[8][2] - o[7][2]]

    jv = [[z[0][1] * o8_0[2] - z[0][2] * o8_0[1], z[0][2] * o8_0[0] - z[0][0] * o8_0[2],
           z[0][0] * o8_0[1] - z[0][1] * o8_0[0]],
          [z[1][1] * o8_1[2] - z[1][2] * o8_1[1], z[1][2] * o8_1[0] - z[1][0] * o8_1[2],
           z[1][0] * o8_1[1] - z[1][1] * o8_1[0]],
          [z[2][1] * o8_2[2] - z[2][2] * o8_2[1], z[2][2] * o8_2[0] - z[2][0] * o8_2[2],
           z[2][0] * o8_2[1] - z[2][1] * o8_2[0]],
          [z[3][1] * o8_3[2] - z[3][2] * o8_3[1], z[3][2] * o8_3[0] - z[3][0] * o8_3[2],
           z[3][0] * o8_3[1] - z[3][1] * o8_3[0]],
          [z[4][1] * o8_4[2] - z[4][2] * o8_4[1], z[4][2] * o8_4[0] - z[4][0] * o8_4[2],
           z[4][0] * o8_4[1] - z[4][1] * o8_4[0]],
          [z[5][1] * o8_5[2] - z[5][2] * o8_5[1], z[5][2] * o8_5[0] - z[5][0] * o8_5[2],
           z[5][0] * o8_5[1] - z[5][1] * o8_5[0]],
          [z[6][1] * o8_6[2] - z[6][2] * o8_6[1], z[6][2] * o8_6[0] - z[6][0] * o8_6[2],
           z[6][0] * o8_6[1] - z[6][1] * o8_6[0]],
          [z[7][1] * o8_7[2] - z[7][2] * o8_7[1], z[7][2] * o8_7[0] - z[7][0] * o8_7[2],
           z[7][0] * o8_7[1] - z[7][1] * o8_7[0]],
          ]

    jac = [[jv[0][0], jv[1][0], jv[2][0], jv[3][0], jv[4][0], jv[5][0], jv[6][0], jv[7][0]],
           [jv[0][1], jv[1][1], jv[2][1], jv[3][1], jv[4][1], jv[5][1], jv[6][1], jv[7][1]],
           [jv[0][2], jv[1][2], jv[2][2], jv[3][2], jv[4][2], jv[5][2], jv[6][2], jv[7][2]],
           [jw[0][0], jw[0][1], jw[0][2], jw[0][3], jw[0][4], jw[0][5], jw[0][6], jw[0][7]],
           [jw[1][0], jw[1][1], jw[1][2], jw[1][3], jw[1][4], jw[1][5], jw[1][6], jw[1][7]],
           [jw[2][0], jw[2][1], jw[2][2], jw[2][3], jw[2][4], jw[2][5], jw[2][6], jw[2][7]]
           ]

    print(jac)
    h = []
    for i in range(0, 9):
        h.append(link_posiiton[i][2] + positions[i][2])

    # rospy.init_node('listener', anonymous=True)


def joint_states_callback(data):
    rospy.loginfo("I heard: ", data.position)


if __name__ == '__main__':
    try:
        cw3_example_script()
    except rospy.ROSInterruptException:
        pass
