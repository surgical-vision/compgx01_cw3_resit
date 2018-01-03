#!/usr/bin/env python

import rospy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, AllowedCollisionMatrix, CollisionObject


class PlanningSceneMonitor(object):
    def __init__(self):
        self.collision_object_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=5,
                                                          latch=True)
        self.object_names = rospy.get_param('object_list', ['RedCylinder', 'BlueCuboid_link_0', 'GreenCube'])

    def run(self):
        while not rospy.is_shutdown():

            acm = response.scene.allowed_collision_matrix
            for object in self.object_names:
                if not object in acm.default_entry_names:
                    # add object to allowed collision matrix
                    acm.default_entry_names.append(object)
                    acm.default_entry_values.append(True)

            planning_scene_diff = PlanningScene(
                is_diff=True,
                allowed_collision_matrix=acm
            )

            self.pubPlanningScene.publish(planning_scene_diff)
            rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node('planning_scene_monitor')
    monitor = PlanningSceneMonitor()
    monitor.run()
