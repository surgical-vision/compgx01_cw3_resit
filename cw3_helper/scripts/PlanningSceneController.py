#!/usr/bin/env python

import rospy
from cw3_helper.srv import *
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, AllowedCollisionMatrix, AllowedCollisionEntry

class PlanningSceneController(object):
    def __init__(self):
        self.planning_scene_publisher = rospy.Publisher('planning_scene', PlanningScene, queue_size=5, latch=True)

        self.object_names = rospy.get_param('object_list', ['RedCylinder__link_0', 'BlueCuboid__link_0',
                                                            'GreenCube__link_0'])

        rospy.wait_for_service('/get_planning_scene', 10.0)
        self.get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)

        self.add_object_service = rospy.Service('/add_object_acm', ChangeCollisionObject, self.add_object_handler)
        self.remove_object_service = rospy.Service('/remove_object_acm', ChangeCollisionObject, self.remove_object_handler)

    def allow_all(self):

        request = PlanningSceneComponents(
                components=(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                            PlanningSceneComponents.SCENE_SETTINGS))
        response = self.get_planning_scene(request)
        acm = response.scene.allowed_collision_matrix
        planning_scene_diff = PlanningScene()
        planning_scene_diff.is_diff = True

        for object_name in self.object_names:
            rospy.logwarn('Adding object {} to allowed collisions...'.format(object_name))
            # acm = acm_allow_grasping(acm, object_name, allow=True)
            acm = add_to_acm(acm, object_name)

        planning_scene_diff.allowed_collision_matrix = acm
        self.planning_scene_publisher.publish(planning_scene_diff)

    def add_object_handler(self, req):
        request = PlanningSceneComponents(
            components=(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                        PlanningSceneComponents.SCENE_SETTINGS))
        response = self.get_planning_scene(request)

        acm = response.scene.allowed_collision_matrix

        planning_scene_diff = PlanningScene()
        planning_scene_diff.is_diff = True

        if not '__link_0' in req.object_name:
            req.object_name += '__link_0'

        acm = add_to_acm(acm, req.object_name)

        planning_scene_diff.allowed_collision_matrix = acm
        self.planning_scene_publisher.publish(planning_scene_diff)
        return ChangeCollisionObjectResponse(True)

    def remove_object_handler(self, req):
        request = PlanningSceneComponents(
            components=(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                        PlanningSceneComponents.SCENE_SETTINGS))
        response = self.get_planning_scene(request)

        acm = response.scene.allowed_collision_matrix

        planning_scene_diff = PlanningScene()
        planning_scene_diff.is_diff = True

        if not '__link_0' in req.object_name:
            req.object_name += '__link_0'

        acm = remove_from_acm(acm, req.object_name)

        planning_scene_diff.allowed_collision_matrix = acm
        self.planning_scene_publisher.publish(planning_scene_diff)
        return ChangeCollisionObjectResponse(True)


def add_to_acm(acm, object_name):
    acm.default_entry_names.append(object_name)
    acm.default_entry_values.append(True)
    return acm


def remove_from_acm(acm, object_name):
    try:
        index = acm.default_entry_names.index(object_name)
        acm.default_entry_values[index] = False
    except ValueError:
        rospy.logwarn('Object not in acm')
        acm.default_entry_names.append(object_name)
        acm.default_entry_values.append(False)
    finally:
        return acm


if __name__ == "__main__":
    rospy.init_node('planning_scene_controller')
    scene_controller = PlanningSceneController()
    rospy.spin()
