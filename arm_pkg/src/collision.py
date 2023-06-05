#!/usr/bin/env python3
import rospy
from hsrb_interface import geometry
from moveit_commander import PlanningSceneInterface
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2


class CollisionUpdater:
    def __init__(self):
        self.planning_scene = PlanningSceneInterface()
        rospy.Subscriber('/occupied_cells_vis_array', PointCloud2, self.callback)

    def callback(self, data):
        points = pc2.read_points(data)
        collision_world = self.planning_scene.get_collision_world()
        for point in points:
            point = geometry.Vector3(point[0], point[1], point[2])
            collision_world.add_box(str(point), [0.02, 0.02, 0.02], point, wait=True)

if __name__ == '__main__':
    rospy.init_node('collision_updater')
    collision_updater = CollisionUpdater()
    rospy.spin()
