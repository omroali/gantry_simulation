#!/usr/bin/env python3

import rospy
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
)
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class SimpleWaypointVisualizer:
    def __init__(self):
        rospy.init_node("simple_waypoint_visualizer")

        yaml_file = rospy.get_param("~waypoint_file")
        with open(yaml_file, "r") as f:
            self.waypoints = yaml.safe_load(f)

        self.marker_pub = rospy.Publisher(
            "waypoint_markers", MarkerArray, queue_size=10, latch=True
        )

        self.publish_markers()
        rospy.Timer(rospy.Duration(2.0), lambda e: self.publish_markers())

    def publish_markers(self):
        marker_array = MarkerArray()

        for idx, wp_data in enumerate(self.waypoints):
            node = wp_data["node"]
            pos = node["pose"]["position"]

            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "waypoints"
            sphere.id = idx * 3
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pos["x"]
            sphere.pose.position.y = pos["y"]
            sphere.pose.position.z = pos.get("z", 0.0)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.4
            sphere.scale.y = 0.4
            sphere.scale.z = 0.4
            sphere.color.a = 0.9
            sphere.color.r = 0.2
            sphere.color.g = 0.8
            sphere.color.b = 0.2
            marker_array.markers.append(sphere)

            text = Marker()
            text.header = sphere.header
            text.ns = "labels"
            text.id = idx * 3 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pos["x"]
            text.pose.position.y = pos["y"]
            text.pose.position.z = pos.get("z", 0.0) + 0.6
            text.text = node["name"]
            text.scale.z = 0.25
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            marker_array.markers.append(text)

            verts = node.get("verts", [])
            if verts:
                zone = Marker()
                zone.header = sphere.header
                zone.ns = "zones"
                zone.id = idx * 3 + 2
                zone.type = Marker.LINE_STRIP
                zone.action = Marker.ADD
                zone.pose.position.x = pos["x"]
                zone.pose.position.y = pos["y"]
                zone.pose.position.z = pos.get("z", 0.0) + 0.01
                zone.pose.orientation.w = 1.0
                zone.scale.x = 0.05
                zone.color.a = 0.5
                zone.color.r = 0.2
                zone.color.g = 0.8
                zone.color.b = 0.2

                for vert in verts:
                    p = Point()
                    p.x = vert["x"]
                    p.y = vert["y"]
                    p.z = 0.0
                    zone.points.append(p)

                if verts:
                    p = Point()
                    p.x = verts[0]["x"]
                    p.y = verts[0]["y"]
                    p.z = 0.0
                    zone.points.append(p)

                marker_array.markers.append(zone)

        self.marker_pub.publish(marker_array)


if __name__ == "__main__":
    try:
        visualizer = SimpleWaypointVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
