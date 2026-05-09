#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class ChassisMarkerNode(Node):
    def __init__(self):
        super().__init__("wheeltec_chassis_marker_node")
        self.pub = self.create_publisher(MarkerArray, "/wheeltec_chassis/markers", 1)
        self.timer = self.create_timer(0.1, self.publish_markers)
        self.get_logger().info("Publishing WHEELTEC chassis markers on /wheeltec_chassis/markers")

    def box_marker(self, marker_id, name, xyz, scale, rgba):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "wheeltec_chassis"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        marker.text = name
        return marker

    def publish_markers(self):
        msg = MarkerArray()
        msg.markers.append(self.box_marker(
            0, "body", (0.0, 0.0, 0.12), (0.42, 0.30, 0.18), (0.1, 0.4, 0.9, 1.0)
        ))
        msg.markers.append(self.box_marker(
            1, "left_track", (0.0, 0.19, 0.055), (0.46, 0.07, 0.11), (0.02, 0.02, 0.02, 1.0)
        ))
        msg.markers.append(self.box_marker(
            2, "right_track", (0.0, -0.19, 0.055), (0.46, 0.07, 0.11), (0.02, 0.02, 0.02, 1.0)
        ))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
