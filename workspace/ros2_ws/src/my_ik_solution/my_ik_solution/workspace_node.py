import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from my_ik_solution.fk_node import forward_kinematics


JOINT_LIMITS = [
    (-np.pi, np.pi),       # Joint 1
    (-np.pi/4, np.pi/4),   # Joint 2
    (-np.pi/4, np.pi/4),   # Joint 3
    (-np.pi/4, np.pi/4),   # Joint 4
]

SAMPLES_PER_JOINT = 12


class WorkspaceNode(Node):
    def __init__(self):
        super().__init__('workspace_node')

        self.marker_pub = self.create_publisher(
            MarkerArray, '/workspace_markers', 10)

        self.timer = self.create_timer(2.0, self.publish_workspace)
        self.markers = None
        self.get_logger().info('Workspace node started. Computing reachable workspace...')

    def publish_workspace(self):
        if self.markers is None:
            self.markers = self.compute_workspace()
            self.get_logger().info(
                f'Workspace computed: {len(self.markers.markers)} points')

        self.marker_pub.publish(self.markers)

    def compute_workspace(self):
        points = []
        max_reach = 0.0

        ranges = [
            np.linspace(lo, hi, SAMPLES_PER_JOINT)
            for lo, hi in JOINT_LIMITS
        ]

        for q1 in ranges[0]:
            for q2 in ranges[1]:
                for q3 in ranges[2]:
                    for q4 in ranges[3]:
                        T = forward_kinematics([q1, q2, q3, q4])
                        pos = T[:3, 3]
                        points.append(pos.copy())
                        dist = np.linalg.norm(pos)
                        if dist > max_reach:
                            max_reach = dist

        self.get_logger().info(f'Total positions sampled: {len(points)}')
        self.get_logger().info(f'Maximum reach: {max_reach:.4f} m')

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        self.get_logger().info(
            f'X range: [{min(xs):.4f}, {max(xs):.4f}]')
        self.get_logger().info(
            f'Y range: [{min(ys):.4f}, {max(ys):.4f}]')
        self.get_logger().info(
            f'Z range: [{min(zs):.4f}, {max(zs):.4f}]')

        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns = 'workspace'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.005
        marker.scale.y = 0.005
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3

        for pos in points:
            p = Point()
            p.x = float(pos[0])
            p.y = float(pos[1])
            p.z = float(pos[2])
            marker.points.append(p)

        marker_array.markers.append(marker)
        return marker_array


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
