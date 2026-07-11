import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np


# DH Parameters for the 4-DOF ASRS robot arm
# Each row: [a, alpha, d, theta_offset]
# theta_offset is added to the joint variable
DH_PARAMS = [
    [0.0,      np.pi/2,  0.14415, 0.0],  # Joint 1: base rotation
    [0.24381,  0.0,      0.0,     0.0],  # Joint 2: shoulder
    [0.12475,  0.0,      0.0,     0.0],  # Joint 3: elbow
    [0.08475, -np.pi/2,  0.0,     0.0],  # Joint 4: wrist
]

JOINT_NAMES = [
    'world_to_baselink',
    'base_to_shoulder',
    'shoulder_to_elbow',
    'elbow_to_wrist1',
]


def dh_matrix(a, alpha, d, theta):
    """Compute the 4x4 homogeneous transformation matrix from DH parameters."""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,       ca,      d],
        [0.0,    0.0,      0.0,    1.0],
    ])


def forward_kinematics(joint_angles):
    """Compute end-effector position from joint angles using DH convention."""
    T = np.eye(4)
    for i, (a, alpha, d, offset) in enumerate(DH_PARAMS):
        theta = joint_angles[i] + offset
        T = T @ dh_matrix(a, alpha, d, theta)
    return T


class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')

        self.declare_parameter('joint_angles', [0.0, 0.0, 0.0, 0.0])

        self.fk_pub = self.create_publisher(Point, '/fk_result', 10)
        self.joint_sub = self.create_subscription(
            JointState, '/fk_joint_input', self.joint_callback, 10)

        initial_angles = self.get_parameter('joint_angles').value
        self.compute_and_publish(initial_angles)

        self.get_logger().info('FK Node ready. Listening on /fk_joint_input')
        self.get_logger().info('Or set joint_angles parameter at launch.')

    def joint_callback(self, msg):
        if len(msg.position) >= 4:
            self.compute_and_publish(list(msg.position[:4]))

    def compute_and_publish(self, joint_angles):
        T = forward_kinematics(joint_angles)
        pos = T[:3, 3]

        msg = Point()
        msg.x = float(pos[0])
        msg.y = float(pos[1])
        msg.z = float(pos[2])
        self.fk_pub.publish(msg)

        self.get_logger().info(
            f'Joint angles: [{joint_angles[0]:.4f}, {joint_angles[1]:.4f}, '
            f'{joint_angles[2]:.4f}, {joint_angles[3]:.4f}]')
        self.get_logger().info(
            f'End-effector position: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]')
        self.get_logger().info(f'Full transform:\n{T}')


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
