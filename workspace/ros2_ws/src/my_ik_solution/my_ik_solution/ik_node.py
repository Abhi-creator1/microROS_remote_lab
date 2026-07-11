import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np
from my_ik_solution.fk_node import forward_kinematics

# Link lengths from DH parameters
D1 = 0.14415   # base height (d1)
A2 = 0.24381   # upper arm length (a2)
A3 = 0.12475   # forearm length (a3)
A4 = 0.08475   # wrist length (a4)

JOINT_NAMES = [
    'world_to_baselink',
    'base_to_shoulder',
    'shoulder_to_elbow',
    'elbow_to_wrist1',
]

# Joint limits from URDF
JOINT_LIMITS = [
    (-np.pi, np.pi),       # Joint 1
    (-np.pi, np.pi),       # Joint 2
    (-np.pi/4, np.pi/4),   # Joint 3
    (-np.pi/4, np.pi/4),   # Joint 4
]


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def geometric_ik(x, y, z, approach_angle=0.0):
    """
    Geometric IK for the 4-DOF ASRS arm.

    The arm is decomposed as:
      - Joint 1: base rotation (yaw) about Z
      - Joints 2,3,4: planar 3-link arm in the vertical plane

    Since we have 3 planar joints but only 2 planar constraints (r, z),
    we fix the end-effector approach angle (phi = theta2 + theta3 + theta4)
    and solve the remaining 2-link problem for the wrist center.

    Returns (theta1, theta2, theta3, theta4) or None if unreachable.
    """
    # Step 1: base rotation
    r_xy = np.sqrt(x**2 + y**2)
    if r_xy < 1e-6:
        theta1 = 0.0
    else:
        theta1 = np.arctan2(y, x)

    # Step 2: work in the vertical plane
    r = r_xy          # horizontal distance from base axis
    z_rel = z - D1    # height relative to shoulder pivot

    phi = approach_angle

    # Step 3: wrist center position (subtract wrist link contribution)
    r_w = r - A4 * np.cos(phi)
    z_w = z_rel - A4 * np.sin(phi)

    # Step 4: 2-link IK (law of cosines) for upper arm + forearm
    D_sq = r_w**2 + z_w**2
    cos_theta3 = (D_sq - A2**2 - A3**2) / (2.0 * A2 * A3)

    if abs(cos_theta3) > 1.0:
        return None  # unreachable

    sin_theta3 = np.sqrt(1.0 - cos_theta3**2)

    # Try both elbow-down (+sin) and elbow-up (-sin) configurations
    solutions = []
    for sign in [1.0, -1.0]:
        t3 = np.arctan2(sign * sin_theta3, cos_theta3)

        k1 = A2 + A3 * cos_theta3
        k2 = A3 * (sign * sin_theta3)
        t2 = np.arctan2(z_w, r_w) - np.arctan2(k2, k1)

        t4 = phi - t2 - t3

        solutions.append((
            clamp(theta1, *JOINT_LIMITS[0]),
            clamp(t2, *JOINT_LIMITS[1]),
            clamp(t3, *JOINT_LIMITS[2]),
            clamp(t4, *JOINT_LIMITS[3]),
        ))

    return solutions


def solve_ik(x, y, z):
    """
    Try multiple approach angles to find a valid IK solution.
    Returns the best solution (smallest position error after clamping).
    """
    best_solution = None
    best_error = float('inf')
    target = np.array([x, y, z])

    for phi in np.linspace(-np.pi/3, np.pi/3, 21):
        candidates = geometric_ik(x, y, z, approach_angle=phi)
        if candidates is None:
            continue

        for result in candidates:
            T = forward_kinematics(list(result))
            pos = T[:3, 3]
            error = np.linalg.norm(pos - target)

            if error < best_error:
                best_error = error
                best_solution = result

    return best_solution, best_error


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_sub = self.create_subscription(
            Point, '/target_position', self.target_callback, 10)

        self.current_angles = [0.0, 0.0, 0.0, 0.0]
        self.get_logger().info('Geometric IK Node ready. Listening on /target_position')

    def target_callback(self, msg):
        x, y, z = msg.x, msg.y, msg.z
        self.get_logger().info(f'Target: [{x:.4f}, {y:.4f}, {z:.4f}]')

        solution, error = solve_ik(x, y, z)

        if solution is None:
            self.get_logger().warn('Target is unreachable!')
            return

        self.current_angles = list(solution)
        self.get_logger().info(
            f'Solution: [{solution[0]:.4f}, {solution[1]:.4f}, '
            f'{solution[2]:.4f}, {solution[3]:.4f}] (error: {error:.6f} m)')

        if error > 0.01:
            self.get_logger().warn(
                f'Position error {error:.4f} m — joint limits prevent exact reach')

        self.publish_joints()

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = list(self.current_angles)
        self.joint_pub.publish(msg)

    def timer_publish(self):
        self.publish_joints()


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()

    timer = node.create_timer(0.05, node.timer_publish)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
