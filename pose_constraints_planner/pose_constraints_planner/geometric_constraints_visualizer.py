#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Point

from pose_constraints_msgs.msg import GeometricConstraintArray, GeometricConstraint


def _norm(v: Tuple[float, float, float]) -> float:
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])


def _normalize(v: Tuple[float, float, float], eps: float = 1e-12) -> Tuple[float, float, float]:
    n = _norm(v)
    if n < eps:
        return (0.0, 0.0, 1.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _dot(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def _cross(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    )


def _z_axis_alignment_matrix(z_axis: Tuple[float, float, float]):
    """
    Return a 3x3 rotation matrix (tuple-of-tuples) such that the local Z axis is aligned to z_axis.
    The remaining axes are chosen to form a right-handed orthonormal basis.
    """
    z = _normalize(z_axis)

    up = (0.0, 0.0, 1.0)
    if abs(_dot(z, up)) > 0.99:
        up = (0.0, 1.0, 0.0)

    x = _normalize(_cross(up, z))
    if _norm(x) < 1e-9:
        up = (1.0, 0.0, 0.0)
        x = _normalize(_cross(up, z))

    y = _normalize(_cross(z, x))

    # Columns are [x y z]
    return (
        (x[0], y[0], z[0]),
        (x[1], y[1], z[1]),
        (x[2], y[2], z[2]),
    )


def _quaternion_from_rotation_matrix(R) -> Quaternion:
    """Convert a 3x3 rotation matrix to a ROS quaternion (x, y, z, w)."""
    r00, r01, r02 = R[0]
    r10, r11, r12 = R[1]
    r20, r21, r22 = R[2]

    tr = r00 + r11 + r22

    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (r21 - r12) / S
        qy = (r02 - r20) / S
        qz = (r10 - r01) / S
    elif (r00 > r11) and (r00 > r22):
        S = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / S
        qx = 0.25 * S
        qy = (r01 + r10) / S
        qz = (r02 + r20) / S
    elif r11 > r22:
        S = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / S
        qx = (r01 + r10) / S
        qy = 0.25 * S
        qz = (r12 + r21) / S
    else:
        S = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / S
        qx = (r02 + r20) / S
        qy = (r12 + r21) / S
        qz = 0.25 * S

    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n > 1e-12:
        qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    else:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

    q = Quaternion()
    q.x, q.y, q.z, q.w = float(qx), float(qy), float(qz), float(qw)
    return q


def _point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


class GeometricConstraintsVisualizer(Node):
    """
    Subscribes to /geometry_constraints (pose_constraints_msgs/GeometricConstraintArray)
    Publishes /geometric_constraints_markers (visualization_msgs/MarkerArray) every 2 seconds.
    """

    def __init__(self):
        super().__init__('geometric_constraints_visualizer')

        self._constraints_msg: Optional[GeometricConstraintArray] = None

        self.create_subscription(
            GeometricConstraintArray,
            '/geometry_constraints',
            self._constraints_cb,
            10
        )

        self._pub = self.create_publisher(
            MarkerArray,
            '/geometric_constraints_markers',
            10
        )

        self.create_timer(2.0, self._on_timer)

        self.get_logger().info("GeometricConstraintsVisualizer started. Waiting for /geometry_constraints...")

    def _constraints_cb(self, msg: GeometricConstraintArray) -> None:
        self._constraints_msg = msg
        self.get_logger().info(f"Received {len(msg.constraints)} constraint(s)")

    def _on_timer(self) -> None:
        if self._constraints_msg is None:
            return

        now = self.get_clock().now().to_msg()
        out = MarkerArray()

        # Clear previous markers to avoid leftovers when the constraint list shrinks.
        clear = Marker()
        clear.action = getattr(Marker, 'DELETEALL', 3)
        out.markers.append(clear)

        marker_id = 0

        for c in self._constraints_msg.constraints:
            frame = c.frame_id if c.frame_id else "world"

            if c.type == GeometricConstraint.PLANE:
                m = Marker()
                m.header.frame_id = frame
                m.header.stamp = now
                m.ns = "geometric_constraints"
                m.id = marker_id; marker_id += 1
                m.action = Marker.ADD
                m.type = Marker.CUBE

                m.pose.position.x = c.plane_origin.x
                m.pose.position.y = c.plane_origin.y
                m.pose.position.z = c.plane_origin.z

                normal = (c.plane_normal.x, c.plane_normal.y, c.plane_normal.z)
                m.pose.orientation = _quaternion_from_rotation_matrix(_z_axis_alignment_matrix(normal))

                thickness = max(0.01, 2.0 * float(c.plane_tolerance))
                m.scale.x = 5.0
                m.scale.y = 5.0
                m.scale.z = thickness

                m.color.a = 0.5
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
                out.markers.append(m)

            elif c.type == GeometricConstraint.LINE:
                m = Marker()
                m.header.frame_id = frame
                m.header.stamp = now
                m.ns = "geometric_constraints"
                m.id = marker_id; marker_id += 1
                m.action = Marker.ADD
                m.type = Marker.CYLINDER

                m.pose.position.x = c.line_origin.x
                m.pose.position.y = c.line_origin.y
                m.pose.position.z = c.line_origin.z

                direction = (c.line_direction.x, c.line_direction.y, c.line_direction.z)
                m.pose.orientation = _quaternion_from_rotation_matrix(_z_axis_alignment_matrix(direction))

                diameter = max(0.02, 2.0 * float(c.line_max_distance))
                m.scale.x = diameter
                m.scale.y = diameter
                m.scale.z = 5.0

                m.color.a = 0.5
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
                out.markers.append(m)

            elif c.type == GeometricConstraint.ANGLE:
                # One arrow per constrained axis (max_angle >= 0.0).
                axes = [
                    ('x', (1.0, 0.0, 0.0), float(c.max_angle.x)),
                    ('y', (0.0, 1.0, 0.0), float(c.max_angle.y)),
                    ('z', (0.0, 0.0, 1.0), float(c.max_angle.z)),
                ]

                arrow_len = 0.5
                shaft_d = 0.02
                head_d = 0.05
                head_l = 0.10

                published_any = False

                for axis_name, axis_vec, max_angle in axes:
                    if max_angle < 0.0:
                        continue
                    published_any = True

                    m = Marker()
                    m.header.frame_id = frame
                    m.header.stamp = now
                    m.ns = "geometric_constraints"
                    m.id = marker_id; marker_id += 1
                    m.action = Marker.ADD
                    m.type = Marker.ARROW

                    m.points = [
                        _point(0.0, 0.0, 0.0),
                        _point(axis_vec[0]*arrow_len, axis_vec[1]*arrow_len, axis_vec[2]*arrow_len)
                    ]

                    m.scale.x = shaft_d
                    m.scale.y = head_d
                    m.scale.z = head_l

                    m.color.a = 0.8
                    if axis_name == 'x':
                        m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
                    elif axis_name == 'y':
                        m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
                    else:
                        m.color.r, m.color.g, m.color.b = 0.0, 0.0, 1.0

                    out.markers.append(m)

                # Fallback marker if all axes are unconstrained.
                if not published_any:
                    m = Marker()
                    m.header.frame_id = frame
                    m.header.stamp = now
                    m.ns = "geometric_constraints"
                    m.id = marker_id; marker_id += 1
                    m.action = Marker.ADD
                    m.type = Marker.ARROW
                    m.points = [_point(0.0, 0.0, 0.0), _point(arrow_len, 0.0, 0.0)]
                    m.scale.x = shaft_d
                    m.scale.y = head_d
                    m.scale.z = head_l
                    m.color.a = 0.8
                    m.color.r, m.color.g, m.color.b = 0.0, 0.0, 1.0
                    out.markers.append(m)

            else:
                self.get_logger().warn(f"Unknown constraint type: {int(c.type)} (name='{c.name}')")

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GeometricConstraintsVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

