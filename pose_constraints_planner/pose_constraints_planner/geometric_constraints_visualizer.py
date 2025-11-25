#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion
import os


def z_axis_alignment_matrix(v):
    """
    Create a 3x3 rotation matrix that aligns the local z-axis to vector v.
    v should be normalized.
    """
    v = np.array(v)
    v = v / np.linalg.norm(v)  # normalize

    # Choose an arbitrary vector not parallel to v
    if abs(v[2]) < 0.99:
        up = np.array([0, 0, 1])
    else:
        up = np.array([0, 1, 0])

    # x-axis of the new frame
    x_axis = np.cross(up, v)
    x_axis /= np.linalg.norm(x_axis)

    # y-axis of the new frame
    y_axis = np.cross(v, x_axis)

    # z-axis is the vector itself
    z_axis = v

    R = np.column_stack((x_axis, y_axis, z_axis))  # rotation matrix 3x3
    return R

def homogeneous_matrix(origin, direction):
    R = z_axis_alignment_matrix(direction)
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = origin
    return T

def quaternion_from_matrix(T):
    q = Rotation.from_matrix(T[0:3,0:3]).as_quat()
    # scipy returns [x, y, z, w] -> matches ROS
    q_msg = Quaternion()
    q_msg.x = q[0]
    q_msg.y = q[1]
    q_msg.z = q[2]
    q_msg.w = q[3]
    return q_msg

class GeometricConstraintVisualizer(Node):
    def __init__(self):
        super().__init__('geometric_constraint_visualizer')
        self.publisher_ = self.create_publisher(MarkerArray, 'geometric_constraints_markers', 10)

        # Declare parameter
        self.declare_parameter('constaints_description', '')

        # Read parameter value
        yaml_path = self.get_parameter('constaints_description').get_parameter_value().string_value

        if not os.path.isfile(yaml_path):
            self.get_logger().error(f"YAML file does not exist: {yaml_path}")
            return
        
        # Load YAML
        with open(yaml_path, 'r') as f:
            self.constraints_yaml = yaml.safe_load(f)

        # Publish markers
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        count = 0
        
        constraints = self.constraints_yaml.get('pose_constraints_planner',[]).get('geometric_constraints',[])

        self.get_logger().info(f"Found {len(constraints)} geometric constraints to visualize.")

        for c in constraints:

            self.get_logger().info(f"Processing constraint: {c}")
            name = c.get('name', f'constraint_{count}')
            ctype = c.get('type')
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "geometric_constraints"
            marker.id = count
            marker.action = Marker.ADD
            marker.lifetime.sec = 0  # forever
            marker.lifetime.nanosec = 0
            marker.color.a = 0.5  # semi-transparent

            if ctype == "plane":
                # Represent plane as a thin cube
                marker.type = Marker.CUBE
                marker.pose.position.x = c['origin'][0]
                marker.pose.position.y = c['origin'][1]
                marker.pose.position.z = c['origin'][2]
                marker.pose.orientation = quaternion_from_matrix(
                    homogeneous_matrix(c['origin'], c['normal'])
                )
                marker.scale.x = 5.0
                marker.scale.y = 5.0
                marker.scale.z = 0.01  # thin
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            elif ctype == "orientation":
                # Represent orientation as an arrow pointing along constrained axis
                marker.header.frame_id = "tool0"
                marker.type = Marker.ARROW
                marker.pose.position.x = 0.0
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.0
                marker.scale.x = 0.5  # length
                marker.scale.y = 0.02  # shaft diameter
                marker.scale.z = 0.05  # head diameter
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0

            elif ctype == "line":
                # Represent line as a long thin cylinder
                marker.type = Marker.CYLINDER
                marker.pose.position.x = c['origin'][0]
                marker.pose.position.y = c['origin'][1]
                marker.pose.position.z = c['origin'][2]
                marker.pose.orientation = quaternion_from_matrix(
                    homogeneous_matrix(c['origin'], c['direction'])
                )
                marker.scale.x = c['max_distance']*2  # diameter
                marker.scale.y = c['max_distance']*2  # diameter
                marker.scale.z = 5.0  
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            else:
                self.get_logger().warn(f"Unknown constraint type: {ctype}")
                continue

            marker_array.markers.append(marker)
            count += 1

        self.publisher_.publish(marker_array)
        self.get_logger().info(f"Published {count} geometric constraint markers")


def main(args=None):
    rclpy.init(args=args)

    print("Starting Geometric Constraint Visualizer Node")
    node = GeometricConstraintVisualizer()
    #rclpy.spin(node)

    print("Publishing markers...")
    node.publish_markers()
    rclpy.spin(node)
    node.destroy_node()
    print("Shutting down Geometric Constraint Visualizer Node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
