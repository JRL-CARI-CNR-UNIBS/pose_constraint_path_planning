// Copyright 2026 JRL-CARI CNR-STIIMA/UNIBS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file pose_constraints_manager.hpp
 * @brief Geometric constraints on the tool pose and their evaluation.
 */
#pragma once

// Standard
#include <string>
#include <vector>
#include <sstream>

// Eigen
#include <Eigen/Geometry>
#include <pose_constraints_msgs/msg/geometric_constraint.hpp>
#include <pose_constraints_msgs/msg/geometric_constraint_array.hpp>
#include <vector>



namespace pose_constraints_planner
{
/**
 * @brief A geometric constraint on the tool pose (all quantities in the world frame).
 *
 * - @c PLANE: the signed distance of the tool origin from the plane (along @c plane_normal)
 *   must be at least @c plane_tolerance, i.e. the tool stays on the side the normal points to;
 * - @c LINE: the distance of the tool origin from the line must be less than @c line_max_distance;
 * - @c ANGLE: for each axis i (x, y, z), the angle between the i-th rows of the current and of
 *   the start rotation matrices of the tool, expressed in the reference frame of the constraint
 *   (R_ref_tool = @c R_ref_world * R_world_tool), must not exceed component i of @c max_angle.
 *   A negative value or a value >= pi leaves the axis free.
 *
 * A constraint defined in another frame is expressed in the world frame with transformConstraint().
 */
struct GeometricConstraint
{
  std::string name;  ///< Name, used in reports.
  std::string frame_id;  ///< Frame in which the constraint was defined (informative after transformConstraint()).

  /**
   * @brief Constraint type (same values as pose_constraints_msgs/GeometricConstraint).
   */
  enum ConstraintType {PLANE, LINE, ANGLE};
  ConstraintType type;  ///< Constraint type.

  // Plane constraint parameters
  Eigen::Vector3d plane_origin;  ///< PLANE: a point of the plane.
  Eigen::Vector3d plane_normal;  ///< PLANE: normal pointing to the allowed side.
  double plane_tolerance;  ///< PLANE: minimum signed distance from the plane.

  // Line constraint parameters
  Eigen::Vector3d line_origin;  ///< LINE: a point of the line.
  Eigen::Vector3d line_dir;  ///< LINE: line direction.
  double line_max_distance;  ///< LINE: maximum distance from the line.

  // Angle constraint parameters
  Eigen::Vector3d max_angle;  ///< ANGLE: maximum angle for each axis [rad].
  Eigen::Vector3d max_angle_cos;  ///< ANGLE: cosine of max_angle (computed by the conversion functions).
  Eigen::Matrix3d R_ref_world = Eigen::Matrix3d::Identity();  ///< ANGLE: orientation of the world in the reference frame.
};


/**
 * @brief Lightweight container + evaluator for geometric pose constraints.
 *
 * This class provides a small public API to manage constraints and evaluate
 * whether a pose (T_current) respects the stored constraints w.r.t. a reference
 * pose (T_start).
 */
class PoseConstraintsManager
{
public:
  /**
   * @brief Constructor with no constraints.
   */
  PoseConstraintsManager();
  /**
   * @brief Constructor.
   * @param constraints Initial constraints.
   */
  explicit PoseConstraintsManager(const std::vector<GeometricConstraint> &constraints);

  /** define shared pointer PoseConstraintsManager::Ptr*/
  using Ptr = std::shared_ptr<PoseConstraintsManager>;

  /** Add a single constraint to the existing set. */
  void addConstraint(const GeometricConstraint& constraint);

  /** Replace the existing set of constraints. */
  void setConstraints(const std::vector<GeometricConstraint>& constraints);

  /**
   * @brief Replace the existing set of constraints with those of a ROS message.
   */
  void setConstraints(const pose_constraints_msgs::msg::GeometricConstraintArray& constraints);


  /**
   * @brief Evaluate all stored constraints.
   *
   * @param T_current Pose to be checked.
   * @param T_start   Reference pose (used by ANGLE constraints).
   * @param report    Optional stream to receive a human-readable report.
   * @return True if all constraints are satisfied, false otherwise.
   */
  bool checkConstraints(const Eigen::Affine3d& T_current,
                        const Eigen::Affine3d& T_start,
                        std::stringstream* report = nullptr) const;

  /** Read-only access to the current constraints. */
  const std::vector<GeometricConstraint>& constraints() const noexcept { return constraints_; }

protected:
  // The following functions are the ones provided in the attached sources,
  // exposed as protected methods as requested.

  /// @brief Checks axis-wise maximum angle constraints between two transforms, in the frame given by R_ref_world.
  bool checkAngleConstraint(const Eigen::Affine3d& T_1,
                            const Eigen::Affine3d& T_2,
                            const Eigen::Vector3d& max_angle_cos,
                            const Eigen::Matrix3d& R_ref_world,
                            double& angle_cos) const;

  /// @brief Checks point-to-plane distance constraint.
  bool checkPlaneConstraint(const Eigen::Affine3d& T,
                            const Eigen::Vector3d& plane_origin,
                            const Eigen::Vector3d& plane_normal,
                            double tolerance,
                            double& distance) const;

  /// @brief Checks point-to-line distance constraint.
  bool checkLineConstraint(const Eigen::Affine3d& T,
                           const Eigen::Vector3d& line_origin,
                           const Eigen::Vector3d& line_dir,
                           double max_distance,
                           double& distance) const;

  std::vector<GeometricConstraint> constraints_;  ///< Stored constraints.
};


/**
 * @brief Express a constraint defined in a frame F in the world frame.
 *
 * Points are mapped with @p T_world_frame, directions with its rotation; for ANGLE constraints the
 * reference frame stays F (R_ref_world becomes R_ref_world * R_world_F^T).
 * @param constraint Constraint whose quantities are expressed in F.
 * @param T_world_frame Pose of F in the world frame.
 * @return The same constraint with its quantities expressed in the world frame.
 */
GeometricConstraint transformConstraint(const GeometricConstraint& constraint, const Eigen::Affine3d& T_world_frame);

// Conversion function: GeometricConstraint to struct
/**
 * @brief Convert a ROS constraint message to a GeometricConstraint (also computes max_angle_cos).
 *
 * The quantities are copied as they are, in the frame @c msg.frame_id: use transformConstraint()
 * to express them in the world frame.
 */
GeometricConstraint convertGeometricConstraint(
    const pose_constraints_msgs::msg::GeometricConstraint& msg);

// Conversion function: GeometricConstraintArray to std::vector of structs
/**
 * @brief Convert a ROS constraint array message to a list of GeometricConstraint.
 */
std::vector<pose_constraints_planner::GeometricConstraint> convertGeometricConstraintArray(
    const pose_constraints_msgs::msg::GeometricConstraintArray& msg_array);


}  // namespace pose_constraints_planner
