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
struct GeometricConstraint
{
  std::string name;

  enum ConstraintType {PLANE, LINE, ANGLE};
  ConstraintType type;

  // Plane constraint parameters
  Eigen::Vector3d plane_origin;
  Eigen::Vector3d plane_normal;
  double plane_tolerance;

  // Line constraint parameters
  Eigen::Vector3d line_origin;
  Eigen::Vector3d line_dir;
  double line_max_distance;

  // Angle constraint parameters
  Eigen::Vector3d max_angle;
  Eigen::Vector3d max_angle_cos;
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
  PoseConstraintsManager();
  explicit PoseConstraintsManager(const std::vector<GeometricConstraint> &constraints);

  /** define shared pointer PoseConstraintsManager::Ptr*/
  using Ptr = std::shared_ptr<PoseConstraintsManager>;

  /** Add a single constraint to the existing set. */
  void addConstraint(const GeometricConstraint& constraint);

  /** Replace the existing set of constraints. */
  void setConstraints(const std::vector<GeometricConstraint>& constraints);

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

  /// @brief Checks axis-wise maximum angle constraints between two transforms.
  bool checkAngleConstraint(const Eigen::Affine3d& T_1,
                            const Eigen::Affine3d& T_2,
                            const Eigen::Vector3d& max_angle_cos,
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

  std::vector<GeometricConstraint> constraints_;
};


// Conversion function: GeometricConstraint to struct
GeometricConstraint convertGeometricConstraint(
    const pose_constraints_msgs::msg::GeometricConstraint& msg);

// Conversion function: GeometricConstraintArray to std::vector of structs
std::vector<pose_constraints_planner::GeometricConstraint> convertGeometricConstraintArray(
    const pose_constraints_msgs::msg::GeometricConstraintArray& msg_array);


}  // namespace pose_constraints_planner
