#include <pose_constraints_planner/pose_constraints_manager.hpp>

#include <cmath>
#include <iostream>

namespace pose_constraints_planner
{

  PoseConstraintsManager::PoseConstraintsManager()
  {
    constraints_.clear();
  }

  PoseConstraintsManager::PoseConstraintsManager(const std::vector<GeometricConstraint>& constraints)
  : constraints_(std::move(constraints))
{
}

void PoseConstraintsManager::addConstraint(const GeometricConstraint& constraint)
{
  constraints_.push_back(constraint);
}

void PoseConstraintsManager::setConstraints(const std::vector<GeometricConstraint>& constraints)
{
  constraints_ = constraints;
}

void PoseConstraintsManager::setConstraints(const pose_constraints_msgs::msg::GeometricConstraintArray &constraints)
{
  setConstraints(convertGeometricConstraintArray(constraints));
}


bool PoseConstraintsManager::checkConstraints(const Eigen::Affine3d& T_current,
                                             const Eigen::Affine3d& T_start,
                                             std::stringstream* report) const
{
  for (const auto& constraint : constraints_)
  {
    double value;
    std::cout<<"Checking constraint: '" << constraint.name << "' of type "<<constraint.type<<std::endl;
    switch (constraint.type)
    {
      case GeometricConstraint::PLANE:
        if (!checkPlaneConstraint(T_current,
                                  constraint.plane_origin,
                                  constraint.plane_normal,
                                  constraint.plane_tolerance,
                                  value))
        {
          if (report)
            *report << "Plane constraint '" << constraint.name << "' violated. Distance: " << value << "\n";
          return false;
        }
        break;

      case GeometricConstraint::LINE:
        if (!checkLineConstraint(T_current,
                                 constraint.line_origin,
                                 constraint.line_dir,
                                 constraint.line_max_distance,
                                 value))
        {
          std::cout<<"Line constraint violated inside checkConstraints(). Value: "<<value<<std::endl;
          if (report)
            *report << "Line constraint '" << constraint.name << "' violated. Distance: " << value << "\n";
          return false;
        }
        std::cout<<"Line constraint satisfied inside checkConstraints(). Value: "<<value<<std::endl;
        break;

      case GeometricConstraint::ANGLE:
        if (!checkAngleConstraint(T_current,
                                  T_start,
                                  constraint.max_angle_cos,
                                  value))
        {
          if (report)
            *report << "Angle constraint '" << constraint.name << "' violated. Cosine of angle: " << value << "\n";
          return false;
        }
        break;
    }
    std::cout<<"Constraint '" << constraint.name << "' satisfied."<<std::endl;
  }
  std::cout<<"All constraints satisfied."<<std::endl;
  return true;
}

/// @brief Checks if the angle between the corresponding axes of two transformations exceeds the maximum allowed angles.
bool PoseConstraintsManager::checkAngleConstraint(const Eigen::Affine3d& T_1,
                                                 const Eigen::Affine3d& T_2,
                                                 const Eigen::Vector3d& max_angle_cos,
                                                 double& angle_cos) const
{
  for (int i = 0; i < 3; i++)
  {
    if (max_angle_cos(i) > -1)  // if max_angle_cos < -1 => no constraint
    {
      angle_cos = T_1.linear().row(i).dot(T_2.linear().row(i)) /
                  (T_1.linear().row(i).norm() * T_2.linear().row(i).norm());

      if (angle_cos < max_angle_cos(i))  // skip if the angle is too large
      {
        return false;
      }
    }
  }
  return true;
}

/// @brief Checks if the pose defined by transformation T satisfies the plane constraint.
bool PoseConstraintsManager::checkPlaneConstraint(const Eigen::Affine3d& T,
                                                 const Eigen::Vector3d& plane_origin,
                                                 const Eigen::Vector3d& plane_normal,
                                                 double tolerance,
                                                 double& distance) const
{
  Eigen::Vector3d vec_plane_to_p = T.translation() - plane_origin;
  distance = vec_plane_to_p.dot(plane_normal.normalized());  // distance from point to plane

  if (std::abs(distance) < tolerance)  // tolerance
  {
    return false;
  }
  return true;
}

/// @brief Checks if the pose defined by transformation T satisfies the line constraint.
bool PoseConstraintsManager::checkLineConstraint(const Eigen::Affine3d& T,
                                                const Eigen::Vector3d& line_origin,
                                                const Eigen::Vector3d& line_dir,
                                                double max_distance,
                                                double& distance) const
{
  Eigen::Vector3d p_to_line = T.translation() - line_origin;
  Eigen::Vector3d projection = p_to_line.dot(line_dir.normalized()) * line_dir.normalized();
  distance = (p_to_line - projection).norm();

  std::cout<<"Distance from line: "<<distance << " Max allowed: "<<max_distance<<std::endl;
  std::cout <<"Point: "<<T.translation().transpose()<<std::endl;
  std::cout <<"Line origin: "<<line_origin.transpose()<<std::endl;
  std::cout <<"Line direction: "<<line_dir.transpose()<<std::endl;
  if (distance < max_distance)
  {
    std::cout<<"Line constraint satisfied."<<std::endl;
    return true;
  }
  std::cout<<"Line constraint violated."<<std::endl;
  return false;
}

GeometricConstraint convertGeometricConstraint(const pose_constraints_msgs::msg::GeometricConstraint &msg)
{
  pose_constraints_planner::GeometricConstraint constraint;

  constraint.name = msg.name;
  constraint.type = static_cast<pose_constraints_planner::GeometricConstraint::ConstraintType>(msg.type);

  // Plane constraint
  if (constraint.type == pose_constraints_planner::GeometricConstraint::PLANE) {
    constraint.plane_origin = Eigen::Vector3d(msg.plane_origin.x, msg.plane_origin.y, msg.plane_origin.z);
    constraint.plane_normal = Eigen::Vector3d(msg.plane_normal.x, msg.plane_normal.y, msg.plane_normal.z);
    constraint.plane_tolerance = msg.plane_tolerance;
  }

  // Line constraint
  if (constraint.type == pose_constraints_planner::GeometricConstraint::LINE) {
    constraint.line_origin = Eigen::Vector3d(msg.line_origin.x, msg.line_origin.y, msg.line_origin.z);
    constraint.line_dir = Eigen::Vector3d(msg.line_direction.x, msg.line_direction.y, msg.line_direction.z);
    constraint.line_max_distance = msg.line_max_distance;
  }

  // Angle constraint
  if (constraint.type == pose_constraints_planner::GeometricConstraint::ANGLE) {
    constraint.max_angle = Eigen::Vector3d(msg.max_angle.x, msg.max_angle.y, msg.max_angle.z);
    constraint.max_angle_cos = Eigen::Vector3d(std::cos(msg.max_angle.x),
                                               std::cos(msg.max_angle.y),
                                               std::cos(msg.max_angle.z));
  }

  return constraint;
}

std::vector<GeometricConstraint> convertGeometricConstraintArray(const pose_constraints_msgs::msg::GeometricConstraintArray &msg_array)
{
  std::vector<pose_constraints_planner::GeometricConstraint> constraints;

  for (const auto& msg : msg_array.constraints) {
    constraints.push_back(convertGeometricConstraint(msg));
  }

  return constraints;
}

}  // namespace pose_constraints_planner
