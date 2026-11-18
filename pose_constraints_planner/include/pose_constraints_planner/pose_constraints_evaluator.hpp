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
 * @file pose_constraints_evaluator.hpp
 * @brief Checks the geometric constraints on the tool pose of a joint configuration.
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <pose_constraints_planner/pose_constraints_manager.hpp>
#include <ik_solver/ik_solver.hpp>

namespace pose_constraints_planner
{

class PoseConstraintsEvaluator;
/** Shared pointer to a PoseConstraintsEvaluator. */
using PoseConstraintsEvaluatorPtr = std::shared_ptr<PoseConstraintsEvaluator>;

/**
 * @brief Computes the tool pose of a configuration (forward kinematics of an ik_solver plugin)
 * and checks it with a PoseConstraintsManager.
 *
 * Configurations are given in the planner joint order, which may differ from the order of the
 * IK solver: they are permuted before the forward kinematics. ANGLE constraints are relative to the
 * start tool pose.
 *
 * The object is immutable after construction; it is shared by samplers and collision checkers.
 * Concurrent use requires a thread-safe forward kinematics in the IK solver.
 */
class PoseConstraintsEvaluator
{
public:
  /**
   * @brief Constructor.
   * @param ik_solver IK solver, used for the forward kinematics.
   * @param constraints Geometric constraints (all quantities in the world frame).
   * @param planner_joint_names Joint order of the configurations given to the evaluator.
   * @param T_w_b Pose of the robot base in the world frame.
   * @param T_f_t Pose of the tool in the flange frame.
   * @param start_q Start configuration (planner joint order): its tool pose is the reference of ANGLE constraints.
   * @throw std::invalid_argument if a pointer is null or the joints of the IK solver are not all in @p planner_joint_names.
   */
  PoseConstraintsEvaluator(const std::shared_ptr<ik_solver::IkSolver>& ik_solver,
                           const PoseConstraintsManager::Ptr& constraints,
                           const std::vector<std::string>& planner_joint_names,
                           const Eigen::Affine3d& T_w_b,
                           const Eigen::Affine3d& T_f_t,
                           const Eigen::VectorXd& start_q);

  /** Tool pose of @p q (planner joint order) in the world frame. */
  Eigen::Affine3d toolPose(const Eigen::VectorXd& q) const;

  /**
   * @brief True if the tool pose of @p q satisfies all the constraints.
   * @param q Configuration, planner joint order.
   * @param report Optional human-readable report.
   */
  bool satisfies(const Eigen::VectorXd& q, std::stringstream* report = nullptr) const;

  /** Tool pose of the start configuration in the world frame. */
  const Eigen::Affine3d& startPose() const { return T_w_start_; }

  /** The constraints. */
  const PoseConstraintsManager::Ptr& constraints() const { return constraints_; }

protected:
  std::shared_ptr<ik_solver::IkSolver> ik_solver_;  ///< Forward kinematics.
  PoseConstraintsManager::Ptr constraints_;  ///< Constraints.
  std::vector<Eigen::Index> ik_from_planner_;  ///< ik_solver joint i is planner joint ik_from_planner_[i].
  bool identity_order_ = true;  ///< True if no permutation is needed.
  Eigen::Affine3d T_w_b_;  ///< World to base.
  Eigen::Affine3d T_f_t_;  ///< Flange to tool.
  Eigen::Affine3d T_w_start_;  ///< Start tool pose in the world frame.
};

}  // namespace pose_constraints_planner
