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

#include <pose_constraints_planner/pose_constraints_evaluator.hpp>

#include <algorithm>
#include <stdexcept>

namespace pose_constraints_planner
{

PoseConstraintsEvaluator::PoseConstraintsEvaluator(const std::shared_ptr<ik_solver::IkSolver>& ik_solver,
                                                   const PoseConstraintsManager::Ptr& constraints,
                                                   const std::vector<std::string>& planner_joint_names,
                                                   const Eigen::Affine3d& T_w_b,
                                                   const Eigen::Affine3d& T_f_t,
                                                   const Eigen::VectorXd& start_q)
  : ik_solver_(ik_solver), constraints_(constraints), T_w_b_(T_w_b), T_f_t_(T_f_t)
{
  if (!ik_solver_ || !constraints_)
    throw std::invalid_argument("PoseConstraintsEvaluator: IK solver and constraints must not be null");

  const std::vector<std::string>& ik_joints = ik_solver_->joint_names();
  ik_from_planner_.resize(ik_joints.size());
  for (size_t i = 0; i < ik_joints.size(); ++i)
  {
    auto it = std::find(planner_joint_names.begin(), planner_joint_names.end(), ik_joints[i]);
    if (it == planner_joint_names.end())
      throw std::invalid_argument("PoseConstraintsEvaluator: ik_solver joint '" + ik_joints[i] +
                                  "' is not a joint of the planning group");
    ik_from_planner_[i] = static_cast<Eigen::Index>(it - planner_joint_names.begin());
    if (ik_from_planner_[i] != static_cast<Eigen::Index>(i))
      identity_order_ = false;
  }
  if (ik_joints.size() != planner_joint_names.size())
    identity_order_ = false;

  T_w_start_ = toolPose(start_q);
}

Eigen::Affine3d PoseConstraintsEvaluator::toolPose(const Eigen::VectorXd& q) const
{
  if (identity_order_)
    return ik_solver_->computeFk(q, T_w_b_, T_f_t_);

  Eigen::VectorXd q_ik(static_cast<Eigen::Index>(ik_from_planner_.size()));
  for (size_t i = 0; i < ik_from_planner_.size(); ++i)
    q_ik(static_cast<Eigen::Index>(i)) = q(ik_from_planner_[i]);
  return ik_solver_->computeFk(q_ik, T_w_b_, T_f_t_);
}

bool PoseConstraintsEvaluator::satisfies(const Eigen::VectorXd& q, std::stringstream* report) const
{
  return constraints_->checkConstraints(toolPose(q), T_w_start_, report);
}

}  // namespace pose_constraints_planner
