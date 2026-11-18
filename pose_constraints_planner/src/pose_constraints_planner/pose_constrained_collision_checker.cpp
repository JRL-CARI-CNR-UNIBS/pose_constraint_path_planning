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

#include <pose_constraints_planner/pose_constrained_collision_checker.hpp>

#include <cmath>
#include <stdexcept>

namespace pose_constraints_planner
{

PoseConstrainedCollisionChecker::PoseConstrainedCollisionChecker(const graph::core::CollisionCheckerPtr& inner,
                                                                 const PoseConstraintsEvaluatorPtr& evaluator,
                                                                 const double& resolution,
                                                                 const graph::core::LoggerPtr& logger)
  : CollisionCheckerBase(logger, resolution), inner_(inner), evaluator_(evaluator), resolution_(resolution)
{
  if (!inner_ || !evaluator_)
    throw std::invalid_argument("PoseConstrainedCollisionChecker: inner checker and evaluator must not be null");
  if (!(resolution_ > 0.0))
    throw std::invalid_argument("PoseConstrainedCollisionChecker: resolution must be positive");
}

bool PoseConstrainedCollisionChecker::constraintsHoldAlong(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
                                                           Eigen::VectorXd* last_valid) const
{
  if (!evaluator_->satisfies(q1))
  {
    if (last_valid)
      *last_valid = q1;
    return false;
  }
  const double dist = (q2 - q1).norm();
  const unsigned int npnt = static_cast<unsigned int>(std::ceil(dist / resolution_));
  if (npnt > 1)
  {
    const Eigen::VectorXd step = (q2 - q1) / npnt;
    for (unsigned int ipnt = 1; ipnt < npnt; ipnt++)
    {
      if (!evaluator_->satisfies(q1 + step * ipnt))
      {
        if (last_valid)
          *last_valid = q1 + step * (ipnt - 1);
        return false;
      }
    }
  }
  if (!evaluator_->satisfies(q2))
  {
    if (last_valid)
      *last_valid = npnt > 1 ? Eigen::VectorXd(q1 + (q2 - q1) * (npnt - 1) / npnt) : q1;
    return false;
  }
  return true;
}

bool PoseConstrainedCollisionChecker::check(const Eigen::VectorXd& configuration)
{
  if (!evaluator_->satisfies(configuration))
  {
    constraint_rejections_++;
    return false;
  }
  return inner_->check(configuration);
}

bool PoseConstrainedCollisionChecker::checkConnection(const Eigen::VectorXd& configuration1,
                                                      const Eigen::VectorXd& configuration2, Eigen::VectorXd& conf)
{
  Eigen::VectorXd last_valid;
  if (constraintsHoldAlong(configuration1, configuration2, &last_valid))
    return inner_->checkConnection(configuration1, configuration2, conf);

  constraint_rejections_++;
  // The edge is invalid anyway: report the last point that is also collision-free.
  if (inner_->checkConnection(configuration1, last_valid, conf))
    conf = last_valid;
  return false;
}

bool PoseConstrainedCollisionChecker::checkConnection(const Eigen::VectorXd& configuration1,
                                                      const Eigen::VectorXd& configuration2)
{
  if (!constraintsHoldAlong(configuration1, configuration2))
  {
    constraint_rejections_++;
    return false;
  }
  return inner_->checkConnection(configuration1, configuration2);
}

bool PoseConstrainedCollisionChecker::checkConnection(const graph::core::ConnectionPtr& conn)
{
  if (!constraintsHoldAlong(conn->getParent()->getConfiguration(), conn->getChild()->getConfiguration()))
  {
    constraint_rejections_++;
    return false;
  }
  return inner_->checkConnection(conn);
}

bool PoseConstrainedCollisionChecker::checkConnections(const std::vector<graph::core::ConnectionPtr>& connections)
{
  for (const graph::core::ConnectionPtr& conn : connections)
  {
    if (!constraintsHoldAlong(conn->getParent()->getConfiguration(), conn->getChild()->getConfiguration()))
    {
      constraint_rejections_++;
      return false;
    }
  }
  return inner_->checkConnections(connections);
}

bool PoseConstrainedCollisionChecker::checkConnFromConf(const graph::core::ConnectionPtr& conn,
                                                        const Eigen::VectorXd& this_conf)
{
  if (!constraintsHoldAlong(this_conf, conn->getChild()->getConfiguration()))
  {
    constraint_rejections_++;
    return false;
  }
  return inner_->checkConnFromConf(conn, this_conf);
}

graph::core::CollisionCheckerPtr PoseConstrainedCollisionChecker::clone()
{
  return std::make_shared<PoseConstrainedCollisionChecker>(inner_->clone(), evaluator_, resolution_, logger_);
}

}  // namespace pose_constraints_planner
