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
 * @file pose_constrained_collision_checker.hpp
 * @brief Collision checker that also rejects configurations and edges violating the pose constraints.
 */
#pragma once

#include <graph_core/collision_checkers/collision_checker_base.h>
#include <pose_constraints_planner/pose_constraints_evaluator.hpp>

namespace pose_constraints_planner
{

class PoseConstrainedCollisionChecker;
/** Shared pointer to a PoseConstrainedCollisionChecker. */
using PoseConstrainedCollisionCheckerPtr = std::shared_ptr<PoseConstrainedCollisionChecker>;

/**
 * @brief Wraps a collision checker and adds the geometric pose constraints.
 *
 * A configuration is valid if it satisfies the constraints and the wrapped checker accepts it.
 * An edge is valid if the constraints hold at points spaced at most @c resolution (joint-space
 * norm) along it, and the wrapped checker accepts it. The constraints are checked first, since
 * the forward kinematics is cheaper than a collision check; the wrapped checker keeps its own
 * (e.g. parallel) edge checking.
 *
 * With this checker the solver, the goal filtering and any path optimizer enforce the constraints
 * on the whole path, not only on the nodes.
 */
class PoseConstrainedCollisionChecker : public graph::core::CollisionCheckerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor.
   * @param inner Wrapped collision checker.
   * @param evaluator Constraint evaluator.
   * @param resolution Maximum joint-space distance between two constraint checks along an edge.
   * @param logger graph_core logger.
   * @throw std::invalid_argument if @p inner or @p evaluator is null, or @p resolution is not positive.
   */
  PoseConstrainedCollisionChecker(const graph::core::CollisionCheckerPtr& inner,
                                  const PoseConstraintsEvaluatorPtr& evaluator,
                                  const double& resolution,
                                  const graph::core::LoggerPtr& logger);

  bool check(const Eigen::VectorXd& configuration) override;

  bool checkConnection(const Eigen::VectorXd& configuration1, const Eigen::VectorXd& configuration2,
                       Eigen::VectorXd& conf) override;

  bool checkConnection(const Eigen::VectorXd& configuration1, const Eigen::VectorXd& configuration2) override;

  bool checkConnection(const graph::core::ConnectionPtr& conn) override;

  bool checkConnections(const std::vector<graph::core::ConnectionPtr>& connections) override;

  bool checkConnFromConf(const graph::core::ConnectionPtr& conn, const Eigen::VectorXd& this_conf) override;

  std::string getGroupName() override { return inner_->getGroupName(); }

  /** Copy with a clone of the wrapped checker (the evaluator is shared). */
  graph::core::CollisionCheckerPtr clone() override;

  /**
   * @brief True if the constraints hold along the segment from @p q1 to @p q2 (end points included).
   * @param q1 First configuration.
   * @param q2 Second configuration.
   * @param[out] last_valid Last checked point that satisfies the constraints (@p q1 if none); unchanged if
   * the whole segment is valid.
   */
  bool constraintsHoldAlong(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
                            Eigen::VectorXd* last_valid = nullptr) const;

  /** Number of configurations or edges rejected because of the constraints. */
  unsigned long getConstraintRejections() const { return constraint_rejections_; }

  /** The wrapped checker. */
  const graph::core::CollisionCheckerPtr& getInnerChecker() const { return inner_; }

protected:
  graph::core::CollisionCheckerPtr inner_;  ///< Wrapped checker.
  PoseConstraintsEvaluatorPtr evaluator_;  ///< Constraint evaluator.
  double resolution_;  ///< Constraint-check spacing along edges.
  unsigned long constraint_rejections_ = 0;  ///< Rejections due to the constraints.
};

}  // namespace pose_constraints_planner
