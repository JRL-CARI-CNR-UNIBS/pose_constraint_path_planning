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
 * @file pose_constrained_multigoal_solver.hpp
 * @brief MI-RRT* (graph::mirrt_star::MultigoalSolver) sampling only configurations that satisfy the pose constraints.
 */
#pragma once

#include <vector>

#include <mirrt_star/solvers/mirrt_star.h>
#include <pose_constraints_planner/pose_constrained_sampler.hpp>

namespace pose_constraints_planner
{

class PoseConstrainedMultigoalSolver;
/** Shared pointer to a PoseConstrainedMultigoalSolver. */
using PoseConstrainedMultigoalSolverPtr = std::shared_ptr<PoseConstrainedMultigoalSolver>;

/**
 * @brief Sampling statistics of the per-goal PoseConstrainedSampler instances.
 */
struct ConstrainedSamplingStatistics
{
  unsigned long attempts = 0;  ///< Total draws.
  unsigned long rejections = 0;  ///< Draws rejected because of the constraints.
  unsigned long failed_samples = 0;  ///< sample() calls that returned an infeasible configuration.
  double reject_time = 0.0;  ///< Time spent on rejected draws [s].
  double sampling_time = 0.0;  ///< Time spent sampling [s].

  /** Fraction of rejected draws (0 if nothing has been drawn). */
  double rejectionRate() const { return attempts > 0 ? static_cast<double>(rejections) / attempts : 0.0; }
};

/**
 * @brief graph::mirrt_star::MultigoalSolver whose per-goal samplers draw only configurations that
 * satisfy the geometric pose constraints.
 *
 * MultigoalSolver creates, for every registered goal, an informed sampler and a tube sampler around
 * the goal's current solution. This class wraps each informed sampler with a PoseConstrainedSampler
 * (rejection sampling on the constraints) and rebuilds the tube sampler on top of it, so that both
 * the informed and the tube samples are feasible (the tube sampler accepts a tube sample only if
 * the wrapped sampler's inBounds() accepts it, which checks the constraints).
 *
 * Sampling makes the nodes feasible; the edges must be checked with a PoseConstrainedCollisionChecker.
 */
class PoseConstrainedMultigoalSolver : public graph::mirrt_star::MultigoalSolver
{
public:
  /**
   * @brief Constructor. config() must be called before adding start and goals.
   * @param metrics Metrics.
   * @param checker Collision checker (a PoseConstrainedCollisionChecker, to enforce the constraints along the edges).
   * @param sampler Sampler; only its bounds are used to build the per-goal samplers.
   * @param goal_cost_fcn Goal cost function.
   * @param evaluator Constraint evaluator.
   * @param max_sampling_attempts Maximum draws per sample() of each PoseConstrainedSampler.
   * @param logger Logger.
   */
  PoseConstrainedMultigoalSolver(const graph::core::MetricsPtr& metrics,
                                 const graph::core::CollisionCheckerPtr& checker,
                                 const graph::core::SamplerPtr& sampler,
                                 const graph::core::GoalCostFunctionPtr& goal_cost_fcn,
                                 const PoseConstraintsEvaluatorPtr& evaluator,
                                 const unsigned int& max_sampling_attempts,
                                 const graph::core::LoggerPtr& logger);

  using graph::mirrt_star::MultigoalSolver::addGoal;

  /**
   * @brief Register a candidate goal (see MultigoalSolver::addGoal()) and replace its samplers with
   * constrained ones.
   */
  bool addGoal(const graph::core::NodePtr& goal_node, const double& max_time, const double& apple_cost) override;

  /** Sum of the statistics of all the constrained samplers created so far. */
  ConstrainedSamplingStatistics samplingStatistics() const;

protected:
  PoseConstraintsEvaluatorPtr evaluator_;  ///< Constraint evaluator.
  unsigned int max_sampling_attempts_;  ///< Maximum draws per sample().
  std::vector<PoseConstrainedSamplerPtr> constrained_samplers_;  ///< One per registered goal.
};

}  // namespace pose_constraints_planner
