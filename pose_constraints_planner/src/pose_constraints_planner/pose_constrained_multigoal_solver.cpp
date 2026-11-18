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

#include <pose_constraints_planner/pose_constrained_multigoal_solver.hpp>

#include <cmath>
#include <stdexcept>

namespace pose_constraints_planner
{

PoseConstrainedMultigoalSolver::PoseConstrainedMultigoalSolver(const graph::core::MetricsPtr& metrics,
                                                               const graph::core::CollisionCheckerPtr& checker,
                                                               const graph::core::SamplerPtr& sampler,
                                                               const graph::core::GoalCostFunctionPtr& goal_cost_fcn,
                                                               const PoseConstraintsEvaluatorPtr& evaluator,
                                                               const unsigned int& max_sampling_attempts,
                                                               const graph::core::LoggerPtr& logger)
  : MultigoalSolver(metrics, checker, sampler, goal_cost_fcn, logger),
    evaluator_(evaluator),
    max_sampling_attempts_(max_sampling_attempts)
{
  if (!evaluator_)
    throw std::invalid_argument("PoseConstrainedMultigoalSolver: evaluator must not be null");
}

bool PoseConstrainedMultigoalSolver::addGoal(const graph::core::NodePtr& goal_node, const double& max_time,
                                             const double& apple_cost)
{
  const size_t n_goals = goal_nodes_.size();
  if (!MultigoalSolver::addGoal(goal_node, max_time, apple_cost))
    return false;
  if (goal_nodes_.size() != n_goals + 1)  // not registered (defensive: addGoal returned true)
    return true;

  // Replace the informed sampler of the new goal with a constrained one (same informed set and cost),
  // and rebuild its tube sampler on top of it with the settings used by MultigoalSolver::addGoal().
  auto constrained =
      std::make_shared<PoseConstrainedSampler>(samplers_.back(), evaluator_, max_sampling_attempts_, logger_);
  auto tube = std::make_shared<graph::core::TubeInformedSampler>(constrained, metrics_);
  tube->setLocalBias(local_bias_);
  tube->setRadius(tube_radius_);
  if (solutions_.back())
    tube->setPath(solutions_.back());
  if (std::isfinite(constrained->getCost()))
    tube->setCost(constrained->getCost());

  samplers_.back() = constrained;
  tube_samplers_.back() = tube;
  constrained_samplers_.push_back(constrained);
  return true;
}

ConstrainedSamplingStatistics PoseConstrainedMultigoalSolver::samplingStatistics() const
{
  ConstrainedSamplingStatistics stats;
  for (const PoseConstrainedSamplerPtr& s : constrained_samplers_)
  {
    stats.attempts += s->getAttempts();
    stats.rejections += s->getRejections();
    stats.failed_samples += s->getFailedSamples();
    stats.reject_time += s->getRejectTime();
    stats.sampling_time += s->getSamplingTime();
  }
  return stats;
}

}  // namespace pose_constraints_planner
