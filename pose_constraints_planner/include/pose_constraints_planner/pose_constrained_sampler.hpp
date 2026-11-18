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
 * @file pose_constrained_sampler.hpp
 * @brief graph_core sampler that returns only configurations whose tool pose satisfies the geometric constraints.
 */
#pragma once

#include <graph_core/samplers/sampler_base.h>
#include <pose_constraints_planner/pose_constraints_evaluator.hpp>

namespace pose_constraints_planner
{

class PoseConstrainedSampler;
/** Shared pointer to a PoseConstrainedSampler. */
using PoseConstrainedSamplerPtr = std::shared_ptr<PoseConstrainedSampler>;

/**
 * @brief Rejection sampler on the geometric pose constraints.
 *
 * Wraps another graph_core sampler (uniform, informed, ...): sample() draws configurations from it
 * and returns the first one whose tool pose satisfies the constraints of a PoseConstraintsEvaluator.
 * The samples are therefore uniformly distributed over the intersection of the wrapped sampler's
 * set and the constrained set.
 *
 * If no feasible configuration is found within @c max_attempts draws, sample() returns the last
 * (infeasible) draw and lastSampleFeasible() returns false: the caller decides what to do.
 *
 * setCost(), collapse(), getLB() and getUB() are forwarded to the wrapped sampler. inBounds() also
 * checks the constraints (graph::core::TubeInformedSampler relies on it to keep its tube samples
 * feasible). getSpecificVolume() is the wrapped sampler's value scaled by the measured acceptance
 * rate, an estimate of the volume of the constrained set.
 *
 * clone() shares the evaluator with the original: if clones are used by parallel threads, the
 * forward kinematics of the IK solver must be thread-safe.
 */
class PoseConstrainedSampler : public graph::core::SamplerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor.
   * @param inner Wrapped sampler, already initialized.
   * @param evaluator Constraint evaluator.
   * @param max_attempts Maximum number of draws per call to sample() (at least 1).
   * @param logger graph_core logger.
   * @throw std::invalid_argument if @p inner or @p evaluator is null.
   */
  PoseConstrainedSampler(const graph::core::SamplerPtr& inner,
                         const PoseConstraintsEvaluatorPtr& evaluator,
                         const unsigned int& max_attempts,
                         const graph::core::LoggerPtr& logger);

  /**
   * @brief Draw a configuration that satisfies the constraints (see the class description).
   */
  Eigen::VectorXd sample() override;

  /** Forwarded to the wrapped sampler. */
  void setCost(const double& cost) override;

  /** Forwarded to the wrapped sampler. */
  bool collapse() override;

  /**
   * @brief True if @p q is within the wrapped sampler's bounds and satisfies the constraints.
   */
  bool inBounds(const Eigen::VectorXd& q) override;

  /** Lower bound of the wrapped sampler. */
  const Eigen::VectorXd& getLB() override;

  /** Upper bound of the wrapped sampler. */
  const Eigen::VectorXd& getUB() override;

  /**
   * @brief Copy with a clone of the wrapped sampler and reset statistics.
   */
  graph::core::SamplerPtr clone() override;

  /** True if the last value returned by sample() satisfies the constraints. */
  bool lastSampleFeasible() const { return last_sample_feasible_; }

  /** Set the maximum number of draws per call to sample() (at least 1). */
  void setMaxAttempts(const unsigned int& max_attempts) { max_attempts_ = std::max(1u, max_attempts); }

  /** Maximum number of draws per call to sample(). */
  unsigned int getMaxAttempts() const { return max_attempts_; }

  /** Total number of draws since construction or the last resetStatistics(). */
  unsigned long getAttempts() const { return attempts_; }

  /** Number of rejected draws since construction or the last resetStatistics(). */
  unsigned long getRejections() const { return rejections_; }

  /** Number of sample() calls that returned an infeasible configuration. */
  unsigned long getFailedSamples() const { return failed_samples_; }

  /** Time spent on rejected draws [s] since construction or the last resetStatistics(). */
  double getRejectTime() const { return reject_time_; }

  /** Time spent in sample() [s] since construction or the last resetStatistics(). */
  double getSamplingTime() const { return sampling_time_; }

  /** Fraction of accepted draws (1 if nothing has been drawn yet). */
  double getAcceptanceRate() const;

  /** Reset attempts, rejections, failed samples and times. */
  void resetStatistics();

  /** The wrapped sampler. */
  const graph::core::SamplerPtr& getInnerSampler() const { return inner_; }

  /** The constraint evaluator. */
  const PoseConstraintsEvaluatorPtr& getEvaluator() const { return evaluator_; }

protected:
  /** Update specific_volume_ from the wrapped sampler and the acceptance rate. */
  void updateSpecificVolume();

  graph::core::SamplerPtr inner_;  ///< Wrapped sampler.
  PoseConstraintsEvaluatorPtr evaluator_;  ///< Constraint evaluator.
  unsigned int max_attempts_;  ///< Maximum draws per sample().

  bool last_sample_feasible_ = false;  ///< Feasibility of the last returned sample.
  unsigned long attempts_ = 0;  ///< Total draws.
  unsigned long rejections_ = 0;  ///< Rejected draws.
  unsigned long failed_samples_ = 0;  ///< sample() calls without a feasible draw.
  double reject_time_ = 0.0;  ///< Time spent on rejected draws [s].
  double sampling_time_ = 0.0;  ///< Time spent in sample() [s].
};

}  // namespace pose_constraints_planner
