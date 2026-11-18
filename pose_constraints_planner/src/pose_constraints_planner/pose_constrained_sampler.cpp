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

#include <pose_constraints_planner/pose_constrained_sampler.hpp>

#include <chrono>
#include <stdexcept>

namespace pose_constraints_planner
{

PoseConstrainedSampler::PoseConstrainedSampler(const graph::core::SamplerPtr& inner,
                                               const PoseConstraintsEvaluatorPtr& evaluator,
                                               const unsigned int& max_attempts,
                                               const graph::core::LoggerPtr& logger)
  : SamplerBase(inner ? inner->getLB() : Eigen::VectorXd(),
                inner ? inner->getUB() : Eigen::VectorXd(),
                logger,
                inner ? inner->getCost() : std::numeric_limits<double>::infinity()),
    inner_(inner),
    evaluator_(evaluator),
    max_attempts_(std::max(1u, max_attempts))
{
  if (!inner_ || !evaluator_)
    throw std::invalid_argument("PoseConstrainedSampler: inner sampler and evaluator must not be null");
  updateSpecificVolume();
}

Eigen::VectorXd PoseConstrainedSampler::sample()
{
  const auto t_start = std::chrono::steady_clock::now();
  Eigen::VectorXd q;
  last_sample_feasible_ = false;
  for (unsigned int i = 0; i < max_attempts_; i++)
  {
    auto t0 = std::chrono::steady_clock::now();
    q = inner_->sample();
    attempts_++;
    if (evaluator_->satisfies(q))
    {
      last_sample_feasible_ = true;
      break;
    }
    rejections_++;
    reject_time_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }

  if (!last_sample_feasible_)
  {
    failed_samples_++;
    GC_DEBUG(logger_, "PoseConstrainedSampler: no feasible sample in " << max_attempts_ << " attempts");
  }

  updateSpecificVolume();
  sampling_time_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  return q;
}

void PoseConstrainedSampler::setCost(const double& cost)
{
  inner_->setCost(cost);
  cost_ = inner_->getCost();
  updateSpecificVolume();
}

bool PoseConstrainedSampler::collapse()
{
  return inner_->collapse();
}

bool PoseConstrainedSampler::inBounds(const Eigen::VectorXd& q)
{
  return inner_->inBounds(q) && evaluator_->satisfies(q);
}

const Eigen::VectorXd& PoseConstrainedSampler::getLB()
{
  return inner_->getLB();
}

const Eigen::VectorXd& PoseConstrainedSampler::getUB()
{
  return inner_->getUB();
}

graph::core::SamplerPtr PoseConstrainedSampler::clone()
{
  return std::make_shared<PoseConstrainedSampler>(inner_->clone(), evaluator_, max_attempts_, logger_);
}

double PoseConstrainedSampler::getAcceptanceRate() const
{
  if (attempts_ == 0)
    return 1.0;
  return static_cast<double>(attempts_ - rejections_) / static_cast<double>(attempts_);
}

void PoseConstrainedSampler::resetStatistics()
{
  attempts_ = 0;
  rejections_ = 0;
  failed_samples_ = 0;
  reject_time_ = 0.0;
  sampling_time_ = 0.0;
  updateSpecificVolume();
}

void PoseConstrainedSampler::updateSpecificVolume()
{
  // Avoid a zero volume (it would make the RRT* rewiring radius zero) before any acceptance.
  double rate = std::max(getAcceptanceRate(), 1.0 / static_cast<double>(attempts_ + 1));
  specific_volume_ = inner_->getSpecificVolume() * rate;
}

}  // namespace pose_constraints_planner
