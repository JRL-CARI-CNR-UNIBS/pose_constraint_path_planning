#pragma once
/*
Copyright (c) 2024, Manuel Beschi and Cesare Tonola, JRL-CARI CNR-STIIMA/UNIBS,
manuel.beschi@unibs.it, c.tonola001@unibs.it All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <graph_core/solvers/path_optimizers/path_optimizer_base.h>
#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>
#include <ik_solver/ik_solver.hpp>
#include <pose_constraints_planner/pose_constraints_manager.hpp>

using namespace graph::core;
namespace pose_constraints_planner
{
  class PoseConstrainedPathLocalOptimizer;
  typedef std::shared_ptr<PoseConstrainedPathLocalOptimizer> PoseConstrainedPathLocalOptimizerPtr;

  /**
   * @class PoseConstrainedPathLocalOptimizer
   * @brief Derived class for local path optimization using warping and
   * simplification techniques.
   *
   * This class inherits from PathLocalOptimizer and provides
   */
  class PoseConstrainedPathLocalOptimizer : public PathLocalOptimizer
  {
  protected:
    bool bisection(const size_t& connection_idx, const Eigen::VectorXd& center, const Eigen::VectorXd& direction,
                   const double min_step_size, double max_distance, double min_distance);

    /* @brief Check if a connection satisfies pose constraints.
     *
     * This function checks if the given connection satisfies the pose constraints
     * defined in the PoseConstraintsManager. It evaluates the connection's
     * configurations against the specified constraints and returns true if all
     * constraints are satisfied, false otherwise.
     *
     * @param connection The connection to be checked against pose constraints.
     * @return True if the connection satisfies all pose constraints, false otherwise.
     */
    bool checkConnectionSatisfiesPoseConstraints(const ConnectionPtr& connection);

    /* @brief Check if the connection between two nodes satisfies pose constraints.
     *
     * This function checks if the connection between two nodes satisfies the pose
     * constraints defined in the PoseConstraintsManager. It evaluates the
     * configurations of both nodes against the specified constraints and returns
     * true if all constraints are satisfied, false otherwise.
     *
     * @param n1 The first node of the connection.
     * @param n2 The second node of the connection.
     * @return True if the connection between the two nodes satisfies all pose
     * constraints, false otherwise.
     */
    bool checkConnectionSatisfiesPoseConstraints(const NodePtr& n1, const NodePtr& n2);

    /* @brief Check if the connection between two configurations satisfies pose constraints.
     *
     * This function checks if the connection between two configurations satisfies
     * the pose constraints defined in the PoseConstraintsManager. It evaluates
     * both configurations against the specified constraints and returns true if
     * all constraints are satisfied, false otherwise.
     *
     * @param parent_config The configuration of the parent node.
     * @param child_config The configuration of the child node.
     * @return True if the connection between the two configurations satisfies all
     * pose constraints, false otherwise.
     */
    bool checkConnectionSatisfiesPoseConstraints(const Eigen::VectorXd& parent_config, const Eigen::VectorXd& child_config);

    pose_constraints_planner::PoseConstraintsManager::Ptr pose_constraints_manager_;

    std::shared_ptr<ik_solver::IkSolver> ik_solver_;
    Eigen::Affine3d T_w_b_;
    Eigen::Affine3d T_f_t_;
    Eigen::Affine3d T_w_start_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseConstrainedPathLocalOptimizer(const CollisionCheckerPtr& checker, const MetricsPtr& metrics,
                       const cnr_logger::TraceLoggerPtr& logger,
                       const std::shared_ptr<ik_solver::IkSolver> ik_solver,
                        const Eigen::Affine3d& T_w_b,
                        const Eigen::Affine3d& T_f_t,
                        const Eigen::Affine3d& T_w_start,
                       const pose_constraints_planner::PoseConstraintsManager::Ptr& pose_constraints_manager);


    using Ptr = std::shared_ptr<PoseConstrainedPathLocalOptimizer>;

    virtual void config(const std::string& param_ns) override;


    /**
     * @brief Simplify the path by skipping nodes.
     *
     * This method simplifies the path by removing connections shorter than a
     * threshold. If the length of a connection is shorter than 'min_conn_length',
     * the algorithm skips the connection's child by connecting the parent of the
     * previous connection with the current child, if it results in a collision
     * free connection.
     *
     * @param min_conn_length The minimum connection's length threshold for node
     * removal.
     * @return True if the path is simplified, false otherwise.
     */
    virtual bool simplify(const double& min_conn_length = 0.1);

  };

} // namespace pose_constraints_planner