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

#include <pose_constraints_planner/pose_constraints_path_local_optimizer.h>

namespace pose_constraints_planner
{

PoseConstrainedPathLocalOptimizer::PoseConstrainedPathLocalOptimizer(const CollisionCheckerPtr& checker, const MetricsPtr& metrics,
            const cnr_logger::TraceLoggerPtr& logger,
            const std::shared_ptr<ik_solver::IkSolver> ik_solver,
            const Eigen::Affine3d& T_w_b,
            const Eigen::Affine3d& T_f_t,
            const Eigen::Affine3d& T_w_start,
            const pose_constraints_planner::PoseConstraintsManager::Ptr& pose_constraints_manager
                                       )
  : PathLocalOptimizer(checker, metrics, logger), ik_solver_(ik_solver), T_w_b_(T_w_b), T_f_t_(T_f_t), T_w_start_(T_w_start)
{
  if (!pose_constraints_manager)
  {
    CNR_ERROR(logger, "PoseConstraintsManager pointer is null.");
    throw std::invalid_argument("PoseConstraintsManager pointer is null.");
  }
  if (!ik_solver)
  {
    CNR_ERROR(logger, "IkSolver pointer is null.");
    throw std::invalid_argument("IkSolver pointer is null.");
  }
  pose_constraints_manager_ = pose_constraints_manager;
}

void PoseConstrainedPathLocalOptimizer::config(const std::string& param_ns)
{
  PathOptimizerBase::config(param_ns);
  get_param(logger_, param_ns_, "simplify_max_conn_length", simplify_max_conn_length_, 0.1);
  get_param(logger_, param_ns_, "warp_min_conn_length", warp_min_conn_length_, 0.01);
  get_param(logger_, param_ns_, "warp_min_step_size", warp_min_step_size_, 0.01);
}

bool PoseConstrainedPathLocalOptimizer::bisection(const size_t& connection_idx, const Eigen::VectorXd& center,
                                   const Eigen::VectorXd& direction, const double min_step_size, double max_distance,
                                   double min_distance)
{
  assert(connection_idx < path_->getConnectionsSize());
  assert(connection_idx > 0);

  std::vector<ConnectionPtr> connections = path_->getConnections();
  ConnectionPtr& conn12 = connections.at(connection_idx - 1);  // ref to connection
  ConnectionPtr& conn23 = connections.at(connection_idx);      // ref to connection

  NodePtr parent = conn12->getParent();
  NodePtr child = conn23->getChild();

  bool improved = false;
  double cost = conn12->getCost() + conn23->getCost();

  unsigned int iter = 0;
  double distance;

  while ((iter++ < 5) && ((max_distance - min_distance) > min_step_size))
  {
    if (iter > 0)
      distance = 0.5 * (max_distance + min_distance);
    else
      distance = min_distance;

    Eigen::VectorXd p = center + direction * distance;
    assert(p.size() == center.size());
    double cost_pn = metrics_->cost(parent->getConfiguration(), p);
    double cost_nc = metrics_->cost(p, child->getConfiguration());
    double cost_n = cost_pn + cost_nc;

    if (cost_n >= cost)
    {
      min_distance = distance;
      continue;
    }
    bool is_valid = checker_->checkConnection(parent->getConfiguration(), p) &&
                    checker_->checkConnection(p, child->getConfiguration());

    bool satisfies_pose_constraints = checkConnectionSatisfiesPoseConstraints(parent->getConfiguration(), p) &&
                                      checkConnectionSatisfiesPoseConstraints(p, child->getConfiguration());


    if (not is_valid || not satisfies_pose_constraints)
    {
      min_distance = distance;
      continue;
    }

    improved = true;
    max_distance = distance;
    cost = cost_n;
    // conn12->remove(); // Do not remove, other nodes could be linked to
    // conn12's child

    bool is_net = conn23->isNet();
    conn23->remove();

    NodePtr n = std::make_shared<Node>(p, logger_);
    conn12 = std::make_shared<Connection>(parent, n, logger_);

    is_net ? (conn23 = std::make_shared<Connection>(n, child, logger_, true)) :
             (conn23 = std::make_shared<Connection>(n, child, logger_, false));

    conn12->setCost(cost_pn);
    conn23->setCost(cost_nc);
    conn12->add();
    conn23->add();

    assert(child->getParentConnectionsSize() == 1);
    assert(conn23->getChild()->getParentConnectionsSize() == 1);

    if (path_->getTree())
      path_->getTree()->addNode(n, false);
  }

  if (improved)
    path_->setConnections(connections);  // updates cost too

  return improved;
}

bool PoseConstrainedPathLocalOptimizer::checkConnectionSatisfiesPoseConstraints(const NodePtr& n1, const NodePtr& n2)
{
  if (!n1)
  {
    CNR_ERROR(logger_, "NodePtr n1 is null.");
    throw std::invalid_argument("NodePtr n1 is null.");
  }
  if (!n2)
  {
    CNR_ERROR(logger_, "NodePtr n2 is null.");
    throw std::invalid_argument("NodePtr n2 is null.");
  }
  return checkConnectionSatisfiesPoseConstraints(n1->getConfiguration(), n2->getConfiguration());
}

bool PoseConstrainedPathLocalOptimizer::checkConnectionSatisfiesPoseConstraints(const Eigen::VectorXd& parent_config, const Eigen::VectorXd& child_config)
{
  std::stringstream report;

  Eigen::Affine3d T_w_parent = ik_solver_->computeFk(parent_config,
                                      T_w_b_,
                                      T_f_t_);
  Eigen::Affine3d T_w_child = ik_solver_->computeFk(child_config,
                                     T_w_b_,
                                     T_f_t_);

  if (!pose_constraints_manager_->checkConstraints(T_w_parent,T_w_start_,&report) || !pose_constraints_manager_->checkConstraints(T_w_child,T_w_start_,&report))
  {
    return false;
  }

  // check intermediate configurations along the connection with distance = warp_min_step_size_
  double check_distance = std::max(0.001, warp_min_step_size_);  // Ensure a minimum check distance

  double connection_length = (parent_config - child_config).norm();
  if (connection_length > check_distance)
  {
    int num_checks = static_cast<int>(std::ceil(connection_length / check_distance));
    for (int i = 1; i < num_checks; ++i)
    {
      double alpha = static_cast<double>(i) / num_checks;
      Eigen::VectorXd intermediate_config = (1 - alpha) * parent_config + alpha * child_config;
      Eigen::Affine3d T_w_t = ik_solver_->computeFk(intermediate_config,
                                                    T_w_b_,
                                                    T_f_t_);
      if (!pose_constraints_manager_->checkConstraints(T_w_t,T_w_start_,&report))
        return false;
    }
  }
  return true;
}

bool PoseConstrainedPathLocalOptimizer::checkConnectionSatisfiesPoseConstraints(const ConnectionPtr& connection)
{
  if (!connection)
  {
    CNR_ERROR(logger_, "ConnectionPtr is null.");
    throw std::invalid_argument("ConnectionPtr is null.");
  }
  return checkConnectionSatisfiesPoseConstraints(connection->getParent(), connection->getChild());
}

bool PoseConstrainedPathLocalOptimizer::simplify(const double& min_conn_length)
{
  bool simplified = false;
  bool reconnect_first_conn = false;

  std::vector<ConnectionPtr> connections = path_->getConnections();

  if (connections.size() > 1)
  {
    if (connections.front()->norm() < min_conn_length)
      reconnect_first_conn = true;
  }

  unsigned int ic = 1;
  while (ic < connections.size())
  {
    if (connections.at(ic)->norm() > min_conn_length)  // connection longer than the threshold, skip
    {
      /* If the connection at pos 1 is longer than the threshold but the first
       * connection was shorter than the threshold, simplify the conneection.*/
      if (not(ic == 1 && reconnect_first_conn))
      {
        ic++;
        continue;
      }
    }

    /* If connection from previous parent and current child is possible connect
     * them and remove the middle node (current parent)*/

    bool satisfies_pose_constraints = checkConnectionSatisfiesPoseConstraints(connections.at(ic - 1)->getParent(),
                                  connections.at(ic)->getChild());

    bool is_valid = checker_->checkConnection(connections.at(ic - 1)->getParent()->getConfiguration(),
                                  connections.at(ic)->getChild()->getConfiguration());
    if (is_valid && satisfies_pose_constraints)
    {
      simplified = true;
      double cost = metrics_->cost(connections.at(ic - 1)->getParent(), connections.at(ic)->getChild());

      ConnectionPtr conn = std::make_shared<Connection>(
          connections.at(ic - 1)->getParent(), connections.at(ic)->getChild(), logger_, connections.at(ic)->isNet());
      conn->setCost(cost);

      // Check pose constraints

      conn->add();

      connections.at(ic)->remove();
      assert(conn->getChild()->getParentConnectionsSize() == 1);

      connections.erase(connections.begin() + (ic - 1), connections.begin() + ic + 1);
      connections.insert(connections.begin() + (ic - 1), conn);

      change_warp_.erase(change_warp_.begin() + ic);
      if (ic > 1)
        change_warp_.at(ic - 1) = true;
    }
    else
      ic++;
  }

  path_->setConnections(connections);

  return simplified;
}

}  // end namespace pose_constraints_planner

