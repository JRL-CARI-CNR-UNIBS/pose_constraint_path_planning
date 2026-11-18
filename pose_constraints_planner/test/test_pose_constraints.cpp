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

// Unit tests of the constraint checks, the evaluator, the constrained sampler, the constrained
// collision checker and the constrained MI-RRT* solver. They use a fake 4-joint robot (no ROS graph,
// no URDF):
//   joints (jx, jy, jz, jyaw), tool position (jx, jy, jz + jx^2), tool orientation Rz(jyaw).
// The jx^2 term makes the forward kinematics nonlinear, so a straight joint-space edge between two
// feasible configurations can leave the feasible set.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <graph_core/graph/node.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/metrics/goal_cost_function_base.h>
#include <graph_core/samplers/uniform_sampler.h>

#include <pose_constraints_planner/pose_constrained_collision_checker.hpp>
#include <pose_constraints_planner/pose_constrained_multigoal_solver.hpp>
#include <pose_constraints_planner/pose_constrained_sampler.hpp>
#include <pose_constraints_planner/pose_constraints_evaluator.hpp>
#include <pose_constraints_planner/pose_constraints_manager.hpp>

using pose_constraints_planner::GeometricConstraint;
using pose_constraints_planner::PoseConstraintsManager;

namespace
{

const std::vector<std::string> kIkJoints = { "jx", "jy", "jz", "jyaw" };

/** Fake kinematics (see the file comment). */
class FakeIkSolver : public ik_solver::IkSolver
{
public:
  FakeIkSolver()
  {
    joint_names_ = kIkJoints;
    base_frame_ = "base";
    flange_frame_ = "flange";
    tool_frame_ = "flange";
  }
  ik_solver::Solutions getIk(const Eigen::Affine3d&, const ik_solver::Configurations&, const int& = -1,
                             const int& = -1, const int& = -1) override
  {
    return ik_solver::Solutions();
  }
  Eigen::Affine3d getFK(const ik_solver::Configuration& q) override
  {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation() << q(0), q(1), q(2) + q(0) * q(0);
    T.linear() = Eigen::AngleAxisd(q(3), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return T;
  }
  bool getTF(const std::string&, const std::string&, Eigen::Affine3d& T) const override
  {
    T.setIdentity();
    return true;
  }
};

/** Collision checker that accepts everything. */
class FreeChecker : public graph::core::CollisionCheckerBase
{
public:
  explicit FreeChecker(const graph::core::LoggerPtr& logger) : CollisionCheckerBase(logger, 0.01) {}
  bool check(const Eigen::VectorXd&) override { return true; }
  graph::core::CollisionCheckerPtr clone() override { return std::make_shared<FreeChecker>(logger_); }
};

graph::core::LoggerPtr logger()
{
  static auto l = std::make_shared<graph::core::ConsoleLogger>("test", graph::core::LogLevel::Error);
  return l;
}

GeometricConstraint plane(const Eigen::Vector3d& origin, const Eigen::Vector3d& normal, double tol = 0.0)
{
  GeometricConstraint c;
  c.name = "plane";
  c.type = GeometricConstraint::PLANE;
  c.plane_origin = origin;
  c.plane_normal = normal;
  c.plane_tolerance = tol;
  return c;
}

GeometricConstraint line(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir, double max_distance)
{
  GeometricConstraint c;
  c.name = "line";
  c.type = GeometricConstraint::LINE;
  c.line_origin = origin;
  c.line_dir = dir;
  c.line_max_distance = max_distance;
  return c;
}

GeometricConstraint angle(const Eigen::Vector3d& max_angle)
{
  pose_constraints_msgs::msg::GeometricConstraint msg;
  msg.name = "angle";
  msg.type = pose_constraints_msgs::msg::GeometricConstraint::ANGLE;
  msg.max_angle.x = max_angle.x();
  msg.max_angle.y = max_angle.y();
  msg.max_angle.z = max_angle.z();
  return pose_constraints_planner::convertGeometricConstraint(msg);
}

Eigen::Affine3d pose(double x, double y, double z, double yaw = 0.0)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translation() << x, y, z;
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return T;
}

pose_constraints_planner::PoseConstraintsEvaluatorPtr evaluator(const std::vector<GeometricConstraint>& constraints,
                                                                const Eigen::VectorXd& start,
                                                                const std::vector<std::string>& joints = kIkJoints)
{
  auto ik = std::make_shared<FakeIkSolver>();
  auto manager = std::make_shared<PoseConstraintsManager>(constraints);
  return std::make_shared<pose_constraints_planner::PoseConstraintsEvaluator>(
      ik, manager, joints, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(), start);
}

Eigen::VectorXd q(double x, double y, double z, double yaw = 0.0)
{
  Eigen::VectorXd v(4);
  v << x, y, z, yaw;
  return v;
}

}  // namespace

// ------------------------------------------------------------------ PoseConstraintsManager

TEST(PoseConstraintsManager, Plane)
{
  PoseConstraintsManager m({ plane({ 0, 0, 0.5 }, { 0, 0, 2 }, 0.1) });  // z >= 0.6, normal not unit
  EXPECT_TRUE(m.checkConstraints(pose(0, 0, 0.7), pose(0, 0, 0)));
  EXPECT_FALSE(m.checkConstraints(pose(0, 0, 0.55), pose(0, 0, 0)));
  std::stringstream report;
  EXPECT_FALSE(m.checkConstraints(pose(0, 0, 0.0), pose(0, 0, 0), &report));
  EXPECT_NE(report.str().find("plane"), std::string::npos);
}

TEST(PoseConstraintsManager, Line)
{
  PoseConstraintsManager m({ line({ 0, 0, 1 }, { 1, 0, 0 }, 0.2) });  // around the line y = 0, z = 1
  EXPECT_TRUE(m.checkConstraints(pose(5.0, 0.1, 1.1), pose(0, 0, 0)));
  EXPECT_FALSE(m.checkConstraints(pose(0.0, 0.3, 1.0), pose(0, 0, 0)));
}

TEST(PoseConstraintsManager, AngleAxesAndFreeAxes)
{
  const Eigen::Affine3d start = pose(0, 0, 0, 0.0);
  // A rotation about z changes rows 0 and 1 of the rotation matrix, row 2 is unchanged.
  PoseConstraintsManager limit_x({ angle({ 0.3, M_PI, M_PI }) });
  EXPECT_TRUE(limit_x.checkConstraints(pose(0, 0, 0, 0.2), start));
  EXPECT_FALSE(limit_x.checkConstraints(pose(0, 0, 0, 0.4), start));

  PoseConstraintsManager limit_z({ angle({ M_PI, M_PI, 0.3 }) });
  EXPECT_TRUE(limit_z.checkConstraints(pose(0, 0, 0, 2.0), start));

  // Negative values leave the axes free (as documented in GeometricConstraint.msg)
  PoseConstraintsManager free_axes({ angle({ -1.0, -1.0, -1.0 }) });
  EXPECT_TRUE(free_axes.checkConstraints(pose(0, 0, 0, 2.5), start));
}

TEST(PoseConstraintsManager, ConvertMessage)
{
  pose_constraints_msgs::msg::GeometricConstraintArray array;
  pose_constraints_msgs::msg::GeometricConstraint c;
  c.name = "table";
  c.frame_id = "table_link";
  c.type = pose_constraints_msgs::msg::GeometricConstraint::PLANE;
  c.plane_origin.z = 0.8;
  c.plane_normal.z = 1.0;
  c.plane_tolerance = 0.1;
  array.constraints.push_back(c);

  const auto converted = pose_constraints_planner::convertGeometricConstraintArray(array);
  ASSERT_EQ(converted.size(), 1u);
  EXPECT_EQ(converted[0].name, "table");
  EXPECT_EQ(converted[0].frame_id, "table_link");
  EXPECT_EQ(converted[0].type, GeometricConstraint::PLANE);
  EXPECT_DOUBLE_EQ(converted[0].plane_origin.z(), 0.8);
  EXPECT_DOUBLE_EQ(converted[0].plane_tolerance, 0.1);
}

// ------------------------------------------------------------------ transformConstraint

TEST(TransformConstraint, PlaneAndLine)
{
  // Frame F: translated by (0, 0, 1) and rotated by 90 deg about x (F's z axis is the world -y axis)
  Eigen::Affine3d T_w_f = Eigen::Affine3d::Identity();
  T_w_f.translation() << 0, 0, 1;
  T_w_f.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();

  // z_F >= 0  <=>  -y_w >= 0
  auto p = pose_constraints_planner::transformConstraint(plane({ 0, 0, 0 }, { 0, 0, 1 }), T_w_f);
  PoseConstraintsManager mp({ p });
  EXPECT_TRUE(mp.checkConstraints(pose(0, -0.1, 5), pose(0, 0, 0)));
  EXPECT_FALSE(mp.checkConstraints(pose(0, 0.1, 5), pose(0, 0, 0)));

  // Line along z_F through F's origin = line along -y_w through (0, 0, 1)
  auto l = pose_constraints_planner::transformConstraint(line({ 0, 0, 0 }, { 0, 0, 1 }, 0.1), T_w_f);
  PoseConstraintsManager ml({ l });
  EXPECT_TRUE(ml.checkConstraints(pose(0, 3.0, 1.05), pose(0, 0, 0)));
  EXPECT_FALSE(ml.checkConstraints(pose(0, 3.0, 0.8), pose(0, 0, 0)));
}

TEST(TransformConstraint, AngleInAnotherFrame)
{
  const Eigen::Affine3d start = pose(0, 0, 0, 0.0);
  // Limit on axis z: in the world frame a yaw does not move the z axis...
  PoseConstraintsManager world({ angle({ M_PI, M_PI, 0.3 }) });
  EXPECT_TRUE(world.checkConstraints(pose(0, 0, 0, 0.4), start));

  // ...but in a frame whose z axis is horizontal (rotated 90 deg about x) the same yaw violates it.
  Eigen::Affine3d T_w_f = Eigen::Affine3d::Identity();
  T_w_f.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  PoseConstraintsManager in_f({ pose_constraints_planner::transformConstraint(angle({ M_PI, M_PI, 0.3 }), T_w_f) });
  EXPECT_TRUE(in_f.checkConstraints(pose(0, 0, 0, 0.2), start));
  EXPECT_FALSE(in_f.checkConstraints(pose(0, 0, 0, 0.4), start));
}

// ------------------------------------------------------------------ PoseConstraintsEvaluator

TEST(PoseConstraintsEvaluator, JointOrderPermutation)
{
  // Planner order differs from the IK order
  const std::vector<std::string> planner_joints = { "jyaw", "jz", "jy", "jx" };
  auto e = evaluator({ plane({ 0, 0, 0.5 }, { 0, 0, 1 }) }, q(0.0, 0.0, 1.0, 0.0), planner_joints);
  Eigen::VectorXd planner_q(4);
  planner_q << 0.3, 0.2, 0.1, 0.5;  // yaw, z, y, x
  const Eigen::Affine3d T = e->toolPose(planner_q);
  EXPECT_NEAR(T.translation().x(), 0.5, 1e-12);
  EXPECT_NEAR(T.translation().y(), 0.1, 1e-12);
  EXPECT_NEAR(T.translation().z(), 0.2 + 0.25, 1e-12);
  EXPECT_FALSE(e->satisfies(planner_q));  // z = 0.45 < 0.5
}

TEST(PoseConstraintsEvaluator, MissingJointThrows)
{
  EXPECT_THROW(evaluator({}, q(0, 0, 0), { "jx", "jy", "jz" }), std::invalid_argument);
}

TEST(PoseConstraintsEvaluator, AngleRelativeToStart)
{
  auto e = evaluator({ angle({ 0.3, M_PI, M_PI }) }, q(0, 0, 0, 1.0));  // start yaw = 1.0
  EXPECT_TRUE(e->satisfies(q(0, 0, 0, 1.2)));
  EXPECT_FALSE(e->satisfies(q(0, 0, 0, 0.0)));
}

// ------------------------------------------------------------------ PoseConstrainedSampler

TEST(PoseConstrainedSampler, SamplesAreFeasible)
{
  auto e = evaluator({ plane({ 0, 0, 0.5 }, { 0, 0, 1 }) }, q(0, 0, 1));
  auto inner = std::make_shared<graph::core::UniformSampler>(q(-1, -1, -1, -M_PI), q(1, 1, 1, M_PI), logger());
  pose_constraints_planner::PoseConstrainedSampler sampler(inner, e, 1000, logger());

  for (int i = 0; i < 500; i++)
  {
    const Eigen::VectorXd s = sampler.sample();
    ASSERT_TRUE(sampler.lastSampleFeasible());
    ASSERT_TRUE(e->satisfies(s));
    ASSERT_TRUE(inner->inBounds(s));
  }
  EXPECT_EQ(sampler.getAttempts() - sampler.getRejections(), 500u);
  EXPECT_EQ(sampler.getFailedSamples(), 0u);
  EXPECT_GT(sampler.getRejections(), 0u);
  EXPECT_GT(sampler.getAcceptanceRate(), 0.0);
  EXPECT_LT(sampler.getAcceptanceRate(), 1.0);
  // The specific volume is scaled by the acceptance rate
  EXPECT_NEAR(sampler.getSpecificVolume(), inner->getSpecificVolume() * sampler.getAcceptanceRate(), 1e-9);

  EXPECT_TRUE(sampler.inBounds(q(0, 0, 0.8)));
  EXPECT_FALSE(sampler.inBounds(q(0, 0, 0.2)));   // violates the plane
  EXPECT_FALSE(sampler.inBounds(q(0, 0, 1.5)));   // out of the joint bounds

  sampler.resetStatistics();
  EXPECT_EQ(sampler.getAttempts(), 0u);
}

TEST(PoseConstrainedSampler, ExhaustedAttempts)
{
  auto e = evaluator({ plane({ 0, 0, 10.0 }, { 0, 0, 1 }) }, q(0, 0, 1));  // unreachable
  auto inner = std::make_shared<graph::core::UniformSampler>(q(-1, -1, -1, -M_PI), q(1, 1, 1, M_PI), logger());
  pose_constraints_planner::PoseConstrainedSampler sampler(inner, e, 20, logger());
  sampler.sample();
  EXPECT_FALSE(sampler.lastSampleFeasible());
  EXPECT_EQ(sampler.getAttempts(), 20u);
  EXPECT_EQ(sampler.getRejections(), 20u);
  EXPECT_EQ(sampler.getFailedSamples(), 1u);
  EXPECT_GT(sampler.getSpecificVolume(), 0.0);  // never zero
}

TEST(PoseConstrainedSampler, NullArgumentsThrow)
{
  auto inner = std::make_shared<graph::core::UniformSampler>(q(-1, -1, -1, -1), q(1, 1, 1, 1), logger());
  EXPECT_THROW(pose_constraints_planner::PoseConstrainedSampler(inner, nullptr, 10, logger()), std::invalid_argument);
}

// ------------------------------------------------------------------ PoseConstrainedCollisionChecker

TEST(PoseConstrainedCollisionChecker, EdgeLeavingTheFeasibleSet)
{
  // z_tool = jz + jx^2 >= 0.5: (-1, 0, 0) and (1, 0, 0) are feasible (z = 1), the midpoint is not (z = 0).
  auto e = evaluator({ plane({ 0, 0, 0.5 }, { 0, 0, 1 }) }, q(-1, 0, 0));
  pose_constraints_planner::PoseConstrainedCollisionChecker checker(std::make_shared<FreeChecker>(logger()), e, 0.01,
                                                                    logger());
  EXPECT_TRUE(checker.check(q(-1, 0, 0)));
  EXPECT_TRUE(checker.check(q(1, 0, 0)));
  EXPECT_FALSE(checker.check(q(0, 0, 0)));

  EXPECT_FALSE(checker.checkConnection(q(-1, 0, 0), q(1, 0, 0)));
  Eigen::VectorXd last_valid;
  EXPECT_FALSE(checker.checkConnection(q(-1, 0, 0), q(1, 0, 0), last_valid));
  EXPECT_TRUE(e->satisfies(last_valid));
  EXPECT_LT(last_valid(0), -0.6);  // stops before the violation (jx^2 >= 0.5 for jx <= -0.707)

  // Raising jz keeps the edge feasible
  EXPECT_TRUE(checker.checkConnection(q(-1, 0, 0.6), q(1, 0, 0.6)));
  EXPECT_GE(checker.getConstraintRejections(), 3u);

  auto clone = checker.clone();
  EXPECT_FALSE(clone->checkConnection(q(-1, 0, 0), q(1, 0, 0)));
}

// ------------------------------------------------------------------ PoseConstrainedMultigoalSolver

TEST(PoseConstrainedMultigoalSolver, PathSatisfiesTheConstraints)
{
  auto e = evaluator({ plane({ 0, 0, 0.5 }, { 0, 0, 1 }) }, q(-1, 0, 0));
  auto checker = std::make_shared<pose_constraints_planner::PoseConstrainedCollisionChecker>(
      std::make_shared<FreeChecker>(logger()), e, 0.01, logger());
  auto metrics = std::make_shared<graph::core::EuclideanMetrics>(logger());
  auto sampler = std::make_shared<graph::core::UniformSampler>(q(-1.2, -1, -1, -1), q(1.2, 1, 1, 1), logger());
  auto goal_cost = std::make_shared<graph::core::GoalCostFunctionBase>();

  pose_constraints_planner::PoseConstrainedMultigoalSolver solver(metrics, checker, sampler, goal_cost, e, 1000,
                                                                  logger());
  graph::core::ParamMap params;
  params["max_distance"] = 0.3;
  params["extend"] = true;
  ASSERT_TRUE(solver.config(params));
  ASSERT_TRUE(solver.addStart(std::make_shared<graph::core::Node>(q(-1, 0, 0), logger())));
  // Two alternative goals: the direct edge to each one crosses the infeasible region
  solver.addGoal(std::make_shared<graph::core::Node>(q(1, 0, 0), logger()), 0.1, 0.0);
  solver.addGoal(std::make_shared<graph::core::Node>(q(1, 0.5, 0), logger()), 0.1, 0.0);
  solver.finalizeProblem();

  graph::core::PathPtr solution;
  for (int i = 0; i < 20000 && !solver.solved(); i++)
    solver.update(solution);
  ASSERT_TRUE(solver.solved());
  solution = solver.getSolution();
  ASSERT_TRUE(solution);

  const auto waypoints = solution->getWaypoints();
  ASSERT_GE(waypoints.size(), 3u);  // a detour is needed
  for (size_t i = 0; i + 1 < waypoints.size(); i++)
    EXPECT_TRUE(checker->constraintsHoldAlong(waypoints[i], waypoints[i + 1])) << "segment " << i;

  const auto stats = solver.samplingStatistics();
  EXPECT_GT(stats.attempts, 0u);
  EXPECT_GE(stats.rejectionRate(), 0.0);
  EXPECT_LE(stats.rejectionRate(), 1.0);
}
