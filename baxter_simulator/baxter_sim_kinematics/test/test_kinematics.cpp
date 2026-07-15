#include <algorithm>
#include <cstring>
#include <memory>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <baxter_core_msgs/SEAJointState.h>
#include <gazebo_msgs/GetLinkProperties.h>
#include <gazebo_msgs/SetLinkProperties.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#define private public
#include <baxter_sim_kinematics/arm_kinematics.h>
#undef private

namespace
{
const char kOneJointUrdf[] =
    "<robot name='one_joint'>"
    "  <link name='base'/>"
    "  <link name='tip'/>"
    "  <joint name='joint_1' type='revolute'>"
    "    <parent link='base'/>"
    "    <child link='tip'/>"
    "    <origin xyz='1 0 0' rpy='0 0 0'/>"
    "    <axis xyz='0 0 1'/>"
    "    <limit lower='-1.0' upper='1.0' effort='10' velocity='10'/>"
    "  </joint>"
    "</robot>";

const char kMissingLimitsUrdf[] =
    "<robot name='missing_limits'>"
    "  <link name='base'/>"
    "  <link name='tip'/>"
    "  <joint name='joint_1' type='revolute'>"
    "    <parent link='base'/>"
    "    <child link='tip'/>"
    "    <axis xyz='0 0 1'/>"
    "  </joint>"
    "</robot>";

void InitOneJointModel(arm_kinematics::Kinematics& kin)
{
  kin.root_name = "base";
  kin.tip_name = "tip";
  ASSERT_TRUE(kin.loadModel(kOneJointUrdf));
  kin.fk_solver.reset(new KDL::ChainFkSolverPos_recursive(kin.chain));
  kin.ik_solver_vel.reset(new KDL::ChainIkSolverVel_pinv(kin.chain));
  kin.ik_solver_pos.reset(new KDL::ChainIkSolverPos_NR_JL(kin.chain, kin.joint_min, kin.joint_max, *kin.fk_solver,
                                                          *kin.ik_solver_vel, 1000, 1e-6));
}

sensor_msgs::JointState OneJointState(double position)
{
  sensor_msgs::JointState state;
  state.name.push_back("joint_1");
  state.position.push_back(position);
  return state;
}

geometry_msgs::PoseStamped OneJointZeroPose()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base";
  pose.pose.position.x = 1.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}
}  // namespace

TEST(KinematicsTest, LoadsJointLimitsAndRejectsMissingLimits)
{
  arm_kinematics::Kinematics kin;
  kin.root_name = "base";
  kin.tip_name = "tip";

  EXPECT_TRUE(kin.loadModel(kOneJointUrdf));
  ASSERT_EQ(1u, kin.info.joint_names.size());
  EXPECT_EQ("joint_1", kin.info.joint_names[0]);
  EXPECT_DOUBLE_EQ(-1.0, kin.joint_min(0));
  EXPECT_DOUBLE_EQ(1.0, kin.joint_max(0));

  arm_kinematics::Kinematics bad_kin;
  bad_kin.root_name = "base";
  bad_kin.tip_name = "tip";
  EXPECT_FALSE(bad_kin.loadModel(kMissingLimitsUrdf));
}

TEST(KinematicsTest, ComputesKnownFkAndIk)
{
  arm_kinematics::Kinematics kin;
  InitOneJointModel(kin);

  geometry_msgs::PoseStamped fk_result;
  ASSERT_TRUE(kin.getPositionFK("base", OneJointState(0.0), fk_result));
  EXPECT_EQ("base", fk_result.header.frame_id);
  EXPECT_NEAR(1.0, fk_result.pose.position.x, 1e-9);
  EXPECT_NEAR(0.0, fk_result.pose.position.y, 1e-9);
  EXPECT_NEAR(0.0, fk_result.pose.position.z, 1e-9);
  EXPECT_NEAR(1.0, fk_result.pose.orientation.w, 1e-9);

  sensor_msgs::JointState ik_result;
  ASSERT_TRUE(kin.getPositionIK(OneJointZeroPose(), OneJointState(0.0), &ik_result));
  ASSERT_EQ(1u, ik_result.name.size());
  ASSERT_EQ(1u, ik_result.position.size());
  EXPECT_EQ("joint_1", ik_result.name[0]);
  EXPECT_NEAR(0.0, ik_result.position[0], 1e-6);
}

TEST(KinematicsTest, RejectsIncompleteFkAndIkRequests)
{
  arm_kinematics::Kinematics kin;
  InitOneJointModel(kin);

  sensor_msgs::JointState missing_position;
  missing_position.name.push_back("joint_1");
  geometry_msgs::PoseStamped fk_result;
  EXPECT_FALSE(kin.getPositionFK("base", missing_position, fk_result));

  sensor_msgs::JointState missing_joint;
  missing_joint.name.push_back("other_joint");
  missing_joint.position.push_back(0.0);
  sensor_msgs::JointState ik_result;
  EXPECT_FALSE(kin.getPositionIK(OneJointZeroPose(), missing_joint, &ik_result));
}

TEST(KinematicsTest, GravityDisabledSizesLeftAndRightIndependently)
{
  arm_kinematics::Kinematics kin;
  kin.num_joints_l = 2;
  kin.num_joints_r = 1;
  kin.left_joint.push_back("left_1");
  kin.left_joint.push_back("left_2");
  kin.right_joint.push_back("right_1");

  baxter_core_msgs::SEAJointState left_gravity;
  baxter_core_msgs::SEAJointState right_gravity;
  ASSERT_TRUE(kin.getGravityTorques(sensor_msgs::JointState(), left_gravity, right_gravity, false));

  EXPECT_EQ(kin.left_joint, left_gravity.name);
  EXPECT_EQ(kin.right_joint, right_gravity.name);
  ASSERT_EQ(2u, left_gravity.gravity_model_effort.size());
  ASSERT_EQ(1u, right_gravity.gravity_model_effort.size());
  EXPECT_DOUBLE_EQ(0.0, left_gravity.gravity_model_effort[0]);
  EXPECT_DOUBLE_EQ(0.0, left_gravity.gravity_model_effort[1]);
  EXPECT_DOUBLE_EQ(0.0, right_gravity.gravity_model_effort[0]);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "baxter_sim_kinematics_tests", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
