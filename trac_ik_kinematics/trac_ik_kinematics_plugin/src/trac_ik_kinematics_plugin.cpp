/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <algorithm>
#include <limits>

// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <trac_ik/trac_ik.hpp>
#include <trac_ik_kinematics_plugin/trac_ik_kinematics_plugin.h>


namespace trac_ik_kinematics_plugin
{
  static rclcpp::Logger LOGGER = rclcpp::get_logger("trac_ik_kinematics_plugin");

  TRAC_IKKinematicsPlugin::TRAC_IKKinematicsPlugin() : initialized_(false)
  {
  }

  bool TRAC_IKKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr &node, 
                                           const moveit::core::RobotModel &robot_model,
                                           const std::string &group_name, 
                                           const std::string &base_frame,
                                           const std::vector<std::string> &tip_frames, 
                                           double search_discretization)
  {
    node_ = node;
    storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
    joint_model_group_ = robot_model_->getJointModelGroup(group_name);
    if (!joint_model_group_)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Unknown planning group: " << group_name);
      return false;
    }

    if (!joint_model_group_->isChain())
    {
      RCLCPP_ERROR(LOGGER, "Group '%s' is not a chain", group_name.c_str());
      return false;
    }
    if (!joint_model_group_->isSingleDOFJoints())
    {
      RCLCPP_ERROR(LOGGER, "Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
      return false;
    }

    KDL::Tree kdl_tree;

    if (!kdl_parser::treeFromUrdfModel(*robot_model.getURDF(), kdl_tree))
    {
      RCLCPP_ERROR(LOGGER, "Could not initialize tree object");
      return false;
    }
    if (!kdl_tree.getChain(base_frame_, getTipFrame(), kdl_chain_))
    {
      RCLCPP_ERROR(LOGGER, "Could not initialize chain object");
      return false;
    }

    dimension_ = joint_model_group_->getActiveJointModels().size() + joint_model_group_->getMimicJointModels().size();
    for (std::size_t i = 0; i < joint_model_group_->getJointModels().size(); ++i)
    {
      if (joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE ||
          joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
      {
        solver_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
        const std::vector<moveit_msgs::msg::JointLimits> &jvec =
            joint_model_group_->getJointModels()[i]->getVariableBoundsMsg();
        solver_info_.limits.insert(solver_info_.limits.end(), jvec.begin(), jvec.end());
      }
    }

    if (!joint_model_group_->hasLinkModel(getTipFrame()))
    {
      RCLCPP_ERROR(LOGGER, "Could not find tip name in joint group '%s'", group_name.c_str());
      return false;
    }
    solver_info_.link_names.push_back(getTipFrame());

    joint_min_.resize(solver_info_.limits.size());
    joint_max_.resize(solver_info_.limits.size());

    for (unsigned int i = 0; i < solver_info_.limits.size(); i++)
    {
      joint_min_(i) = solver_info_.limits[i].min_position;
      joint_max_(i) = solver_info_.limits[i].max_position;
    }
 
    num_joints_ = kdl_chain_.getNrOfJoints();

    RCLCPP_INFO(LOGGER, "number of joints: %i", num_joints_);

    // ###

    state_.reset(new moveit::core::RobotState(robot_model_));

    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    // ###

    lookupParam(node_, "position_only_ik", position_ik_, false);
    RCLCPP_INFO_STREAM(LOGGER, "Position only IK: " << (position_ik_ ? "TRUE" : "FALSE"));
    std::string solve_type_str;
    lookupParam(node_, "solve_type", solve_type_str, std::string("Speed"));
    if (solve_type_str == "Manipulation1")
      solve_type_ = TRAC_IK::Manip1;
    else if (solve_type_str == "Manipulation2")
      solve_type_ = TRAC_IK::Manip2;
    else if (solve_type_str == "Distance")
      solve_type_ = TRAC_IK::Distance;
    else
    {
      if (solve_type_str != "Speed")
      {
        RCLCPP_WARN_STREAM(LOGGER, solve_type_str << " is not a valid solve_type; setting to default: Speed");
        solve_type_str = "Speed";
      }
      solve_type_ = TRAC_IK::Speed;
    }
    RCLCPP_INFO_STREAM(LOGGER, "Solver type: " << solve_type_str);

    initialized_ = true;
    return true;
  }

  int TRAC_IKKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
  {
    int i = 0;
    // while (i < (int)chain.getNrOfSegments())
    // {
    //   if (chain.getSegment(i).getName() == name)
    //   {
    //     return i + 1;
    //   }
    //   i++;
    // }
    while (i < (int)kdl_chain_.getNrOfSegments())
    {
      if (kdl_chain_.getSegment(i).getName() == name)
      {
        return i + 1;
      }
      i++;
    }
    return -1;
  }

  bool TRAC_IKKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                              const std::vector<double> &joint_angles,
                                              std::vector<geometry_msgs::msg::Pose> &poses) const
  {
    if (!initialized_)
    {
      RCLCPP_ERROR(LOGGER, "kinematics not active");
      return false;
    }
    poses.resize(link_names.size());
    if (joint_angles.size() != num_joints_)
    {
      RCLCPP_ERROR(LOGGER, "Joint angles vector must have size: %d", num_joints_);
      return false;
    }

    KDL::Frame p_out;
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::Pose tf_pose;

    KDL::JntArray jnt_pos_in(num_joints_);
    for (unsigned int i = 0; i < num_joints_; i++)
    {
      jnt_pos_in(i) = joint_angles[i];
    }

    // KDL::ChainFkSolverPos_recursive fk_solver(chain);
    // KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);

    bool valid = true;
    for (unsigned int i = 0; i < poses.size(); i++)
    {
      RCLCPP_DEBUG(LOGGER, "End effector index: %d", getKDLSegmentIndex(link_names[i]));
      // if (fk_solver.JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >= 0)
      if (fk_solver_->JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >= 0)
      {
        // poseKDLToMsg(p_out, poses[i]);
        poses[i] = tf2::toMsg(p_out);
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Could not compute FK for %s", link_names[i].c_str());
        valid = false;
      }
    }

    return valid;
  }

  bool TRAC_IKKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              std::vector<double> &solution,
                                              moveit_msgs::msg::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
  {
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            default_timeout_,
                            solution,
                            solution_callback,
                            error_code,
                            consistency_limits,
                            options);
  }

  bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 std::vector<double> &solution,
                                                 moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
  {
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            timeout,
                            solution,
                            solution_callback,
                            error_code,
                            consistency_limits,
                            options);
  }

  bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 const std::vector<double> &consistency_limits,
                                                 std::vector<double> &solution,
                                                 moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
  {
    const IKCallbackFn solution_callback = 0;
    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            timeout,
                            solution,
                            solution_callback,
                            error_code,
                            consistency_limits,
                            options);
  }

  bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 std::vector<double> &solution,
                                                 const IKCallbackFn &solution_callback,
                                                 moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
  {
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            timeout,
                            solution,
                            solution_callback,
                            error_code,
                            consistency_limits,
                            options);
  }

  bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 const std::vector<double> &consistency_limits,
                                                 std::vector<double> &solution,
                                                 const IKCallbackFn &solution_callback,
                                                 moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
  {
    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            timeout,
                            solution,
                            solution_callback,
                            error_code,
                            consistency_limits,
                            options);
  }

  bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 std::vector<double> &solution,
                                                 const IKCallbackFn &solution_callback,
                                                 moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                 const std::vector<double> &consistency_limits,
                                                 const kinematics::KinematicsQueryOptions &options) const
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "getPositionIK");

    if (!initialized_)
    {
      RCLCPP_ERROR(LOGGER, "kinematics not active");
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    if (ik_seed_state.size() != num_joints_)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Seed state must have size " << num_joints_ << " instead of size " << ik_seed_state.size());
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    KDL::Frame frame;
    tf2::fromMsg(ik_pose, frame);

    KDL::JntArray in(num_joints_), out(num_joints_);

    for (uint z = 0; z < num_joints_; z++)
      in(z) = ik_seed_state[z];

    KDL::Twist bounds = KDL::Twist::Zero();

    if (position_ik_)
    {
      bounds.rot.x(std::numeric_limits<float>::max());
      bounds.rot.y(std::numeric_limits<float>::max());
      bounds.rot.z(std::numeric_limits<float>::max());
    }

    double epsilon = 1e-5; //Same as MoveIt's KDL plugin

    TRAC_IK::TRAC_IK ik_solver(kdl_chain_, joint_min_, joint_max_, timeout, epsilon, solve_type_);

    int rc = ik_solver.CartToJnt(in, frame, out, bounds);

    solution.resize(num_joints_);

    if (rc >= 0)
    {
      for (uint z = 0; z < num_joints_; z++)
        solution[z] = out(z);

      // check for collisions if a callback is provided
      if (!solution_callback.empty())
      {
        solution_callback(ik_pose, solution, error_code);
        if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          RCLCPP_DEBUG(LOGGER, "Solution passes callback");
          return true;
        }
        else
        {
          RCLCPP_DEBUG(LOGGER, "Solution has error code %d", error_code.val);
          return false;
        }
      }
      else
        return true; // no collision check callback provided
    }

    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  const std::vector<std::string> &TRAC_IKKinematicsPlugin::getJointNames() const
  {
    return solver_info_.joint_names;
  }

  const std::vector<std::string> &TRAC_IKKinematicsPlugin::getLinkNames() const
  {
    return solver_info_.link_names;
  }

} // end namespace

// //register TRAC_IKKinematicsPlugin as a KinematicsBase implementation
// #include <class_loader/class_loader.hpp>
// CLASS_LOADER_REGISTER_CLASS(trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin, kinematics::KinematicsBase)

// register TRAC_IKKinematicsPlugin as a KinematicsBase implementation
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin, kinematics::KinematicsBase);

