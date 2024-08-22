// Copyright 2023 ros2_control Development Team
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

#include "gravcomp_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace moteus_hardware_interface
{

template <typename Scalar, int Options,
  template <typename, int> class JointCollectionTpl>
void BuildModel(pinocchio::ModelTpl<Scalar, Options, JointCollectionTpl>* model) {
  using namespace pinocchio;

  using M = Model;
  using JC = JointCollectionTpl<Scalar, Options>;
  using CV = typename JC::JointModelRX::ConfigVector_t;
  using TV = typename JC::JointModelRX::TangentVector_t;

  M::JointIndex idx = 0;

  constexpr double kFudge = 0.95;

  SE3 Tlink (SE3::Matrix3::Identity(), SE3::Vector3(0, 0, 0.15));
  Inertia Ilink1(kFudge * 0.29, Tlink.translation(),
                 Inertia::Matrix3::Identity() * 0.001);
  Inertia Ilink2(kFudge * 0.28, Tlink.translation(),
                 Inertia::Matrix3::Identity() * 0.001);

  CV qmin = CV::Constant(-4);
  CV qmax = CV::Constant(4);
  TV vmax = CV::Constant(10);
  TV taumax = CV::Constant(10);

  idx = model->addJoint(idx, typename JC::JointModelRY(), Tlink,
                        "link1_joint", taumax, vmax, qmin, qmax);
  model->appendBodyToJoint(idx, Ilink1);
  model->addJointFrame(idx);
  model->addBodyFrame("link1_body", idx);

  idx = model->addJoint(idx, typename JC::JointModelRY(), Tlink,
                        "link2_joint", taumax, vmax, qmin, qmax);
  model->appendBodyToJoint(idx, Ilink2);
  model->addJointFrame(idx);
  model->addBodyFrame("link2_body", idx);
}

double WrapAround0(double v) {
  const auto v1 = std::fmod(v, 1.0);
  const auto v2 = (v1 < 0.0) ? (v1 + 1.0) : v1;
  return v2 > 0.5 ? (v2 - 1.0) : v2;
}

RobotController::RobotController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RobotController::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  BuildModel(&model_);
  data_ = pinocchio::Data(model_);

  q_ = Eigen::VectorXd::Zero(model_.nv);
  v_ = Eigen::VectorXd::Zero(model_.nv);
  a_ = Eigen::VectorXd::Zero(model_.nv);

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
  // clear out vectors in case of restart
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_effort_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  joint_effort_state_interface_.clear();

  // assign command interfaces
  for (auto & interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // assign state interfaces
  for (auto & interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double current_pos[2] = {0};

  if (joint_position_state_interface_.size() >= 3)
  {
      current_pos[0] = joint_position_state_interface_[1].get().get_value();
      current_pos[1] = joint_position_state_interface_[2].get().get_value();
  }
  else
  {
    return controller_interface::return_type::OK;
  }


  for (size_t i = 1; i < joint_position_state_interface_.size(); i++)
  {
    current_pos[i] = joint_position_state_interface_[i].get().get_value();
  }

  for (size_t i = 0; i < joint_position_command_interface_.size(); i++)
  {
    // set to nan for torque control mode
    joint_position_command_interface_[i].get().set_value(std::numeric_limits<float>::quiet_NaN());
  }
  for (size_t i = 0; i < joint_velocity_command_interface_.size(); i++)
  {
    // set to 0 for torque control mode
    joint_velocity_command_interface_[i].get().set_value(0.0);
  }

  //for (size_t i = 0; i < joint_effort_command_interface_.size(); i++)
  //{
    // command torque dependent on position
  //  joint_effort_command_interface_[i].get().set_value(-sin(current_pos*2*M_PI));
  //}

  q_(0) = WrapAround0(current_pos[0] + 0.5) * 2 * M_PI * 0.2;
  q_(1) = WrapAround0(current_pos[1]) * 2 * M_PI;

  const Eigen::VectorXd& tau = pinocchio::rnea(model_, data_, q_, v_, a_);

  joint_effort_command_interface_[1].get().set_value(tau(0));
  joint_effort_command_interface_[2].get().set_value(tau(1));  

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_cleanup(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace moteus_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  moteus_hardware_interface::RobotController, controller_interface::ControllerInterface)
