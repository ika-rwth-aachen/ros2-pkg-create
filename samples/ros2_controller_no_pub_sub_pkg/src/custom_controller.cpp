#include <ros2_controller_no_pub_sub_pkg/custom_controller.hpp>

#include <cstddef>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <controller_interface/controller_interface_base.hpp>

namespace ros2_controller_no_pub_sub_pkg {

  controller_interface::InterfaceConfiguration CustomController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t index = 1; index <= N_JOINTS; index++) {
    config.names.push_back("fr3_joint" + std::to_string(index) + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration CustomController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t index = 1; index <= N_JOINTS; index++) {
    config.names.push_back("fr3_joint" + std::to_string(index) + "/position");
    config.names.push_back("fr3_joint" + std::to_string(index) + "/velocity");
  }

  return config;
}

controller_interface::CallbackReturn CustomController::on_init() {
  try {
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<double>("velocity_filter_alpha_gain", 0.99);
  } catch (const std::exception &e) {
    (void)fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CustomController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/
  ) {
  // Get k-gains from parameter
  auto k_gains_param = get_node()->get_parameter("k_gains").as_double_array();
  if (k_gains_param.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains_param.size() != N_JOINTS) {
    RCLCPP_FATAL(
      get_node()->get_logger(), "k_gains should be of size %zu but is of size %ld", N_JOINTS, k_gains_param.size()
    );
    return CallbackReturn::FAILURE;
  }

  // Get d-gains from parameter
  auto d_gains_param = get_node()->get_parameter("d_gains").as_double_array();
  if (d_gains_param.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains_param.size() != N_JOINTS) {
    RCLCPP_FATAL(
      get_node()->get_logger(), "d_gains should be of size %zu but is of size %ld", N_JOINTS, d_gains_param.size()
    );
    return CallbackReturn::FAILURE;
  }

  // Store gains
  for (size_t index = 0; index < N_JOINTS; index++) {
    d_gains_(static_cast<int>(index)) = d_gains_param.at(index);
    k_gains_(static_cast<int>(index)) = k_gains_param.at(index);
  }

  // Get single-valued parameters
  velocity_filter_alpha_gain_ = get_node()->get_parameter("velocity_filter_alpha_gain").as_double();

    RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
    return CallbackReturn::SUCCESS;
  }

controller_interface::CallbackReturn CustomController::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/
  ) {

  // Reset parameters
  velocity_filter_alpha_gain_ = 0.0;
  for (size_t index = 0; index < N_JOINTS; index++) {
    d_gains_(static_cast<int>(index)) = 0;
    k_gains_(static_cast<int>(index)) = 0;
  }


  RCLCPP_INFO(get_node()->get_logger(), "Cleanup successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CustomController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/
  ) {

  // Reset internal variables
  filtered_joint_velocities_.setZero();

  // Reset command buffer in case a command came in when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CustomController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/
  ) {

  // Reset internal variables
  filtered_joint_velocities_.setZero();

  // Reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "Deactivate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CustomController::update(
const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
  ) {

  // Obtain current joint states from state interfaces
  Vector7d curr_joint_positions;
  Vector7d curr_joint_velocities;
  for (size_t index = 0; index < N_JOINTS; index++) {
    // This is currently a pretty "dumb" way to obtain Franka states, but we assume 
    // that every even state interface is a position and every odd is velocity.
    const auto &position_interface = state_interfaces_.at(2 * index);
    const auto &velocity_interface = state_interfaces_.at(2 * index + 1);

    if (position_interface.get_interface_name() != "position") {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 1000,
        "Interface with name \"%s\" does not match \"position\" at state index %zu for joint %zu",
        position_interface.get_interface_name().c_str(), 2 * index, index);
      // The controller should stop (or in Humble, spam errors)
      return controller_interface::return_type::ERROR;
    }
    if (velocity_interface.get_interface_name() != "velocity") {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 1000,
        "Interface with name \"%s\" does not match \"velocity\" at state index %zu for joint %zu",
        position_interface.get_interface_name().c_str(), 2 * index + 1, index);
      // The controller should stop (or in Humble, spam errors)
      return controller_interface::return_type::ERROR;
    }

    curr_joint_positions(static_cast<int>(index)) = position_interface.get_value();
    curr_joint_velocities(static_cast<int>(index)) = velocity_interface.get_value();
  }

  // TODO: Compute joint commands internally


  // Calculate the torque command
  filtered_joint_velocities_ = (1 - velocity_filter_alpha_gain_) * filtered_joint_velocities_ +
                                    velocity_filter_alpha_gain_ * curr_joint_velocities;
  Vector7d commanded_joint_torques = k_gains_.cwiseProduct(joint_commands - curr_joint_positions) +
                                   d_gains_.cwiseProduct(-filtered_joint_velocities_);

  // Set the commands as output
  for (size_t index = 0ul; index < N_JOINTS; ++index) {
    command_interfaces_[index].set_value(commanded_joint_torques(static_cast<int>(index)));
  }

  return controller_interface::return_type::OK;
}

}

// Export the controller as a pluginlib plugin

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_controller_no_pub_sub_pkg::CustomController, 
  controller_interface::ControllerInterface
)