#include <algorithm>
#include <functional>

#include <ros2_cpp_lifecycle_pkg/ros2_cpp_node.hpp>


namespace ros2_cpp_lifecycle_pkg {


/**
 * @brief Constructor
 *
 * @param options node options
 */
Ros2CppNode::Ros2CppNode() : rclcpp_lifecycle::LifecycleNode("ros2_cpp_node") {

  int startup_state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  this->declareAndLoadParameter("startup_state", startup_state, "Initial lifecycle state", false, false, false);
  if (startup_state > lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (startup_state > lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}


/**
 * @brief Declares and loads a ROS parameter
 *
 * @param name name
 * @param param parameter variable to load into
 * @param description description
 * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
 * @param is_required whether failure to load parameter will stop node
 * @param read_only set parameter to read-only
 * @param from_value parameter range minimum
 * @param to_value parameter range maximum
 * @param step_value parameter range step
 * @param additional_constraints additional constraints description
 */
template <typename T>
void Ros2CppNode::declareAndLoadParameter(const std::string& name,
                                                         T& param,
                                                         const std::string& description,
                                                         const bool add_to_auto_reconfigurable_params,
                                                         const bool is_required,
                                                         const bool read_only,
                                                         const std::optional<double>& from_value,
                                                         const std::optional<double>& to_value,
                                                         const std::optional<double>& step_value,
                                                         const std::string& additional_constraints) {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE); // TODO: rclpy shutdown?
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "]");
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
    }
  }

  if (add_to_auto_reconfigurable_params) {
    // why so complicated, storing lambda functions? / why vector, not map?
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}


/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult Ros2CppNode::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}


/**
 * @brief Sets up subscribers, publishers, etc. to configure the node
 */
void Ros2CppNode::setup() {

  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(std::bind(&Ros2CppNode::parametersCallback, this, std::placeholders::_1));

  // subscriber for handling incoming messages
  subscriber_ = this->create_subscription<std_msgs::msg::Int32>("~/input", 10, std::bind(&Ros2CppNode::topicCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());

  // publisher for publishing outgoing messages
  publisher_ = this->create_publisher<std_msgs::msg::Int32>("~/output", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());
}


/**
 * @brief Processes messages received by a subscriber
 *
 * @param msg message
 */
void Ros2CppNode::topicCallback(const std_msgs::msg::Int32& msg) {

  RCLCPP_INFO(this->get_logger(), "Message received: '%d'", msg.data);
}


/**
 * @brief Processes 'configuring' transitions to 'inactive' state
 *
 * @param state previous state
 * @return transition result
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_configure(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Configuring to enter 'inactive' state from '%s' state", state.label().c_str());

  this->declareAndLoadParameter("param", param_, "TODO", true, false, false, 0.0, 10.0, 1.0);
  setup();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


/**
 * @brief Processes 'activating' transitions to 'active' state
 *
 * @param state previous state
 * @return transition result
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_activate(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Activating to enter 'active' state from '%s' state", state.label().c_str());

  publisher_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


/**
 * @brief Processes 'deactivating' transitions to 'inactive' state
 *
 * @param state previous state
 * @return transition result
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_deactivate(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Deactivating to enter 'inactive' state from '%s' state", state.label().c_str());

  publisher_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


/**
 * @brief Processes 'cleningup' transitions to 'unconfigured' state
 *
 * @param state previous state
 * @return transition result
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_cleanup(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Cleaning up to enter 'unconfigured' state from '%s' state", state.label().c_str());

  subscriber_.reset();
  publisher_.reset();
  parameters_callback_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


/**
 * @brief Processes 'shuttingdown' transitions to 'finalized' state
 *
 * @param state previous state
 * @return transition result
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_shutdown(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Shutting down to enter 'finalized' state from '%s' state", state.label().c_str());

  if (state.id() >= lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    on_deactivate(state);
  if (state.id() >= lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    on_cleanup(state);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_cpp_lifecycle_pkg::Ros2CppNode>()->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}