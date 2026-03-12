#include <algorithm>
#include <chrono>
#include <functional>
#include <thread>

#include <ros2_cpp_all_pkg/ros2_cpp_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_cpp_all_pkg::Ros2CppNode)


namespace ros2_cpp_all_pkg {


Ros2CppNode::Ros2CppNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("ros2_cpp_node", options) {

  int startup_state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  this->declareAndLoadParameter("startup_state", startup_state, "Initial lifecycle state", false, false, false);
  if (startup_state > lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (startup_state > lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}


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
    if constexpr(std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.integer_range = {range};
    } else if constexpr(std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr(is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
      ss << "]";
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr(is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}


rcl_interfaces::msg::SetParametersResult Ros2CppNode::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s' to: %s", param.get_name().c_str(), param.value_to_string().c_str());
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}


void Ros2CppNode::setup() {

  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(std::bind(&Ros2CppNode::parametersCallback, this, std::placeholders::_1));

  /*  subscriber for handling incoming messages with suitable quality-of-service settings:
        1. default:     reliable, durability volatile, keep last 10 received messages in queue
                        default ROS 2 setting, requires reliable publisher
        2. sensor data: best effort, durability volatile, keep last message
                        take most recent message, no guarantee to receive every single message
        3. transient:   durability transient local, reliable, keep last message
                        take most recent message, even if it was published while the 
                        subscriber was offline, requires transient local publisher
  */
  auto subscriber_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  // auto subscriber_qos_profile = rclcpp::SensorDataQoS().keep_last(1);
  // auto subscriber_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("~/input", subscriber_qos_profile, std::bind(&Ros2CppNode::topicCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());

  /*  publisher for publishing outgoing messages with suitable quality-of-service settings:
        1. default:     reliable, durability volatile, keep 10 messages in queue for sending
                        default ROS 2 setting
        2. sensor data: best effort, durability volatile, send last published message
                        no re-transmission if message could not be delivered
        3. transient:   durability transient local, reliable, send last published message
                        keep last published message available for late-joining subscribers,
                        requires transient local subscriber
  */
  auto publisher_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  // auto publisher_qos_profile = rclcpp::SensorDataQoS().keep_last(1);
  // auto publisher_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("~/output", publisher_qos_profile);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());

  // service server for handling service calls
  service_server_ = this->create_service<std_srvs::srv::SetBool>("~/service", std::bind(&Ros2CppNode::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));

  // action server for handling action goal requests
  action_server_ = rclcpp_action::create_server<ros2_cpp_all_pkg_interfaces::action::Fibonacci>(
    this,
    "~/action",
    std::bind(&Ros2CppNode::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Ros2CppNode::actionHandleCancel, this, std::placeholders::_1),
    std::bind(&Ros2CppNode::actionHandleAccepted, this, std::placeholders::_1)
  );

  // setup diagnostic updater
  diagnostic_updater_.setHardwareID("none");
  diagnostic_updater_.add("Health", this, &Ros2CppNode::health);

  // add diagnostic task for monitoring topic subscription with a moving average over min. 5 incoming messages based on expected minimum frequency
  const int topic_diagnostic_frequency_window_size = std::ceil(5 / (diagnostic_updater_.getPeriod().seconds() * topic_diagnostic_config_.min_frequency));
  topic_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
    "~/input",
    diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(&topic_diagnostic_config_.min_frequency, &topic_diagnostic_config_.max_frequency, 0.0, topic_diagnostic_frequency_window_size),
    diagnostic_updater::TimeStampStatusParam(topic_diagnostic_config_.min_acceptable_timestamp_delta, topic_diagnostic_config_.max_acceptable_timestamp_delta)
  );

  // add diagnostic task for monitoring topic publisher with a moving average over min. 5 incoming messages based on expected minimum frequency
  const int diagnosed_publisher_frequency_window_size = std::ceil(5 / (diagnostic_updater_.getPeriod().seconds() * diagnosed_publisher_config_.min_frequency));
  diagnosed_publisher_ = std::make_unique<diagnostic_updater::DiagnosedPublisher<geometry_msgs::msg::PointStamped>>(
    publisher_,
    diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(&diagnosed_publisher_config_.min_frequency, &diagnosed_publisher_config_.max_frequency, 0.0, diagnosed_publisher_frequency_window_size),
    diagnostic_updater::TimeStampStatusParam(diagnosed_publisher_config_.min_acceptable_timestamp_delta, diagnosed_publisher_config_.max_acceptable_timestamp_delta)
  );
}


void Ros2CppNode::topicCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg) {

  topic_diagnostic_->tick(msg->header.stamp);
  RCLCPP_INFO(this->get_logger(), "Message received with stamp: '%d'", msg->header.stamp.sec);

  // publish message
  geometry_msgs::msg::PointStamped::UniquePtr out_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  *out_msg = *msg;
  RCLCPP_INFO(this->get_logger(), "Message published with stamp: '%d'", out_msg->header.stamp.sec);
  diagnosed_publisher_->publish(std::move(out_msg));
}


void Ros2CppNode::serviceCallback(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {

  (void)request;

  RCLCPP_INFO(this->get_logger(), "Received service request");

  response->success = true;
}


rclcpp_action::GoalResponse Ros2CppNode::actionHandleGoal(const rclcpp_action::GoalUUID& uuid, ros2_cpp_all_pkg_interfaces::action::Fibonacci::Goal::ConstSharedPtr goal) {

  (void)uuid;
  (void)goal;

  RCLCPP_INFO(this->get_logger(), "Received action goal request");

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse Ros2CppNode::actionHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_cpp_all_pkg_interfaces::action::Fibonacci>> goal_handle) {

  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received request to cancel action goal");

  return rclcpp_action::CancelResponse::ACCEPT;
}


void Ros2CppNode::actionHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_cpp_all_pkg_interfaces::action::Fibonacci>> goal_handle) {

  // execute action in a separate thread to avoid blocking
  std::thread{std::bind(&Ros2CppNode::actionExecute, this, std::placeholders::_1), goal_handle}.detach();
}


void Ros2CppNode::actionExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_cpp_all_pkg_interfaces::action::Fibonacci>> goal_handle) {

  RCLCPP_INFO(this->get_logger(), "Executing action goal");

  // define a sleeping rate between computing individual Fibonacci numbers
  rclcpp::Rate loop_rate(1);

  // create handy accessors for the action goal, feedback, and result
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ros2_cpp_all_pkg_interfaces::action::Fibonacci::Feedback>();
  auto result = std::make_shared<ros2_cpp_all_pkg_interfaces::action::Fibonacci::Result>();

  // initialize the Fibonacci sequence
  auto& partial_sequence = feedback->partial_sequence;
  partial_sequence.push_back(0);
  partial_sequence.push_back(1);

  // compute the Fibonacci sequence up to the requested order n
  for (int i = 1; i < goal->order && rclcpp::ok(); ++i) {

    // cancel, if requested
    if (goal_handle->is_canceling()) {
      result->sequence = feedback->partial_sequence;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Action goal canceled");
      return;
    }

    // compute the next Fibonacci number
    partial_sequence.push_back(partial_sequence[i] + partial_sequence[i - 1]);

    // publish the current sequence as action feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publishing action feedback");

    // sleep before computing the next Fibonacci number
    loop_rate.sleep();
  }

  // finish by publishing the action result
  if (rclcpp::ok()) {
    result->sequence = partial_sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}


void Ros2CppNode::timerCallback() {

  RCLCPP_INFO(this->get_logger(), "Timer triggered");

  // TODO: Remove this demonstration of health status and implement real health checks using `setHealth()`
  if(health_.status == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Node is non-functional, unable to obtain, and/or yielding implausible data.", { {"uptime", std::to_string(this->get_clock()->now().seconds())} });
    health_.status = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else if(health_.status == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Node is able to assess its own performance level, but is not able to reach its desired performance level.");
    health_.status = diagnostic_msgs::msg::DiagnosticStatus::OK;
  } else if(health_.status == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::OK, "Node is able to assess its own performance level and is reaching its desired performance level.");
    health_.status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  } else {
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::STALE, "Node performance level cannot be assessed");
    health_.status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  // end of demonstration
}


void Ros2CppNode::health(diagnostic_updater::DiagnosticStatusWrapper& stat) {

  stat.summary(health_.status, health_.message);
  for (const auto& [key, value] : health_.key_value_pairs) {
    stat.add(key, value);
  }
}


void Ros2CppNode::setHealth(const unsigned char status, const std::string& msg, const std::map<std::string, std::string>& key_value_pairs) {

  health_.status = status;
  health_.message = msg;
  health_.key_value_pairs = key_value_pairs;
  diagnostic_updater_.force_update();
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_configure(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Configuring to enter 'inactive' state from '%s' state", state.label().c_str());

  this->declareAndLoadParameter("param", param_, "TODO", true, false, false, 0.0, 10.0, 1.0);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostic.min_frequency", topic_diagnostic_config_.min_frequency, "Minimum frequency for incoming messages", true, true, false);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostic.max_frequency", topic_diagnostic_config_.max_frequency, "Maximum frequency for incoming messages", true, true, false);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostic.min_acceptable_timestamp_delta", topic_diagnostic_config_.min_acceptable_timestamp_delta, "Minimum acceptable timestamp delta for incoming messages", true, true, false);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostic.max_acceptable_timestamp_delta", topic_diagnostic_config_.max_acceptable_timestamp_delta, "Maximum acceptable timestamp delta for incoming messages", true, true, false);
  this->declareAndLoadParameter("diagnostic_updater.diagnosed_publisher.min_frequency", diagnosed_publisher_config_.min_frequency, "Minimum frequency for outgoing messages", true, true, false);
  this->declareAndLoadParameter("diagnostic_updater.diagnosed_publisher.max_frequency", diagnosed_publisher_config_.max_frequency, "Maximum frequency for outgoing messages", true, true, false);
  this->declareAndLoadParameter("diagnostic_updater.diagnosed_publisher.min_acceptable_timestamp_delta", diagnosed_publisher_config_.min_acceptable_timestamp_delta, "Minimum acceptable timestamp delta for outgoing messages", true, true, false);
  this->declareAndLoadParameter("diagnostic_updater.diagnosed_publisher.max_acceptable_timestamp_delta", diagnosed_publisher_config_.max_acceptable_timestamp_delta, "Maximum acceptable timestamp delta for outgoing messages", true, true, false);
  setup();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_activate(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Activating to enter 'active' state from '%s' state", state.label().c_str());

  publisher_->on_activate();
  timer_ = this->create_wall_timer(std::chrono::duration<double>(param_), std::bind(&Ros2CppNode::timerCallback, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_deactivate(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Deactivating to enter 'inactive' state from '%s' state", state.label().c_str());

  timer_.reset();
  publisher_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_cleanup(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Cleaning up to enter 'unconfigured' state from '%s' state", state.label().c_str());

  subscriber_.reset();
  publisher_.reset();
  service_server_.reset();
  action_server_.reset();
  parameters_callback_.reset();
  timer_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_shutdown(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Shutting down to enter 'finalized' state from '%s' state", state.label().c_str());

  if (state.id() >= lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    on_deactivate(state);
  if (state.id() >= lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    on_cleanup(state);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


}
