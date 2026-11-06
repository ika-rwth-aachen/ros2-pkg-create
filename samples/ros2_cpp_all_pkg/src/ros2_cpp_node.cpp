#include <algorithm>
#include <chrono>
#include <functional>
#include <thread>

#include <ros2_cpp_all_pkg/ros2_cpp_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_cpp_all_pkg::Ros2CppNode)


namespace ros2_cpp_all_pkg {


Ros2CppNode::Ros2CppNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("ros2_cpp_node", options) {

  initializeDiagnostics();
  diagnostic_updater_->add("health", this, &Ros2CppNode::health);
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

  if (read_only)
    this->declare_parameter(name, rclcpp::ParameterValue(param), param_desc);
  else
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
  std::lock_guard<std::mutex> lock(callback_mutex_);

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

  // Topic tasks run before the health summary in the same updater cycle.
  diagnostic_updater_->removeByName("health");
  if (topic_diagnostic_config_.enabled) {
    topic_diagnostic_ = std::make_unique<CapturedTopicDiagnostic>(
      subscriber_->get_topic_name(), *diagnostic_updater_, topic_diagnostic_config_, get_clock());
  }
  if (diagnosed_publisher_config_.enabled) {
    output_diagnostic_ = std::make_unique<CapturedTopicDiagnostic>(
      publisher_->get_topic_name(), *diagnostic_updater_, diagnosed_publisher_config_, get_clock());
  }
  diagnostic_updater_->add("health", this, &Ros2CppNode::health);
}


void Ros2CppNode::topicCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (!callbacks_active_ || !subscriber_ || !publisher_) return;

  if (topic_diagnostic_) topic_diagnostic_->tick(msg->header.stamp);
  RCLCPP_INFO(this->get_logger(), "Message received with stamp: '%d'", msg->header.stamp.sec);

  // publish message
  geometry_msgs::msg::PointStamped::UniquePtr out_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  *out_msg = *msg;
  RCLCPP_INFO(this->get_logger(), "Message published with stamp: '%d'", out_msg->header.stamp.sec);
  publisher_->publish(std::move(out_msg));
  if (output_diagnostic_) output_diagnostic_->tick(msg->header.stamp);
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

}


namespace {
int diagnosticSeverity(unsigned char level) {
  switch (level) {
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR: return 3;
    case diagnostic_msgs::msg::DiagnosticStatus::STALE: return 2;
    case diagnostic_msgs::msg::DiagnosticStatus::WARN: return 1;
    default: return 0;
  }
}

const char* diagnosticLevelName(unsigned char level) {
  switch (level) {
    case diagnostic_msgs::msg::DiagnosticStatus::OK: return "OK";
    case diagnostic_msgs::msg::DiagnosticStatus::WARN: return "WARN";
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR: return "ERROR";
    default: return "STALE";
  }
}
}

void Ros2CppNode::initializeDiagnostics() {
  bool use_fqn = true;
  double period = 1.0;
  declareAndLoadParameter("diagnostic_updater.use_fqn", use_fqn, "Prefix statuses with the fully qualified node name", false, false, true);
  declareAndLoadParameter("diagnostic_updater.period", period, "Diagnostic publication period in seconds", false, false, true);
  if (!std::isfinite(period) || period <= 0.0)
    throw std::invalid_argument("Diagnostic period must be finite and positive");
  topic_diagnostic_config_ = loadTopicDiagnosticConfig("topic_diagnostic");
  diagnosed_publisher_config_ = loadTopicDiagnosticConfig("diagnosed_publisher");
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this, period, diagnostic_msgs::msg::DiagnosticStatus::STALE);
  diagnostic_updater_->setHardwareID("none");
}

TopicDiagnosticConfig Ros2CppNode::loadTopicDiagnosticConfig(const std::string& name) {
  TopicDiagnosticConfig config;
  const std::string prefix = "diagnostic_updater." + name + ".";
  declareAndLoadParameter(prefix + "enabled", config.enabled, "Monitor this periodic topic", false, false, true);
  declareAndLoadParameter(prefix + "min_frequency", config.min_frequency, "Minimum frequency in Hz", false, false, true);
  declareAndLoadParameter(prefix + "max_frequency", config.max_frequency, "Maximum frequency in Hz", false, false, true);
  declareAndLoadParameter(prefix + "min_acceptable_timestamp_delta", config.min_acceptable_timestamp_delta, "Minimum message age in seconds", false, false, true);
  declareAndLoadParameter(prefix + "max_acceptable_timestamp_delta", config.max_acceptable_timestamp_delta, "Maximum message age in seconds", false, false, true);
  if (!std::isfinite(config.min_frequency) || config.min_frequency <= 0.0 ||
      !std::isfinite(config.max_frequency) || config.max_frequency < config.min_frequency ||
      !std::isfinite(config.min_acceptable_timestamp_delta) ||
      !std::isfinite(config.max_acceptable_timestamp_delta) ||
      config.max_acceptable_timestamp_delta < config.min_acceptable_timestamp_delta)
    throw std::invalid_argument(prefix + "requires positive ordered frequencies and finite ordered timestamp bounds");
  config.timeout = std::max(3.0 / config.min_frequency, 0.5);
  declareAndLoadParameter(prefix + "timeout", config.timeout, "Live topic timeout in seconds", false, false, true);
  if (!std::isfinite(config.timeout) || config.timeout <= 0.0)
    throw std::invalid_argument(prefix + "timeout must be finite and positive");
  return config;
}

void Ros2CppNode::removeTopicDiagnostics() {
  if (topic_diagnostic_) {
    diagnostic_updater_->removeByName(topic_diagnostic_->getName());
    topic_diagnostic_.reset();
  }
  if (output_diagnostic_) {
    diagnostic_updater_->removeByName(output_diagnostic_->getName());
    output_diagnostic_.reset();
  }
}

void Ros2CppNode::health(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  std::map<std::string, DiagnosticCheck> checks;
  {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    checks = diagnostic_checks_;
  }
  const auto state = get_current_state();
  if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    checks["lifecycle"] = {diagnostic_msgs::msg::DiagnosticStatus::STALE,
                           "node not active (" + state.label() + ")", {}};
  } else {
  if (topic_diagnostic_) {
    checks["topic.input"] = topic_diagnostic_->result();
    checks["inputs"] = topic_diagnostic_->progress("received_since_start");
  }
  if (output_diagnostic_) {
    checks["topic.output"] = output_diagnostic_->result();
    checks["outputs"] = output_diagnostic_->progress("published_since_start");
  }
  }
  // Evaluate application-specific checks here; do not register extra updater tasks.
  unsigned char level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "All checks OK";
  if (checks.empty()) {
    level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    message = "No assessments registered";
  }
  for (const auto& [name, check] : checks) {
    if (diagnosticSeverity(check.level) > diagnosticSeverity(level)) {
      level = check.level;
      message = name + ": " + check.message;
    }
    stat.add(name, std::string(diagnosticLevelName(check.level)) + ": " + check.message);
    const auto previous = diagnostic_levels_.find(name);
    if (previous == diagnostic_levels_.end() || previous->second != check.level) {
      if (check.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR)
        RCLCPP_ERROR(get_logger(), "%s: %s", name.c_str(), check.message.c_str());
      else if (check.level == diagnostic_msgs::msg::DiagnosticStatus::WARN)
        RCLCPP_WARN(get_logger(), "%s: %s", name.c_str(), check.message.c_str());
      else if (check.level == diagnostic_msgs::msg::DiagnosticStatus::OK && previous != diagnostic_levels_.end())
        RCLCPP_INFO(get_logger(), "%s: %s", name.c_str(), check.message.c_str());
    }
    diagnostic_levels_[name] = check.level;
  }
  for (const auto& [name, check] : checks)
    for (const auto& [key, value] : check.details)
      stat.add(name + "." + key, value);
  stat.summary(level, message);
}

void Ros2CppNode::setDiagnosticCheck(const std::string& name, const DiagnosticCheck& check) {
  if (check.level > diagnostic_msgs::msg::DiagnosticStatus::STALE)
    throw std::invalid_argument("Unknown diagnostic level");
  std::lock_guard<std::mutex> lock(diagnostic_mutex_);
  diagnostic_checks_[name] = check;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_configure(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Configuring to enter 'inactive' state from '%s' state", state.label().c_str());

  if (!has_parameter("param"))
    this->declareAndLoadParameter("param", param_, "TODO", true, false, false, 0.0, 10.0, 1.0);

  setup();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_activate(const rclcpp_lifecycle::State& state) {
  std::lock_guard<std::mutex> lock(callback_mutex_);

  RCLCPP_INFO(get_logger(), "Activating to enter 'active' state from '%s' state", state.label().c_str());

  if (topic_diagnostic_) topic_diagnostic_->setActive(true);
  if (output_diagnostic_) output_diagnostic_->setActive(true);
  publisher_->on_activate();
  timer_ = this->create_wall_timer(std::chrono::duration<double>(param_), std::bind(&Ros2CppNode::timerCallback, this));

  callbacks_active_ = true;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callbacks_active_ = false;

  RCLCPP_INFO(get_logger(), "Deactivating to enter 'inactive' state from '%s' state", state.label().c_str());

  timer_.reset();
  if (topic_diagnostic_) topic_diagnostic_->setActive(false);
  if (output_diagnostic_) output_diagnostic_->setActive(false);
  publisher_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2CppNode::on_cleanup(const rclcpp_lifecycle::State& state) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callbacks_active_ = false;

  RCLCPP_INFO(get_logger(), "Cleaning up to enter 'unconfigured' state from '%s' state", state.label().c_str());

  removeTopicDiagnostics();
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
