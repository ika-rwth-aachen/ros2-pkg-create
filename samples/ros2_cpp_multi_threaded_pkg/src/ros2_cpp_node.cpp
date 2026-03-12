#include <functional>
#include <thread>

#include <ros2_cpp_multi_threaded_pkg/ros2_cpp_node.hpp>


namespace ros2_cpp_multi_threaded_pkg {


Ros2CppNode::Ros2CppNode() : Node("ros2_cpp_node") {

  this->declareAndLoadParameter("num_threads", num_threads_, "number of threads for MultiThreadedExecutor", false, false, false, 1, std::thread::hardware_concurrency(), 1);
  this->declareAndLoadParameter("param", param_, "TODO", true, false, false, 0.0, 10.0, 1.0);
  this->setup();
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

  // create callback group for potentially multi-threaded execution
  rclcpp::SubscriptionOptions subscriber_options;
  subscriber_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
  subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("~/input", subscriber_qos_profile, std::bind(&Ros2CppNode::topicCallback, this, std::placeholders::_1), subscriber_options);
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
}


void Ros2CppNode::topicCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg) {

  RCLCPP_INFO(this->get_logger(), "Message received with stamp: '%d'", msg->header.stamp.sec);

  // publish message
  geometry_msgs::msg::PointStamped out_msg;
  out_msg = *msg;
  publisher_->publish(out_msg);
  RCLCPP_INFO(this->get_logger(), "Message published with stamp: '%d'", out_msg.header.stamp.sec);
}


}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_cpp_multi_threaded_pkg::Ros2CppNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), node->num_threads_);
  RCLCPP_INFO(node->get_logger(), "Spinning node '%s' with %s (%d threads)", node->get_fully_qualified_name(), "MultiThreadedExecutor", node->num_threads_);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
