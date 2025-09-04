#pragma once

#include <memory>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <ros2_cpp_all_pkg_interfaces/action/fibonacci.hpp>


namespace ros2_cpp_all_pkg {

template <typename C> struct is_vector : std::false_type {};
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;


/**
 * @brief Ros2CppNode class
 */
class Ros2CppNode : public rclcpp_lifecycle::LifecycleNode {

 public:

  /**
   * @brief Constructor
   *
   * @param options node options
   */
  explicit Ros2CppNode(const rclcpp::NodeOptions& options);

 protected:

  /**
   * @brief Processes 'configuring' transitions to 'inactive' state
   *
   * @param state previous state
   * @return transition result
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Processes 'activating' transitions to 'active' state
   *
   * @param state previous state
   * @return transition result
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Processes 'deactivating' transitions to 'inactive' state
   *
   * @param state previous state
   * @return transition result
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Processes 'cleaningup' transitions to 'unconfigured' state
   *
   * @param state previous state
   * @return transition result
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Processes 'shuttingdown' transitions to 'finalized' state
   *
   * @param state previous state
   * @return transition result
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:

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
  void declareAndLoadParameter(const std::string &name,
                               T &param,
                               const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");

  /**
   * @brief Handles reconfiguration when a parameter value is changed
   *
   * @param parameters parameters
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  /**
   * @brief Processes messages received by a subscriber
   *
   * @param msg message
   */
  void topicCallback(const std_msgs::msg::Int32::ConstSharedPtr& msg);

  /**
   * @brief Processes service requests
   *
   * @param request service request
   * @param response service response
   */
  void serviceCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request>request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Processes action goal requests
   *
   * @param uuid unique goal identifier
   * @param goal action goal
   * @return goal response
   */
  rclcpp_action::GoalResponse actionHandleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ros2_cpp_all_pkg_interfaces::action::Fibonacci::Goal> goal);

  /**
   * @brief Processes action cancel requests
   *
   * @param goal_handle action goal handle
   * @return cancel response
   */
  rclcpp_action::CancelResponse actionHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_cpp_all_pkg_interfaces::action::Fibonacci>> goal_handle);

  /**
   * @brief Processes accepted action goal requests
   *
   * @param goal_handle action goal handle
   */
  void actionHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_cpp_all_pkg_interfaces::action::Fibonacci>> goal_handle);

  /**
   * @brief Executes an action
   *
   * @param goal_handle action goal handle
   */
  void actionExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_cpp_all_pkg_interfaces::action::Fibonacci>> goal_handle);

  /**
   * @brief Processes timer triggers
   */
  void timerCallback();

 private:

  /**
   * @brief Auto-reconfigurable parameters for dynamic reconfiguration
   */
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;

  /**
   * @brief Callback handle for dynamic parameter reconfiguration
   */
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  /**
   * @brief Subscriber
   */
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;

  /**
   * @brief Publisher
   */
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  /**
   * @brief Service server
   */
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_;

  /**
   * @brief Action server
   */
  rclcpp_action::Server<ros2_cpp_all_pkg_interfaces::action::Fibonacci>::SharedPtr action_server_;

  /**
   * @brief Timer
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Dummy parameter (parameter)
   */
  double param_ = 1.0;
};


}
