#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <map>
#include <mutex>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <ros2_cpp_all_pkg_interfaces/action/fibonacci.hpp>


namespace ros2_cpp_all_pkg {

template <typename C> struct is_vector : std::false_type {};
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;


struct DiagnosticCheck {
  unsigned char level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  std::string message = "Awaiting assessment";
  std::map<std::string, std::string> details;
};

struct TopicDiagnosticConfig {
  bool enabled = true;
  double min_frequency = 0.5;
  double max_frequency = 1.0;
  double min_acceptable_timestamp_delta = -0.1;
  double max_acceptable_timestamp_delta = 0.1;
  double timeout = 6.0;
};

/**
 * @brief Captures one topic assessment per updater cycle and tracks live progress
 */
class CapturedTopicDiagnostic : public diagnostic_updater::HeaderlessTopicDiagnostic {
  class TimestampTask : public diagnostic_updater::DiagnosticTask {
   public:
    TimestampTask(const TopicDiagnosticConfig& config, const rclcpp::Clock::SharedPtr& clock)
      : DiagnosticTask("Timestamp Status"),
        params_(config.min_acceptable_timestamp_delta, config.max_acceptable_timestamp_delta), clock_(clock) {
      reset();
    }
    void reset() { task_ = std::make_unique<diagnostic_updater::TimeStampStatus>(params_, clock_); }
    void tick(const rclcpp::Time& stamp) { task_->tick(stamp); }
    void run(diagnostic_updater::DiagnosticStatusWrapper& stat) override { task_->run(stat); }
   private:
    diagnostic_updater::TimeStampStatusParam params_;
    rclcpp::Clock::SharedPtr clock_;
    std::unique_ptr<diagnostic_updater::TimeStampStatus> task_;
  };

 public:
  CapturedTopicDiagnostic(const std::string& topic, diagnostic_updater::Updater& updater,
                         TopicDiagnosticConfig& config, const rclcpp::Clock::SharedPtr& clock)
    : HeaderlessTopicDiagnostic(topic, updater,
        diagnostic_updater::FrequencyStatusParam(&config.min_frequency, &config.max_frequency, 0.0,
          frequencyWindow(updater.getPeriod().seconds(), config.min_frequency)),
        std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
      timeout_(config.timeout), stamp_(config, clock) {
    addTask(&stamp_);
  }

  void setActive(bool active) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = active;
    received_ = false;
    baseline_ = std::chrono::steady_clock::now();
    clear_window();
    stamp_.reset();
  }

  void tick(const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) return;
    stamp_.tick(stamp);
    HeaderlessTopicDiagnostic::tick();
    received_ = true;
    last_event_ = std::chrono::steady_clock::now();
    ++events_since_start_;
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper& stat) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "Node not active");
    } else {
      HeaderlessTopicDiagnostic::run(stat);
      if (!received_ && age() <= timeout_)
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "Awaiting first message");
    }
    result_ = {stat.level, stat.message.empty() ? "Frequency and timestamps within bounds" : stat.message, {}};
  }

  DiagnosticCheck result() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return result_;
  }

  DiagnosticCheck progress(const std::string& counter) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const double elapsed = age();
    DiagnosticCheck check;
    if (elapsed > timeout_)
      check = {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Required topic timed out", {}};
    else if (!received_)
      check = {diagnostic_msgs::msg::DiagnosticStatus::STALE, "Awaiting first message", {}};
    else
      check = {diagnostic_msgs::msg::DiagnosticStatus::OK, "Topic current", {}};
    check.details = { {"age_s", std::to_string(elapsed)}, {counter, std::to_string(events_since_start_)} };
    return check;
  }

 private:
  static int frequencyWindow(double period, double frequency) {
    const double size = std::ceil(5.0 / (period * frequency));
    if (!std::isfinite(size) || size > 100000.0)
      throw std::invalid_argument("Diagnostic frequency window too large");
    return static_cast<int>(std::max(1.0, size));
  }

  double age() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        (received_ ? last_event_ : baseline_)).count();
  }

  mutable std::mutex mutex_;
  bool active_ = false;
  bool received_ = false;
  std::uint64_t events_since_start_ = 0;
  double timeout_;
  std::chrono::steady_clock::time_point baseline_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_event_;
  DiagnosticCheck result_;
  TimestampTask stamp_;
};


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
  void topicCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg);

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

  /**
   * @brief Function called by diagnostic updater to populate diagnostics status
   */
  void health(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /**
   * @brief Stores an application check for the periodic health assessment
   */
  void setDiagnosticCheck(const std::string& name, const DiagnosticCheck& check);

  void initializeDiagnostics();
  TopicDiagnosticConfig loadTopicDiagnosticConfig(const std::string& name);
  void removeTopicDiagnostics();

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
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;

  /**
   * @brief Publisher
   */
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;

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

  std::mutex callback_mutex_;
  std::mutex diagnostic_mutex_;
  std::map<std::string, DiagnosticCheck> diagnostic_checks_;
  std::map<std::string, unsigned char> diagnostic_levels_;
  TopicDiagnosticConfig topic_diagnostic_config_;
  std::unique_ptr<CapturedTopicDiagnostic> topic_diagnostic_;
  TopicDiagnosticConfig diagnosed_publisher_config_;
  std::unique_ptr<CapturedTopicDiagnostic> output_diagnostic_;
  // Destroy the updater timer before the task objects it references.
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};


}
