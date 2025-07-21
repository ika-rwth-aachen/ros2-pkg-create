#ifndef ROS2_CONTROLLER_ALL_PKG_CUSTOM_CONTROLLER
#define ROS2_CONTROLLER_ALL_PKG_CUSTOM_CONTROLLER

#include <memory>

#include <Eigen/Eigen>
#include <Eigen/src/Core/IO.h>
#include <Eigen/src/Core/Matrix.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>


#include <ros2_controller_all_pkg_interfaces/msg/joint_positions.hpp>
#include <ros2_controller_all_pkg_interfaces/msg/joint_torques.hpp>

namespace ros2_controller_all_pkg
{

  /**
   * 
   */
  class CustomController : public controller_interface::ControllerInterface {

  public:
    /// Number of joints in the controlled robot
    static constexpr const size_t N_JOINTS = 7;
  
    /// Type alias for a vector with values for each joint
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    /// Type alias for command input type
    using CmdMsg = ros2_controller_all_pkg_interfaces::msg::JointPositions;

    /// Type alias for commanded state output type
    using StateMsg = ros2_controller_all_pkg_interfaces::msg::JointTorques;

  protected:
    /**
     * Specify which hardware command interfaces are used.
     *
     * This method is called in the `active` or `inactive` state. This means 
     * that the configuration can be changed during the `on_configure` state 
     * but not afterwards.
     */
    [[nodiscard]] controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    /**
     * Specify which hardware state interfaces are used.
     *
     * This method is called in the `active` or `inactive` state. This means 
     * that the configuration can be changed during the `on_configure` state 
     * but not afterwards.
     */
    [[nodiscard]] controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    /**
     * Declare all parameters.
     */
    CallbackReturn on_init() override;

    /**
     * Perform tasks that must be performed once during the node life time.

     * The tasks to perform can be e.g. obtaining parameter values and
     * setting up topic publications/subscriptions that do not change.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
        override;

    /**
     * Clear all state and return the node to a functionally
     * equivalent state as when first created.
     *
     * This should revert all changes during the configure
     * transition, e.g. destroying all managed
     * objects transition.
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
        override;

    /**
     * Do any final preparations to start executing.
     *
     * This may include acquiring resources that are only held while the node is
     * actually active, such as access to hardware and setting up real-time
     * buffers.
     *
     * Ideally, no preparation that requires significant time (such as
     * lengthy hardware initialisation) should be performed in this callback.
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
        override;

    /**
     * Do any cleanup after executing is stopped.
     *
     * This should reverse all changes during the activate transition.
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
        override;

    /**
     * Perform a single controller update step.
     *
     * This function should ONLY execute real-time capable code.
     *
     * @param time Current time.
     * @param period Time since the last controller update step.
     */
    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    rclcpp::Subscription<CmdMsg>::SharedPtr subscriber_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>> rt_command_ptr_;

    using RtStatePublisher = realtime_tools::RealtimePublisher<StateMsg>;
    rclcpp_lifecycle::LifecyclePublisher<StateMsg>::SharedPtr publisher_;
    std::unique_ptr<RtStatePublisher> realtime_publisher_;


    // Parameter values
    Vector7d k_gains_;
    Vector7d d_gains_;
    double velocity_filter_alpha_gain_;

    // Internal variables (controller state while active)
    Vector7d filtered_joint_velocities_;
  };
}

#endif // ROS2_CONTROLLER_ALL_PKG_CUSTOM_CONTROLLER