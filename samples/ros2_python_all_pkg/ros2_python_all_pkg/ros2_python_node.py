import time
from typing import Any, Optional, Union

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
import rclpy.exceptions
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult)
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
from ros2_python_all_pkg_interfaces.action import Fibonacci


class Ros2PythonNode(Node):

    def __init__(self):
        """Constructor"""
        super().__init__("ros2_python_node")

        self.subscriber = None

        self.publisher = None

        self.auto_reconfigurable_params: list[str] = []
        self.param = self.declare_and_load_parameter(name="param",
                                                    param_type=rclpy.Parameter.Type.DOUBLE,
                                                    description="TODO",
                                                    default=1.0,
                                                    from_value=0.0,
                                                    to_value=10.0,
                                                    step_value=0.1)

        self.setup()

    def declare_and_load_parameter(self,
        name: str,
        param_type: rclpy.Parameter.Type,
        description: str,
        default: Optional[Any] = None,
        add_to_auto_reconfigurable_params: bool = True,
        is_required: bool = False,
        read_only: bool = False,
        from_value: Optional[Union[int, float]] = None,
        to_value: Optional[Union[int, float]] = None,
        step_value: Optional[Union[int, float]] = None,
        additional_constraints: str = "") -> Any:
        """Declares and loads a ROS parameter

        Args:
            name (str): name
            param_type (rclpy.Parameter.Type): parameter type
            description (str): description
            default (Optional[Any], optional): default value
            add_to_auto_reconfigurable_params (bool, optional): enable reconfiguration of parameter
            is_required (bool, optional): whether failure to load parameter will stop node
            read_only (bool, optional): set parameter to read-only
            from_value (Optional[Union[int, float]], optional): parameter range minimum
            to_value (Optional[Union[int, float]], optional): parameter range maximum
            step_value (Optional[Union[int, float]], optional): parameter range step
            additional_constraints (str, optional): additional constraints description

        Returns:
            Any: parameter value
        """

        # declare parameter
        param_desc = ParameterDescriptor()
        param_desc.description = description
        param_desc.additional_constraints = additional_constraints
        param_desc.read_only = read_only
        if from_value is not None and to_value is not None:
            if param_type == rclpy.Parameter.Type.INTEGER:
                range = IntegerRange(from_value=from_value, to_value=to_value)
                if step_value is not None:
                    range.step = step_value
                param_desc.integer_range = [range]
            elif param_type == rclpy.Parameter.Type.DOUBLE:
                range = FloatingPointRange(from_value=from_value, to_value=to_value)
                if step_value is not None:
                    range.step = step_value
                param_desc.floating_point_range = [range]
            else:
                self.get_logger().warn(f"Parameter type of parameter '{name}' does not support specifying a range")
        self.declare_parameter(name, param_type, param_desc)

        # load parameter
        try:
            param = self.get_parameter(name).value
            self.get_logger().info(f"Loaded parameter '{name}': {param}")
        except rclpy.exceptions.ParameterUninitializedException:
            if is_required:
                self.get_logger().fatal(f"Missing required parameter '{name}', exiting")
                raise SystemExit(1)
            else:
                self.get_logger().warn(f"Missing parameter '{name}', using default value: {default}")
                param = default
                self.set_parameters([rclpy.Parameter(name=name, value=param)])

        # add parameter to auto-reconfigurable parameters
        if add_to_auto_reconfigurable_params:
            self.auto_reconfigurable_params.append(name)

        return param

    def parameters_callback(self,
                           parameters: list[rclpy.Parameter]) -> SetParametersResult:
        """Handles reconfiguration when a parameter value is changed

        Args:
            parameters (list[rclpy.Parameter]): parameters

        Returns:
            SetParametersResult: parameter change result
        """

        for param in parameters:
            if param.name in self.auto_reconfigurable_params:
                setattr(self, param.name, param.value)
                self.get_logger().info(f"Reconfigured parameter '{param.name}' to: {param.value}")

        result = SetParametersResult()
        result.successful = True

        return result

    def setup(self):
        """Sets up subscribers, publishers, etc. to configure the node"""

        # callback for dynamic parameter configuration
        self.add_on_set_parameters_callback(self.parameters_callback)

        # subscriber for handling incoming messages
        self.subscriber = self.create_subscription(Int32,
                                                   "~/input",
                                                   self.topic_callback,
                                                   qos_profile=10)
        self.get_logger().info(f"Subscribed to '{self.subscriber.topic_name}'")

        # publisher for publishing outgoing messages
        self.publisher = self.create_publisher(Int32,
                                               "~/output",
                                               qos_profile=10)
        self.get_logger().info(f"Publishing to '{self.publisher.topic_name}'")

        # service server for handling service calls
        self.service_server = self.create_service(SetBool, "~/service", self.service_callback)

        # action server for handling action goal requests
        self.action_server = ActionServer(
            self,
            Fibonacci,
            "~/action",
            execute_callback=self.action_execute,
            goal_callback=self.action_handle_goal,
            cancel_callback=self.action_handle_cancel,
            handle_accepted_callback=self.action_handle_accepted
        )

        # timer for repeatedly invoking a callback to publish messages
        self.timer = self.create_timer(1.0, self.timer_callback)

    def topic_callback(self, msg: Int32):
        """Processes messages received by a subscriber

        Args:
            msg (Int32): message
        """

        self.get_logger().info(f"Message received: '{msg.data}'")

    def service_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Processes service requests

        Args:
            request (SetBool.Request): service request
            response (SetBool.Response): service response

        Returns:
            SetBool.Response: service response
        """

        self.get_logger().info("Received service request")
        response.success = True

        return response

    def action_handle_goal(self, goal: Fibonacci.Goal) -> GoalResponse:
        """Processes action goal requests

        Args:
            goal (Fibonacci.Goal): action goal

        Returns:
            GoalResponse: goal response
        """

        self.get_logger().info("Received action goal request")

        return GoalResponse.ACCEPT

    def action_handle_cancel(self, goal: Fibonacci.Goal) -> CancelResponse:
        """Processes action cancel requests

        Args:
            goal (Fibonacci.Goal): action goal

        Returns:
            CancelResponse: cancel response
        """

        self.get_logger().info("Received request to cancel action goal")

        return CancelResponse.ACCEPT

    def action_handle_accepted(self, goal: Fibonacci.Goal):
        """Processes accepted action goal requests

        Args:
            goal (Fibonacci.Goal): action goal
        """

        # execute action in a separate thread to avoid blocking
        goal.execute()

    async def action_execute(self, goal: Fibonacci.Goal) -> Fibonacci.Result:
        """Executes an action

        Args:
            goal (Fibonacci.Goal): action goal

        Returns:
            Fibonacci.Result: action goal result
        """

        self.get_logger().info("Executing action goal")

        # define a sleeping rate between computing individual Fibonacci numbers
        loop_rate = 1

        # create the action feedback and result
        feedback = Fibonacci.Feedback()
        result = Fibonacci.Result()

        # initialize the Fibonacci sequence
        feedback.partial_sequence = [0, 1]

        # compute the Fibonacci sequence up to the requested order n
        for i in range(1, goal.request.order):

            # cancel, if requested
            if goal.is_cancel_requested:
                result.sequence = feedback.partial_sequence
                goal.canceled()
                self.get_logger().info("Action goal canceled")
                return result

            # compute the next Fibonacci number
            feedback.partial_sequence.append(feedback.partial_sequence[i] + feedback.partial_sequence[i-1])

            # publish the current sequence as action feedback
            goal.publish_feedback(feedback)
            self.get_logger().info("Publishing action feedback")

            # sleep before computing the next Fibonacci number
            time.sleep(loop_rate)

        # finish by publishing the action result
        if rclpy.ok():
            result.sequence = feedback.partial_sequence
            goal.succeed()
            self.get_logger().info("Goal succeeded")

        return result

    def timer_callback(self):
        """Processes timer triggers
        """

        self.get_logger().info("Timer triggered")


def main():

    rclpy.init()
    node = Ros2PythonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
