"""Runtime contract checks for generated packages in a ROS workspace."""

import argparse
import importlib
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PointStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rosgraph_msgs.msg import Clock as ClockMessage


def level_number(value):
    return int.from_bytes(value, "little") if isinstance(value, (bytes, bytearray)) else int(value)


def check_package(args):
    rclpy.init()
    observer = Node("diagnostics_test", parameter_overrides=[Parameter("use_sim_time", value=args.sim_time)])
    steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
    if args.sim_time:
        clock_publisher = observer.create_publisher(ClockMessage, "/clock", 10)
        started = time.monotonic()

        def advance_clock():
            message = ClockMessage()
            message.clock = rclpy.time.Time(seconds=100.0 + time.monotonic() - started).to_msg()
            clock_publisher.publish(message)

        observer.create_timer(0.02, advance_clock, clock=steady_clock)
    fqn = f"/{args.namespace}/{args.package}".replace("//", "/")
    received = []
    default_topic = []
    outputs = []
    observer.create_subscription(DiagnosticArray, "/test_diagnostics", received.append, 10)
    observer.create_subscription(DiagnosticArray, "/diagnostics", default_topic.append, 10)
    observer.create_subscription(PointStamped, f"{fqn}/output", outputs.append, 10)
    publisher = observer.create_publisher(PointStamped, f"{fqn}/input", 10)
    emitting = False
    old_stamp = False

    def publish():
        if emitting:
            msg = PointStamped()
            msg.header.stamp = (observer.get_clock().now() - rclpy.duration.Duration(
                seconds=2.0 if old_stamp else 0.0)).to_msg()
            publisher.publish(msg)

    publish_timer = observer.create_timer(0.05, publish, clock=steady_clock)

    def wait_for(predicate, description, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.05)
            if process.poll() is not None:
                raise AssertionError(f"Node exited during {description}; see process log")
            if predicate():
                return
        recent = [[(s.name, level_number(s.level), s.message) for s in m.status] for m in received[-2:]]
        raise AssertionError(f"Timed out waiting for {description}: {recent}")

    def health():
        for message in reversed(received):
            for status in message.status:
                if status.name == f"{fqn}: health" and status.message != "Node starting up":
                    return status
        return None

    def expect(level, description):
        received.clear()
        wait_for(lambda: health() is not None and level_number(health().level) == level, description)

    def transition(value):
        client = observer.create_client(ChangeState, f"{fqn}/change_state")
        assert client.wait_for_service(timeout_sec=5.0), "Missing lifecycle service"
        request = ChangeState.Request()
        request.transition.id = value
        future = client.call_async(request)
        wait_for(future.done, "lifecycle transition")
        assert future.result().success, f"Transition {value} failed"
        observer.destroy_client(client)

    config = '''/**:
  ros__parameters:
    diagnostic_updater:
      period: 0.1
      topic_diagnostic:
        min_frequency: 16.0
        max_frequency: 24.0
        timeout: 0.6
      diagnosed_publisher:
        min_frequency: 16.0
        max_frequency: 24.0
        timeout: 0.6
'''
    if args.disabled_topics:
        config = config.replace("      topic_diagnostic:\n", "      topic_diagnostic:\n        enabled: false\n")
        config = config.replace("      diagnosed_publisher:\n", "      diagnosed_publisher:\n        enabled: false\n")
    with tempfile.TemporaryDirectory() as temp:
        params = Path(temp) / "params.yml"
        params.write_text(config)
        command = ["ros2", "launch", args.package,
                   f"{args.package}_launch.{args.launch_file_type}",
                   f"params:={params}",
                   "diagnostics_topic:=/test_diagnostics"]
        if args.namespace:
            command.append(f"namespace:={args.namespace}")
        if args.sim_time:
            command.append("use_sim_time:=true")
        if args.num_threads is not None:
            command.append(f"num_threads:={args.num_threads}")
        log_path = Path(temp) / "node.log"
        with log_path.open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                if args.disabled_topics or (args.no_subscriber and args.no_publisher):
                    expect(3, "unassessed health")
                    assert health().message == "No assessments registered"
                else:
                    expect(2, "never-started stream timeout")
                    if not args.no_subscriber:
                        emitting = True
                        expect(0, "valid stream")
                        if not args.no_publisher:
                            assert outputs, "Application output is missing"
                        old_stamp = True
                        expect(2, "invalid timestamps")
                        old_stamp = False
                        expect(0, "timestamp recovery")
                        emitting = False
                        expect(2, "stopped input watchdog")
                        wait_for(lambda: any(v.key == "inputs" and v.value.startswith("ERROR:")
                                             for v in health().values), "bounded input watchdog")
                        emitting = True
                        expect(0, "stream recovery")

                client = AsyncParameterClient(observer, fqn)
                assert client.wait_for_services(timeout_sec=5.0)
                future = client.set_parameters([Parameter("diagnostic_updater.period", value=0.2)])
                wait_for(future.done, "read-only parameter rejection")
                assert not future.result().results[0].successful

                if args.num_threads is not None:
                    future = client.get_parameters(["num_threads"])
                    wait_for(future.done, "executor thread count")
                    assert future.result().values[0].integer_value == args.num_threads

                if args.lifecycle:
                    if args.stress_lifecycle:
                        publish_timer.timer_period_ns = 1_000_000
                    for _ in range(20 if args.stress_lifecycle else 2):
                        transition(Transition.TRANSITION_DEACTIVATE)
                        expect(3, "inactive lifecycle health")
                        assert "node not active (inactive)" in health().message
                        transition(Transition.TRANSITION_CLEANUP)
                        expect(3, "unconfigured lifecycle health")
                        transition(Transition.TRANSITION_CONFIGURE)
                        expect(3, "reconfigured inactive health")
                        transition(Transition.TRANSITION_ACTIVATE)
                        if not args.stress_lifecycle:
                            expect(0, "reactivated stream")
                    if args.stress_lifecycle:
                        publish_timer.timer_period_ns = 50_000_000
                        expect(0, "stream recovery after concurrent lifecycle transitions")
                    assert "publisher is not activated" not in log_path.read_text(), \
                        "A data callback attempted to publish while the lifecycle publisher was inactive"

                prefix = f"{fqn}: "
                expected = {prefix + "health"}
                if not args.no_subscriber and not args.disabled_topics:
                    expected.add(prefix + f"{fqn}/input topic status")
                if not args.no_publisher and not args.disabled_topics:
                    expected.add(prefix + f"{fqn}/output topic status")
                cycles = [m for m in received if any(s.name == prefix + "health" and
                          s.message != "Node starting up" for s in m.status)]
                assert cycles, "No periodic health cycle"
                for message in cycles:
                    names = [s.name for s in message.status]
                    assert set(names) == expected, (names, expected)
                    assert len(names) == len(expected), "Duplicate diagnostic task"
                    assert names[-1] == prefix + "health", "Health must run last"
                    overall = message.status[-1]
                    summaries = {v.key: v.value for v in overall.values}
                    for status in message.status[:-1]:
                        key = "topic.input" if "/input topic status" in status.name else "topic.output"
                        assert summaries[key].startswith(("OK", "WARN", "ERROR", "STALE")[level_number(status.level)] + ":")
                assert not default_topic, "Diagnostics ignored the launch remapping"
            except BaseException:
                print("".join(log_path.read_text().splitlines(keepends=True)[-100:]))
                raise
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                observer.destroy_node()
                rclpy.shutdown()

    if args.python_checks:
        check_application_checks(args.package)
    if args.validate_parameters:
        invalid = [("diagnostic_updater.period", "0.0"),
                   ("diagnostic_updater.topic_diagnostic.min_frequency", "0.0"),
                   ("diagnostic_updater.topic_diagnostic.timeout", "-1.0"),
                   ("diagnostic_updater.topic_diagnostic.max_frequency", "0.1")]
        for name, value in invalid:
            result = subprocess.run(["ros2", "run", args.package, args.package, "--ros-args",
                                     "-p", f"{name}:={value}"], capture_output=True, text=True, timeout=10)
            assert result.returncode != 0, f"Invalid {name} was accepted"
    print(f"Diagnostics contract passed: {args.package} ({args.launch_file_type}, /{args.namespace})")


def check_application_checks(package):
    module = importlib.import_module(f"{package}.{package}")
    cls = next(value for value in vars(module).values()
               if isinstance(value, type) and issubclass(value, Node) and value is not Node)
    rclpy.init()
    node = cls()
    try:
        node.set_diagnostic_check("initialization", module.DiagnosticCheck(DiagnosticStatus.ERROR, "Backend failed"))
        node.set_diagnostic_check("optional", module.DiagnosticCheck(DiagnosticStatus.STALE, "Unavailable"))
        for _ in range(2):
            result = node.health(module.DiagnosticStatusWrapper())
            assert level_number(result.level) == 2, "ERROR must dominate STALE and persist"
        node.set_diagnostic_check("initialization", module.DiagnosticCheck(0, "Backend ready", {"ready": True}))
        assert level_number(node.health(module.DiagnosticStatusWrapper()).level) == 3
        node.set_diagnostic_check("optional", module.DiagnosticCheck(1, "Degraded"))
        assert level_number(node.health(module.DiagnosticStatusWrapper()).level) == 1
        node.set_diagnostic_check("optional", module.DiagnosticCheck(0, "Available"))
        result = node.health(module.DiagnosticStatusWrapper())
        assert level_number(result.level) == 0
        assert result.message == "All checks OK"
        assert all(isinstance(v.value, str) for v in result.values)
        keys = [v.key for v in result.values]
        assert keys[:2] == ["initialization", "optional"]
        assert keys[2:] == ["initialization.ready"]
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package")
    parser.add_argument("--launch-file-type", default="py", choices=("py", "xml", "yml"))
    parser.add_argument("--namespace", default="test")
    parser.add_argument("--lifecycle", action="store_true")
    parser.add_argument("--stress-lifecycle", action="store_true")
    parser.add_argument("--num-threads", type=int)
    parser.add_argument("--no-subscriber", action="store_true")
    parser.add_argument("--no-publisher", action="store_true")
    parser.add_argument("--python-checks", action="store_true")
    parser.add_argument("--sim-time", action="store_true")
    parser.add_argument("--disabled-topics", action="store_true")
    parser.add_argument("--validate-parameters", action="store_true")
    check_package(parser.parse_args())
