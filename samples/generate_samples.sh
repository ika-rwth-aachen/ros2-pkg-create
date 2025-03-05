#!/bin/bash

script_dir=$(dirname $0)
template_dir="$script_dir/.."

copier copy --trust --defaults --overwrite --vcs-ref=HEAD \
            -d template=ros2_cpp_pkg \
            -d package_name=ros2_cpp_pkg \
            -d node_name=ros2_cpp_node \
            -d is_component=false \
            -d is_lifecycle=false \
            -d has_launch_file=true \
            -d has_params=true \
            -d has_subscriber=true \
            -d has_publisher=true \
            -d has_service_server=false \
            -d has_action_server=false \
            -d has_timer=false \
            -d has_docker_ros=true \
            $template_dir $script_dir

copier copy --trust --defaults --overwrite --vcs-ref=HEAD \
            -d template=ros2_cpp_pkg \
            -d package_name=ros2_cpp_component_pkg \
            -d node_name=ros2_cpp_node \
            -d is_component=true \
            -d is_lifecycle=false \
            -d has_launch_file=true \
            -d has_params=true \
            -d has_subscriber=true \
            -d has_publisher=true \
            -d has_service_server=false \
            -d has_action_server=false \
            -d has_timer=false \
            -d has_docker_ros=true \
            $template_dir $script_dir

copier copy --trust --defaults --overwrite --vcs-ref=HEAD \
            -d template=ros2_cpp_pkg \
            -d package_name=ros2_cpp_lifecycle_pkg \
            -d node_name=ros2_cpp_node \
            -d is_component=false \
            -d is_lifecycle=true \
            -d has_launch_file=true \
            -d has_params=true \
            -d has_subscriber=true \
            -d has_publisher=true \
            -d has_service_server=false \
            -d has_action_server=false \
            -d has_timer=false \
            -d has_docker_ros=true \
            $template_dir $script_dir

copier copy --trust --defaults --overwrite --vcs-ref=HEAD \
            -d template=ros2_cpp_pkg \
            -d package_name=ros2_cpp_all_pkg \
            -d node_name=ros2_cpp_node \
            -d is_component=true \
            -d is_lifecycle=true \
            -d has_launch_file=true \
            -d has_params=true \
            -d has_subscriber=true \
            -d has_publisher=true \
            -d has_service_server=true \
            -d has_action_server=true \
            -d has_timer=true \
            -d has_docker_ros=true \
            $template_dir $script_dir

copier copy --trust --defaults --overwrite --vcs-ref=HEAD \
            -d template=ros2_python_pkg \
            -d package_name=ros2_python_pkg \
            -d node_name=ros2_python_node \
            -d has_launch_file=true \
            -d has_params=true \
            -d has_subscriber=true \
            -d has_publisher=true \
            -d has_service_server=false \
            -d has_action_server=false \
            -d has_timer=false \
            -d has_docker_ros=true \
            $template_dir $script_dir

copier copy --trust --defaults --overwrite --vcs-ref=HEAD \
            -d template=ros2_python_pkg \
            -d package_name=ros2_python_all_pkg \
            -d node_name=ros2_python_node \
            -d has_launch_file=true \
            -d has_params=true \
            -d has_subscriber=true \
            -d has_publisher=true \
            -d has_service_server=true \
            -d has_action_server=true \
            -d has_timer=true \
            -d has_docker_ros=true \
            $template_dir $script_dir

copier copy --trust --defaults --overwrite --vcs-ref=HEAD \
            -d template=ros2_interfaces_pkg \
            -d package_name=ros2_interfaces_pkg \
            $template_dir $script_dir

copier copy --trust --defaults --overwrite --vcs-ref=HEAD \
            -d template=ros2_controller_pkg \
            -d package_name=ros2_controller_all_pkg \
            -d controller_name=custom_controller \
            -d has_launch_file=true \
            -d has_subscriber=true \
            -d has_publisher=true \
            -d has_docker_ros=false \
            $template_dir $script_dir

copier copy --trust --defaults --overwrite --vcs-ref=HEAD \
            -d template=ros2_controller_pkg \
            -d package_name=ros2_controller_no_pub_sub_pkg \
            -d controller_name=custom_controller \
            -d has_launch_file=true \
            -d has_subscriber=false \
            -d has_publisher=false \
            -d has_docker_ros=false \
            $template_dir $script_dir