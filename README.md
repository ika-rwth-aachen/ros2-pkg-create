# *ros2-pkg-create* – Powerful ROS 2 Package Generator

<p align="center">
  <img src="https://img.shields.io/github/license/ika-rwth-aachen/ros2-pkg-create"/>
  <a href="https://github.com/ika-rwth-aachen/ros2-pkg-create/actions/workflows/generate-and-test.yml"><img src="https://github.com/ika-rwth-aachen/ros2-pkg-create/actions/workflows/generate-and-test.yml/badge.svg"/></a>
  <a href="https://pypi.org/project/ros2-pkg-create/"><img src="https://img.shields.io/pypi/v/ros2-pkg-create?label=PyPI"/></a>
  <a href="https://pypi.org/project/ros2-pkg-create/"><img src="https://img.shields.io/pypi/dm/ros2-pkg-create?color=blue&label=PyPI%20downloads"/></a>
</p>

*ros2-pkg-create* is an interactive CLI tool for quickly generating ROS 2 packages from basic pub/sub nodes to complex lifecycle components. It is meant to replace the official [`ros2 pkg create`](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#create-a-package) command.

- [Quick Demo](#quick-demo)
- [Installation](#installation)
- [Templates \& Features](#templates--features)
- [Usage](#usage)
- [Acknowledgements](#acknowledgements)

> [!IMPORTANT]  
> This repository is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).  
> **ROS is the backbone** of many research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.  
> If you would like to learn more about how we can support your advanced driver assistance and automated driving efforts, feel free to reach out to us!  
> :email: ***opensource@ika.rwth-aachen.de***


## Quick Demo

```bash
pip install ros2-pkg-create
ros2-pkg-create --template ros2_cpp_pkg .
```

<img src="https://github.com/ika-rwth-aachen/ros2-pkg-create/raw/main/assets/cli.png" width=600>


## Installation

```bash
pip install ros2-pkg-create

# (optional) bash auto-completion
activate-global-python-argcomplete
eval "$(register-python-argcomplete ros2-pkg-create)"
```

> [!WARNING]  
> Outside of a virtual environment, *pip* may default to a user-site installation of executables to `~/.local/bin`, which may not be present in your shell's `PATH`.  If running `ros2-pkg-create` errors with `ros2-pkg-create: command not found`, add the directory to your path. [*(More information)*](https://packaging.python.org/en/latest/tutorials/installing-packages/#installing-to-the-user-site)
> ```bash
> echo "export PATH=\$HOME/.local/bin:\$PATH" >> ~/.bashrc
> source ~/.bashrc
> ```


## Templates & Features

*ros2-pkg-create* provides multiple templates, each covering a different questionnaire for generating all the components you need. See below for the list of supported features and questionnarie options. Note that all options can also be passed directly to the command, bypassing the interactive questionnaire (see [Usage](#usage)).

- [C++ Package](#c-package---template-ros2_cpp_pkg)
- [Python Package](#python-package---template-ros2_python_pkg)
- [Interfaces Package](#interfaces-package---template-ros2_interfaces_pkg)

### C++ Package (`--template ros2_cpp_pkg`)

**Supported Features:** publisher, subscriber, parameter loading, launch file, service server, action server, timer callback, component, lifecycle node, executor, docker-ros

<details>
<summary>Questionnaire</summary>

- Package name
- Description
- Maintainer | Maintainer email
- Author | Author email
- License
- Node name
- Class name of node
- Make it a component?
- Make it a lifecycle node?
- Type of Executor
- Add a launch file? | Type of launch file
- Add parameter loading?
- Add a subscriber?
- Add a publisher?
- Add a service server?
- Add an action server?
- Add a timer callback?
- Add the docker-ros CI integration?
</details>

### Python Package (`--template ros2_python_pkg`)

**Supported Features:** publisher, subscriber, parameter loading, launch file, service server, action server, timer callback, docker-ros

<details>
<summary>Questionnaire</summary>

- Package name
- Description
- Maintainer | Maintainer email
- Author | Author email
- License
- Node name
- Class name of node
- Add a launch file? | Type of launch file
- Add parameter loading?
- Add a subscriber?
- Add a publisher?
- Add a service server?
- Add an action server?
- Add a timer callback?
- Add the docker-ros CI integration?
</details>

### Interfaces Package (`--template ros2_interfaces_pkg`)

**Supported Features:** message, service, action

<details>
<summary>Questionnaire</summary>

- Package name
- Description
- Maintainer | Maintainer email
- Author | Author email
- License
- Interfaces types
- Message name
- Service name
- Action name
- Add the docker-ros CI integration?
</details>


## Usage

```
usage: ros2-pkg-create [-h] [--defaults] [--use-local-templates] --template {ros2_interfaces_pkg,ros2_cpp_pkg,ros2_python_pkg} [--package-name PACKAGE_NAME] [--description DESCRIPTION] [--maintainer MAINTAINER] [--maintainer-email MAINTAINER_EMAIL] [--author AUTHOR]
                       [--author-email AUTHOR_EMAIL] [--license {Apache-2.0,BSL-1.0,BSD-2.0,BSD-2-Clause,BSD-3-Clause,GPL-3.0-only,LGPL-2.1-only,LGPL-3.0-only,MIT,MIT-0}] [--node-name NODE_NAME] [--node-class-name NODE_CLASS_NAME] [--is-component] [--no-is-component] [--is-lifecycle]
                       [--no-is-lifecycle] [--executor-type {SingleThreadedExecutor,MultiThreadedExecutor,StaticSingleThreadedExecutor}] [--has-launch-file] [--no-has-launch-file] [--launch-file-type {xml,py,yml}] [--has-params] [--no-has-params] [--has-subscriber] [--no-has-subscriber]
                       [--has-publisher] [--no-has-publisher] [--has-service-server] [--no-has-service-server] [--has-action-server] [--no-has-action-server] [--has-timer] [--no-has-timer] [--auto-shutdown] [--no-auto-shutdown] [--interface-types {Message,Service,Action}]
                       [--msg-name MSG_NAME] [--srv-name SRV_NAME] [--action-name ACTION_NAME] [--has-docker-ros] [--docker-ros-type {github,gitlab}] [--version]
                       destination

Creates a ROS 2 package from templates

positional arguments:
  destination           Destination directory

options:
  -h, --help            show this help message and exit
  --defaults            Use defaults for all options
  --use-local-templates
                        Use locally installed templates instead of remotely pulling most recent ones
  --template {ros2_interfaces_pkg,ros2_cpp_pkg,ros2_python_pkg}
                        Template
  --package-name PACKAGE_NAME
                        Package name
  --description DESCRIPTION
                        Description
  --maintainer MAINTAINER
                        Maintainer
  --maintainer-email MAINTAINER_EMAIL
                        Maintainer email
  --author AUTHOR       Author
  --author-email AUTHOR_EMAIL
                        Author email
  --license {Apache-2.0,BSL-1.0,BSD-2.0,BSD-2-Clause,BSD-3-Clause,GPL-3.0-only,LGPL-2.1-only,LGPL-3.0-only,MIT,MIT-0}
                        License
  --node-name NODE_NAME
                        Node name
  --node-class-name NODE_CLASS_NAME
                        Class name of node
  --is-component        Make it a component?
  --no-is-component
  --is-lifecycle        Make it a lifecycle node?
  --no-is-lifecycle
  --executor-type {SingleThreadedExecutor,MultiThreadedExecutor,StaticSingleThreadedExecutor}
                        Type of Executor
  --has-launch-file     Add a launch file?
  --no-has-launch-file
  --launch-file-type {xml,py,yml}
                        Type of launch file
  --has-params          Add parameter loading
  --no-has-params
  --has-subscriber      Add a subscriber?
  --no-has-subscriber
  --has-publisher       Add a publisher?
  --no-has-publisher
  --has-service-server  Add a service server?
  --no-has-service-server
  --has-action-server   Add an action server?
  --no-has-action-server
  --has-timer           Add a timer callback?
  --no-has-timer
  --auto-shutdown       Automatically shutdown the node after launch (useful in CI/CD)?
  --no-auto-shutdown
  --interface-types {Message,Service,Action}
                        Interfaces types
  --msg-name MSG_NAME   Message name
  --srv-name SRV_NAME   Service name
  --action-name ACTION_NAME
                        Action name
  --has-docker-ros      Add the docker-ros CI integration?
  --docker-ros-type {github,gitlab}
                        Type of docker-ros CI integration
  --version             show program's version number and exit
```

## Acknowledgements

This work is accomplished within the projects [6GEM](https://6gem.de/en/) (FKZ 16KISK036K) and [autotech.agil](https://www.autotechagil.de/) (FKZ 01IS22088A). We acknowledge the financial support for the projects by the *Federal Ministry of Education and Research of Germany (BMBF)*.
