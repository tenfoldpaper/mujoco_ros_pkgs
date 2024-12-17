# MuJoCo ROS

Tools that combine the MuJoCo simulator with ROS. Meant to recreate a base port of [gazebo\_ros\_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) for MuJoCo.

This is a ROS software Project that wraps the [MuJoCo physics engine](https://mujoco.org/) into a ROS package.
It is an extension of the MuJoCo [simulate](https://github.com/deepmind/mujoco/blob/3.2.0/sample/simulate.cc) program, with ROS integration and the possibility to load plugins via pluginlib.

### ROS Versions

This project is mainly built for Ubuntu Focal with ROS Noetic. But we are working on adaptations for more recent Ubuntu systems with ROS One and Humble (ROS 2).

#### ROS2 Humble Development
This branch is for porting the project into ROS2 Humble.

In the current state, there is a headless MuJoCo simulation integrated with a working, but pretty basic, `ros2_control` plugin.
You can clone this branch and build it normally, as all the ROS1 packages have `COLCON_IGNORE` in them, which prevents them from being built by `colcon`.
It's currently comprised of 3 packages:
- `mujoco_ros2_base` with the parent `MujocoPlugin` class in `plugin_utils.h`, and a main node (`mujoco_ros2_main.cpp`) that loads the `ros2_control` plugin using a matching `pendulum.xml/urdf` file found in `sample_xml`.
- `mujoco_ros2_control` plugin package that implements a `ros2_control` plugin using `MujocoPlugin` class, mainly taken from gazebo's ros2_control, and provides an interface class for the SystemInterfaces.
- `mujoco_ros2_control_system` package that implements a very basic SystemInterface using the interface class from above, and is loaded in by `mujoco_ros2_control`. It has a working initialization, `read` and `write` functions. An example controller much in the style of Gazebo's demos is in the works.

Basically, in the current structure, `mujoco_ros2_base` loads `mujoco_ros2_control` loads `mujoco_ros2_control_system`.

To try out, install ROS2 and MuJoCo 3.2.0, build the package and then run `ros2 launch mujoco_ros2_base mujoco_ros2_main_launch.py`.

The pendulum can be visualized in RVIZ. You can open up the `pendulum.rviz` file under `mujoco_ros2_base/config/rviz` to see it.

Currently, we use MuJoCo's simulation time (`mjData* d_->time`) to synchronize the controller's read/write loops. This means the main loop's `sleep` function needs to be adjusted, if you want the simulation to run faster.
This is mainly to give users more control over how the simulation and control are executed, and to not be bound by the computer's resources should super short timesteps be required.
This will, of course, cause some issues when using it with other nodes that run dependent on ROS' own time. A feature for changing the source of the clock is planned.

### Continuous Integration

service    | Noetic / One | Humble (coming soon)
---------- | :-----: | :----:
GitHub | [![Format](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/format.yaml/badge.svg?branch=noetic-devel)](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/format.yaml?query=branch%3Anoetic-devel) [![CI](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/ci.yaml/badge.svg?branch=noetic-devel)](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/ci.yaml?query=branch%3Anoetic-devel) | - |
CodeCov | [![codecov](https://codecov.io/gh/ubi-agni/mujoco_ros_pkgs/branch/noetic-devel/graph/badge.svg?token=W7uHKcY0ly)](https://codecov.io/gh/ubi-agni/mujoco_ros_pkgs) | - |


# Build Instructions
1. Make sure MuJoCo is installed (the current build uses version 3.2.0) and runs on your machine.
2. Create a new ROS workspace or include this repository into an existing workspace.
3. Before building, make sure that your compiler knows where to find the MuJoCo library, e.g. by running
```bash
export MUJOCO_DIR=PATH/TO/MUJOCO/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
export LIBRARY_PATH=$LIBRARY_PATH:$MUJOCO_DIR/lib
```
where `PATH/TO/MUJOCO/DIR` is `~/.mujoco/mujoco-3.2.0` if you used the recommended location to install mujoco (if downloaded as tarball). If you built MuJoCo from source and the install path is known to catkin, you can skip this step.

4. Build with `catkin_build`, `catkin b` or `colcon build`.
5. Source your workspace and try `roslaunch mujoco_ros launch_server.launch use_sim_time:=true` to test if it runs.

> **Warning**
> To prevent action servers ignoring actions for a limited time after resetting the simulation, until https://github.com/ros/actionlib/pull/203 is merged, you need to build the PR branch and any packages implementing action servers (like MoveIt) yourself.


### Plugin Examples

As an example for extended functionality through plugins, take a look at [mujoco_ros_control](https://github.com/ubi-agni/mujoco_ros_pkgs/tree/noetic-devel/mujoco_ros_control), [mujoco_screw_plugin](https://github.com/ubi-agni/mujoco_screw_plugin), [mujoco_contact_surfaces](https://github.com/ubi-agni/mujoco_contact_surfaces) or [mujoco_ros_sensors](https://github.com/ubi-agni/mujoco_ros_pkgs/tree/noetic-devel/mujoco_ros_sensors).

We provide some code examples in our [demo repository](https://github.com/ubi-agni/mujoco_ros_demos)


### Documentation

We are currently working on setting up more detailed documentation including tutorials and guides. The current prototype can be found [here](davidpl1.github.io/mujoco_ros_pkgs) (though note that this will migrate once its ready for an initial proper release).

Some more structural and configuration info, which is not yet included in the documentation, can be found [here](./mujoco_ros/README.md).

# Licensing

This work is licensed under the BSD 3-Clause License (see LICENSE).
It is built on top of MuJoCo, which was released under an Apache 2.0 License. For the original MuJoCo and further third party licenses, see [THIRD_PARTY_NOTICES](./THIRD_PARTY_NOTICES).
