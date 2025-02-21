## Installation

### Dependencies

Ensure the following dependencies are installed before proceeding:

#### 1. Log4cxx
If not already installed, install Log4cxx:

```bash
sudo apt-get install liblog4cxx-dev
```

#### 2. POCO C++ Libraries
Download and install the [POCO C++ Libraries](https://pocoproject.org/index.html).

#### 3. ROS 2 Humble
Install ROS 2 Humble by following the [official installation guide](https://docs.ros.org/en/humble/Installation.html).

#### 4. MoveIt 2 for ROS 2 Humble
Install MoveIt 2 using the following command:

```bash
sudo apt install ros-humble-moveit
```

#### 5. Required ROS 2 Package
Install the necessary ROS 2 package:

```bash
sudo apt install ros-humble-ur
```

## Create a Workspace

Set up your ROS 2 workspace:

```bash
mkdir ros2_ws
cd ros2_ws
mkdir src
colcon build
```

## Clone the Repositories

Navigate to the `src` directory of your workspace and clone the required repositories:

```bash
git clone -b modern_cmake https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader.git
git clone https://github.com/CNR-STIIMA-IRAS/cnr_logger.git
git clone https://github.com/CNR-STIIMA-IRAS/cnr_param.git
git clone https://github.com/CNR-STIIMA-IRAS/cnr_yaml.git

git clone https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git

git clone -b ros2 https://github.com/JRL-CARI-CNR-UNIBS/moveit_collision_checker.git
git clone -b ros2 https://github.com/JRL-CARI-CNR-UNIBS/cnr_scene_manager.git
git clone -b ros2 https://github.com/CNR-STIIMA-IRAS/cnr_tf_named_object_loader.git

git clone https://github.com/JRL-CARI-CNR-UNIBS/ik_solver.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs.git
git clone -b ros2 https://github.com/JRL-CARI-CNR-UNIBS/ur_ik_solver.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/pose_constraint_path_planning.git
git clone -b ros2 https://github.com/JRL-CARI-CNR-UNIBS/graph_display.git
```

## Configure Colcon Defaults to avoid using all the cpus to compile

Create the `.colcon` folder and a `defaults.yaml` file with the following content:

```bash
mkdir -p ~/.colcon
echo "build:
  parallel-workers: 1" > ~/.colcon/defaults.yaml
```

if colcon uses all the RAM, call ulimit in the shell before compiling, to limit the usage (for example to 4GB):
```bash
ulimit -v 4000000
```

## Build the Workspace

Build the workspace with the following command:

```bash
colcon build --symlink-install --cmake-args -DUSE_ROS1=False
```
## Environment Variables

You can set the required environment variables manually in each terminal session or add them to your `.bashrc`.

```bash
export CNR_PARAM_ROOT_DIRECTORY="/tmp/cnr_param"
```
If not present, you need to create a `default_logger.yaml` file with the following content

```yaml
appenders: ['screen']                             # Mandatory
                                                  # A vector od dimension 1 or 2, where you can select if the output will be streamed to file, to screen or to both
                                                  # (the order in the vector is not important)

levels: ['debug']                                   # Optional
                                                  # A vector where you can select the verbosity level for each appender.
                                                  # If not present, or if the size of the vector is different from the dimension of the appenders,
                                                  # the default value is superimposed.
                                                  # Default: 'debug' for all the appenders
```
You need to point to the `default_logger.yaml` file in an environment variable

```bash
export IK_SOLVER_LOGGER_CONFIG_PATH=~/ros2_ws/install/ik_solver_test/share/ik_solver_test/config/default_logger.yaml
```

## Execution

Use the following commands to launch the necessary components (source ros2_ws/install/setup.bash in each terminal before start, or automatically source it in your `.bashrc` file):

1. **Terminal 1**: Launch the UR robot driver with fake hardware:
   ```bash
   ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=127.0.0.1 activate_joint_controller:=true
   ```

2. **Terminal 2**: Launch MoveIt configuration for the UR robot:
   ```bash
   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e use_fake_hardware:=true launch_rviz:=true
   ```

   here you can add a Marker (add-> marker) with topic /marker_visualization_topic to see the path.

3. **Terminal 3**: Launch the pose constraints planner:
   ```bash
   ros2 launch pose_constraints_planner test_solver.launch.yaml
   ```
