## Installation

Install the required ROS 2 package:

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

git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
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

## Execution

Use the following commands to launch the necessary components:

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
