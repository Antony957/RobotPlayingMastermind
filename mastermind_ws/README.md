# Mastermind

## Building
We need to build `mastermind_interfaces` and `mastermind` separately. `mastermind_interfaces` has the custom messages.

In one terminal pane, run
```bash
cd /root/workspaces/mastermind_ws &&
colcon build --symlink-install --packages-select mastermind_interfaces &&
colcon build --symlink-install --packages-select mastermind
```

## Running
Open a total of 5 terminal panes. To start, in each pane: 
```bash
cd /root/workspaces/mastermind_ws &&
source /opt/ros/$ROS_DISTRO/setup.bash &&
source ./install/setup.bash
```

### Pane 1: Start Gazebo
For simulation:
```bash
ros2 launch kortex_bringup kortex_sim_control.launch.py sim_gazebo:=true robot_type:=gen3_lite gripper:=gen3_lite_2f robot_name:=gen3_lite dof:=6 use_sim_time:=true launch_rviz:=false robot_controller:=joint_trajectory_controller
```

For physical implementation:
```bash
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10 gripper:=gen3_lite_2f launch_rviz:=false
```

### Pane 2: Start RViz
For simulation:
```bash
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true
```

For physical implementation:

```bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10 launch_driver:=false launch_rviz:=true
```

### Pane 3: Spawn world, then run game
To spawn world (make sure we're in `/root/workspaces/mastermind_ws`):
```bash
./src/mastermind/world/spawn_blocks.sh
```
This will take a while.

When done, start game with (remember to change secret):
```bash
ros2 run mastermind main --ros-args -p secret:="red blue green yellow"
```

### Pane 4: Start player 2

#### Human mode
Start player 2 in human mode with
```bash
ros2 run mastermind player2 --ros-args -p mode:=human
```

Then we'll see
```
Enter your guess (e.g. 'red blue black green'):

```

Enter valid guesses as
```
red blue black green
```

and so on.

#### AI mode
Start player 2 in human mode with
```bash
ros2 run mastermind player2 --ros-args -p mode:=ai
```
