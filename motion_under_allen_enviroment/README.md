1.How to run my solution
A.Build & source
cd ~/workspaces/jz499_robotics_fall2025/lab06/ros2_ws
colcon build --symlink-install
source install/setup.bash
B.Launch Gazebo + MoveIt
ros2 launch kortex_bringup kortex_sim_control.launch.py \
  	sim_gazebo:=true \
  	robot_type:=gen3_lite \
  	gripper:=gen3_lite_2f \
  	robot_name:=gen3_lite \
 	 dof:=6 \
  	use_sim_time:=true \
 	 launch_rviz:=false \
  	robot_controller:=joint_trajectory_controller
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true
C.spawning objects
cd/workspaces/jz499_robotics_fall2025/lab06/ros2_ws/src/mems-toolkit/lab06_files
./spawn_blocks.sh
d.adding the scene
ros2 run lab06_moveit pick_and_place --ros-args -p task:=add_scene
e.running command to stack 3 blocks
ros2 run lab06_moveit pick_and_place --ros-args -p task:=stack_three
2. summary of my changes:
a.Gripper reliability: set ignore_new_calls_while_executing=False so open() isn’t ignored while a previous gripper action is finishing.
b.New task: implemented task_stack_three() (your section may name this stack_all) that performs two stack steps:
c.Helpers added to keep structure intact: _block_id(idx), _prepose_over(x,y), _grasp_pose_at(x,y), _place_pose_at_level(x,y,level), and a post-release _micro_lift(...) (1 cm up) before detach.
Pose math inside the sequence: for each layer level, the place pose is computed as  z_place = grasp_height + level * block_size with pre_place at the same (x,y) as the center column and orientation fixed to the top-down quaternion.
