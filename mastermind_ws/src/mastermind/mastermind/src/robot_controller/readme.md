vm terminal:
xhost +local:docker

docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/jz499/workspaces:/root/workspaces \
  --gpus all \
  --name ros2_lab06 \
  gitlab-registry.oit.duke.edu/introtorobotics/mems-robotics-toolkit:kinova-jazzy-latest bash


docker pane1:
cd ~/workspaces/jz499_robotics_fall2025/final_project/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch kortex_bringup kortex_sim_control.launch.py \
  sim_gazebo:=true \
  robot_type:=gen3_lite \
  gripper:=gen3_lite_2f \
  robot_name:=gen3_lite \
  dof:=6 \
  use_sim_time:=true \
  launch_rviz:=false \
  robot_controller:=joint_trajectory_controller

docker pane2:
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true

docker pane3:
cd ~/workspaces/jz499_robotics_fall2025/final_project/ros2_ws/src/mems-toolkit/lab06_files
./spawn_blocks.sh

docker pane4:
cd /root/workspaces/jz499_robotics_fall2025/final_project/ros2_ws
colcon build --symlink-install
source install/setup.bash

# mirror from Gazebo into MoveIt
ros2 run lab06_moveit pick_and_place --ros-args -p task:=add_scene

# run the task
ros2 run lab06_moveit pick_and_place --ros-args -p task:=place_four_in_line

ros2 run lab06_moveit pick_and_place \
  --ros-args -p task:=place_by_color \
             -p order:="purple blue black green"


ros2 run lab06_moveit pick_and_place \
  --ros-args -p task:=place_by_color \
             -p order:="black yellow blue red"



ros2 run lab06_moveit pick_and_place \
  --ros-args -p task:=place_by_color \
             -p order:="red red blue blue"

ros2 run lab06_moveit pick_and_place \
  --ros-args -p task:=place_by_color \
             -p order:="black black purple purple"


docker pane 5: 
cd ~/workspaces/jz499_robotics_fall2025/final_project/ros2_ws
source install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
"/world/empty/model/lab06_overhead_camera/link/camera_link/sensor/overhead_camera/image@sensor_msgs/msg/Image@gz.msgs.Image"




cd ~/workspaces/jz499_robotics_fall2025/final_project/ros2_ws
source install/setup.bash

ros2 run color_vision block_color_scanner \
  --ros-args \
  -p image_topic:=/world/empty/model/lab06_overhead_camera/link/camera_link/sensor/overhead_camera/image \
  -p row_fraction:=0.40 \
  -p x_fracs:="[0.25, 0.45, 0.65, 0.85]" \
  -p debug_hsv:=false

