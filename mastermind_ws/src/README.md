# mastermind game

## terminal1 : launch robot arm

cd mastermind_ws/

source install/setup.bash

ros2 launch mastermind arm.launch.py


## terminal2 : add game scene to gazebo

cd mastermind_ws/src/mastermind/world

./mastermind.sh


## terminal3 : moveit2

ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true


## terminal4 : run the whole game
not finished

### test code separately

@vision

terminal 1:launch robot arm

terminal 2: add game scene to gazebo

terminal 3: run vision.py

            python3 src/mastermind/mastermind/src/vision_model/vision.py 
            
terminal 4: send code to vision to test it

            ros2 topic pub --once /game_status mastermind_interfaces/msg/Status "{sender: 'UserTest', status: 4}"



@pick and place

ros2 run mastermind pick_and_place --ros-args -p task:=place_by_color -p order:="red green yellow black"


@gamestate






#check topic 
ros2 topic list

#check gazebo topic
gz topic -l

#check topic data flow
ros2 topic info /topic_name
ros2 topic hz /topic_name
