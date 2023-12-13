# ee585_carla_project

This is a guidance for executing Final Term project in class EE585 made by Jiheyok Kim
Please follow the guidance below

## Install
Please install the requirements for this code.

```bash
    cd /home/carla_melodic/catkin_ws/src/ee585_carla_project/ros-bridge
    ./install.sh
    
    cd /home/carla_melodic/catkin_ws && catkin_make
    source devel/setup.bash
```

## Start
Please open 3 terminals and insert the command:

```bash

# Terminal 1
cd /home/carla_melodic/CARLA_0.9.10.1 && ./CarlaUE4.sh

# Terminal 2
cd /home/carla_melodic/catkin_ws && source devel/setup.bash
rosrun carla_ad_agent spawn_npc.py

# Terminal 3
cd /home/carla_melodic/catkin_ws &&
roslaunch carla_ad_demo carla_ad_demo_with_rviz.launch
```
# Thank You
