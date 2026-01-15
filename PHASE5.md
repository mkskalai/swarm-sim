Terminal 1: Rebuild and Start Gazebo with perception test world


source ~/venv-ardupilot/bin/activate
source /opt/ros/jazzy/setup.bash

# Rebuild to get new messages
cd ~/ros2_ws
colcon build --packages-select swarm_ros
source install/setup.bash

# Start Gazebo with perception test world (has people/vehicles to detect)
source "/home/maks/Desktop/python projects/claude/swarm/scripts/setup_env.sh"
gz sim -r "/home/maks/Desktop/python projects/claude/swarm/worlds/perception_test.sdf"





Terminal 2: Start SITL instances


source ~/venv-ardupilot/bin/activate
cd "/home/maks/Desktop/python projects/claude/swarm"
python scripts/launch_sitl.py --num-instances 3




Terminal 3: ROS bridge

source ~/venv-ardupilot/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch swarm_ros simulation.launch.py num_drones:=3 enable_cameras:=true






Terminal 4: Perception nodes (YOLO)


source ~/venv-ardupilot/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch swarm_ros perception.launch.py num_drones:=3






Terminal 5: Monitor detections


source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Check camera images coming through
ros2 topic hz /drone_0/camera/image

# Watch detections
ros2 topic echo /drone_0/detections







Terminal 6: Fly drones over targets


source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 service call /swarm/connect std_srvs/srv/Trigger
ros2 service call /swarm/takeoff_all std_srvs/srv/Trigger

# Fly formation to get drones moving over the target area
ros2 service call /swarm/set_formation swarm_ros/srv/SetFormation "{formation_type: 4, spacing: 10.0, altitude: 15.0}"