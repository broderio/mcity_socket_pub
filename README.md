## Docker
```
docker run -i -t osrf/ros:humble-desktop
source /opt/ros/humble/setup.sh
git clone https://github.com/broderio/mcity_socket_pub.git
cd mcity_socket_pub
```

## MCity Installation
```
cp env_sample .env
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```
## Run ROS2 Node
```
colcon build --packages-select mcity_socket_pub
source install/setup.bash
ros2 run mcity_socket_pub talker
```