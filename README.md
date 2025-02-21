## Project YARA's repo for its ROS2 controller. 
ROS2 distribution used in this project is ROS2 Jazzy.<br />
![alt text](https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.intrinsic.ai%2Fblog%2Fposts%2Fwelcoming-jazzy-jalisco-as-the-latest-ros-2-release&psig=AOvVaw2qjYvsk162Jfom5OrdJWKx&ust=1740244407321000&source=images&cd=vfe&opi=89978449&ved=0CBQQjRxqFwoTCJiW8-Ch1YsDFQAAAAAdAAAAABAE)
In ordere to use this code, you need to clone this repo, and cloned code represents ROS2 src folder.<br />
For cloning use command : `git clone https://github.com/aSrki/ProjectYaraROS2`.<br />
After succesfull clone, run command `colcon build`.<br />
After succesfull `colcon build`, run command `source install/setup.bash`.<br />
Finally run command `ros2 launch yara_driver yara_no_rviz.launch.py joystick:=False hardware:=False`.
