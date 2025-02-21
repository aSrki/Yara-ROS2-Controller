## Project YARA's repo for its ROS2 controller. 
ROS2 distribution used in this project is ROS2 Jazzy.<br />
![alt text](https://cdn.prod.website-files.com/660efb77a041930767e2971a/664e3b4b2640ffefe5b9a158_Welcoming%20Jazzy%20Jalisco%20Meta%20Asset.png)
YARA harware is not necessary in order to run, and try out, its ROS2 controller.
In ordere to use this code, you need to clone this repo, and cloned code represents ROS2 src folder.<br />
For cloning use command : `git clone https://github.com/aSrki/ProjectYaraROS2`.<br />
After succesfull clone, run command `colcon build`.<br />
After succesfull `colcon build`, run command `source install/setup.bash`.<br />
Finally run command `ros2 launch yara_driver yara_no_rviz.launch.py joystick:=False hardware:=False`.
ROS2 parameters `joystick` is used if you want to move the arm manually with joystick.
ROS2 parameters `hardware` is used if you want to run the code on the real hardware of the robot.
