## Project YARA's repo for its ROS2 controller. 
<p align="center">
   <img src="https://github.com/aSrki/Yara-ROS2-Controller/blob/master/videos/Screencast%20from%202025-02-23%2017-58-11.gif" width="400"/>
</p>
ROS2 distribution used in this project is ROS2 Jazzy.<br />
<p align="center">
   <img src="https://cdn.prod.website-files.com/660efb77a041930767e2971a/664e3b4b2640ffefe5b9a158_Welcoming%20Jazzy%20Jalisco%20Meta%20Asset.png" width="550"/>
</p>
YARA harware is not necessary in order to run, and try out, its ROS2 controller.
In ordere to use this code, you need to clone this repo, and cloned code represents ROS2 src folder.<br />
For cloning use command : <code>git clone https://github.com/aSrki/ProjectYaraROS2</code>.<br />
After succesfull clone, run command <code>colcon build</code>.<br />
After succesfull <code>colcon build</code>, run command <code>source install/setup.bash</code>.<br />
Finally run command <code>ros2 launch yara_driver yara_no_rviz.launch.py joystick:=False hardware:=False</code>.<br />
ROS2 parameters <code>joystick</code> is used if you want to move the arm manually with joystick.<br />
ROS2 parameters <code>hardware</code> is used if you want to run the code on the real hardware of the robot.

Requirements:
   - `sudo apt-get update`
   - `sudo apt-get upgrade`
   - `pip3 install pygame`
   - `sudo apt install ros-jazzy-moveit`
   - `sudo apt-get install libompl-dev ompl-demos`
   - `sudo apt-get install ros-jazzy-moveit-planners-chomp`

