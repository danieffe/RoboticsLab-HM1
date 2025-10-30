# Bring up your robot

## Available Packages in this Repository ##
- `armando_description` 
- `armando_gazebo`
- `armando_controller`

## Getting Started

```shell
git clone https://github.com/danieffe/RoboticsLab-HM1.git
colcon build 
source install/setup.bash
```
# **Usage**
 ## **1. Launch the Manipulator in Rviz**
To start the manipulator simulation in Rviz run the command:
```shell
ros2 launch armando_description armando_display.launch.py 
```
rviz will be started.

 ## **2. Launch the Manipulator in Gazebo**
To start the manipulator simulation in Gazebo run the command:
```shell
ros2 launch armando_gazebo armando_world.launch.py
```
## **3. Camera Sensor**
After launching the manipulator in Gazebo, open another terminal and run:
```shell
ros2 run rqt_image_view rqt_image_view
```
## **4. Controller**
There are two controllers available, position controller and trajectory controller.
To active a controller you have to tun this command:
```shell
ros2 launch armando_gazebo armando_world.launch.py controller_type:=<type>
```
where `<type>` can be `position` or `trajectory`, by defult is set to position.

## **5. Subscriber and Publisher node**
To launch the subscriber and the publisher node you have to run:
```shell
ros2 run armando_controller arm_controller_node --ros-args -p controller_type:=<value>
```
where `<value>` can be `position` for the position controller, or `trajectory` for the trajectory controller.

## 📝 Additional Notes

> **Note: Manipulator Not Moving?**
>
> By default, the velocities for the trajectory controller are set to `0`, so the manipulator will not move until they are manually defined.
>
> You can change the velocities in two ways:
>
> 1.  **Configuration File:** Set the velocity and acceleration parameters within the controller's config file.
> 2.  **ROS 2 Topic:** Send commands with the desired velocity values using a dedicated ROS 2 node or from the terminal via `ros2 topic pub` to the `/joint_trajectory_controller/joint_trajectory` topic.
>
> This allows you to adapt the arm's movement speed according to your simulation or testing requirements.
