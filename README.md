# ProjectTeamG04
Using the package developed by [Saif Sidhik](https://github.com/justagist), the team made a demo on impedance control on panda robot.
This repository is the environment to simulate the impedance control on Franka robot using Gazebo
You would need to run the simulation on Ubuntu 18.04 and ROS Melodic
The impedance control code, which is located in src/Panda_impedance_with_graph.py works with Python 2.7

You would need to install following dependency to use the simulation.
Execute following command

`sudo apt install ros-melodic-libfranka ros-melodic-franka-ros`

`sudo apt install ros-melodic-gazebo-ros-control ros-melodic-rospy-message-converter ros-melodic-effort-controllers ros-melodic-joint-state-controller ros-melodic-moveit ros-melodic-moveit-commander ros-melodic-moveit-visual-tools`

`pip install numpy`

`pip install numpy-quaternion==2020.5.11.13.33.35`

Here are the steps to set up the environment

1. Open terminal
2. Head to desired empty directory
3. git clone the repository to desired directory
4. cd into the cloned repository
5. Execute `source devel/setup.bash`
6. Execute `roslaunch panda_gazebo panda_world.launch`. This would open up gazebo with panda robot.
7. Open new terminal window
8. Head to same directory that the repository is cloned in
9. Execute `source devel/setup.bash`
10. head to `src` directory (`cd src`)
11. Type in `python Panda_impedance_with_graph.py` to start impedance control

This would set up the environment for impedance control on simulated panda robot.
You can select any joints of the panda robot, right click, and select Apply Force/Torque to apply force/torque and see how robot reacts to external forces
