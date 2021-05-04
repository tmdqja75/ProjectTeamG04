# ProjectTeamG04
This repository is the environment to simulate the impedance control on Franka robot using Gazebo
You would need to run the simulation on Ubuntu 18.04 and ROS Melodic
The impedance control code, which is located in src/Panda_impedance_with_graph.py works with Python 2.7

Here are the steps to set up the environment

1. Open terminal
2. Head to desired empty directory
3. git clone the repository to desired directory
4. cd into the cloned repository
5. Execute source devel/setup.bash
6. Execute roslaunch panda_gazebo panda_world.launch. This would open up gazebo with panda robot.
7. Open new terminal window
8. Head to same directory that the repository is cloned in
9. Execute source devel/setup.bash
10. head to 'src' directory
11. Type in python sour
