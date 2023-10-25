![visitor badge](https://visitor-badge.glitch.me/badge?page_id=abirhossen786.486687358-badge)

# Reval
Reval is an open-source framework to evaluate the performance of Robotics platforms. Currently it supports [Husky platform](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/), [Turtblebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). The useres can evalute the performance of a mission for a given gazebo envirnoment (or on their own gazebo envirnment) for different configurations in an automated fashion and log the results. In addition, Reval supports the following metrics to evaluate the quality of a mission:

**Evaluation metrics**
Metrics         |    Description    |
-----------     | ------------------|
DWA F           | # of failed produced path by DWA planner
DWA NP          | # of re-planning by DWA planner
DWA IT          | # of DWA invalid trajectory
RR              | # of rotate recovery excuted
RCU             | # of ClearCostMaps recovery executed for unstuck robot 
RCL             | # of ClearCostMaps layer recovery executed
IRC             | # of invalid rotation cmd
ERG             | # of error rotating on the goal
DRMS            | The square root of the average of the squared horizontal position errors, <img src="https://latex.codecogs.com/png.image?\inline&space;\small&space;\dpi{110}\bg{white}DRMS=\sqrt{\sigma_x^2&plus;\sigma_y^2}" title="https://latex.codecogs.com/png.image?\inline \small \dpi{110}\bg{white}DRMS=\sqrt{\sigma_x^2&plus;\sigma_y^2}" /> ;   where standard deviation of the delta x and y,  <img src="https://latex.codecogs.com/png.image?\inline&space;\small&space;\dpi{120}\bg{white}\sigma&space;=&space;\sqrt{\frac{1}{N}\sum_{i=1}^N(x_i-\mu)^2}" title="https://latex.codecogs.com/png.image?\inline \small \dpi{120}\bg{white}\sigma = \sqrt{\frac{1}{N}\sum_{i=1}^N(x_i-\mu)^2}" />. Probability of 65%
2DRMS           | Twice the Distance Root Mean Squared (DRMS) of the horizontal position error, <img src="https://latex.codecogs.com/png.image?\inline&space;\small&space;\dpi{120}\bg{white}2DRMS=2\sqrt{\sigma_x^2&plus;\sigma_y^2}" title="https://latex.codecogs.com/png.image?\inline \small \dpi{120}\bg{white}2DRMS=2\sqrt{\sigma_x^2&plus;\sigma_y^2}" />. Probability of 95%
CPE             | The radius of circle centered at the true position, containing the position estimate with probability of 50%. <img src="https://latex.codecogs.com/png.image?\inline&space;\small&space;\dpi{120}\bg{white}CEP=0.59(\sigma_x&plus;\sigma_y)" title="https://latex.codecogs.com/png.image?\inline \small \dpi{120}\bg{white}CEP=0.59(\sigma_x+\sigma_y)" />
ED              | Euclidean distance between actual goal location and the location the robot reached. <img src="https://latex.codecogs.com/png.image?\inline&space;\small&space;\dpi{110}\bg{white}d&space;=&space;\sqrt{(x_2&space;-&space;x_1)^2&space;&plus;&space;(y_2&space;-&space;y_1)^2}" title="https://latex.codecogs.com/png.image?\inline \small \dpi{110}\bg{white}d = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}" />
RNS             | robustness in narrow spaces, <img src="https://latex.codecogs.com/png.image?\inline&space;\small&space;\dpi{120}\bg{white}RNS=\frac{1}{N_{s}}\sum_{i=1}^{N_{s}}&space;(passed_{Ns})" title="https://latex.codecogs.com/png.image?\inline \small \dpi{120}\bg{white}RNS=\frac{1}{N_{s}}\sum_{i=0}^{N_{s}}&space;(passed_{Ns})" /> ; where Ns is the total narrow spaces in the gazebo environment, and passed_Ns is the narrow spaces that the robot successfully crossed.
DT             | total distance traveled during a mission
BP             | battery percentage. For more details: [Gazebo-ROS battery plugin](src/husky_ws/src/gazebo_ros_battery/#gazebo-ros-battery-plugin)
Col            | number of collisions in a mission
MT             | time taken to complete a mission
MS             | mission success. Example: if the robot successfully reached point A to B

The instructions provided below are for Turtlebot3. Follow the initial setup for [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).

## Build status
Build Type      |    Status     |
-----------     | --------------|
ROS melodic     | [![ROS melodic](https://img.shields.io/badge/ROS_meoldic-failing-FF0000)](http://wiki.ros.org/melodic/Installation/Ubuntu)
ROS noetic      | [![ROS noetic](https://img.shields.io/badge/ROS_noetic-passing-success)](http://wiki.ros.org/noetic/Installation/Ubuntu)

Platform        |    Status     |
-----------     | --------------|
Husky UGV     | [![Husky UGV](https://img.shields.io/badge/Husky_UGV-passing-success)](https://www.clearpathrobotics.com/assets/guides/noetic/husky/SimulatingHusky.html)
TurtleBot3      | [![TurtleBot3](https://img.shields.io/badge/TurtleBot3-passing-success)](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
OceanWATERS     | [![OW](https://img.shields.io/badge/OceanWATERS-coming_soon-ff69b4)](https://github.com/nasa/ow_simulator)



## Requirements
* Ubuntu 20
* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 
* Python 3.6+

## Installations
```sh
sudo apt remove ros-noetic-dynamixel-sdk
sudo apt remove ros-noetic-turtlebot3-msgs
sudo apt remove ros-noetic-turtlebot3
git clone -b turtlebot3 https://github.com/softsys4ai/Reval.git
mkdir -p ~/Reval/src/turtlebot3
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/Reval && catkin build
```

### Installing [rosbag](http://wiki.ros.org/rosbag) for Python
#### Download and install [ros_readbag.py](http://wiki.ros.org/ROS/Tutorials/reading%20msgs%20from%20a%20bag%20file) using these commands:
Download the file
```sh
wget https://raw.githubusercontent.com/ElectricRCAircraftGuy/eRCaGuy_dotfiles/master/useful_scripts/ros_readbagfile.py
```
Make it executable
```sh
chmod +x ros_readbagfile.py
```
Ensure you have the ~/bin directory for personal binaries
```sh
mkdir -p ~/bin
```
Move this executable script into that directory as `ros_readbagfile`, so that it will be available as that command
```sh
mv ros_readbagfile.py ~/bin/ros_readbagfile
```
Create a symlink in `~/bin` to this script so you can run it from anywhere:
```sh
ln -si "${PWD}/ros_readbagfile.py" ~/bin/ros_readbagfile
```
If this is the first time ever creating the "~/bin" dir, then log out and log back in to your Ubuntu user account to cause Ubuntu to automatically add your ~/bin dir to your executable PATH.

Re-source your `~/.bahsrc` file
```sh
source ~/.bashrc
```
## Install dependencies
Installing the dependencies
```sh
sudo apt install ripgrep
pip install pandas
pip install tqdm
pip install tabulate 
```
Or `cd Reval/` run `./requirements.sh`. If you face permission denied, first run `chmod +x requirements.sh` 


## Running Reval
```sh
cd Reval/
```
source your new `setup.sh` file. You need source this `setup.sh` file everytime you open a new Terminal
```sh
source devel/setup.bash
```
To evaluate the mission run
```sh
python reval.py
```

## Demo
https://user-images.githubusercontent.com/73362969/206578791-1d3d5b52-10c3-4271-aded-67ce19be1f08.mp4

https://user-images.githubusercontent.com/73362969/206736365-6e5f7486-bd51-42f0-b39f-656d039a13cb.mov


## Customizations
- To define your custom configuration options: [Set Configuration](/src/benchmark/README.md#cahnging-configuration-options)
- To change the goal locations: [Update Goal location](/src/benchmark/README.md#define-mission-specifications)


## Contacts
Please feel free to contact via email if you have any feedbacks. Thank you for using Reval!
|Name|Email|     
|---------------|------------------|      
|Md Abir Hossen|mhossen@email.sc.edu|          
|Pooyan Jamshidi|pjamshid@cse.sc.edu|  
