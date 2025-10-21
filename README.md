# Qt powered Dashboard for turtle-bot relocalization
- - - 

_this was something i chose make out of blue for no specific reason and I had to make a qt assignment.
thinking back i believe it might have made more sense to make one for mapping using cartographer. But since I had already started working on this.. I guess no options... It took me about 5 hrs to make this and another 4 to document it and beautify the code_

### Basic Idea:
Your robot is in an known space, of which you already have a map. The robot moves around the map avoiding obstacles trying to predict it's own location based on the current sensor readings and previously recorded map.

### video rec:
Please refer this video on drive:
[click here to watch the vid](https://drive.google.com/file/d/1C92PzVYZZPKT3HYrwKkkxy6R4OIZKh4m/view?usp=sharing)


### Pre-requisites:
- ROS2-Humble
- linux
- pyQt
- python
- c++
- bash
- turtle bot

## Install:

#### 1. ROS installation
start by ROS2 installation if you don't already have it follow this link: [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

#### 2. turtlebot installation
start by turtlebot installation if you don't already have it follow this link: [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)


#### 3. custom installation
I have wrtitten all steps into install.sh just run it with 
``` bash
bash install.sh
```

and alll done 
now

## Running the application
``` bash
source ~/ros2_ws/install/setup.sh
export TURTLEBOT3_MODEL=waffle
python3 src/main.py
```

## desiegn philosophies:
so basically I made use of turtle bot 
I believe its the best place to begin any rover based project from ideation to protypin, it't always been helpful.
So i wrote a new package over this #random_localizer
this package is what send cmd_vel to the rover to move around while trying to avoid obstacles
this package has 3 launch files:
- random_localizer.launch.py
- stop_random_localizer.launch.py
- turtlebot3_sim_nav.launch.py

as the name suggests turtlebot3_sim_nav.launch.py launches gazebo and also brings uo nav2 and rviz
the other 2 are just to start and stop the the reandon localizer package

the start and stop buttons basically call these 2 launch files

on the logs you can see cmd_vel, odom and laserscan, these were chosen cause i believe these are what are requiredto recrreate the entire situation, rest of the data is redundant and can be recomputed based on this if at all required.

the telemetry shows the number of laser point the lidar recieves, showing qulity of incoming data.
also it shows the graph for uncertainity in position clculated by the AMCL package

other than the 2 dials on right show the average velocity over past 2 sec from the odom 

Finally the status tab has battery status, this is a dummy value but this helps keep track of time and the power we are expending on the robot
