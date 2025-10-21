# Qt powered Dashboard for turtle-bot relocalization
- - - 

_this was something i chose make out of blue for no specific reason and I had to make a qt assignment.
thinking back i believe it might have made more sense to make one for mapping using cartographer. But since I had already started working on this.. I guess no options... It took me about 5 hrs to make this and another 4 to document it and beautify the code_

### Basic Idea:
Your robot is in an known space, of which you already have a map. The robot moves around the map avoiding obstacles trying to predict it's own location based on the current sensor readings and previously recorded map.

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
cd
python3 src/main.py
```

## desiegn philosophies:
