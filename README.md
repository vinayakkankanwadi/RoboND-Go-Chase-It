[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# RoboND-Go-Chase-It
The **Go-Chase-It** is part of RoboND ROS Essentials. The purpose is to Design and build a mobile robot, and house it in a world.
Then, program a robot with C++ nodes in ROS to chase white colored ball.

### Directory Structure
```
    .RoboND-Go-Chase-It                # Go Chase It Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── <yourworld>.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └── model      
        
```
### Steps to launch the simulation

#### Step 1 Update and upgrade the Workspace image
```sh
$ sudo apt-get update
$ sudo apt-get upgrade -y
```

#### Step 2 Create the catkin workspace
```sh
$ mkdir -p $HOME/catkin_ws/src
$ cd $HOME/catkin_ws/src
$ catkin_init_workspace
$ git clone https://github.com/vinayakkankanwadi/RoboND-Go-Chase-It.git
```

#### Step 3 add Model Library and custom models
```sh
$ cd $HOME
$ git clone https://github.com/osrf/gazebo_models
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/gazebo_models:$HOME/catkin_ws/src/RoboND-Go-Chase-It/model

```

#### Step 4 Compile the code
```sh
$ cd $HOME/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

#### Step 5 Run the Simulation 
##### in terminal 1:

```sh
$ source $HOME/catkin_ws/devel/setup.bash
$ roslaunch my_robot world.launch

```

##### in terminal 2:

```sh
$ source $HOME/catkin_ws/devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch

```

##### in Gazebo:

move the white ball in front of the robot and have fun.


# License
MIT license
