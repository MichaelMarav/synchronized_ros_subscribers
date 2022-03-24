# Synchronized ROS-topic subscribers

This package usage is to subscribe to various ROS topics and store data simultaneously.

## Installation

```
$ cd ~/<ros_workspace>/src/
$ git clone https://github.com/MichaelMarav/synchronized_ros_subscribers.git
$ cd ..
$ catkin_make
```

## Usage

Navigate to config/config.yaml and change the names of the topics, to the desired ones. Then change the path to fit the desired location to save the samples. The default way is each data point to a column in a .csv file and each row represents a different time step. From the config.yaml you can also change the sampling rate and toggle the option to save the data. Finally, modify subscriber.cpp and change the messages to fit the corresponding topics you want to subscribe.

When everything is set up, run:
```
$ roslaunch synchronized_ros_subscribers subscriber.launch
```
