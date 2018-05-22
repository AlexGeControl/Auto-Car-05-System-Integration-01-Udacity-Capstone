# Udacity Self-Driving Car Engineer Capstone -- System Integration

---

## Environment Setup

### 1. Clone This Repo

Clone the project repository
```bash
git clone https://github.com/AlexGeControl/Auto-Car-05-System-Integration-01-Udacity-Capstone.git
```

### 2. Set Up Docker Environment

1. [Install Docker](https://docs.docker.com/engine/installation/)

2. Change to project directory
    ```bash
    cd [YOUR_PROJECT_DIRECTORY]
    ```

3. Build the docker container for project
    ```bash
    docker build . -t capstone
    ```

4. Run the docker file
    ```bash
    docker run -it -v $PWD:/capstone -v /tmp/log:/root/.ros/ -p 4567:4567 capstone
    ```

### 3. Launch ROS
After entering docker, in the default entrypoint directory, execute the following commands:
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

### 4. Run Simulator

1. Download [Udacity Capstone Project Simulator](https://github.com/udacity/CarND-Capstone/releases) 

2. Extract the file and make the programs runnable
```bash
unzip linux_sys_int.zip
chmod +x sys_int.x86
chmod +x sys_int.x86_64
```
3. Run the simulator

---

## Real World Testing

1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
