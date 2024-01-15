<h1 align="center"> Setting Up Intel NUC </h1>
Before you can start using the Intel NUC computer, you need to install the operating system (Ubuntu 22.04) and ROS2 Humble. This section will guide you through the installation of the required software.

## Step 1: Burning the Ubuntu 22.04 Image into a USB Drive ##

First, you need to download the Ubuntu 22.04 OS image to your computer from
https://ubuntu.com/download/desktop

After downloading, you need to follow the same steps as burning the LeoROS image onto the SD Card. First, open the Etcher application, select the downloaded Ubuntu 22.04 image, choose the USB device, and finally, click on Flash. This process can take around 10 minutes.

## Step 2: Installing Ubuntu 22.04 on the NUC ##

Plug in the USB with Ubuntu 22.04 into one of the USB ports of the Intel NUC and turn it on. Since there is no operating system installed on the NUC, it will automatically recognize your USB device and start the setup process. Then, you can follow the steps provided https://ubuntu.com/tutorials/install-ubuntu-desktop#5-installation-setup

## Step 3: Installing ROS2 Humble on the NUC ##

### Set Locale ###
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
### Setup Sources ###

```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
### Install ROS 2 packages ###

```
sudo apt update
```
```
sudo apt upgrade
```
```
sudo apt install ros-humble-desktop
```
```
sudo apt install ros-dev-tools
```

To test your installation, please open two terminal windows and source your ROS2 workspace in both of them:

```
source /opt/ros/humble/setup.bash
```

In one of the terminals, run a data publisher node:
```
ros2 run demo_nodes_cpp talker
```
In the other terminal, run a data listener node:

```
ros2 run demo_nodes_py listener
```

You should see the following output:

<img title="ROS2_Test"  src="../Images/ROS2/ros_test2.png"  width=80% height=auto>

To automatically source the ROS2 workspace, you can use the following commands:
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

```
source ~/.bashrc
```
## Step 4: Connecting NUC with LeoRover ##

You are provided with an Ethernet cable to establish a connection between LeoRover and the Intel NUC. After connecting them, you can observe topics published by LeoRover and send velocity commands to the robot from the NUC. Just as you did inside LeoRover, try sending velocity commands from the NUC.




