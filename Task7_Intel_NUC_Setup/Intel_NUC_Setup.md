<h1 align="center"> Setting Up Intel NUC </h1>
Before you can start using the Intel NUC computer, you need to install the operating system (Ubuntu 22.04) and ROS2 Humble. This section will guide you through the installation of the required software.

## Step 1: Burning the Ubuntu 22.04 Image into a USB Drive ##

First, you need to download the Ubuntu 22.04 OS image to your computer from
https://ubuntu.com/download/desktop

After downloading, you need to follow the same steps as burning the LeoROS image onto the SD Card. First, open the Etcher application, select the downloaded Ubuntu 22.04 image, choose the USB device, and finally, click on Flash. This process can take around 10 minutes.

## Step 2: Installing Ubuntu 22.04 on the NUC ##

Plug in the USB with Ubuntu 22.04 into one of the USB ports of the Intel NUC and turn it on. Since there is no operating system installed on the NUC, it will automatically recognize your USB device and start the setup process. Then, you can follow the steps provided https://ubuntu.com/tutorials/install-ubuntu-desktop#5-installation-setup

## Step 3: Installing ROS2 Humble on the NUC ##
To install ROS2 on the NUC, please follow the instructions in the official ROS2 Humble document (https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) until you reach the **Environment setup** section. Additionally, you do not need to complete the **Install additional DDS implementations (optional)** step. Please precisely replicate the installation instructions."


To test your installation, please open two terminal windows and source your ROS2 workspace in both of them:
```
. ~/ros2_humble/install/local_setup.bash
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
echo "source ~/ros2_humble/install/local_setup.bash" >> ~/.bashrc
```

```
source ~/.bashrc
```





