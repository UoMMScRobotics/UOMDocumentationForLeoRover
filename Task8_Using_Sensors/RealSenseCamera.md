<h1 align="center"> Using Realsense with ROS2 </h1>
This section is dedicated to the installation and usage of Realsense Camera on your NUC. Once you complete this section, you will be able to visualize Depth and RGB data with Rviz and Realsense-Viewer.

 ## Step 1: Install latest Intel® RealSense™ SDK 2.0 ##

Register the server's public key

```
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

Install HTTPS support 

```
sudo apt-get install apt-transport-https
```


