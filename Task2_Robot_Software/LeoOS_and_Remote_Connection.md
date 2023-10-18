<h1 align="center"> Installing LeoOS and Establishing Remote Connection</h1>

This section focuses on installing the LeoOS operating system on the Raspberry Pi and connecting to the Raspberry Pi using two different methods. Once you have successfully completed the steps in this section, you will be able to access and modify the folders and settings inside the Raspberry Pi.

## Step 1: Installing Required Softwares ##

**LeoOS** : This is the official operating system for Raspberry Pi, which includes ROS Noetic pre-installed. You will need to burn this image onto your Raspberry Pi. You can download the LeoOS image from

https://github.com/LeoRover/LeoOS/releases

Install the latest **full** version of the LeoOS.

**Balena Ethcer** : You will use it to burn the LeoOS image onto the SD card.

Download Etcher from https://etcher.balena.io

and install it on your computer.

**For Linux**
Please select the following option to dowload

<img title="Ethcer App Linux"  src="../Images/LeoOS/linux_etcher.png">

To run the application, first give the permission:

```
chmod +x balenaEtcher-1.18.11-x64.AppImage
```

and run it
```
./balenaEtcher-1.18.11-x64.AppImage
```


**PuTTY** :You will use it to connect to the Raspberry Pi using your computer (SSH connection).

Download PuTTY from https://www.PuTTY.org
and install it on your computer.
You do not need to install PuTTY if have a linux installed computer.

## Step 2: Burning LeoOS Image into an SD Card Using Etcher ###

Connect the SD Card to your computer. Open Etcher, and select the LeoOS you downloaded in Step 1.

<img title="Ethcer App"  src="../Images/LeoOS/OpenEtcher.png">

Select your USB device

<img title="USB Device"  src="../Images/LeoOS/SelectDrive.png">

Finally, click 'Flash.' It will take around 5 minutes to finish the flashing process.

## Step 3: Turning On LeoRover ###
Mount the SD Card into the Raspberry Pi using the plastic tool provided in the package.

<img title="SD Mount"  src="../Images/LeoOS/MountSD.png">

Turn on LeoRover by pressing the button shown in the following figure
<img title="Power Button"  src="../Images/LeoOS/TuronON.jpg"  width=60% height=auto>

The green LED of the power button will start blinking, and after some time (15-20 seconds), you will be able to see the Wi-Fi network of the LeoRover.

**Important Note** : To avoid confusion when setting up multiple robots, please follow these steps:

<ol>
<li>Do not turn on your robot at the same time to prevent difficulty in determining which Wi-Fi network belongs to your robot.</li>
<li>One group should turn on their robot.</li>
<li>Connect to the robot using the Wi-Fi network specific to their robot.</li>
<li>Change the network ID to "LeoRover-GROUPX," where X represents your group number.</li>
<li>Following groups should then follow the same procedure for their robots.</li>
</ol>
  
Please note that these steps are only necessary during the initial setup of your robots. Once the network IDs are customized to include group numbers, you can run all the robots simultaneously without confusion, as each will have a distinctive name.

## Step 4: Connecting LeoRover ###
### Windows ###
You should see a LeoRover network specific to your robot, similar to the image below.

<img title="Wifi Leo"  src="../Images/LeoOS/WifiImage1.png"  width=40% height=auto>

The password for all networks (for all robots) is 'password' by default. Enter the password and connect to your robot.

After establishing the connection, run PuTTY, which you downloaded in Step 1. Type your robot's IP address (10.0.0.1) in the 'Host Name (or IP Address)' field and leave the other settings unchanged. Your PuTTY window should resemble the following image.
<img title="PuTTY Window"  src="../Images/LeoOS/PuttyImage.png"  width=40% height=auto>

Click **Open** . The following warning will be raised, click **Accept**.

<img title="PuTTY Warning"  src="../Images/LeoOS/PuttySecurityAlert.png"  width=40% height=auto>

Now, you should see a login screen like the image below. The login information should be as follows:

**login as:** pi

**password:** raspberry

<img title="SSH Login"  src="../Images/LeoOS/LoginRasperrySSH.png"  width=70% height=auto>

You should log in after entering the provided ID and password. Please note that the password will not be visible as you type.

### Linux ###

You have SSH client already installed on ubuntu. Open a new terminal in your computer and type:

```
ssh pi@10.0.0.1
```
Then, type **yes** and **raspberry**.

## Step 5: Changing Wifi ID ###

Before proceeding, you need to change the name of your robot's Wi-Fi network ID, as explained in Step 3. To do this, open the network settings by pasting the following code into the PuTTY console:
```
sudo nano /etc/hostapd/hostapd.conf
```
Change the Wi-Fi name (SSID) according to your group number. For example, the first group should name their network as 'LeoRover-GROUP1'.

<img title="Changing Wifi Name"  src="../Images/LeoOS/ChangeWifiName.png"  width=60% height=auto>

Please only change the name of the network; do not modify the password or other parameters. After renaming it correctly, press **Ctrl+o**, **Enter**, **Ctrl+x**.

Now, your new configuration is saved. Finally, restart your network using the following code:

```
sudo systemctl restart hostapd
```
It will disconnect you from the LeoRover Wi-Fi, and the Wi-Fi name will be updated as follows:

<img title="Updated Wifi Name"  src="../Images/LeoOS/WifiImage2.png"  width=60% height=auto>

## Step 6: Connecting via Remote Desktop Connection ###

### Windows ###

With an SSH connection (PuTTY), you have access to a console that allows you to modify your robot's software. Another way to connect to the Raspberry Pi is to use Remote Desktop Connection, which enables you to directly view the desktop of the Raspberry Pi instead of a console. To use Remote Desktop Connection, follow these steps:

<img title="Remote Desktop"  src="../Images/LeoOS/RemoteDesktop.png"  width=60% height=auto>

Type your LeoRover's IP address and click 'Connect'.

<img title="Remote Desktop Connection"  src="../Images/LeoOS/RemoteDesktopConnect.png"  width=60% height=auto>

Select **Yes** for following alert

<img title="Remote Desktop Warning"  src="../Images/LeoOS/RemoteDesktopAlert.png"  width=60% height=auto>

Enter the password and id 

**username:** pi

**password:** raspberry

<img title="Remote Desktop Login"  src="../Images/LeoOS/RemoteDesktopLogin.png"  width=60% height=auto>

Now, you should see the desktop of the LeoRover as shown below:

<img title="Raspberry Desktop"  src="../Images/LeoOS/LeoRoverDesktop.png"  width=60% height=auto>

You will notice that ROS Noetic is already installed on the robot.

<img title="ROS Folders"  src="../Images/LeoOS/LeoRoverFolders.png"  width=60% height=auto>

Now, you can navigate through the folders to explore what is installed within the LeoRover operating system.

### Linux ###

Run the remmina application

<img title="Open Remmina"  src="../Images/LeoOS/remmina_1.png"  width=60% height=auto>

Add a new remote using button on left top corner

<img title="Add Remote"  src="../Images/LeoOS/remmina_2.png"  width=60% height=auto>

Type the raspberry pi ip, id and password and then click connect.

<img title="Connect Raspberry"  src="../Images/LeoOS/remmina_3.png"  width=60% height=auto>

















