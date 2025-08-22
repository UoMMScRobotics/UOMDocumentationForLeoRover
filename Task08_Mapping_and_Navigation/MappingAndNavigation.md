<h1 align="center"> Mapping and Navigation with ROS2 </h1>

The [Navigation Stack](https://navigation.ros.org/) (Nav2) for ROS2 provides a ready built solution for mapping and navigation for mobile robots.

---
<h2 align="center">Mapping in 2D</h2>

In the navigation stack, environments are represented using 2D maps called occupancy grids.  These are generated using sensor data such as lidar.  These maps are not only used to highlight obstacles and to plan paths, but also to localise where a robot is by comparing the map to sensor data (known as localisation).

Maps can be premade, or generated whilst the robot is moving using SLAM (Simultaneous Localisation and Mapping).  In the Nav2 stack, the [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) is the most feature rich and well documented solution for SLAM and pure localisation.

---
<h2 align="center">What About Costmaps?</h2>

[Costmaps](https://navigation.ros.org/configuration/packages/configuring-costmaps.html) are a continuation of occupancy grids.  Where occupancy grids typically only record where obstacles are, a costmap can expand this representation to [multiple layers](https://navigation.ros.org/plugins/index.html#costmap-layers).  The most common is the [inflation layer](https://navigation.ros.org/configuration/packages/costmap-plugins/inflation.html), to convert obstacles into the configuration space of a mobile platform.

The Navigation Stack also allows for filtering and triggers based on costmaps, e.g. [keep out zones](https://navigation.ros.org/configuration/packages/costmap-plugins/keepout_filter.html).  It also allows for 3D sensors such as depth cameras to be converted into 2D representations for obstacle avoidance.

---
<h2 align="center">Mapping in 3D</h2>

It may be necessary to represent obstacles and features in 3D (e.g. for a manipulator).  [Voxel grids](https://index.ros.org/p/nav2_voxel_grid/github-ros-planning-navigation2/) are an extension of an occupancy grid, where instead of a 2D grid of pixels in an image, a 3D grid of voxels are used.  As the memory requirements for N^{3} voxels compared to a 2D grid is much greater, it may be that the total volume needs to be limited, or the voxels more coarse.  [Octomap](https://index.ros.org/p/octomap/github-octomap-octomap/) provides an alternative means to generate voxel grids.

---
<h2 align="center">Path Planning and Trajectory Control in 2D</h2>

For a mobile platform, the navigation stack once again provides the necessary tools for path planning.  This includes various [path planners](https://navigation.ros.org/plugins/index.html#planners), filters and modifiers (e.g. [smoothers](https://navigation.ros.org/plugins/index.html#smoothers)), as well as [controllers](https://navigation.ros.org/plugins/index.html#controllers).

The planner provides a route from the start to the goal, whereas the controller aims to keep the robot on that path.  Each path planner and controller is better or worse suited to certain types of robot locomotion and degrees of freedom, so read the documentation.

---
<h2 align="center">Path Planning and Trajectory Control in 3D</h2>

For manipulators, path planning is now a 3D affair, often with many more degrees of freedom in the system compared to a non-holonomic ground vehicle.  [ROS Industrial](https://rosindustrial.org/) and other endevours have lead to solutions such as [MoveIt](https://moveit.ros.org/) to provide motion planning for manipulators.

For mobile robots, the hardware input for control has been neglected, as is it nearly always velocity control (for a wheeled platform).  The hardware control for robots in ROS2 is handled by the [ROS2 Control](https://control.ros.org/) framework.  This usually means that a manufacturer needs to supply a hardware controller rather than the end-user have to write one.

---
<h2 align="center">ROS2 Control</h2>

ROS2 Control allows for the same joints or group of joints to be managed using different control methods.  For example, imagine a pan and tilt camera which has two degrees of freedom (pan and tilt, unsurprisingly).  __Position control__ would get the two axes to move to the desired position as quickly as possible, this likely means that one axis may still be moving whilst the other has reached its goal position.  __Velocity control__ simply gives the rate of movement for each joint, this is the obvious choice for a wheeled mobile robot.  It may be useful for a single joint in our pan and tilt example, perhaps perform a slow pan from left to right as a given speed.  __Trajectory control__ is probably the most desirable control scheme for a manuipulator, i.e. a goal position and a specific time duration.

The trajectory control enables both axes of the pan and tilt to reach their respective goal positions at the same time.  This also can help with smoother movement, for example if a jerk minimised trajectory controller is in use.  The choice of controller often needs to be defined as part of a launch file or yaml file, so read the documentation for your hardware.  Ideally, a solution such as MoveIt will provide the trajectory, and the hardware (via ROS2 Control) will execute that trajectory.

Solutions such as [MoveIt](https://moveit.ros.org/) for manipulators incorporate a great deal of different planners, and will likely always use trajectory control.
