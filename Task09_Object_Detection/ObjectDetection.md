<h1 align="center"> Object Detection </h1>

> [!NOTE]
> Think: How can we convert the output of object detection into coordinates that can be provided to a manipulator. 

There are many approaches to identifying objects for a robot to interact with.  This is largely dependent on what type of object the robot is interested in.  For example, human detection may be of benefit during path planning and navigation in dynamic environments, whereas using text recognition may be more suitable for identifying which pill bottle is correct for an elderly person with a robot assistant. Lastly, it's important to identify what output results are required for your application. Different approaches give outputs such as bounding boxes around objects or segmentation masks. 

**THESE ARE ONLY SUGGESTIONS OR SIGNPOSTING TO FURTHER RESOURCES, THE CHOICE OF APPROACH IS DOWN TO YOU**


---
<h2 align="center">Camera Real World Considerations</h2>
When using any sensor, it is important to ensure your sensors are well calibrated. For cameras, there will be instructions provided by the manufacturer on how to best calibrate your device. Examples include [realsense docs](https://dev.intelrealsense.com/docs/self-calibration-for-depth-cameras) and [Azure Kinect docs](https://learn.microsoft.com/en-us/azure/kinect-dk/use-calibration-functions).
Also, several factors, such as lighting, reflections from objects, and shadows can affect your camera images. It may be beneficial to consider if these are important to your application and how to tackle them.


---
<h2 align="center">Visual</h2>

Typically, visual object detection relies on characteristics of an image to segment a portion of the image.  For example, the brightness or colour may be used to identify objects.  Furthermore, filters such as edge detection may be used to help segment or perform further tasks such as text recognition.  It may be beneficial to convert from RGB colour space to HSV colour space.  Many online tutorials and numerous textbooks are available that discuss image processing.


---
<h2 align="center">OpenCV</h2>

[OpenCV](https://opencv.org/) is a powerful suite of tools for object recognition, with many examples and tutorials (which you may wish to read).  It is the backbone of many image processing workflows, and conveniently there are [packages available](https://github.com/ros-perception/vision_opencv) to convert ROS images messages to OpenCV images for manipulation.  With a calibrated camera, is it even possible to find the 3D pose of a highly textured object from 2D images.


---
<h2 align="center">Fiducial Markers</h2>

It is possible to make objects more recognisable by using images which are readily observed and understood by a machine.  In this case, markers such as barcodes, QR codes or spatial fiducial markers such as [AprilTag](https://april.eecs.umich.edu/software/apriltag) are ideal.  The benefit is that these can be readily utilised to provide pose estimation and identification of objects and locations, the compromise is that a human or other agent needs to attach and identify each object/tag _a priori_.


---
<h2 align="center">Depth</h2>

Depth (either from stereo vision, lidar, RGBD etc) can be used to aid in segmentation of a scene.  For example a table top can be modelled as a plane and removed to better filter out where an object may be.  It is likely that these depth images or point clouds will need to be converted into a 3D voxel occupancy grid when being used with [MoveIt](https://moveit.ros.org/) or other existing packages.
Depth images can also be processed directly to estimate object location using approaches such as background subtraction. There are also libraries for processing point cloud and depth data, such as [open3D](https://www.open3d.org/docs/release/introduction.html) and [point cloud library](https://pointclouds.org/), which could be useful.


---
<h2 align="center">Traditional Geometric Methods</h2>
When the approximate shape and dimensions of an object are already known, it may not be necessary to use machine learning at all. Traditional computer vision and geometric approaches can make direct use of this prior information to estimate an object's 3D position and orientation (pose).

For a manipulator, the desired result will typically be a 6-DoF pose, consisting of a translation ((x,y,z)) and a rotation. This pose is initially estimated relative to the camera and can then be transformed into the robot's coordinate frame using the appropriate camera/robot calibration.

For a geometrically simple object such as a cube, depth information can make this particularly straightforward. One possible processing pipeline is:

1. Crop the point cloud to the robot's workspace or another region of interest.
2. Remove invalid measurements and optionally downsample/filter the point cloud.
3. Identify the table or supporting surface using a plane-fitting method such as RANSAC and remove it from the scene.
4. Cluster the remaining points to identify candidate objects.
5. Select the cluster whose dimensions and geometry are consistent with the expected cube.
6. Estimate the cube centre and orientation using its surfaces, edges, surface normals or a fitted 3D bounding box.
7. Transform the resulting pose from the camera coordinate system into the robot coordinate system.

Libraries such as Open3D and the Point Cloud Library (PCL) contain implementations of many of these operations, including filtering, clustering, plane fitting, surface-normal estimation and bounding-volume calculations.

If only a conventional RGB camera is available, geometric pose estimation is still possible when known points on the object can be identified in the image. For example, the corners of an object can be associated with corresponding points in a known 3D model and a Perspective-n-Point (PnP) algorithm can then estimate the translation and rotation of the object relative to a calibrated camera. OpenCV provides several implementations of PnP through its solvePnP family of functions.

Traditional geometric methods are particularly useful in robotics because they are interpretable, predictable and relatively easy to debug. Each stage of the processing pipeline has a clear purpose, and parameters such as object dimensions, distance thresholds and angular tolerances can be related directly to the physical scene. They typically require less computation than deep learning approaches, do not need a labelled training dataset, and can provide highly repeatable results in controlled environments. Known geometric constraints can also be enforced directly; for example, detections that do not match the expected dimensions of a cube can be rejected. This makes traditional methods well suited to manipulation tasks, where pose estimates must be reliable enough to avoid failed grasps or collisions. Their main limitations are sensitivity to factors such as occlusion, clutter, reflective surfaces, poor depth measurements and inaccurate calibration, particularly as object geometry and environmental conditions become more complex.

---
<h2 align="center">Machine Learning and Deep Learning</h2>

Machine Learning (ML), and particularly Deep Learning (DL), has become extremely powerful for computer vision. Neural networks can learn features directly from training data and can perform tasks such as classification, object detection, instance segmentation, keypoint detection and tracking. These approaches are particularly useful when objects cannot easily be described using simple colours, edges or geometric rules, or when the environment contains significant variation and clutter.

A well-known example is the YOLO (You Only Look Once) family of models. YOLO-style object detectors can identify objects in an image and return a class, confidence value and bounding box. Related models can also perform instance segmentation and return a pixel-level mask for each detected object.

These capabilities are extremely useful, but it is important to consider whether their output actually solves the robotics problem.

For example, a conventional YOLO object detector may successfully produce:

`cube -> bounding box -> confidence`

but a manipulator may actually require:

`cube -> x, y, z -> rotation -> grasp pose`

A 2D bounding box therefore does not directly provide the 3D pose of the object. Similarly, a segmentation network provides a useful object mask, but additional geometric processing is normally required to convert this information into metric 3D coordinates and orientation.

Pose or keypoint networks can provide more information, but they may still require a custom dataset and a subsequent geometric step to recover a metric object pose. Consequently, a deep-learning model can sometimes move the problem rather than completely solve it.

Machine learning can be very effective, but it may introduce unnecessary complexity for a known geometric object in a controlled environment. Models such as YOLO generally identify what and where an object is in an image, but this does not directly provide the accurate 3D position and rotation required for robotic manipulation. ML approaches may also require labelled training data, additional computational resources, and can be harder to interpret when they fail. 

he [papers with code](https://paperswithcode.com/task/object-detection) is a good place to get an overview with implementations available. The absence of a dedicated GPU does not completely prevent the use of deep learning. Modern inference frameworks can execute neural networks on CPUs and other edge-processing hardware. Frameworks such as [OpenVINO](https://docs.openvino.ai/) can target CPUs, GPUs and supported neural-processing hardware, while frameworks such as [PyTorch](https://pytorch.org/) and [TensorFlow](https://www.tensorflow.org/) also provide a variety of deployment options. Processing speed, memory consumption and power requirements should still be considered for the target robot.

> [!WARNING]
> In robotics we are deploying onto real hardware, in real time. We do not have endless cloud compute power readily available, consider your deployment hardware in software decisions. 

---
<h2 align="center">A hybrid approach</h2>

Traditional computer vision and deep learning should not necessarily be considered competing approaches. In more difficult environments they can complement one another.

For example:

`RGB image -> ML segmentation -> cube mask`

followed by:

`cube mask + depth -> 3D point cloud -> geometric pose estimation -> grasp pose`

In this approach, the neural network handles the difficult problem of identifying which pixels belong to the object, while deterministic geometric processing is used to calculate the physically meaningful position and orientation required by the manipulator.
