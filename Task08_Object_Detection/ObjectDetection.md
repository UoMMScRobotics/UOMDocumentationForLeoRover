<h1 align="center"> Object Detection </h1>

There are many approaches to identifying objects for a robot to interact with.  This is largely dependent on what type of object the robot is interested in.  For example, human detection may be of benefit during path planning and navigation in dynamic environments, whereas using text recognition may be more suitable for identifying which pill bottle is correct for an elderly person with a robot assistant. Lastly, it's important to identify what output results are required for your application. Different approaches give outputs such as bounding boxes around objects or segmentation masks. 

**THESE ARE ONLY SUGGESTIONS OR SIGNPOSTING TO FURTHER RESOURCES, THE CHOICE OF APPROACH IS DOWN TO YOU**

## Camera Real World Considerations ##
When using any sensor, it is important to ensure your sensors are well calibrated. For cameras, there will be instructions provided by the manufacturer on how to best calibrate your device. Examples include [realsense docs](https://dev.intelrealsense.com/docs/self-calibration-for-depth-cameras) and [Azure Kinect docs](https://learn.microsoft.com/en-us/azure/kinect-dk/use-calibration-functions).
Also, several factors, such as lighting, reflections from objects, and shadows can affect your camera images. It may be beneficial to consider if these are important to your application and how to tackle them.

## Visual ##

Typically, visual object detection relies on characteristics of an image to segment a portion of the image.  For example, the brightness or colour may be used to identify objects.  Furthermore, filters such as edge detection may be used to help segment or perform further tasks such as text recognition.  It may be beneficial to convert from RGB colour space to HSV colour space.  Many online tutorials and numerous textbooks are available that discuss image processing.

## OpenCV ##

[OpenCV](https://opencv.org/) is a powerful suite of tools for object recognition, with many examples and tutorials (which you may wish to read).  It is the backbone of many image processing workflows, and conveniently there are [packages available](https://github.com/ros-perception/vision_opencv) to convert ROS images messages to OpenCV images for manipulation.  With a calibrated camera, is it even possible to find the 3D pose of a highly textured object from 2D images.

## Fiducial Markers ##

It is possible to make objects more recognisable by using images which are readily observed and understood by a machine.  In this case, markers such as barcodes, QR codes or spatial fiducial markers such as [AprilTag](https://april.eecs.umich.edu/software/apriltag) are ideal.  The benefit is that these can be readily utilised to provide pose estimation and identification of objects and locations, the compromise is that a human or other agent needs to attach and identify each object/tag _a priori_.

## Depth ##

Depth (either from stereo vision, lidar, RGBD etc) can be used to aid in segmentation of a scene.  For example a table top can be modelled as a plane and removed to better filter out where an object may be.  It is likely that these depth images or point clouds will need to be converted into a 3D voxel occupancy grid when being used with [MoveIt](https://moveit.ros.org/) or other existing packages.
Depth images can also be processed directly to estimate object location using approaches such as background subtraction. There are also libraries for processing point cloud and depth data, such as [open3D](https://www.open3d.org/docs/release/introduction.html) and [point cloud library](https://pointclouds.org/), which could be useful.

## Machine Learning ##

With no GPU onboard, it may appear as though Machine Learning (ML) and more sophisicated machine vision approaches are not particularly useful in this scenario.  However, companies such as Intel and Google have USB devices known as Tensor Processing Units (TPU) which can be used to provide machine learning capabilities without the need for a high-end graphics card.  The neural network models have been trained in a traditional method with large GPU clusters and datasets, but the eventual weights of the nodes (and a representation of the architecture) represent a much smaller computational footprint, which can run on a USB stick at reasonable speeds. 

Intel's Neural Compute Stick 2 can be loaded with existing OpenVino models which can then be accessed via Python, with [Intel made models](https://docs.openvino.ai/2022.3/omz_models_group_intel.html) and [Publicly available models](https://docs.openvino.ai/2022.3/omz_models_group_public.html).  Google's Coral Accelerator line of products also [provides models](https://coral.ai/models).  It is also possible to use models using different frameworks (e.g., [PyTorch](https://pytorch.org/), [TensorFlow](https://www.tensorflow.org/)).  This enables techniques such as object tracking, detection, segmentation, and classification, which may make your object detection more robust.

The [papers with code](https://paperswithcode.com/task/object-detection) is a good place to get an overview of machine learning techniques with implementations available. Most modern frameworks such as [PyTorch](https://pytorch.org/) or [TensorFlow](https://www.tensorflow.org/) can be run on a CPU with increased processing time. It is important to consider both the required memory and computing time if you decide to use machine learning.

