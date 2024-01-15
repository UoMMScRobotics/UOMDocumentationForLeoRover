<h1 align="center"> Object Detection </h1>
The [Navigation Stack](https://navigation.ros.org/) (Nav2) for ROS2 provides a ready built solution for mapping and navigation for mobile robots.

## Visual ##

Typically, visual object detection relies on characteristics of an image to segment a portion of the image.  For example, the brightness or colour may be used to identify objects.  Furthermore, filters such as edge detection may be used to help segment or perform further tasks such as text recognition.  It may be beneficial to convert from RGB colour space to HSV colour space.  Many online tutorials and numerous textbooks are aavilable which discuss image processing.

## OpenCV ##

OpenCV is a powerful suite of tools for object recognition, with many examples and tutorials.  It is the backbone of many image processing workflows, and conveniently there are [packages available](https://github.com/ros-perception/vision_opencv) to convert ROS images messages to OpenCV images for manipulation.

## Fiducial Markers ##

It is possible to make objects more recognisable by using images which are readily observed and understood by a machine.  In this case, markers such as barcodes, QR codes or spatial fiducial markers such as [AprilTag](https://april.eecs.umich.edu/software/apriltag) are ideal.  The benefit is that these can be readily utilised to provide pose estimation and identification of objects and locations, the compromise is that a human or other agent needs to attach and identify each object/tag _a priori_.

## Depth ##



## Machine Learning ##

With no GPU onboard, it may appear as though Machine Learning (ML) and more sophisicated machine vision approaches are not particularly useful in this scenario.  However, companies such as Intel and Google have USB devices known as Tensor Processing Units (TPU) which can be used to provide machine learning capabilities without the need for a high-end graphics card.  The neural network models have been trained in a traditional method with large GPU clusters and datasets, but the eventual weights of the nodes (and a representation of the architecture) represent a much smaller computational footprint, which can run on a USB stick at reasonable speeds.

Intel's Neural Compute Stick 2 can be loaded with existing OpenVino models which can then be accessed via Python, with [Intel made models](https://docs.openvino.ai/2022.3/omz_models_group_intel.html) and [Publicly available models](https://docs.openvino.ai/2022.3/omz_models_group_public.html).  Google's Coral Accelerator line of products also [provide models](https://coral.ai/models).  It is also possible to use models using different frameworks (e.g. TensorFlow).  This enables techniques such as object tracking, detection, segmentation and classification which may make your object detection more robust.