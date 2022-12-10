---
layout: default
title: Design/Implementation
---

# Design

Naturally, the project is splitted into two parts, each as a ROS package: 
1. Formula recognition, i.e., the "CV" part
1. Path planning part

## I Formula Recognition

The package relyies on `cv_bridge`, `geometry_msgs`, `intera_interface`, `rospy` and `std_msgs`.

There are two nodes here:

#### `src/intera_cam.py`

This node serve as an test node, with no topic or service interaction with the controller or
path planning part. It:

- retrieves images from one of the Sawyer's cameras (by default `head_camera`, but testing shows the `right_hand_camera` on the wrist is better),
- feeds them into a pretrained model (defined in `src/model.py` whose trained parameters are indexed by the `model.lst`) to interpret the Latex expression,
- parse the expressions,
- evaluate results of the parsed expression, and
- respond to a client's query with the results.

#### `src/server.py`

This node relies on `src/intera_cam.py`, shouldering almost the same functionalities but:

- will not start CV recognition until a service request from `/formula/get_solution_int` or `/formula/get_solution_str`
- will run the recognition model continuously after a request is received until a specified number of answers is generated
- will return the service client with the **most frequent** answer among all the answers as many as the specified number
- will turns itself off to the silent mode and waiting for a new request again. 

### Tech details

#### A. CV Methodolgy

The CV model we used are opensourced by Intel (see [B. Preparation](#b--preparation) for details). It is
an AutoEncoder. We found it is not very robust and requires the input images to be properly caputred and
tuned. Hence, we set up an image-preprocessing pipeline as the callback function for each frame we retrieve
from the camera: 
1. Transferring the image into grayscale for further preprocessing,
1. Resize or crop the image to a default size, according to the runtime argument,
1. Blur the image using Gaussian kernel to filter the high-frequence noises, and
1. Taking adaptive thresholding for recongnition

<center>

<img width='70%' src='/assets/img/cv_pipeline.jpg'/>

</center>

_**Upper left**: the raw image; **upper right**: after cropping; 
**lower left**: gaussian blur + grayscale; **lower right**: thresholding._



Actually, the pipeline contains two callback in order to give a better runtime visualization. The first
display the grayscale image directly to the users so that we can understand what is in front of the camera
easier; the second shows the results after resizing/cropping, blurring and thresholding to debug the CV codes.



#### B. Preparation

After cloning the repository, the first thing is download the pretrained models. The downloading relies on [Intel's Open Model Zoo Downloader](https://docs.openvino.ai/latest/omz_tools_downloader.html#doxid-omz-tools-downloader).
After the downloaded is installed, you may run

```sh
omz_downloader --list model.lst
```

which will download all the necessary models under the package's root directory. Four models will be downloaded: 
recognizing handwriten or printed characters, and each contains both the FP32 and FP16 virations. The directory
storing all the models are named `intel`. For each model, there will be two `.XML` files, one for the encoder and
the other for the decoder, whose paths will be used to launch the node.

Of course, never forget to `catkin_make` and `source devel/setup.bash`.

Moreover, the Sawyer robotic arm should be started. Occasionally, the camera may fail to response. You may need to
first SSH into the Sawyer and re-enable the robot, and then **exit the SSH session** to launch the node again.

#### C. Launch file and parameters

##### `launch/start.launch`

This starts the `intera_cam.py` node, with the following customizable parameters:

- **`encoder`**: path to the encoder model's XML file.
- **`decoder`**: path to the decoder model's XML file.
- **`vocab`**: path the decoder model's `vocab.json`.
- **`camera`**: specify which camera to use, either `head_camera` (default) or `right_hand_camera` (recommended).
- **`confidence`**: confidence level, below which the parsed expression will be discarded.
- **`preprocessing`**: the image preprocessing method, either `crop` or `resize` (default).

##### `launch/serve.launch`

This starts the `server.py`

- **`encoder`**: path to the encoder model's XML file.
- **`decoder`**: path to the decoder model's XML file.
- **`vocab`**: path the decoder model's `vocab.json`.
- **`camera`**: specify which camera to use, either `head_camera` or `right_hand_camera` (default and recommended).
- **`confidence`**: confidence level, below which the parsed expression will be discarded.
- **`preprocessing`**: the image preprocessing method, either `crop` (default) or `resize`.

#### D. ROS Service

The ROS service requires a number as the request, which specifies how many confident answers the CV model should
produced until it returns a result. The result will be the most frequent one among the answers. There are two
service channels, differing only by the returned types:

##### `/formula_rec/get_solution_int`

This returns an integral result. Should the answer is a floating point number, it will round the number
to the nearest integer.

##### `/formula_rec/get_solution_str`

This returns a floating point number as a string. Two digits after decimal point will be preserved.

## II. Path Planning

The ROS package shoulders the following functionalities:
1. Plan paths for the robot so that it can move its own camera to a desired place to read 
   the question and then move its end-effector to write down the answer,
2. Query [the CV node](#i-ros-node-details) to parse the mathe
   expressions it can see and retrieve the corresponding answer, and
3. Connect with the MoveIt! action server to control the robot to execute the planned path.

### Tech Details

#### A. Code structure

Recall this is **a ROS package**, so basically the code structure is quite standarized following
the ROS's paradigm:

##### `src` directory

The source codes, where the python file as the entry to the path planning is the `cartesian_test.py`.
The file imports the `digit_path.py` which is in charge of gernerating each individual digit's path
so that this file can connect them together to form a complete motion.

*The other two python files: `gripper_test.py` and `multipose.py` are used for testing and debugging.
They instructs the robotic arm to complete simple task such as closing the gripper or moving its
end-effector following a certain path to demonstrate the basic functionalities of a robotic arm is
normal (so that we can know it is our codes that corrupts X_X).*

##### `config` directory

`YAML` configuration files are here:

- **`digits.yml`**: storing each character's glyph


#### B. Path Planning

The `config/digits.yml` stores the local path to write down each digit. Here,
a path is defined as a sequence of **catesian** waypoints. When the node received
the answer from the CV part, it retrieves the local path to write each single
digit and adds offsets to them according to their positions to compute a global
path. Also, it deserves notice that the path is not on a plane. When the end-
effector enters a digit's local path, the Z-axis (height) will have a negative
offset to make the end-effector lower; when the end-effector leaves the digit and
starts to be transfered to the next digit's position, the waypoints' Z-axis will
observe a positive offset to disengage the dry marker from the white board.  
