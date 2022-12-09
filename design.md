---
layout: default
title: Design/Implementation
---

# Design

1. Formula Recognition

2. Path Planning

## Formula Recognition

*The repo serves as a server node to recognize a math expression, transfer it into an expression tree,
evaluate the result and publish it to ROS topic.*

The repository contains a ROS package, relying on `cv_bridge`, `geometry_msgs`, `intera_interface`, `rospy` and `std_msgs`.

There are two nodes here:

### `src/intera_cam.py`

This node serve as an test node, with no topic or service interaction with the controller or
path planning part. It:

- retrieves images from one of the Sawyer's cameras (by default `head_camera`, but testing shows the `right_hand_camera` on the wrist is better),
- feeds them into a pretrained model (defined in `src/model.py` whose trained parameters are indexed by the `model.lst`) to interpret the Latex expression,
- parse the expressions,
- evaluate results of the parsed expression, and
- respond to a client's query with the results.

### `src/server.py`

This node relies on `src/intera_cam.py`, shouldering almost the same functionalities but:

- will not start CV recognition until a service request from `/formula/get_solution_int` or `/formula/get_solution_str`
- will run the recognition model continuously after a request is received until a specified number of answers is generated
- will return the service client with the **most frequent** answer among all the answers as many as the specified number
- will turns itself off to the silent mode and waiting for a new request again

### ROS Node details

#### A. Preparation

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

#### B. Launch file and parameters

#### `launch/start.launch`

This starts the `intera_cam.py` node, with the following customizable parameters:

- **`encoder`**: path to the encoder model's XML file.
- **`decoder`**: path to the decoder model's XML file.
- **`vocab`**: path the decoder model's `vocab.json`.
- **`camera`**: specify which camera to use, either `head_camera` (default) or `right_hand_camera` (recommended).
- **`confidence`**: confidence level, below which the parsed expression will be discarded.
- **`preprocessing`**: the image preprocessing method, either `crop` or `resize` (default).

#### `launch/serve.launch`

This starts the `server.py`

- **`encoder`**: path to the encoder model's XML file.
- **`decoder`**: path to the decoder model's XML file.
- **`vocab`**: path the decoder model's `vocab.json`.
- **`camera`**: specify which camera to use, either `head_camera` or `right_hand_camera` (default and recommended).
- **`confidence`**: confidence level, below which the parsed expression will be discarded.
- **`preprocessing`**: the image preprocessing method, either `crop` (default) or `resize`.

#### C. ROS Service

The ROS service requires a number as the request, which specifies how many confident answers the CV model should
produced until it returns a result. The result will be the most frequent one among the answers. There are two
service channels, differing only by the returned types:

#### `/formula_rec/get_solution_int`

This returns an integral result. Should the answer is a floating point number, it will round the number
to the nearest integer.

#### `/formula_rec/get_solution_str`

This returns a floating point number as a string. Two digits after decimal point will be preserved.

### Logistics

Depending on Intel Open Model Zoo's models and sample implementation.

Open-sourced under Apache licence.

## Path Planning

*This repository serves as a ROS package for the Group 5's final project.*

### ROS Package Description

The ROS package contained in this repository is called `path_planning`, which shoulders
the following functionalities:

1. Plan paths for the robot so that it can move its own camera to a desired place to read 
   the question and then move its end-effector to write down the answer,
2. Query [the CV node](https://github.com/PlayWithRobot-Berkeley/FormulaRec) to parse the mathe
   expressions it can see and retrieve the corresponding answer, and
3. Connect with the MoveIt! action server to control the robot to execute the planned path.

### Deployment

#### A. Before running the node

[The CV node](https://github.com/PlayWithRobot-Berkeley/FormulaRec) shall be 
cloned and prepared (models being downloaded, etc., see that repo's README.md for
more details)

**NOTE**

Both this repository and the CV node's repository contain individual packages, but **not** ROS workspace. Hence, it would be advisable to **first create a workspace**, 
clone both repositories into the `src` directory and then `catkin_make`, so that
both packages can reside in the same workspace, making the life easier (and neater).

Since the Sawyer robotic arm are to be used as well, do not forget to include the `intera.sh` as well.

In summary, basicall you need to:

```sh
mkdir final_project # the workspace
cd final_project
mkdir src
cd src
git clone https://github.com/PlayWithRobot-Berkeley/FormulaRec.git
git clone https://github.com/PlayWithRobot-Berkeley/PathPlanning.git

# THEN FOLLOW THE README in FormulaRec to complete the CV setup

cd .. # back to final_project
ln -s /opt/ros/eecsbot_ws/intera.sh .

catkin_make
```

#### B. Run the node

1. Start the action server
   
   ```
   rosrun intera_interface joint_trajectory_action_server.py
   ```

2. Run MoveIt! via RVIZ **in a new terminal**
   
   ```sh
   roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
   ```

3. Start the CV server node **in a new terminal**
   
   ```sh
   roslaunch formula_rec server.launch
   ```

4. Finally, run the path planning node **in a new terminal**
   
   ```sh
   rosrun path_planning cartesian_test.py
   ```

### Collaboration

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

Two `YAML` configuration files are here:

- **`camera_capture_pose.yml`**: recording the pose where the robotic arm should move to right after it
  is launched so that its `right_hand_camera` can see the question best
- **`digits.yml`**: storing each character's glyph

#### B. An Ideal Collaboration Cycle

1. Record what is to be completed in our [project watchboard](https://github.com/orgs/PlayWithRobot-Berkeley/projects)

2. Checkout to a new branch:
   
   ```sh
   git checkout -b dev_[something to do]
   git push --set-upstream origin dev_[something to do]
   ```

3. `git commit -a -m "[commit message]"` for several times

4. Before push, always remember to
   
   ```sh
   git pull -r
   ```

5. Then,
   
   ```sh
   git push
   ```

6. Repeat the previous two steps for several times until something is completed

7. Start a PR and **link the PR back to the item in the project watchboard created in the first step**
