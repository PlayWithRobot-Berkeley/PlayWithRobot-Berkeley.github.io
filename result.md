---
layout: default
title: Result/Conclusion
---

# Deployment

Install the [Intel OpenVINO Open Model Zoo (OMZ)](https://docs.openvino.ai/latest/model_zoo.html)
before going through the following steps! 

## A. Preparation

```sh
mkdir final_project # the workspace
cd final_project
mkdir src
cd src
git clone https://github.com/PlayWithRobot-Berkeley/FormulaRec.git
git clone https://github.com/PlayWithRobot-Berkeley/PathPlanning.git
cd FormuleRec
omz_downloader --list model.lst # download the pretrained models
cd ../.. # back to final_project
ln -s /opt/ros/eecsbot_ws/intera.sh .
catkin_make
```

The README files in
[FormulaRec](https://github.com/PlayWithRobot-Berkeley/FormulaRec)
and
[PathPlanning](https://github.com/PlayWithRobot-Berkeley/PathPlanning)
gives a detailed instructions and are strongly recommended to go through. 

## B. Run the codes

1. Enable the robotic arm
   
   ```sh
   cd [workspace dir]
   ./intera.sh # Enter the SSH session
   rosrun intera_interface enalbe_robot.py -e
   exit # exit the SSH session
   ```

2. Test the robotic arms' movibility and the cameras
   
   ```sh
   roslaunch intera_examples sawyer_tuck.launch
   rosrun intera_examples camera_display.py -c right_hand_camera
   ```

3. Start the action server
   
   ```
   rosrun intera_interface joint_trajectory_action_server.py
   ```

4. Run MoveIt! via RVIZ **in a new terminal**
   
   ```sh
   roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
   ```

5. Start the CV server node **in a new terminal**
   
   ```sh
   roslaunch formula_rec server.launch
   ```

6. Finally, run the path planning node **in a new terminal**
   
   ```sh
   rosrun path_planning cartesian_test.py
   ```

# Results and Evaluations

We lists our results as follows: 

### Success demo 1

This video demonstrates that our robot can successfully perform addition: 

<video id="video" controls="" preload="auto">
    <source id="mp4" src="/assets/video/addition_single_digit.mp4" type="video/mp4">
</video>

### Success demo 2

The robot can also write down a solution contains multiple digits: 

<video id="video" controls="" preload="auto">
    <source id="mp4" src="/assets/video/subtraction_multiple_digits.mp4" type="video/mp4">
</video>

## Summary of our capabilities

- [x] Integral addition, substration, division and multiplication
- [x] Fraction operations (rounded to the nearest integer)
- [x] Complicated algebra including parentheses and power operations
- [ ] Matrix operation: _no due to the CV model tends to fail in recognizing handwritten matrices_
- [ ] Writing floating point numbers

# Limitations

The major limitation is the robustness and the capability of the CV model we used. 
Although it is claimed this model is able to recognize **any** Latex formula, however,
due to its somehow "oversimplified" AutoEncoder architecture, its capability and
robustness deserve further improvement. Our observed its recognition rate significantly
droped when there is a fraction in the mathematical expression, or it might easily
mistake a cross product for character "x". The illumination and the color of the dry
marker we used to write the expression also matters. 

There are more advanced methods to overcome the aforementioned limitations, especially
from the CV side. An alternative (and better way) is to use RNN or vision transformer
which view the algebra expression as a time squential data. Their published paper claims
a state-of-the-art outcome. Yet, due to a lack of sufficient training resources and
since the machine learning is not a major focus of this course, we leave this improvement
for future follow-up. 

### Failure demo

This video demonstrates Sawyer failed to recognize the formula due to our purple handwriting.

<video id="video" controls="" preload="auto">
    <source id="mp4" src="/assets/video/failure_case_1.mp4" type="video/mp4">
</video>
