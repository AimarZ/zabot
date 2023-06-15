<p align="center"><img src="zabotReal.gif"/></p>

# Zabot: Robotic Garbage Sorter

## What is it about?

This project aims to develop the logic and control necessary for a robotic arm, so that it is capable of detecting, collecting and classifying garbage from its environment. It has 3 branches:
    
   - `master` branch: Cube type object sorter in a Unity simulation environment.
   - `real_garbage` branch: Garbage sorter in a Unity simulation environment. 
   - `real_world` branch: Object sorter in a real environment. It is capable of detecting complex objets, but the classification is made by their color.
    
> Note: This Readme explains the project in the `real_world` branch. If you are interested in the other projects, switch to the desired branch.

The robot that has been used is Niryo Ned. The robot has been installed on a circular table. Also, a RealSense camera has been installed above the robot. That way, the system is able to detect the objects that are nearby. 2 different segmentation techniques have been inplemented to process the images:
  - Color segmentation: It uses methods from the CV2 Python package. It is fast but it only detects objects that have predefined colors.
  - Segmentation through SAM: It uses the Segment Anything Model developed and trained by facebook (https://github.com/facebookresearch/segment-anything). It is slower but it is capable of detecting objects of any shape and color.


## Requirements

  - ROS Melodic
  - Python 2 and Python 3.8
  - The following packages:
    ```bash
        pip install 'git+https://github.com/facebookresearch/segment-anything.git'
        pip install -q roboflow supervision
      ```
  - SAM requires `python>=3.8`, as well as `pytorch>=1.7` and `torchvision>=0.8`.
  - Download the model checkpoint `vit_h`: [ViT-H SAM model.](https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth)
  
## How to try it?

1. Clone the repository.
2. Place `imageMaskv3.jpg`, `imageMaskv4.jpg` and `sam_segment_blob.py` in `Desktop/`
3. Download [ned_ros.](https://github.com/NiryoRobotics/ned_ros) packages.
4. Place `niryo_pick_and_place` package inside a catkin workspace, open a terminal and run the following commands:
    ```bash
    source devel/setup.bash
    catkin_make
    ```
5. 
6. Next, the ROS TCP connection needs to be created. Select `Robotics -> ROS Settings` from the top menu bar.

   In the ROS Settings window, the `ROS IP Address` should be the IP address of your ROS machine.

   - Find the IP address of your ROS machine. In Ubuntu, open a terminal window, and enter `hostname -I`.
5. Select `Robotics -> Generate ROS Messages...` from the top menu bar.

   ![](img/2_menu.png)

   In the ROS Message Browser window, click `Browse` next to the ROS message path. Navigate to and select the ROS directory of this cloned repository. This window will populate with all msg and srv files found in this directory.

6. Go to https://www.dropbox.com/sh/i4ymvbv1k4jel83/AAAvNKmJ5uzqyKQza3c2NXvCa?dl=0 and download the desired trained models. The `master` branch (cube sorter) works with the following 2 models:
    - Niryo_1by1_model_class_ep64.tar
    - Niryo_1by1_model_translation_ep52.tar
    
7. Place the downloaded models inside `ROS/src/niryo_moveit/models/`.
8. Open a terminal in the `ROS/` folder and run the following commands:
    ```bash
    source devel/setup.bash
    catkin_make
    ```
9. Once completed, run this command: 
    ```bash
    roslaunch niryo_moveit part_3.launch
    ```
10. Go back to Unity and hit the Play button. You can use the `Randomize` button to change the distribution of the cubes, and hit `Publish` to communicate with ROS and start the pose estimation pipeline.
