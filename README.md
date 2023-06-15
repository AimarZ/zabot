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
5. Depending if you want to use SAM or color segmentation, change the `use_sam` variable value inside `niryo_pick_and_place/scripts/pose_estimation.py`.
6. Run this command:
    ```bash
    roslaunch niryo_pick_and_place pick_and_place.launch
    ```
7. In case you want to use SAM, open another terminal in the `Desktop/` folder, and run the following command:
    ```bash
    python3.8 sam_segment_blob.py
    ```
8. Finally, run the client through the following command in the catkin workspace:
    ```bash
    rosrun niryo_pick_and_place pick_and_place.py
    ```
