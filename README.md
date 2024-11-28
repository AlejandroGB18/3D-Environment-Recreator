# INTERACTIVE DIGITAL RECREATION [ROS2 - Gazebo]
![Transformation of Physical Environments through Integrated Sensor Fusion and Computer Vision for Interactive Digital Recreation](https://github.com/user-attachments/assets/978d1a82-4d0d-4813-bcdf-6324d8f05f72)


This repository contains all the nodes, models for gazebo environment, launch files, simulations and all the archictechture verify the theory and hypothesis related to the research "Transformation of Physical Environments through Integrated Sensor Fusion and Computer Vision for Interactive Digital Recreation" thus developing a in order to development a minimun viable product.

As you can see, we have three folders: **_Gazebo_models_**, **_environment_tiago_** and **_ros2_ws_**. The first one contains all the mesh models used in the gazebo environment (necessary to insert them in our simulation), the second one, contains the simulation worlds along with the TIAGo models to use them in the simulation (These elements were obtained from Francisco Rico's book). Finally, **_ros2_ws_** contains all the nodes and launch files created for the 3D reconstruction. Each folder will be explained to create this environment in your own device.

## Requirements
* Ubuntu Systems (NOTE: This project was developed for Ubuntu 22.04 (Jammy), I recommend installing this version to reduce the existing incompatibilities in Ubuntu 24.04 (Noble) for the time being).
* ROS 2 - Humble. [How to install ROS 2?](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
* Gazebo Classic.

## SETUP: Gazebo_models
When you have finally installed Gazebo on your device, a hidden folder _.gazebo_ will be created. This folder is located in \home and may contain the following folders:

<div align="center">
  <img src="https://github.com/user-attachments/assets/2a2ac833-d014-4c9d-9fe1-830bf6e2fb6e" alt="Screenshot from 2024-11-28 10-48-08" width="500">
</div>

The model folder is the one we are interested, in it we will put the models from the repository folder _Gazebo_models_ (it contains the models for: **apple**, **can** and **coffee**). If you doing this, you can use the 3D models in gazebo.

## SETUP: Ros2_ws


## SETUP: Environment_tiago
For the gazebo simulation environment with TIAGo integrated in it we rely on the work of Francisco Rico, who previously worked with a Gazebo simulation with TIAGo, so we will explain the construction of his project on our devices and how to link it with our 3D transformation project. 

_**NOTE:** Due to certain dependencies and requirements that his repository requests when configuring it, we will work it in a particular workspace which not only allows to streamline the construction of changes in the transformation environment (as Francisco's work has 38 packages), but also the reduction of errors._

1. Create a workspace only by creating a directory with a _src_ directory within.
  ```bash
   cd
   mkdir -p environment_tiago/src  ## This is a example name, you can use other.
   cd environment_tiago/src
  ```
2. Then add the packages developed by Francisco Rico through _git clone_.
   ```bash
   git clone -b humble-devel https://github.com/fmrico/book_ros2.git
   ```
3. In this workspace, there are many packages with dependencies on other packages not part of the ROS 2 Humble distribution. To add the sources of these packages to the workspace, we will use the vcstool.
   ```bash
   cd ~/environment_tiago/src
   vcs import . < book_ros2/third_parties.repos
   ```
4. Before building, let's use _rosdep_ to install any package missing  to build the entire workspace.
   ```bash
   cd ~/environment_tiago
   rosdep install --from-paths src --ignore-src -r -y
   ```
5. Build the workspace, always from its root, using the colcon command.
   ```bash
   cd ~/environment_tiago
   colcon build --symlink-install
   ```
6. To use the packages of the workspace, activate it.
   ```bash
   source ~/environment_tiago/install/setup.bash
   ```
