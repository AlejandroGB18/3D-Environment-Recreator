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

## SETUPS
The following is a description of how to set up the working environments for the TIAGo simulation packages in Gazebo and the package developed for the 3D transformation. The order of creation of each one is indistinct, however, the execution of these packages has a specific order, which we will discuss later in the run section.

### SETUP: Ros2_ws
To generate the workspace with the 3D Transformation package on your device, it is quite simple. Since we don't need to create anything from scratch, and using GitHub commands, the process will be as follows:

1. Create a workspace with a _src_ directory.
   ```bash
   cd
   mkdir -p Ros2_ws/src  ## This is a example name, you can use other.
   ```
2. Go to the _src_ folder and **_git clone_** the repository superficially, because we are only interested in keeping the _ros2_ws_ folder to build the package.
   ```bash
   cd Ros2_ws/src
   git clone --no-checkout --filter=blob:none https://github.com/AlejandroGB18/3D-Environment-Recreator.git
   ```
3. Once you are inside the newly cloned repository, initialize _sparse-checkout_.
   ```bash
   cd 3D-Environment-Recreator
   git sparse-checkout init --cone
   ```
4. Configure the folder to be cloned, in this case _ros2_ws_.
   ```bash
   git sparse-checkout set ros2_ws
   ```
5. Checkout from the _main_ branch to download the selected files.
   ```bash
   git checkout main
   ```

### SETUP: Environment_tiago
For the gazebo simulation environment with TIAGo integrated in it we rely on the work of Francisco Rico, who previously worked with a Gazebo simulation with TIAGo, so we will explain the construction of his project on our devices and how to link it with our 3D transformation project. 

_**NOTE:** Due to certain dependencies and requirements that his repository requests when configuring it, we will work it in a particular workspace which not only allows to streamline the construction of changes in the transformation environment (as Francisco's work has 38 packages), but also the reduction of errors._

1. Create a workspace only by creating a directory with a _src_ directory within.
   ```bash
   cd
   mkdir -p environment_tiago/src  ## This is a example name, you can use other.
   ```
2. Then add the packages developed by Francisco Rico through _git clone_.
   ```bash
   cd environment_tiago/src
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

## RUNS
To be able to execute the packages of the working environments, it is necessary to understand the order of execution, it will always be the same, even if you decide to make modifications, the order is as follows: 
1. _environment_tiago_
2. _ros2_ws_

Below is indicated how to run each one.
### RUN: environment_tiago
#### Running the Gazebo environment
1. Use the environment created for the simulation and build the workspace.
   ```bash
   cd ~/environment_tiago
   colcon build
   ```
2. Activate the worspace to be able to use the packages.
   ```bash
   source install/setup.bash
   ```
3. Once activated, run the launch file, if all dependencies and processes were performed correctly, a Gazebo window will be executed as shown below.
   ```bash
   ros2 launch br2_tiago sim.launch.py is_public_sim:=True
   ```
<div align="center">
  <a href="https://youtu.be/x29LwgAedmQ">
    <img src="https://github.com/user-attachments/assets/2bb27509-8756-4f74-ad18-8c463526727e" alt="gazebo_video" width="500">
  </a>
</div>

#### TIAGo Teleoperation
1. In order to move the TIAGo in the simulation, in a new window linked to the working environment you must execute the following line.
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=key_vel
   ```
A video demonstration of how to do this is below.

<div align="center">
  <a href="https://youtu.be/YGhs_Fo3G1w">
    <img src="https://github.com/user-attachments/assets/7e325336-c7cb-43cf-94aa-454a6869c8e9" alt="teleop_video" width="500">
  </a>
</div>

#### TIAGo Grasping
1. To move the TIAGo arm in the simulation, you need in a new tab linked to the workspace to rebuild and activate the packages followed by the launch to moveit.
   ```bash
   colcon build
   source install/setup.bash
   ros2 launch tiago_moveit_config moveit_rviz.launch.py
   ```
A video demonstration of how to do this is below.

<div align="center">
  <a href="https://youtu.be/QFFMCqCtOJQ">
    <img src="https://github.com/user-attachments/assets/20abc207-b55c-4d22-9c68-e24015e13331" alt="grasping_video" width="500">
  </a>
</div>





### RUN: ros2_ws


