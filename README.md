# uneven-terrain-driver
Intro to Reinforcement learning term project

## Links 
1. Drive folder : https://drive.google.com/drive/u/1/folders/1IWBaSYP6VH7vti9cb_MgK4lh1kEbcMzq
2. First meeting minutes : https://docs.google.com/document/d/1b4IXfd1U2ebf4b-UMGvzG1uoiOsij3C3kAkuit6AaRk/edit
3. Project proposal : https://www.overleaf.com/8236317519rqfskxkjrkyn#9f34f4

## Important papers/projects
1. https://arxiv.org/abs/2107.04034 (Rapid Motor Adaptation - DP 2021)
2. https://cs.gmu.edu/~xiao/Research/Verti-Wheelers/ (Only learning based off road navigation work we know yet)
3. https://arxiv.org/pdf/2107.08325.pdf (Imitation + Reinforcement learning example)
4. https://drive.google.com/file/d/1YFdlU5zrgtFw-yxGwb9kknlHN_dyCmfw/view?usp=sharing (Wenli's slides, relevant work)
5. https://theairlab.org/offroad/ (Airlab's work on off-road driving)
 
## Installation instructions

1. Clone this repo
```
git clone https://github.com/dvij542/uneven-terrain-driver.git
```

1. Download and install unity hub from here : https://unity.com/unity-hub . Currently, only Windows (7+) and MacOS support unity game engine. Ubuntu 2022 support is a WIP
2. Open Unity hub and navigate to Installs tab from the left bar. Click on Install Editor and install Unity 2021.3 (NOTE : My version was 2021.3.24f1 and now it looks to be 2021.3.31f1, hopefully there should only be minor changes/ bug fixes from 24f1 to 31f1)
   <img width="796" alt="image" src="https://github.com/dvij542/uneven-terrain-driver/assets/43860166/2b004035-b45c-4468-b7e0-d2d033696623">
4. Click on New project. This will point to a window as shown below
   <img width="768" alt="image" src="https://github.com/dvij542/uneven-terrain-driver/assets/43860166/393032d2-daaf-4313-ba57-87b8cde918b6">
5. Once you are in the Unity editor window, close it and open from Unity Hub, point to this github repo. make sure the editor version is 2021.3. This will take some time when initializing for the first time. This should open the editor window as shown here :-
<img width="996" alt="image" src="https://github.com/dvij542/uneven-terrain-driver/assets/43860166/71c932c6-9194-458c-be98-3a7aa28d38fd">
6. If the Train_env scene is not open by default, open it from Assets/Karting/Scenes.
7. If the objets in the scene appear pink, please follow these steps to change the rendering pipeline of Unity to Universal Rendering Pipeline : [Link](https://www.youtube.com/watch?v=rtZcuAPJVwo)
8. Click on Play and if everything worked fine, it should take about 10-15s to generate a new environment and spawn the buggy car which you should be able to control with arrow keys. You can change the active display to display 1 for front camera + depth image and Display 2 for 3rd person camera view.
<img width="995" alt="image" src="https://github.com/dvij542/uneven-terrain-driver/assets/43860166/15fc01f5-5d84-4acd-9895-73926049e16f">
Enjoy!!!

## Link with VS Code

Follow these instructions : [Link](https://code.visualstudio.com/docs/other/unity) . You might need to install C# dev tools if vscode doesn't install by default on adding the plugin. Autocompletes should work if everythin worked well. To test, just double click on one of the scripts in unity and that should open the VSCode project. Check if the code is well formatted by VSCode, try typing something to check autocomplete.

## Code structure

Easist way to read code is to just directly go to individual objects and see what scripts are attached to them. 
1. If you point to Terrain object on the left pane, it should have a procedural_heightmap.cs script attached to it as shown below. Just double click on the script pointer and that should take you to that script in vscode. This is the script used to generate the procedurally random heightmap and the features (trees, rocks, grass etc)
<img width="1006" alt="image" src="https://github.com/dvij542/uneven-terrain-driver/assets/43860166/f2f5cf39-ed58-4c03-91fb-526ccfc3379c">
2. Select Main camera object and see all the scripts attached to it
3. Select Gaddi object to see all the scripts attached on it. AracdeKart.cs is the one which controls the vehicle dynamics, KartPlayerAnimation is just for animating the wheel rotations and steers. You can enable the 'Take keyboard input' checkbox to take commands from the keyboard to move the car. You can also select 'Take expert command' to get the expert expert commands from greedy controller. 
<img width="996" alt="image" src="https://github.com/dvij542/uneven-terrain-driver/assets/43860166/a29e3654-522f-4242-8500-9c522c534e5a">
4. utils/plot_traj.py can be run as a utility after each run to visualize the trajectory followed with the heightmap in the background

You can directly use the built environment inside Build-ubuntu and Build-windows. Some of the additional custom built environments can be found here: https://drive.google.com/drive/u/1/folders/1IWBaSYP6VH7vti9cb_MgK4lh1kEbcMzq

## Current WIP's 

1. Sim-to-real transfer to the RC car

## Creating the executable to use the Unity env as a gym env

Follow this [Link](https://github.com/gzrjzcx/ML-agents/blob/476504b547b39e0bd6974d4b2951dd0e97c95f79/docs/Learning-Environment-Executable.md)

## Build with Ubuntu
1. Make sure you change the permission of the Unity directory from Read-Only.
2. Install Linux Build System in Unity Hub
3. Follow the above link and choose Linux as the target platform while building

## Making a gym environment

```
conda create -n terrain_env python=3.6
conda activate terrain_env
git clone https://github.com/gzrjzcx/ML-agents.git
cd ML-agents/
cd ml-agents-envs/
python3.6 -m pip install -e ./
cd ../
cd ml-agents
python3.6 -m pip install -e ./
```

## Installing and running executable on Ubuntu

Ref: [Link](https://alexisrozhkov.github.io/unity_rl/). Run the following commands 
```
pip install mlagents==0.28.0
cd Build-ubuntu
sudo chmod 777 *
cd ..
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
mlagents-learn config.yaml --env "Build-ubuntu/exec" --run-id ubuntu_try15 --num-envs 10 --resume
```

## ROS2 interface

You can also interact with the simulator as a ROS2 node. Full ROS2 integration is a WIP. To get started, follow the given steps:-

1. Install [ROS2](https://docs.ros.org/en/foxy/Installation.html) (Foxy, Iron or any other version)
2. Install dependencies :-
```
sudo apt-get install ros-<ROS_VERSION>-ackermann-msgs
sudo apt-get install ros-<ROS_VERSION>-sensor-msgs
sudo apt-get install ros-<ROS_VERSION>-geometry-msgs
```
3. If on linux, also install [mono-project](https://www.mono-project.com/download/stable/#download-lin) required for reading images within the application
4. Change permissions of the ros2-env built environment with :-
```
cd ros2-env
sudo chmod 777 *
cd ..
```
5. Your params.yaml file should be in the same directory you run from. It should contain the path to the heightmap, the spawn position and which sensors to use. Run 'ros2_interface.py'
```
source /opt/ros/<ROS_VERSION>/setup.bash
python3 ros2_interface.py
```
This should open a new simulator session with ROS2 wrapper. You can see the available topics with 'ros2 topic list' in a new terminal and it should list all sensor topics, pose, velocity and acceleration topics and the /cmd topic from where it listens to acceleration and throttle commands
6. (optional) You can launch the example keyboard teleop script under Scripts/keyboard_controller.py that publishes to '/cmd' topic from the arrow keys to control the car. You may need to install pygame to run the script
```
source /opt/ros/<ROS_VERSION>/setup.bash
python3 Scripts/keyboard_controller.py
```
7. (optional) You can open rviz2 to visualize the sensor readings

```
rviz2
```
