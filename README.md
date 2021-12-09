# ROS-manipulator-panda
BARAM ROS seminar
KwangWoon University BARAM 2021 winter seminar Using franka panda model

## Requirements
-----------
* ROS1 (melodic)
* Eigen (more than 3)
* RBDL & RBDL URDFReader (not ORB version)
* Mujoco (MUlti-JOint dynamics with COntact)
* GLEW
* pthread
* OpenGL

## Installations

1. ROS1
```
sudo apt install ros-melodic-desktop-full
```
***
2. Eigen (Eigen is automatically installed when installing Ubuntu)

You can check either the Eigen is installed or not by entering the command
```
pkg-config --modversion eigen3
```
if the Eigen library is successfully installed, you will see the version of Eigen. Or not, just enter the command below to terminal
```
sudo apt install libeigen3-dev
```
***
3. RBDL
```
git clone https://github.com/rbdl/rbdl
cd rbdl
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -DRBDL_BUILD_ADDON_URDFREADER=ON -DRBDL_USR_ROS_URDF_LIBRARY=OFF ..
make all
make install
sudo ldconfig
```
***
4. Mujoco

+ Mujoco installation link : <https://mujoco.org/download>

If you enter the link successfully, all you need to do is just push the button which has a name of "Linux"
```
cd
mkdir Simulations
```
After download the file, you can Extract the file to the location of "~/Simulations"
***
5. GLEW
```
sudo apt install libglew-dev
```
***
6. pthread
> pthread will be installed automatically when you first install the ubuntu to your pc
***
7. OpenGL
```
sudo apt install build-essential
sudo apt install freeglut3-dev libglu1-mesa-dev mesa-common-dev
```
After that you can check either the OpenGL is successfully installed or not by entering the command below
```
ls /usr/include/GL
```

## Contents
-----------

```sh
├─include
│   | control_math.h
│   | controller.h
│   | model.h
│   | roswrapper.h
|   └─trajectory.h
├─launch
|   └─display.launch
├─model
│   ├─crane (you can ignore this directory. It is just tested model)
│   ├─meshes
|   |   ├─collision
|   |   └─visual
│   | assets.xml
│   | franka_panda.urdf
│   | franka_panda.xml
│   | franka_panda_rviz.urdf
│   | gripper.xml
|   └─objects.xml
├─rviz
|   └─franka_panda.rviz
├─src
│   | controller.cpp
│   | generatecommand.cmpp
│   | model.cpp
│   | roswrapper.cpp
│   | simulate.cpp
|   └─trajectory.cpp
├─srv
|   └─pandaSrv.srv
| CMakeLists.txt
| README.md
└─package.xml
```

## Descriptions
