# Webots ROS2 Interface

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![Version](https://img.shields.io/github/v/tag/cyberbotics/webots_ros2?label=version)](http://wiki.ros.org/webots_ros2)  

`webots_ros2` is a package that provides the necessary interfaces to simulate a robot in the [Webots](https://cyberbotics.com/) open-source 3D robot simulator. It integrates with ROS2 using ROS2 messages, services, and actions.

![Webots](docs/cover.png)

Please visit the [documentation](https://github.com/cyberbotics/webots_ros2/wiki) that contains the following sections:  
- [Getting Started](https://github.com/cyberbotics/webots_ros2/wiki/Getting-Started)  
- [Build and Install](https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install)  
- [Tutorials](https://github.com/cyberbotics/webots_ros2/wiki/Tutorials)  
- [Examples](https://github.com/cyberbotics/webots_ros2/wiki/Examples)  
- [References](https://github.com/cyberbotics/webots_ros2/wiki/References)  

---

## Modifications by SoftIllusion  
SoftIllusion contributed to this repository by adding the `webots_ros2_tutorials` directory and developing projects such as:  
- **Line-Following Robot with SLAM**  
- **ARuCo Tag Detection**  

These enhancements expanded the repository's capabilities for various robotics applications and tutorials.

## Further Modifications by me: 
I have made the following additional changes:  
- Updated the repository to ensure compatibility with the latest ROS2 Foxy version.  
- Fixed building errors caused by changes in the `urdf2webots` package and integrated it as a submodule (previously absent).  
- Enhanced integration for use with my **Robotics-Lab** repository, extending its functionality for custom robotics projects and simulations.  

These updates ensure robust performance and seamless compatibility with advanced robotics development in ROS2 Foxy.

---

## Installation Steps  

### Part 1: Install ROS2 Foxy  
Follow the official guide for installing ROS2 Foxy on Ubuntu:  
[ROS2 Foxy Installation Guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)  

**Additional Setup:**  
Edit your `.bashrc` file:  
```bash
sudo nano ~/.bashrc
```
Add the following lines at the end of the file:  
```bash
source /opt/ros/foxy/local_setup.bash
```
Save and close, then apply the changes:  
```bash
source ~/.bashrc
```

### Part 2: Install Webots R2020b  
Download Webots R2020b:  
- Visit [cyberbotics.com](https://cyberbotics.com/) → Older Versions → R2020b  

### Part 3: Install `webots_ros2`  
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/cyberbotics/webots_ros2.git
```
Ensure the package is visible:  
```bash
ls
```
Install necessary ROS dependencies:  
```bash
sudo apt-get install ros-foxy-webots-ros2
```

### Part 4: Install Visual Studio Code  
Download and install Visual Studio Code:  
- [VSCode Download Page](https://code.visualstudio.com/)  

Install the Debian package and open it:  
```bash
code
```
To open your ROS2 workspace:  
- File → Open Folder → `~/ros2_ws/src`  

---

## Building and Running  

### Build with `colcon`  
```bash
cd ~/ros2_ws
colcon build
```

### Run an Example  
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch webots_ros2_tiago tiago.launch.py
```

---

## Troubleshooting Errors  

### 1. **URDF2Webots Error**  
If you encounter a `urdf2webots` error during build:  
```bash
cd ~/ros2_ws/src/webots_ros2
git submodule update --init --recursive
cd ~/ros2_ws
colcon build
```

### 2. **CPP-Dev Error**  
If there’s an issue with missing C++ dependencies:  
```bash
sudo apt-get update
sudo apt-get install libyaml-cpp-dev
```

---

## Acknowledgement  

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a></br>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>  

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left">  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.  
