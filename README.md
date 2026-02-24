# Automated Guided Vehicle (DiffDrive)
This repository contains all the necessary ROS2 Jazzy packages to set up and run a generic AGV differential drive robot. It implements a hardware interface using ROS2 Control for sending motor commands, the navigation stack, and a bringup package to start the robot with a single command.

## Prerequisites
1. Ubuntu 24.04
2. ROS2 Jazzy

## Software setup
### ROS2 Installation
For the ROS installation you can follow the official guide in the ROS documentation: 
You need to install Ros2 on the onboard computer of the AGV (in this case a Raspberry PI5) and on the development workstation, for visualization through Rviz2

1. Set locale 
```bash
 locale  # check for UTF-8
 sudo apt update && sudo apt install locales
 sudo locale-gen en_US en_US.UTF-8
 sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
 export LANG=en_US.UTF-8
 locale  # verify settings
```
2. Enable ROS repositories
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
3. Install ros apt source packages
```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
4. Install development tools
```bash
sudo apt update && sudo apt install ros-dev-tools
```
5. Install ROS
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
```
6. Environment setup
```bash
source /opt/ros/jazzy/setup.bash
```
   It's useful to add this line of code to the shell startup script, in order to source the environment every time you open a new shell
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### External dependecies installation 
All the external dependencies can be installed only on the onboard computer, because it's the responsible for running all the necessary node. It's not necessary to install everything on the fixed workstation used for visualization, because only Rviz2 is needed. 
1. C++ Compilation tools 
   These tools are necessary for compiling the ROS2 control package, that has been written in C++
```bash
sudo apt install -y build-essential cmake git
```
2. Python libraries

   For handling all the pip packages it's necessary to install pip  and also the math and system libraries of python

```bash
sudo apt install -y python3-pip
pip install transforms3d PyYAML scipy numpy
```
3. Hardware and comunication libraries

   These libraries are necessary to interface the onboard computer with the IMU (model BNO055) that is connected through i2c protocol
```bash
sudo apt install -y i2c-tools
```
4. Serial libraries 

   For debug or external python nodes
```bash
pip install pyserial smbus2
```

### Hardware configuration (udev rules)
To ensure consistent communication with the hardware, you must configure **udev rules**. These rules assign static names to the USB serial ports, preventing the system from swapping device paths (e.g., from `/dev/ttyUSB0` to `/dev/ttyUSB1`) upon reboot or reconnection.
The hardware interface is specifically programmed to look for `/dev/ttyESP32`, and the LiDAR launch file expects `/dev/ttyLiDAR`.
1. Create the Udev rules file   
```bash
sudo nano /etc/udev/rules.d/99-robot-devices.rules
```
2. Define the symlinks

   Add the following lines to the file. If both devices use the same USB to serial chip (same vendor and product IDs), there's the need to distinguish them by adding the unique serial number using 'ATTRS{serial}=="SERIAL_NUMBER"   
```bash
#Symlink for ESP32
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyESP32"
#Symlink for LiDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyLiDAR"
```
*How to verify the hardware IDs: if the devices are not recognized or if different hardware is used, is possible to verify the `idVendor` and `idProduct` using the `lsusb` command, the output should be something like `Bus 001 Device 004: ID 1a86:7523 QinHeng Electronics CH340 serial board` the first 4 digits are the `idVendor` and the last 4 digits are the `idProduct`

3. Apply changes

   Reload the udev service and trigger the rules
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
4. User permission

   In order to grant access to the dialout and i2c permissions for writing and reading data it's needed to add the user to the `dialout` and `i2c` groups, without requiring root privileges 
```bash
sudo usermod -aG dialout,i2c $USER
```
   *It's necessary to log out and log back in (or reboot) for these group changes to take effect*

### Workspace setup
Now it's needed to create a workspace folder, to contain all the necessary packages, it's important that all the packages are inside the `src` folder, because ROS2 build only that directory
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
Now you can clone this repository and all the packages needed to interface with the LiDAR and the IMU;
```bash
git clone https://github.com/rdedo99-byte/Diff_Drive_AGV.git
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/dheera/ros-imu-bno055.git
```

*In order to correctly establish a connection between the `slllidar_ros2` package and the physical LiDAR mounted on the robot, it's needed to change the `sllidar_a1_launch.py` launch file, modifying the `serial_port` parameter from `/dev/ttyUSB0` to `/dev/ttyLiDAR`*

### Install ROS2 specific dependencies
Before building the ws, it's important to install all the required ROS2 packages that are used by the AGV (such as `navigation2`, `slam_toolbox`, `robot_localization` and `ros2_control`) with `rosdep` it's possible to scan all the `package.xml` files in the `src` folder for automatically installing all the required dependencies:

1. Initialize and update rosdep 

```bash
#required only once after ROS installation
sudo rosdep init

rosdep update
```

2. Install the missing packages

```bash
rosdep install --from-paths src --ignore-src -r -y
```
*It's crucial to run the `rosdep install` command inside the root of the workspace, not form inside the `src` folder, you can get into the correct directory with the following command: `cd ~/ros2_ws`*

### Building the workspace
Once all dependecies are installed, it's possible to build the workspace. 

1. Build command
   Inside the workspace folder *not the src directory* run:
```bash
colcon build --symlink-install
```
2. Source the environment

   After a succesful build, source the workspace for making the source files visible to ROS

```bash
source install/setup.bash
```

## Operating modes
Now it's possible to launch the bringup file, there are two operating modes: 

### 1. SLAM mode
In this mode, the robot maps the real environment to create a `2d occupancy grid` representing walls and obstacles.

```bash
ros2 launch bringup robot_bringup_slam.launch.py
```

   After completing a mapping session, the created map must be saved, and it's needed to configure the system to recognize it for future localization. For moving the robot use either Rviz2 for publishing the `2D Goal Pose` and let the robot go to the clicked point autonomously or run in terminal the `teleop_twist_keyboard` node.  

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '~/ros2_ws/src/navigation/maps/my_map_name'}"
```

   After completing the mapping session, you must serialize the map. Unlike the standard Nav2 map saver, Slam Toolbox needs to save its internal state to allow for future localization.This command will generate a `.posegraph` file (and a `.data` file) inside the specified directory. Then, inside the `slam_params_localization.yaml` it's possible to update the `map_file_name` parameter with the path of the `.posegraph` file. 

```yaml
map_file_name: /home/<your_user>/ros2_ws/src/navigation/maps/my_map_name.posegraph
```

   *make sure to specify the absolute path of the map file, otherwise it won't work*

### Localization mode
After saving the newly created map and updated the path in the config `slam_params_localization.yaml` file it's possible to launch the SLAM toolbox node in localization mode with the command: 

```bash
ros2 launch bringup robot_bringup.launch.py
```
   in this mode the robot can localize itself inside the map, and after publishing a `2D Goal Pose` command, it will start to navigate towards that point. 

## Visualization 

### Rviz2
To visualize the robot's current position within the map, detected obstacles, and the planned path, you can use **RViz2** on your workstation. If both the AGV and the workstation are on the same network, ROS 2 topics will be discovered automatically and remain accessible on both devices. A reliable way to manage the AGV and verify network connectivity is via **SSH**.

*to enable the remote access, installation of the **OpenSSH** serve package on both devices*

```bash
sudo apt update
sudo apt install openssh-server
```
*agter that, configure the service to start automatically on boot and launch it immediately*

```bash
sudo systemctl enable --now ssh
```

*to confirm the service is active and running use*

```bash
sudo systemctl status ssh
```

### Foxglove
If a workstation with Ubuntu 24.04 and ROS 2 Jazzy is unavailable, **Foxglove Studio** provides an alternative for visualization. To bridge topics between the onboard computer and Foxglove, install and run the **Foxglove Bridge** in a new terminal on the AGV

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
