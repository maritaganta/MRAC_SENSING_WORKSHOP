# MRAC_SENSING_WORKSHOP
This is the repository for MRAC workshop 2.1, ROBOTICS SOLUTIONS FOR 3D SPACE ANALYSIS. This ROS-based part focuses on integrating advanced 3D scanning technologies, OpenCV-powered navigation, and 3D Lidar mapping for the Husky robot.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Installation
**Supported Operating Systems:** Ubuntu 20.04 and Ubuntu 22.04
### Docker Engine Installation
This project uses Docker for containerization. If you don't have Docker Engine installed, follow this [link](https://docs.docker.com/engine/install/ubuntu/)
To manage docker as a non-root user, please follow the post-installation steps in this [link](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

Add the following to your bashrc file for ros and the workspace to be sourced when opening a new terminal in dev_ws, make sure your are not sourcing ROS in your bashrc file.
```
if [ -f "/dev_ws/setup.bash" ]; then
    source /dev_ws/setup.bash
fi
```
### Foxglove Studio Installation
If you want to visualize the pointcloud and images from husky robot please install foxglove studio.
```
sudo snap install foxglove-studio
```

### Vscode Extensions
To access the files inside container you need to install **Remote Development** and **Docker** extension in your vscode.
To modify the files in container:
  - click the Docker icon in vscode
  - Right click the container you want to access
  - Attach vscode
  - Open the folder, type /dev_ws/src

## Usage
**Fork this repository and clone to your computer**
Add the submodles
```
cd MRAC_SENSING_WORKSHOP
git submodule update --init
```
**Build the ros_sim image**
```
cd ros_sim
.docker/build_image.sh
```
**Build the turtlebot_remote image**
```
cd turtlebot_remote
.docker/build_image.sh
```
**Run the container**
```
.docker/run_user.sh
```
Once you run the container, you will see the terminal return the request to change the ownership of the folder, copy this line from the terminal
```
sudo chown -R $USER /dev_ws
```
Then run the terminator
```
Terminator
```
Congratulation, you can roslaunch your node in the terminator now!

### ros_sim Tutorial
[ros_sim](ros_sim/README.md)
### Husky robot Tutorial
[husky_tutorial](husky/README.md)
