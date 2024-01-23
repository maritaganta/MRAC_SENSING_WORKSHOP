# Husky Robot Contorl

The pacakges of this workshop has been installed on husky on board computer. In order to access the machine you have to have openssh server installed.  
**To install openssh-server**
```
sudo apt install openssh-server
```
## Tmux
tmux is a terminal multiplexer for Unix-like operating systems. It enables users to create and manage multiple terminal sessions within a single terminal window or remote terminal. With tmux, you can divide your terminal into panes, each displaying its own terminal session or command-line interface. This allows you to multitask, run commands simultaneously, and organize your workflow efficiently.

[cheat sheet](https://tmuxcheatsheet.com/)

## Connection

1. Connect to the iaac_husky hotspot (WIFI password: EnterIaac22@)
2. Open a terminal in your computer and ssh to the husky (user password: iaac)
   ```
   ssh iaac@10.42.0.1
   ```
3. Use tmux as the terminal on husky robot
   ```
   tmux new -s husky
   ```
4. launch the ros2 nodes for mobile base (first tmux terminal)
   ```
   ros2 launch /etc/clearpath/platform/launch/husky.launch.py
   ```
5. launch the ros2 nodes for lidar (second tmux terminal)
   ```
   ros2 launch livox_ros_driver msg_MID360.launch.py
   ```
6. launch the mapping node (third tmux terminal)
   ```
   ros2 launch fast_lio mapping.launch.py
   ```
7. launch the usb_cam node to bring up the camera (fourth tmux terminal)
   ```
   ros2 launch usb_cam camera.launch
   ```
8. launch the foxglove bridge for visualization (fifth tmux terminal)
   ```
   ros2 launch foxglove_bridge foxglove_bridge.launch
   ```
9. Open foxglove-studio and change the web address to 10.42.0.1  
10. Load the panel from this repository  
