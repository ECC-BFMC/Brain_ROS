# BFMC - Brain ROS Project

The project includes the software already present on your SD cards. It has all the pre-given packages and examples on how to communicate with them.
The pre-installed sd card is already set-up to communicate with the demo scripts inside the ECC-BFMC/Computer project. 
The sd-card just has the scripts set to run at start-up (to communicate with the Computer project). If you wish to remove this configuration (for when you start developing), just clena out the /etc/rc.local file.

By following the next steps, you can create a clean instalation of the OS and ROS on your car.
## 1. Download the Raspbery Pi Imager on your OS.
[software](https://www.raspberrypi.com/software/) 

If you are unfamiliar with Linux and ROS, we suggest starting with the desktop versions of the SWs and later migrate to the lite versions. 

## 2. Mount Raspbian

With the help of Raspbery Pi Imager, mount the Raspbery py Debian Buster OS (Legacy) on the SD card.
Don't forge to set the SSH Connection to true, 

## 3. Connect to the raspbery pi:
Insert the card in the raspbery and power it up.
Check the Raspbery IP on the network: The easiest way is either to use nmap for scanning the network or directly connect to your router. 
Use ssh to connect to the raspbery with the given IP:
	ssh pi@192.168.x.x


## 4. Add the ROS Debian repo to the OS
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'


## 5. Add official ROS key
To validate that the downloaded file is the needed one, you need to add the verification key

		sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 6. Pull all meta info from ROS noetic packages
		sudo apt-get update && sudo apt-get upgrade

## 7. Install build dependencies
		sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

## 8. Install pip3
		sudo apt install python3-pip

## 9. Install opencv:
		sudo apt install libopencv-dev python3-opencv

## 10. Setup ROS dependency sources/repos
		sudo rosdep init
		rosdep update


## 11. Fetch and install ROS dependencies
		mkdir -p ~/ros_catkin_ws
		cd ~/ros_catkin_ws

		rosinstall_generator ros_comm sensor_msgs cv_bridge --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall 
		wstool init src noetic-ros_comm-wet.rosinstall

		rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster


## 12. Compile ROS packages
Since the ROS project is resource consuming, it is also recommended, but not mandatory, to increase the swap memory to 1 GB. You can decrease it afterwards. By following the same steps and setting it back to 100

		sudo dphys-swapfile swapoff

		sudoedit /etc/dphys-swapfile


CONF_SWAPSIZE=1024


		sudo dphys-swapfile setup

		sudo dphys-swapfile swapon

		sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3

Don't forget to set back the swap memory.

## 13. Source the environment and set it run at the start of any new terminal communication.
		source /opt/ros/noetic/setup.bash
		echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc


## 14. Verify the installation
		roscore

## 15. Fix missing package
		sudo apt install libatlas-base-dev

## 16. clone the given ROS project
		git clone https://github.com/ECC-BFMC/Brain_ROS.git

## 17. Install python dependencies
		
		pip3 install -r requirements_rpi.txt
		sudo apt install -y python3-picamera

## 18. Set up the i2c communication for the IMU by following the Setting up the Raspberry Pi side from this tutorial: 
		https://github.com/RPi-Distro/RTIMULib/tree/master/Linux 

## 19. Build and run the prepared project
		catkin_make

		source devel/setup.bash

		roslaunch utils run_automobile.launch

## 20. For additional topics, check the official ROS documentation for Raspbery Pi: 
http://wiki.ros.org/ROSberryPi/

## 21. If you wish to install additional ROS packages after the installation, you will have to:
cd ~/ros_catkin_ws

sudo rm -rf build_isolated/ devel_isolated/ src/

sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

rosinstall_generator name_of_new_pkg --deps --exclude RPP > new_pkg.rosisntall

wstool init src new_pkg.rosisntall

sudo -s

nano /root/.bashrc

	add source /opt/ros/noetic/setup.bash

source /root/.bashrc

catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release  --install-space /opt/ros/noetic

nano /root/.bashrc

	remove source /opt/ros/noetic/setup.bash

exit
