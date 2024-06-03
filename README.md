# PPE-SoftHand-ENIB

This repository contains the scripts made by students doing their student professional project. It contains the necessary ROS packages for communicate and control a qbrobotics&reg; Soft Hand device, and the control scripts made by the students.

- [Cloning this repository](#cloning-this-repository)
- [Attention](#attention)
- [Installing Modules](#installing-modules)
- [Using the Script](#using-the-scripts)

#### Cloning this repository
This repository uses QB Soft Hand repositories as submodules. In order to correctly clone this repository and include the submodules, is necessary to proceed in one of the following ways:

- Clone with extra option  
	```
	git clone --recurse-submodules link-to-this-repo
	```
- Adding submodules after cloning  
	```
	git clone link-to-this-repo
	cd PPE-SoftHand-ENIB
	git submodule update --init --recursive
	```

#### Attention 
This README only intents to guide with the essential to install the ROS packages. Further detailed explanation can be found in the official site ([ROS](https://wiki.ros.org/ "ROS")).

All of this files have been tested and used in Ubuntu 20.04 with the Noetic version of ROS. There is no guarantee that any of this will work in versions other than the mentioned above.

After having installed ROS, all the necessary files for using the scripts are in this repository.

The **recommended** directory hierarchy when using a catkin workspace is at follows:

	catkin_ws/
			...
			src/
				...
				qbdevice-ros/
				qbhand-ros/
			scripts/
				...
				handController.py
				qbhand_cmd.py


#### Installing Modules

After cloning the repository, the packages must be compiled, wich is done by issuing the `catkin_make` command on a terminal opened in the same directory as the packages. Catkin_make is a util of catkin, wich is included by default when ROS is installed, but can also be installed from source or prebuilt packages (e.g. `sudo apt-get install ros-noetic-catkin`)


#### Using the scripts
Both scripts must be marked as executables. This is achieved using the commands

    chmod +x handController.py
    chmod +x qbhand_cmd.py

Only the qbhand_cmd.py script must be launched:
	`python3 qbhand_cmd.py`
The list of possible commands can be seen by issuing the command `help` or `?`, and documentation on how to use the commands is shown by `help <command>`