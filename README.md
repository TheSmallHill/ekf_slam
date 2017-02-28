# Welcome to ekf_slam
Please note that development for this this project is on going, and the use of
it is free, but please do not hold us responsible for any damage or injuries that
may occur.

A description of directories and their contents can be found in DIRECTORIES.md within the root directory.

## Parts
Here is the list of parts we are using for our project.
- Beaglebone Black rev C (https://beagleboard.org/black)
- 200 step/rev stepper motor (http://www.digikey.com/product-detail/en/sparkfun-electronics/ROB-09238/1568-1105-ND/5318747)
- Big Easy Driver stepper motor controller (http://www.digikey.com/product-detail/en/sparkfun-electronics/ROB-12859/1568-1066-ND/5172292)
- XBee s2c (http://www.digikey.com/product-detail/en/XB24CZ7WIT-004/602-1560-ND/5322374?WT.mc_id=IQ_7595_G_pla5322374&wt.srch=1&wt.medium=cpc&WT.srch=1&gclid=CjwKEAiAlZDFBRCKncm67qihiHwSJABtoNIgd-NB0uezDv2EFoaoVTWRJEV3_bQeuC2OznSORwJfrRoCnnzw_wcB)
- Pioneer 3-DX (http://www.mobilerobots.com/researchrobots/PioneerP3DX.aspx)

You can use parts other than the ones listed above but obviously the code will need to be changed accordingly.

## Licensing
ekf_slam is licensed using GPLv3 from v3.0.5 onwards. For more
information on what this means for you, please see LICENSE.

## Installation
This project is being written for the use of a Beaglebone Black running Ubuntu 16.04 LTS
(available at https://rcn-ee.net/) and ROS kinetic. Make sure to install ROS kinetic 
according to the installation instructions on http://www.ros.org/ before downloading 
and installing this package. To install the package, clone or download this repository
into the top level source directory of your ROS workspace.

The project requires 2 libraries in order to be able to build: libxbee3 and BlackLib. 
These libraries are included in the lib directory as gitmodules. To install libxbee3, follow
the installation instructions found in its README. For BlackLib, I have added a CMakeLists.txt
to the version used in this project so it will build the necessary objects. The gitmodule I 
included should be for the version with CMakeLists.txt included but I am still learning Git so
check to make sure the file is present. If it is not there, it can be found in the master branch
of the BlackLib repository forked to my account.

Here are links to both repositories that I have forked for this project:
- https://github.com/attie/libxbee3
- https://github.com/yigityuce/BlackLib

## Building
In order to build the package, it is advised to use the combination of cmake and catkin tools
provided with ROS kinetic. In the root directory of your ROS workspace, use this command to build
all ROS packages:
  
  $ catkin_make

Or, for just the ekf_slam package:

  $ catkin_make --pkg ekf_slam
  
Obviously this will only work in a Linux distribution supported by ROS kinetic and has it installed.

## Usage
Currently this package is written as a standalone node. To run the node, use this command in the
root directory of your ROS workspace:

  $ roslaunch ekf_slam node.launch
  
Although there are no current plans to convert this node to a ROS package for use on other
platforms, it is possible that this may be done after the completion of this project.

## Interfaces
Right now the package is written in a way so you must add changes to main.cpp and recompile in order
to make changes to the program. However, due to the way ROS works, it is possible to add control of
the operation of the package from another language over a network (such as MATLAB using the 
Robotics System Toolbox). If (and when) these interfaces are created, the source for the executable
will be added to the Interfaces directory of this repository. A new launch file for the 
associated executable will also most likely be added to the launch directory.

## Future work
Although not currently necessary, I may add CMakeLists.txt to libxbee3 so it can be built
and installed using cmake. This would make building and installation much simpler. I may also
continue adding CMakeLists.txt to the BlackLib library so it will build the library in its
entirety.
