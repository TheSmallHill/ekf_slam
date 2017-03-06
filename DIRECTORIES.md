# Directories in this Repository
This is a list of the directories in this repository with a description of the files located in each.

## src
- main.cpp - the code that is turned into an executable to run the node
- Motor.cpp - motor object
- Observer.cpp - observer object
- remotenode.cpp - necessary for observer object
- atcon.cpp - necessary for observer object

## include
- xbeep.h - C++ header for libxbee3 (currently not used correctly since slamLib.cpp uses C functions still)
- Motor.h - for Motor.cpp
- Observer.h - for Observer.cpp
- remotenode.h - for remotenode.cpp
- atcon.h - for atcon.cpp

## launch
- node.launch - roslaunch file for starting the node created from src/main.cpp
- matlabInterface.launch - roslaunch file for starting the node created from interfaces/matlab/main.cpp

## interfaces 
### matlab
- main.cpp - contains an object that is created when the node starts, handles publishers and subscribers
- ekfSlam.m - an example matlab file for interfacing with the matlab interface

## lib
- libxbee3
- BlackLib
