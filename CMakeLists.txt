cmake_minimum_required(VERSION 2.8.3)
project(ekf_slam)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
endif()

# Set the name and location of the BlackLib library
set (BlackLib lib/BlackLib/v3_0)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs
	message_generation
	)

catkin_package(
	CATKIN_DEPENDS message_runtime
	)

add_message_files(
  FILES
  RadioScan.msg
  RadioObservation.msg
  RadioResponse.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Include directories
include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
)

# Include the directory containing BlackLib in the project
include_directories ("${PROJECT_SOURCE_DIR}/${BlackLib}")
add_subdirectory (${BlackLib})

add_library(Motor src/Motor.cpp)
target_link_libraries(Motor BlackLib)

add_library(atcon src/atcon.cpp)
target_link_libraries(atcon xbeep)

add_library(remotenode src/remotenode.cpp)
target_link_libraries(atcon remotenode)

add_library(Observer src/Observer.cpp)
target_link_libraries(Observer remotenode atcon BlackLib)
#target_link_libraries(Observer atcon)
#target_link_libraries(Observer BlackLib)

# Make matlab interface executable
add_executable(matlabInterface interfaces/matlab/main.cpp)
target_link_libraries(matlabInterface Motor BlackLib remotenode atcon Observer)
#target_link_libraries(matlabInterface BlackLib)
# target_link_libraries(matlabInterface remotenode)
# target_link_libraries(matlabInterface atcon)
#target_link_libraries(matlabInterface Observer)

# Make other executable
add_executable(slamExperiment src/main.cpp)

# Link libraries to executable
target_link_libraries(slamExperiment ${catkin_LIBRARIES} atcon Motor BlackLib remotenode atcon Observer)
#target_link_libraries(slamExperiment atcon)

# More linking
#target_link_libraries(slamExperiment Motor)
#target_link_libraries(slamExperiment BlackLib)
# target_link_libraries(slamExperiment remotenode)
# target_link_libraries(slamExperiment atcon)
#target_link_libraries(slamExperiment Observer)
