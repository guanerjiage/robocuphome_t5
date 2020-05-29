cmake_minimum_required(VERSION 3.0.2)
project(from2Dto3D)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf
  image_geometry
  darknet_ros_msgs
  perception_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS include

	LIBRARIES object_recognition
	CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf image_geometry
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${PCL_INCLUDE_DIRS}
 
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_library(from2Dto3D_function src/from2Dto3D_function.cpp)
target_link_libraries(from2Dto3D_function
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(from2Dto3D_function ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_executable(from2Dto3D_main src/from2Dto3D_main.cpp)
add_dependencies(from2Dto3D_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(from2Dto3D_main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} from2Dto3D_function)





#add_dependencies(node_objectRecognition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS from2Dto3D_function 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)













