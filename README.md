# robocuphome_t5

This is a meta package of robocup@home class tutorial 5.

You should put it inside the tiago workspace along with tiago package and darknet_ros package in order to work together.

add these to Cmake after put test_arm in a package.

add_executable(arm_test src/arm_test.cpp)
target_link_libraries(arm_test ${catkin_LIBRARIES} ${roscpp_LIBRARIES} )
add_dependencies(arm_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
