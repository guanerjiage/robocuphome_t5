# robocuphome_t5

This is a meta package of robocup@home class tutorial 5.

You should put it inside the tiago workspace along with tiago package and darknet_ros package in order to work together.

### Things to refine later

1. copy the world file ./ics_gazebo/worlds/tutorial5.world into tiago_simulation/tiago_gazebo/worlds/
2. running setup.bash
3. running roslaunch state_machine state_machine.launch

### Problems

The problem we encounter is the second call to move to the second desk never returns. so it cannot continue place action.