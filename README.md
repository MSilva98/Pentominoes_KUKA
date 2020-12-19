# Pentaminoes_KUKA

**Inside ./Pentaminoes_KUKA/kuka_pentomino**

## Simulator

```properties
catkin config --extend /opt/ros/melodic

. /opt/ros/melodic/setup.bash

catkin b

. devel/setup.bash

roslaunch ec2_bringup miiwa.simulation.launch 
```

**In another terminal in same path**
## Solver
```properties
. devel/setup.bash

rosrun ec2_solvers solver
```