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

## IMPORTANT NOTES
- In **grasp** function in **gripper_interface.cpp**, the **move_msg.request.position** parameter must be **0.03515**
- The **setTCPPose** in **arm_interface.cpp**, must have the following if condition between **gik.response.error_message** if condition and **gik.response.joint_position.values[6]** attribution.
```
if(gik.response.joint_position.values.size() <= 0){
    ROS_ERROR("ArmInterface: Joint Position Values Vector size: %d", gik.response.joint_position.values.size());
    return false;
}
```