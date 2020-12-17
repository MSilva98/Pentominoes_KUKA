#!/usr/bin/env python

import roslib; roslib.load_manifest('ec2_sim')
import rospy

from kmriiwa_wrapper import KmrIiwaWrapper
from kmriiwa_simulated import KmrIiwaSimulated

from schunk_wsg50_wrapper import SchunkWsg50Wrapper
from schunk_wsg50_simulated import SchunkWsg50Simulated

from schunk_pw70_wrapper import SchunkPw70Wrapper
from schunk_pw70_simulated import SchunkPw70Simulated

class SimulationWrapper():
    def __init__(self):
        urdf = rospy.get_param("robot_description")

        print "Connecting to gazebo core simulation: Waiting for services and topics..."

        self.kmr_iiwa_simulated = KmrIiwaSimulated(urdf)
        self.kmr_iiwa_wrapper = KmrIiwaWrapper(self.kmr_iiwa_simulated)

        self.wsg50_simulated = SchunkWsg50Simulated()
        self.wsg50_wrapper = SchunkWsg50Wrapper(self.wsg50_simulated)

        self.pw70_simulated = SchunkPw70Simulated()
        self.pw70_wrapper = SchunkPw70Wrapper(self.pw70_simulated)

        print "Successfully connected to gazebo core simulation."

    def run(self, rate):
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.kmr_iiwa_wrapper.publish()
            rate.sleep()



if __name__ == '__main__':
    rospy.init_node("euroc_c2s2_wrapper_node", anonymous=True)
    wrapper = SimulationWrapper()
    wrapper.run(30)

