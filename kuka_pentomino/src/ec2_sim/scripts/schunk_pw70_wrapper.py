import rospy

import pantilt_msgs.srv as pantilt_srvs

class SchunkPw70Wrapper:
    def __init__(self, pw70_simulated):

        self.pw70_simulated = pw70_simulated;

        # List of available services and their types
        pw70_services = [
            ('/move_pan_tilt', pantilt_srvs.MovePanTilt, self.on_move_pan_tilt),
            ('/get_pan_tilt_position', pantilt_srvs.GetPanTilt, self.on_get_pan_tilt),
        ]

        pw70_node = '/pan_tilt'
        self.services = dict()
        for name, msg, callback in pw70_services:
            service_name =  pw70_node + name
            self.services[name] = rospy.Service(service_name, msg, callback)

    def on_move_pan_tilt(self, req):
        error_message=""
        self.pw70_simulated.move([req.pan, -req.tilt])
        return pantilt_srvs.MovePanTiltResponse(error_message)
        
    def on_get_pan_tilt(self, req):
        error_message=""
        pan, tilt = self.pw70_simulated.get_position()
        return pantilt_srvs.GetPanTiltResponse(pan, -tilt, error_message)
