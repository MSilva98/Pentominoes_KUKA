import rospy

import gripper_msgs.srv as gripper_srvs

class SchunkWsg50Wrapper:
    def __init__(self, wsg50_simulated):

        self.wsg50_simulated = wsg50_simulated;

        # List of available services and their types
        wsg50_services = [
            ('/grasp', gripper_srvs.Grasp, self.on_grasp),
            ('/release', gripper_srvs.Release, self.on_release),
            ('/move', gripper_srvs.Move, self.on_move),
            ('/get_position', gripper_srvs.GetPosition, self.on_get_position)
        ]

        self.max_position = 0.11
        self.min_position = 0.007

        wsg50_node = '/gripper'
        self.services = dict()
        for name, msg, callback in wsg50_services:
            service_name =  wsg50_node + name
            self.services[name] = rospy.Service(service_name, msg, callback)

    def on_grasp(self, req):
        error_message=""
        if req.direction == gripper_srvs.GraspRequest.GRASP_DIRECTION_CLOSE:
            self.wsg50_simulated.move(self.min_position)
        elif req.direction == gripper_srvs.GraspRequest.GRASP_DIRECTION_OPEN:
            self.wsg50_simulated.move(self.max_position)
        else:
            error_message="Invalid grasp direction specified."
        return gripper_srvs.GraspResponse(error_message);

    def on_release(self, req):
        error_message=""
        if req.direction == gripper_srvs.ReleaseRequest.RELEASE_DIRECTION_CLOSE:
            self.wsg50_simulated.move(self.min_position)
        elif req.direction == gripper_srvs.ReleaseRequest.RELEASE_DIRECTION_OPEN:
            self.wsg50_simulated.move(self.max_position)
        else:
            error_message="Invalid grasp direction specified."
        return gripper_srvs.ReleaseResponse(error_message);

    def on_move(self, req):
        result = -1
        error_message = ""
        if req.position > self.max_position:
            error_message = "position larger than maximum position"
        elif req.position < self.min_position:
            error_message = "position beyond maximum position"
        else:
            self.wsg50_simulated.move(req.position)
            result = gripper_srvs.MoveResponse.MOVE_POSITION_REACHED
        return gripper_srvs.MoveResponse(result, error_message)

    def on_get_position(self, req):
        error_message=""
        position = self.wsg50_simulated.get_position()
        return gripper_srvs.GetPositionResponse(position, error_message)

