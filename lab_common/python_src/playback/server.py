#!/usr/bin/env python
import source_code
import rospy
import actionlib
from dynamic_reconfigure.server import Server

from lab_common.cfg import (
    PositionJointTrajectoryActionServerConfig,
    VelocityJointTrajectoryActionServerConfig,
    PositionFFJointTrajectoryActionServerConfig,
)

def start_server(limb="both", rate=100.0, mode="velocity"):
    print("Initializing joint trajectory action server...")

    if mode == 'velocity':
        dyn_cfg_srv = Server(VelocityJointTrajectoryActionServerConfig,
                             lambda config, level: config)
    elif mode == 'position':
        dyn_cfg_srv = Server(PositionJointTrajectoryActionServerConfig,
                             lambda config, level: config)
    else:
        dyn_cfg_srv = Server(PositionFFJointTrajectoryActionServerConfig,
                             lambda config, level: config)
    jtas = []
    if limb == 'both':
        jtas.append(source_code.JointTrajectoryActionServer('right', dyn_cfg_srv,
                                                rate, mode))
        jtas.append(source_code.JointTrajectoryActionServer('left', dyn_cfg_srv,
                                                rate, mode))
    else:
        jtas.append(source_code.JointTrajectoryActionServer(limb, dyn_cfg_srv, rate, mode))

    def cleanup():
        for j in jtas:
            j.clean_shutdown()

    rospy.on_shutdown(cleanup)
    
if __name__ == '__main__':
    rospy.init_node("server")
    start_server()
    rospy.spin()
