#!/usr/bin/env python

import rospy
import os
import baxter_interface
from baxter_interface import joint_trajectory_action_server, CHECK_VERSION
from baxter_examples import JointRecorder, Trajectory


def record(filepath, rate): 
    print("Initializing node... ")
    rospy.init_node("rsdk_joint_recorder")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    recorder = baxter_examples.JointRecorder(filepath, rate)
    rospy.on_shutdown(recorder.stop)

    print("Recording. Press Ctrl-C to stop.")
    recorder.record()
    

def playback(arms, rate, mode, filepath, loops):
    start_server(arms, rate, mode)
    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_file_playback")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
 
    traj = Trajectory()
    traj.parse_file(filepath)
    
    # for safe interrupt handling
    rospy.on_shutdown(traj.stop)
    result = True
    loop_cnt = 1
    loopstr = str(loops)
    if loops == 0:
        loops = float('inf')
        loopstr = "forever"
    while (result == True and loop_cnt <= loops
           and not rospy.is_shutdown()):
        print("Playback loop %d of %s" % (loop_cnt, loopstr))
        traj.start()
        result = traj.wait()
        loop_cnt = loop_cnt + 1
    print("Exiting - File Playback Complete")


if __name__ == '__main__':
    record(os.path.join(os.getcwd(), "playback.txt"), 100.0)
    playback("both", 100.0, "velocity", os.path.join(os.getcwd(), "playback.txt"), 1)

    
    
