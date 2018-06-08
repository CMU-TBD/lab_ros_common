#!/usr/bin/env python

from pathlib2 import Path
import source_code
import baxter_interface
import rospkg
import rospy
import os
import actionlib
from dynamic_reconfigure.server import Server
from baxter_interface import CHECK_VERSION

from lab_common.cfg import (
    PositionJointTrajectoryActionServerConfig,
    VelocityJointTrajectoryActionServerConfig,
    PositionFFJointTrajectoryActionServerConfig,
)

def record(filename, rate=100.0): 
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    
    print("Enabling robot... ")
    rs.enable()
    
    rospack = rospkg.RosPack()
    ros_package = raw_input("In which ros package would you like your library? ")
    filepath = os.path.join(rospack.get_path(ros_package), "playback_library")
    
    # make library if non-existent
    if not os.path.isdir(filepath):
 	    os.mkdir(filepath)
 	
    filename = os.path.join(filepath, filename)
    if Path(filename).is_file():
        print("Cannot record! File already exists.")
        return
        
    recorder = source_code.JointRecorder(filename, rate)

    print("Recording... Press both arm wheel buttons to stop")
    recorder.record()


def playback(filename, arms="both", rate=100.0, mode="velocity", loops=int(raw_input("How many times should the playback loop? "))):
    while loops < 1:
        loops = int(raw_input("How many times should the playback loop?" ))
        
    rospack = rospkg.RosPack()
    ros_package = raw_input("In which ros package is your library? ")
    filepath = os.path.join(rospack.get_path(ros_package), "playback_library/" + filename)
    
    if not Path(filepath).is_file():
        print("Cannot playback! File doesn't exist!")
        
    traj = source_code.Trajectory()
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
    filename = raw_input("Enter desired full name of .txt file: ")
    while filename == '':
        filename = raw_input("Enter desired full name of .txt file: ")

	# get directory path
    rospack = rospkg.RosPack()
    path = os.path.join(rospack.get_path("lab_common"), "playback_library/" + filename)
    
    # initialize node
    print("Initializing node... ")
    rospy.init_node("joint_playback")
    
    if Path(path).is_file():
        print("File already exists. Can only playback.")
        loop_count = int(raw_input("How many times would you like to loop playback? "))
        while not loop_count < 0 :
            loop_count = int(raw_input("How many times would you like to loop playback? "))
        playback("both", 100.0, "velocity", path, loop_count)
    else:
    	print("File does not exist. Must at least record.")

        # record/playback
        R = ['r', 'R']
        B = ['b', 'B']
    
        command = raw_input("Record(R)/Both(B)? ")
        if command in B:
            loop_count = int(raw_input("How many times would you like to loop playback? "))
            while loop_count < 0:
                loop_count = int(raw_input("How many times would you like to loop playback? "))
        if command in R or command in B:
            record(path, 100.0)
        if command in B:
            rospy.sleep(2)
            playback("both", 100.0, "velocity", path, loop_count)

    # cleanup and shutdown
    os.system("rosnode cleanup")
    rospy.signal_shutdown("All done!")
    

    
    
