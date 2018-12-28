#!/usr/bin/env python

###############################################################################
# RosLaunch Automator
#
# Authors: Joe Connolly <joe.connolly@yale.edu>,
#          Amal Nanavati <arnanava@alumni.cmu.edu>
#
# DESCRIPTION
#
# This rosnode is intended to automate the running of multiple roslaunch files
# sequentially. It is especially useful for running simulation experiments,
# or processing/analyzing collected data. A user just has to specify the
# exact command line commands they would use to run their launchfiles in a list,
# and this script will iterate through them and run them one at a time. The
# only requirement is that a node from the roslaunch file publishes an Empty
# message to the /simulationFinished topic when the launchfile is finished,
# so this node knows to terminate the launchfile and move onto the next one.
#
# USAGE: `rosrun lab_common roslaunchAutomator.py`
#
# REQUIREMENTS
#     - When the user wants the launchfile terminated, a node must send a
#       std_msgs/Empty message to the /simulationFinished topic (in the top-level
#       namespace).
#     - This code assumes that the underlying rosnodes are already compiled and
#       won't be changing. Therefore, everything that the user wants to change
#       between roslaunch files must be passed into the launchfile as an arg,
#       and dealt with appropiately by the launchfile (probably by passing them
#       as params to the appropriate node).
#     - As with any python rosnode, this node must be made executable using
#       `chmod +x path/to/node/roslaunchAutomator.py`
#
# TIPS:
#     - It is crucial for this package to recieve the /simulationFinished
#       message. Since cometimes messages get lost, we recommend sending the
#       /simulationFinished message multiple times (10-100) with a brief ros sleep
#       after every publish. This package will still work if it recieves multiple
#       messages on that topic, but will not work if it recieves none.
#     - If your roslaunch file or any of the nodes requires sudo access, this
#       script must be run with root access, either through `sudo` or `sudo su`
#     - As with any time you are running code over a long period of time, first
#       run it on smaller examples to make sure it works! Similarly with this
#       file, test it on each launchfile to make sure there were no typos
#       before the final run!
#     - A word of warning: if you want to run multiple of these scripts at once,
#       be sure to: 1) change the name of each concurrent instance of this node,
#       so they don't conflict and kill one, and 2) change the name of the
#       /simulationFinished topic so that one launchfile finishing does not
#       cause the other iteration of this file to terminate early.
#
# Future Work:
#     - Add a timeout to this file, so in case a node fails to publish to
#       /simulationFinished, or in case a roslaunch file fails to launch or
#       terminates early, this node can still continue with the other roslaunch
#       files it has in its list.
#
###############################################################################

import rospy
from std_msgs.msg import Empty
import subprocess
import signal

class Simulation:
    def __init__(self):
        self._result_sub = rospy.Subscriber("/simulationFinished", Empty, self._result_cb, queue_size=1000)
        self._processEnded = True
        self._processIsKilled = True

    def _result_cb(self, msg):
        rospy.logwarn("Simulation Finished")
        if (not self._processIsKilled): self._processEnded = True

    def _simulate(self, args):
        if self._processIsKilled:
            self._child = subprocess.Popen(args)
            self._processIsKilled = False
            self._processEnded = False
        else:
            rospy.logerr("Do not simulate a new process before the old process is killed")

    def _kill(self):
        if (not self._processIsKilled):
            self._child.send_signal(signal.SIGINT)
            self._child.wait()
            self._processIsKilled = True

    def _wait(self):
        r = rospy.Rate(2.0)
        while (not rospy.is_shutdown() and not s._processEnded):
            r.sleep()


if __name__ == '__main__':
    rospy.init_node("roslaunchAutomator")

    s = Simulation()

    try:
        commands = []

        pkgName = "pkg"
        launchFileName = "launchfile.launch"

        # All args should be strings, to avoid inconsitencies between how
        # python string-ifies datatypes and how roslaunch un-stringifies them.
        for arg0 in ["foo", "bar", "foobar"]:
            for arg1 in ["0", "1", "2"]:
                arg2 = "true"
                arg3 = "[Hello, World]"

                commands.append(["roslaunch", pkgName, launchFileName, "arg0:=%s" % arg0, "arg1:=%s" % arg1, "arg2:=%s" % arg2, "arg3:=%s" % arg3])

        for args in commands:
            rospy.loginfo("Running %s" % repr(args))
            s._simulate(args)
            s._wait()
            s._kill()
    except Exception as e:
        rospy.logerr("Got Exception %s", repr(e))
        s._kill()
