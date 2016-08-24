
#system modules
import os
import struct

#ros packages
import roslib
import rospy
#roslib.load_manifest('lab_common') #load the lab_common's manifest
import actionlib
import rospkg #to get information about packages
from lab_common.msg import ttsAction, ttsGoal
from lab_common.msg import playAudioAction, playAudioGoal
from lab_common.msg import(
    speakAction,
    speakGoal,
    speakResult
) 
from actionlib_msgs.msg import *


""" Audio Controller and Sound Representation """

class SoundFile(object):
    """
    Sound File representation
    """
    rate = 0
    size = 0
    data = []

class AudioController(object):
    """
    Class that is a wrapper for our audio modules
    """
    def __init__(self):
        #

        self.ac = actionlib.SimpleActionClient('lab_common/synthesize', ttsAction)
        self.acp = actionlib.SimpleActionClient('lab_common/playAudio', playAudioAction)
        self._speak_client = actionlib.SimpleActionClient('lab_common/speak',speakAction)


        self.ac.wait_for_server()
        self.acp.wait_for_server()
        self.soundFiles = [];

    def stopPlaying(self):
        self.acp.cancel_all_goals()

    def add_file(self, file_dir):
        rospy.loginfo("test")
        fData =[];
        #find the path to the package and then the res file
        #rospack = rospkg.RosPack() #get an instance of the rospack
        wavPath = file_dir
        #wavPath = os.path.join(rospack.get_path('hand_input'),'res','beep.wav');
        #print(wavPath);
        with open(wavPath, "rb") as f:
            fData = f.read()
        print(len(fData));

        print(fData[0:4])
        if (fData[0:4] != "RIFF"):
            print("not valid format")
            return
        if (fData[8:12] != "WAVE"):
            print("not wave file")
            return

        #first get the rate
        rate = struct.unpack('1i',fData[24:28])
        size = struct.unpack('1i',fData[40:44])
        print(rate[0])
        print(size[0])
        
        sf = SoundFile()
        print(len(fData[44:]))
        sf.data = struct.unpack(str(len(fData[44:])) + 'b',fData[44:])
        sf.size = size[0]
        sf.rate = rate[0]

        #save the soundfiler
        self.soundFiles.append(sf)
        return len(self.soundFiles) - 1 #return the identifier for that sound file
        #load some statically available sounds file

    def play_sounds(self,type):
        #decode the type of files and feed them to the audio part
        sfres = self.soundFiles[type]
        self.playAudio(sfres.data,sfres.rate,sfres.size)

    def playAudio(self, data, rate, size, block=False):

        self.stopPlaying()

        #call the action lib for audio
        paGoal = playAudioGoal()
        paGoal.soundFile = data
        paGoal.rate = rate
        paGoal.size = size
        self.acp.send_goal(paGoal)
        if block:
            self.acp.wait_for_result()       


    def speak(self, text, wait=True):
        """
        Wrapper for calling the speak modules

        text -- str:the text to be spoke by TTS
        wait -- bool:Whether the function should be blocking (default True)

        return -- bool:whether the action was success or not
        """
        goal = speakGoal()
        goal.text = text
        #send the goal to the client
        self._speak_client.send_goal(goal)
        rospy.loginfo("speaking:" + text)
        #if wait is true, wait for it
        if wait:
            self._speak_client.wait_for_result()
            result = self._speak_client.get_result()
            return result.complete
        else:
            return True

    def wait_for_speak(self, timeout = rospy.Duration()):
        """
        Blocking function that waits until the speak finishes
        """
        #first see if there is a goal currently go on
        goal_state = self._speak_client.get_state()
        print(goal_state)
        if goal_state == GoalStatus.ACTIVE:
            #only when it's active
            #we wait for result
            if self._speak_client.wait_for_result(timeout):
                result = self._speak_client.get_result()
                return result.complete
            else:
                #timeout
                return False
        elif goal_state == GoalStatus.SUCCEEDED:
            #The action is done
            result = self._speak_client.get_result()
            return result.complete
        else:
            #Might just don't have a result
            return False



    def speak_old(self, text,block=False,rate=16000):

        self.stopPlaying()

        print "SPEAK:" + text

        synthesizeGoal =ttsGoal()
        synthesizeGoal.text = text
        self.ac.send_goal(synthesizeGoal)
        #now wait for the result to return
        if self.ac.wait_for_result(rospy.Duration.from_sec(5.0)):
            #get the resullt
            result = self.ac.get_result()
            paGoal = playAudioGoal()
            paGoal.soundFile = result.soundFile
            paGoal.rate = rate
            paGoal.size = result.size
            self.acp.send_goal(paGoal)
            if block:
                self.acp.wait_for_result()