#!/usr/bin/env python

import rospy
from lab_common.msg import(
    playAudioGoal,
    playAudioAction,
    polly_speechAction,
    polly_speechGoal,
    polly_speechResult
)
import boto3
from botocore.exceptions import BotoCoreError, ClientError
import actionlib
from contextlib import closing
import struct
import rospkg
import os
from masterfile import Item, HashTable

# set hash size at 32    
h = HashTable(10 ** 32)

class PollyAudioLibrary(object):
    def __init__(self):
        # library directory, we could save the file but use of DB will be better in the future
        rospack = rospkg.RosPack()

        self._lib_directory = os.path.join(rospack.get_path("lab_polly_speech"),'audio_library')

        if not os.path.exists(self._lib_directory):
            os.makedirs(self._lib_directory)

    def save_text(self, text, voice_id, data):
        key = Item(voice_id, text)
        file_name = str(h.hashing(key)) + ".mp3"

        # only save if unique
        if h.find(key) == None:
            h.insert(key)
            with open(os.path.join(self._lib_directory, file_name),'wb') as file:
                file.write(data)

    def find_text(self, text, voice_id):
        key = Item(voice_id, text)
        file_name = str(h.hashing(key)) + ".mp3"
        if h.find(key) != None:
            with open(os.path.join(self._lib_directory, file_name),'r') as file:
                data = file.read()
            return data
        rospy.logdebug("audiofile doesn't exist")
        return None
    

class PollyNode(object):

    def __init__(self):
        self._polly = boto3.client('polly', region_name='us-east-1')

        self._audio_client = actionlib.SimpleActionClient("lab_common/playAudio", playAudioAction)
        self._audio_client.wait_for_server()


        self._speak_server = actionlib.SimpleActionServer("lab_polly_speech/speak", polly_speechAction, self._speak_callback, auto_start=False)
        
        self._speak_server.register_preempt_callback(self._preempt_callback)
        self._speak_server.start()

        self._audio_lib = PollyAudioLibrary()

        # debug flag
        self._no_audio_flag = rospy.get_param('polly_node/no_audio', False)
        self._voice_id = rospy.get_param('polly_node/polly_voice_id')

        rospy.loginfo("PollyNode2 ready")

    def _synthesize_speech(self, text, voice_id):
        # this appends the pitch changes, so we can get a different voice
        # ignore this change if it's already in ssml
        if not text.startswith('<speak>'):
            ammended_text = '<speak><prosody pitch="20%">{}</prosody></speak>'.format(text)
        else:
            ammended_text = text

        try:
            response = self._polly.synthesize_speech(Text=ammended_text, OutputFormat='pcm', VoiceId=voice_id, TextType='ssml')
        except(BotoCoreError, ClientError) as error:
            print(error)
            return None

        if("AudioStream" in response):
            data = None
            with closing(response["AudioStream"]) as stream:
                data = stream.read()
            return data
        else:
            print("ERROR")
            return None

    def _preempt_callback(self):
        #check if audio is running
        if self._audio_client.get_state() == actionlib.SimpleGoalState.ACTIVE:
            self._audio_client.cancel_goal()

    def _speak_callback(self, goal):

        text = goal.input
        complete = True
        rospy.logdebug('POLLY_SPEAK:{}'.format(text[0]))
        if not self._no_audio_flag:
            complete = self.speak(text)
        #create the result
        result = polly_speechResult()
        result.complete = complete
        #check if speak failed because it is being preempted
        if not complete and self._speak_server.is_preempt_requested():
            self._speak_server.set_preempted(result)
        else:
            self._speak_server.set_succeeded(result)
        
    def speak(self, text):
        data = None

        # try finding the text if it's not an an ssml file
        if not text[0].startswith('<speak>'):
            data = self._audio_lib.find_text(text[0], text[1])

        if data is None:
            rospy.loginfo("synthesizing speech with AWS")
            data = self._synthesize_speech(text[0], text[1])

        if self._speak_server.is_preempt_requested():
            return False

        if data is not None:

            # save it if not ssml file
            if not text[0].startswith('<speak>'):
                self._audio_lib.save_text(text[0], text[1], data)

            goal = playAudioGoal()
            goal.soundFile = data
            goal.rate = 16000
            goal.size = len(data)

            self._audio_client.send_goal_and_wait(goal)

        return data is not None


if __name__ == '__main__':
    rospy.init_node("polly_node")
    pl = PollyNode()
    rospy.spin()
