#!/usr/bin/env python

import rospy
from lab_common.msg import(
    playAudioGoal,
    playAudioAction,
    speakResult,
    speakAction
)
import boto3
from botocore.exceptions import BotoCoreError, ClientError
import actionlib
from contextlib import closing
import struct
import rospkg
import os

class PollyAudioLibrary(object):
    def __init__(self):
        #library directory, we could save the file but use of DB will be better in the future
        rospack = rospkg.RosPack()

        self._lib_directory = os.path.join(rospack.get_path("lab_polly_speech"),'audio_library')
        if not os.path.exists(self._lib_directory):
            os.makedirs(self._lib_directory)
        self._lib_list = dict()
        
    def _scan_library(self):
        file_list = os.listdir(self._lib_directory)
        for file in file_list:
            voice_id = file.split('_')[0]
            formatted_text = file.split('_',1)[1]
            if voice_id not in self._lib_list:
                self._lib_list[voice_id] = dict()
            self._lib_list[voice_id][formatted_text] = None
            
    def save_text(self, text, voice_id, data):
        formatted_text = self._format_text(text)
        file_name = voice_id + "_" + formatted_text

        if voice_id not in self._lib_list:
            self._lib_list[voice_id] = dict()
        
        #we won't save it if it's already in the list    
        if formatted_text in self._lib_list[voice_id]:
            return 

        #we won't save if the file name is too long
        if len(file_name.encode('utf8')) > 256:
            rospy.loginfo("File name too long, not saving it in files")
        else:
            with open(os.path.join(self._lib_directory,file_name),'wb') as file:
                file.write(data)
            
        self._lib_list[voice_id][formatted_text] = data
        
    def _format_text(self, text):
        return ('_').join(text.lower().split(' '))

    def find_text(self, text, voice_id):
        formatted_text = self._format_text(text)
        #print(formatted_text)
        #print(self._lib_list)
        if self.sythesized_audio_exist(text, voice_id):
            data = self._lib_list[voice_id][formatted_text]
            if data is None:
                rospy.loginfo("reading audio file from disk")
                file_name = voice_id + '_' + formatted_text
                with open(os.path.join(self._lib_directory,file_name),'r') as file:
                    data = file.read()
                self._lib_list[voice_id][formatted_text] = data
            return data
        rospy.logdebug("audiofile doesn't existing")
        return None

    def sythesized_audio_exist(self, text, voice_id):
        formatted_text = self._format_text(text)
        if voice_id not in self._lib_list:
            return False
        return formatted_text in self._lib_list[voice_id]



class PollyNode(object):

    def __init__(self):
        self._polly = boto3.client('polly', region_name='us-east-1')

        self._audio_client = actionlib.SimpleActionClient("lab_common/playAudio", playAudioAction)
        self._audio_client.wait_for_server()


        self._speak_server = actionlib.SimpleActionServer("lab_polly_speech/speak", speakAction, self._speak_callback, auto_start=False)
        self._speak_server.start()

        self._audio_lib = PollyAudioLibrary()
        self._audio_lib._scan_library()
        
        #debug flag
        self._no_audio_flag = rospy.get_param('polly_node/no_audio',False)
        self._voice_id = rospy.get_param('polly_node/polly_voice_id','Ivy')

        rospy.loginfo("PollyNode ready")

    def _synthesize_speech(self, text, voice_id):
        
        #this appends the pitch changes, so we can get a different voice
        #ignore this change if it's already in ssml
        if not text.startswith('<speak>'):
            ammended_text = '<speak><prosody pitch="20%">{}</prosody></speak>'.format(text)
        else:
            ammended_text = text
        
        try:
            response = self._polly.synthesize_speech(Text=ammended_text, OutputFormat='pcm',VoiceId=voice_id, TextType='ssml')
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

    def _speak_callback(self, goal):

        text = goal.text
        complete = True
        rospy.logdebug('POLLY_SPEAK:{}'.format(text))
        if not self._no_audio_flag:
            complete = self.speak(text, self._voice_id)
        result = speakResult()
        result.complete = complete
        self._speak_server.set_succeeded(result)


    def speak(self,text,voice_id):
        
        data = None

        #try finding the text if it's not an an ssml file
        if not text.startswith('<speak>'):
            data = self._audio_lib.find_text(text,voice_id)
        
        if data is None:
            rospy.loginfo("sythesizing speech with AWS")
            data = self._synthesize_speech(text, voice_id)

        if data is not None:

            #save it if not ssml file
            if not text.startswith('<speak>'):
                self._audio_lib.save_text(text,voice_id,data)

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
    # pl.speak("Hello, My name is Rathu")
    # pl.speak("Hello, My name is Rathu")
    #print(pl._synthesize_speech("hello"))

    #main()
