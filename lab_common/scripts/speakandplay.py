#!/usr/bin/env python

import rospy
from playback.playback import record, playback
from lab_polly_speech.polly_speech import PollySpeech

if __name__ == '__main__':
    rospy.init_node("multinode")
    ps = PollySpeech()
    #record("multi.txt")
    ps.speak("This is a sentence for sure", voice_id="Matthew")
    #playback("multi.txt")
