
import actionlib
from lab_common.msg import(
    speakGoal,
    speakAction
)

class PollySpeech(object):

    def __init__(self):
        self._polly_client = actionlib.SimpleActionClient("lab_polly_speech/speak", speakAction)
        self._polly_client.wait_for_server()

    def speak(self, text, block=True):
        goal = speakGoal()
        goal.text = text
        if block:
            self._polly_client.send_goal_and_wait(goal)
        else:
            self._polly_client.send_goal(goal)

    def wait(self):
        pass
        #self._polly_client