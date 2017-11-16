
import actionlib
from actionlib_msgs.msg import GoalStatus
from lab_common.msg import(
    speakGoal,
    speakAction
)

class PollySpeech(object):

    def __init__(self):
        self._polly_client = actionlib.SimpleActionClient("lab_polly_speech/speak", speakAction)
        self._polly_client.wait_for_server()

    def speak(self, text, block=True):
        """
        Command the robot to speak the given text.

        Parameters
        ----------
        text : str
            The text that the robot will speak
        block : bool, optional
            Whether this call is a blocking call.
        """
        goal = speakGoal()
        goal.text = text
        if block:
            self._polly_client.send_goal_and_wait(goal)
        else:
            self._polly_client.send_goal(goal)

    def wait(self, duration=None):
        """
        Wait for the speech to finish. Note, sometimes the last few seconds of the speech will still be playing when it ends
        
        Parameters
        ----------
        duration : rospy.Duration
            Ros's implementation of Duration

        """
        if self._polly_client.gh:
            if duration is not None:
                result = self._polly_client.wait_for_result(duration)
            else:
                result = self._polly_client.wait_for_result()