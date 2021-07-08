
import actionlib
from actionlib_msgs.msg import GoalStatus
from lab_polly_speech.msg import(
    pollySpeechGoal,
    pollySpeechAction
)

class PollySpeech(object):

    def __init__(self):
        self._polly_client = actionlib.SimpleActionClient("lab_polly_speech/speak", pollySpeechAction)
        self._polly_client.wait_for_server()

    def speak(self, text, block=True, cancel=False, voice_id="Joanna", pitch="20%"):
        """
        Command the robot to speak the given text.

        Parameters
        ----------
        text : string, required
            The text that the robot will speak
        voice_id : string, required
            Name of desired voice from AWS Polly Speech
        pitch : string, required 
            Desired pitch of voice as a percentage (ex: "20%")
        block : bool, optional
            Whether this call is a blocking call.
        cancel : bool, optional
            Whether to cancel all the current speech request to say this instead of 
            queue it
        """
        goal = pollySpeechGoal()
        goal.text = text
        goal.voice_id = voice_id
        goal.pitch = pitch

        if cancel:
            self._polly_client.cancel_all_goals()
        if block:
            self._polly_client.send_goal_and_wait(goal)
        else:
            self._polly_client.send_goal(goal)

    def stopAll(self):
        """Stop all of the current speech executing by the robot. This will also stop
        speech uterances from other programs
        """
        self._polly_client.cancel_all_goals()

    def stop(self):
        """Stop the current speech that is being executed
        """ 
        self._polly_client.cancel_goal()

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


import rospy

def main():
    rospy.init_node('polly_test')
    ps = PollySpeech()
    ps.speak("I am a scary robot",voice_id='Joanna')
    ps.speak('Hello World',voice_id='Emma', block=False, cancel=True)
    rospy.sleep(0.5)
    ps.stopAll()
    rospy.spin()

if __name__ == '__main__':
    main()