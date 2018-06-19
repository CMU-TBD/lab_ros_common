# lab_ros_common
Shared ROS packages that should be able to work across platforms

## Usage

### Speech Module
Import in a script using:
```
from lab_polly_speech.polly_speech import PollySpeech

ps = PollySpeech()
ps.speak('Hello World',voice_id='Emma', block=False, cancel=True)
ps.wait(timeout=rospy.Duration()) #wait for speech to complete if not blocking
ps.stop() #stop the speech running right now. This cuts off the audio if playing
ps.stopAll() #stop all speech
```
Then to start the action server:
```
roslaunch lab_polly_speech polly_speech.launch
```
