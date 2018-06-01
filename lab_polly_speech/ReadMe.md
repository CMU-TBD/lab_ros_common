# Lab_Polly_Speech
CMU - Code released under MIT License

Contact - Zhi - zhi.tan@ri.cmu.edu


A Ros wrapper for Amazon Polly Text-to-Speech. It depends on the audio package to run.

## Usage

### Running 
Roslaunch the backend service that runs an actionlib server.
```
roslaunch lab_polly_speech polly_speech.launch
```

### Parameters
There are two ros parameters in the the launch file. `no_audio`, true if you just want to simulate it and not actually running the code. `polly_voice_id`, the voice ID that you want the generator to use (Don't use this if you want multiple voices in the same script. For that, see below). A list of voice ID can be found here: https://docs.aws.amazon.com/polly/latest/dg/voicelist.html

### Voices
To choose the voice of the robot in a script, follow the example of setting the voice_id parameter with every call of the speak function of the actionlib client in polly_test.py.

### Other Info
The actionlib server receives request at the topic `lab_polly_speech/speak` with the goal `speakAction` 
