#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <lab_common/ttsAction.h>
#include <lab_common/playAudioAction.h>
#include <lab_common/speakAction.h>
#include <vector>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <chrono>
#include <thread>


void speak(std::string text, actionlib::SimpleActionClient<lab_common::playAudioAction>* acp){
	//create the clients
	actionlib::SimpleActionClient<lab_common::ttsAction> ac("lab_common/synthesize", true);
	//actionlib::SimpleActionClient<lab_common::playAudioAction> acp("lab_common/playAudio", true);
	//wait for them to connect
	ac.waitForServer();
	acp->waitForServer();
	//set a goal
	lab_common::ttsGoal goal;
	goal.text = text;
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		ROS_INFO("finish TTS");
		actionlib::SimpleClientGoalState state = ac.getState();
		//get the result
		boost::shared_ptr<const lab_common::ttsResult> result = ac.getResult();
	    ROS_INFO("Action finished: %s",state.toString().c_str());

		lab_common::playAudioGoal audioGoal;
		audioGoal.soundFile = result->soundFile;
		audioGoal.size = result->size;
		audioGoal.rate = 16000;

		acp->sendGoal(audioGoal);
		return;
	}
	else{
		ROS_INFO("Action did not finish before the time out.");
		return;
	}
}

void _speak(std::string text, actionlib::SimpleActionClient<lab_common::speakAction>* asc){
	std::cout << "waiting for server" << std::endl;
	asc->waitForServer();
	std::cout << "found server" << std::endl;
	lab_common::speakGoal goal;
	goal.text = text;
	asc->sendGoal(goal);
	std::cout << "sent goal" << std::endl;
	asc->waitForResult();
}


int main(int argc, char **argv){

	ros::init(argc, argv, "audio_tester");

	actionlib::SimpleActionClient<lab_common::speakAction> asc("lab_common/speak", true);
	_speak("Hi, I am Joe", &asc);
	_speak("This is a very long sentence, I'm hoping we can actually finish this", &asc);
	_speak("This is a second sentence", &asc);
	_speak("This is the third sentence", &asc);

	/*
	actionlib::SimpleActionClient<lab_common::playAudioAction> acp("lab_common/playAudio", true);
	speak("Hello World",&acp);
	acp.waitForResult();
	speak("Another Sentence",&acp);
	acp.waitForResult();
	speak("Another very long sentence",&acp);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	acp.cancelAllGoals();
	speak("It works",&acp);
	*/
}