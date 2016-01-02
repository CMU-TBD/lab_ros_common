#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <chrono>
#include <thread>

//stuff for action lib
#include <lab_common/ttsAction.h>
#include <lab_common/playAudioAction.h>
#include <lab_common/speakAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionServer<lab_common::speakAction> ActionServer;

void speak(const lab_common::speakGoalConstPtr& ori_goal, actionlib::SimpleActionClient<lab_common::ttsAction>* ac, 
	 actionlib::SimpleActionClient<lab_common::playAudioAction>* acp, ActionServer *as){
	//get the text we want to speak
	std::string text = ori_goal->text;

	//set a goal
	lab_common::ttsGoal goal;
	goal.text = text;
	ac->sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac->waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		ROS_INFO("finish TTS");
		actionlib::SimpleClientGoalState state = ac->getState();
		//get the result
		boost::shared_ptr<const lab_common::ttsResult> result = ac->getResult();
	    ROS_INFO("Action finished: %s",state.toString().c_str());

		lab_common::playAudioGoal audioGoal;
		audioGoal.soundFile = result->soundFile;
		audioGoal.size = result->size;
		audioGoal.rate = 16000;

		acp->sendGoal(audioGoal);
		acp->waitForResult(); //wait for the end
		//complete now we send the result back
	    lab_common::speakResult callResult;
	    callResult.complete = true;
	    //set the method as success and pass in the result object
	    as->setSucceeded(callResult);
		return;
	}
	else{
		ROS_INFO("Action did not finish before the time out.");
		return;
	}
}

void callback(actionlib::SimpleActionClient<lab_common::ttsAction>* ac, 
	 actionlib::SimpleActionClient<lab_common::playAudioAction>* acp, ActionServer *as){
	//nothing now, but who knows
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "audio_controller");
	ros::NodeHandle n;

	//create the clients
	actionlib::SimpleActionClient<lab_common::playAudioAction> acp("playAudio", true);
	actionlib::SimpleActionClient<lab_common::ttsAction> ac("synthesize", true);
	//actionlib::SimpleActionClient<lab_common::playAudioAction> acp("lab_common/playAudio", true);
	//wait for them to connect
	ac.waitForServer();
	acp.waitForServer();


	ActionServer actionServer(n,"speak",boost::bind(&speak, _1, &ac, &acp, &actionServer), false);

  	//actionServer.registerPreemptCallback(boost::bind(&callback,&ac,&acp,&actionServer));
  	actionServer.start();
  	//ros::Subscriber sub = n.subscribe<std_msgs::Empty>("stopAudio",1,callback);
  	ROS_INFO("Ready for speak");
  	ros::spin();
  	return 0;
}