/**
C++ self made audio client for maximum control
**/

#include <ros/ros.h>
#include <alsa/asoundlib.h>
#include <iostream>
#include <thread> //this is C++11, might break ros
#include <mutex>

//stuff for action lib
#include <lab_common/playAudioAction.h>
#include <actionlib/server/simple_action_server.h>


typedef actionlib::SimpleActionServer<lab_common::playAudioAction> ActionServer;

bool breakFlag = false;
snd_pcm_t *playback_handle;



//locks due to closing playback_handle halfway
std::mutex breakMtx;
std::mutex sndMtx;


void initialize(){
	const char *CARD_NAME = "default";
	int err;

	if ((err = snd_pcm_open (&playback_handle, CARD_NAME, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf (stderr, "cannot open audio device %s (%s)\n", 
             CARD_NAME,
             snd_strerror (err));
        exit (1);
    }
}

void setParam(unsigned int sampleRate){
	snd_pcm_hw_params_t *hw_params;	
	int err;
	if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0) {
        fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n",
             snd_strerror (err));
        exit (1);
    }
             
    if ((err = snd_pcm_hw_params_any (playback_handle, hw_params)) < 0) {
        fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_set_access (playback_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        fprintf (stderr, "cannot set access type (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_set_format (playback_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
        fprintf (stderr, "cannot set sample format (%s)\n",
             snd_strerror (err));
        exit (1);
    }
    if ((err = snd_pcm_hw_params_set_rate_near (playback_handle, hw_params, &sampleRate, 0)) < 0) {
        fprintf (stderr, "cannot set sample rate (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_set_channels (playback_handle, hw_params, 1)) < 0) {
        fprintf (stderr, "cannot set channel count (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params (playback_handle, hw_params)) < 0) {
        fprintf (stderr, "cannot set parameters (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    snd_pcm_hw_params_free (hw_params);
}

void release(){
	snd_pcm_drain(playback_handle); 
    snd_pcm_close(playback_handle);
}

bool _playSound(int8_t* data, int size, unsigned int sampleRate, ActionServer* as){

	//we always set breakFlag to false
	//since we assume its a restart every time _playSound starts
	//breakMtx.lock();
	//breakFlag = false;
	//breakMtx.unlock();

  int err;

  if ((err = snd_pcm_prepare (playback_handle)) < 0) {
    fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
    snd_strerror (err));
    exit (1);
  }
  
  //try play sound
  int remaining = size;
  int index = 0;
  int SIZE = 512;
  while(remaining > SIZE){
    
    //check if the user request a stop
    if(as->isPreemptRequested()){
      snd_pcm_drop(playback_handle);
      return false;
    }

    int _size = SIZE/2;
    if ((err = snd_pcm_writei (playback_handle, data + index, _size)) != _size) {
        fprintf (stderr, "write to audio interface failed (%s)\n",
             snd_strerror (err));
    }
    index += SIZE;
    remaining -= SIZE;
    //to sync it up,we going to slow down a bit here
    //TODO  
  }
  //play all the remaining stuff.
  
  //check if the user request a stop
  if(as->isPreemptRequested()){
    snd_pcm_drop(playback_handle);
    return false;
  }

  if (remaining > 0 && (err = snd_pcm_writei (playback_handle, data + index, remaining/2)) != remaining/2) {
      fprintf (stderr, "write to audio interface failed (%s)\n",
          snd_strerror (err));
  }
  //std::cout << "pre-drain" << std::endl;
  int rtn = snd_pcm_drain(playback_handle);
  //std::cout << "done:" << rtn << std::endl;
  return true;
}

void callback(ActionServer *as){

	//drop all of it.
	ROS_INFO("stopping audio playback");
	//set break flag to signal the thread
	//to stop
	//breakMtx.lock();
	//breakFlag = true;
	//breakMtx.unlock();
	//drop all the current frames in the system
	snd_pcm_drop(playback_handle);
	/*
	//close and reinitialize it
	snd_pcm_close(playback_handle);
	initialize();
	setParam(16000);
	*/
}


//std::thread soundThread;

void playSound(const lab_common::playAudioGoalConstPtr& goal, ActionServer *as){

	//make sure the previous instance of the soundThread is done
	//if(soundThread.joinable()){
  //		soundThread.join();
	//}


	//call the new sound thread
	int8_t* data = (int8_t*) &(goal->soundFile)[0];
	int size = goal->size;
	unsigned int sampleRate = (unsigned int)goal->rate;
  if(_playSound(data,size,sampleRate,as)){
    lab_common::playAudioResult callResult;
    callResult.complete = true;
    //set the method as success and pass in the result object
    as->setSucceeded(callResult);
  }
  else{
    //set the action by empty
    as->setPreempted();
  }

	//soundThread = std::thread(_playSound, data, size, sampleRate);
	//res.complete = true;
	//soundThread.join();
	//soundThread.detach();
	//return true;
}



int main(int argc, char **argv)
{

	initialize();
	setParam(16000);
  
  ros::init(argc, argv, "audioServer");
  ros::NodeHandle n;

  ActionServer actionServer(n,"playAudio",boost::bind(&playSound, _1, &actionServer), false);
  actionServer.registerPreemptCallback(boost::bind(&callback, &actionServer));
  actionServer.start();
  //ros::Subscriber sub = n.subscribe<std_msgs::Empty>("stopAudio",1,callback);
  ROS_INFO("Ready to playAudio");
  ros::spin();
  release();

  return 0;
}