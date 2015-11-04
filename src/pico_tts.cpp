//pico-tts.cpp

#include <picoapi.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <stdio.h>
#include "ros/ros.h"

//stuff for action lib
#include <lab_common/ttsAction.h>
#include <actionlib/server/simple_action_server.h>
//action file

typedef actionlib::SimpleActionServer<lab_common::ttsAction> ActionServer;

#define MEM_SIZE 2500000
#define MAX_BUFF_SIZE 200

class TTSServer{
private:
	const unsigned char* taFileLoc;
	const unsigned char* sgFileLoc;
	char* voiceName;
	pico_Char* taResourceName;
	pico_Char* sgResourceName;
	pico_Resource taResource;
	pico_Resource sgResource;
	pico_System sys;
	pico_Engine engine;
	void* memSpace;
	bool initialized = false;

public:
	//constructor
	TTSServer(){
		taResourceName = (pico_Char*) malloc(PICO_MAX_RESOURCE_NAME_SIZE);
		sgResourceName = (pico_Char*) malloc(PICO_MAX_RESOURCE_NAME_SIZE);	
		memSpace = (void*)malloc(MEM_SIZE);
		taFileLoc = (const unsigned char*)"/usr/share/pico/lang/en-US_ta.bin";
		sgFileLoc = (const unsigned char*)"/usr/share/pico/lang/en-US_lh0_sg.bin";
		voiceName = (char*)"English";
	}

	~TTSServer(){
		free(taResourceName);
		free(sgResourceName);
		free(memSpace);
	}

	//initialize engine
	bool initialize(){
		pico_Status state = pico_initialize(memSpace, MEM_SIZE,&sys);
		if(state != PICO_OK){
			std::cout << "pico_initialize failed" << std::endl;
			return false;
		}
		state = pico_loadResource(sys,taFileLoc, &taResource);
		if(state != PICO_OK){
			std::cout << "failed to load resources" << std::endl;
		}
		state = pico_loadResource(sys,sgFileLoc, &sgResource);
		if(state != PICO_OK){
			std::cout << "error 3" << std::endl;
		}
		state = pico_createVoiceDefinition(sys,(const pico_Char*) voiceName);
		if(state != PICO_OK){
			std::cout << "error 4" << std::endl;
		}

		//Get resource name
		state = pico_getResourceName(sys, taResource,(char *)taResourceName);
		if(state != PICO_OK){
			std::cout << "error 4.1" << std::endl;
		}
		state = pico_getResourceName(sys, sgResource,(char *)sgResourceName);
		if(state != PICO_OK){
			std::cout << "error 4.2" << std::endl;
		}


		state = pico_addResourceToVoiceDefinition(sys,(const pico_Char*) voiceName, taResourceName);
		if(state != PICO_OK){
			std::cout << "error 5 " << std::endl;
		}
		state = pico_addResourceToVoiceDefinition(sys,(const pico_Char*) voiceName, sgResourceName);
		if(state != PICO_OK){
			std::cout << "error 6 " << std::endl;
		}
		state = pico_newEngine(sys,(const pico_Char*) voiceName, &engine);
		if(state != PICO_OK){
			std::cout << "error 7" << std::endl;
		}
		initialized = true;
	}

	//synthesize, might have to change this to a callback
	int synthesize(std::string str, int8_t* &outBuffer){
		//check whether initialize
		if(!initialized){
			ROS_INFO("not initialized");
			return -1;
		}
		//first convert the string to c_string
		const char* cString = str.c_str();
		//create a buffer for the out buffer
		//estimate based on the string
		int bufferSize = strlen(cString) * 1024 * 1024 * sizeof(int8_t);
		//std::cout << "bufferSize:" << bufferSize << std::endl;
		outBuffer = (int8_t*) malloc(1024 * 1024 * sizeof(int8_t));
		//calculate how much text is left
		//extra one for terminate string 
		pico_Int16 textLeft = strlen(cString) + 1;
		pico_Int16 sum = 0;
		pico_Int16 textRead = 0;
		//put the text in
		while(textLeft != sum){
			pico_putTextUtf8(engine, (const pico_Char*)cString, textLeft, &textRead);
			sum += textRead;
		}

		//loop until the get Data is done
		pico_Status status;
		pico_Int16 type;
		pico_Int16 bytesReturned;
		int ptr = 0;
		short tmpBuffer[MAX_BUFF_SIZE/2];
		do{
			//try getting data
			status = pico_getData(engine, (void*)tmpBuffer ,MAX_BUFF_SIZE, &bytesReturned, &type);
			if(bytesReturned > 0){
				//copy the tmpbuffer to outbuffer
				//check if we are going to overflow
				if(ptr + bytesReturned > bufferSize){
					//OVERFLOWWWW!!!!!
					ROS_INFO("buffer overflow");
					std::cerr << "buffer overflow, way too big";
					return -1;
				}
				memcpy(outBuffer+ptr, (int8_t*)tmpBuffer, bytesReturned);
				ptr += (bytesReturned);
				// /std::cout << ptr << std::endl;
			}
		}while(status != PICO_STEP_IDLE);
		//done!


		//playSound(outBuffer,ptr,(unsigned int)16000);
		return ptr;
	}

	void close(){
		initialized = false;
		pico_Status status = pico_disposeEngine(sys, &engine);
		if(status != PICO_OK){
			std::cout << "failed to dispose engine" << std::endl;
		}
		status = pico_releaseVoiceDefinition(sys, (const pico_Char*) voiceName);
			if(status != PICO_OK){
			std::cout << "failed release voice definition" << std::endl;
		}
		status = pico_unloadResource(sys, &taResource);
		if(status != PICO_OK){
			std::cout << "failed release ta Resource" << std::endl;
		}
		status = pico_unloadResource(sys, &sgResource);
		if(status != PICO_OK){
			std::cout << "failed release sg resource" << std::endl;
		}
		status = pico_terminate(&sys);
		if(status != PICO_OK){
			std::cout << "failed to terminate system " << std::endl;
		}
	}
};


TTSServer ttsServer;

void wrapperService(const lab_common::ttsGoalConstPtr& goal, ActionServer *as){
	int size = 0;
	int8_t* buf;	
	ROS_INFO("text:%s",goal->text.c_str());
	size = ttsServer.synthesize(goal->text,buf);
	ROS_INFO("size:%d",size);

	//definite the result and feedback
	lab_common::ttsFeedback progressFeedback;
	lab_common::ttsResult callResult;

	//check to make sure that the user didn't cancel
	if(as->isPreemptRequested()){
		free(buf);
		return;
	}

	if(size > 0){
		callResult.soundFile = std::vector<int8_t>(buf,buf+size);
		callResult.size = size;
	}	
	else{
		//something went wrong return empty vector
		callResult.soundFile = std::vector<int8_t>();
		callResult.size = 0;
	}
	free(buf);
	//set the method as success and pass in the result object
	as->setSucceeded(callResult);
}


int main(int argc, char **argv)
{
	//initialize TTS engine and service
	//TTSServer ttsServer;
	ttsServer.initialize();

  	ros::init(argc, argv, "pico_tts_server");
  	ros::NodeHandle n;
  	ActionServer actionServer(n,"synthesize",boost::bind(&wrapperService, _1, &actionServer), false);
  	actionServer.start();
  	ROS_INFO("pico tts server online");
  	ros::spin();
	ttsServer.close();
  return 0;
}






   // - pico_initialize
   // - pico_loadResource
   // - pico_createVoiceDefinition
   // - pico_addResourceToVoiceDefinition
   // - pico_newEngine
   // - pico_putTextUtf8
   // - pico_getData (several times)
   // - pico_disposeEngine
   // - pico_releaseVoiceDefinition
   // - pico_unloadResource
   // - pico_terminate

