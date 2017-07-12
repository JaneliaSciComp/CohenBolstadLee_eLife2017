/* Implementation of the Treadmill interface for acquisition from a given camera
	This is the Image Acquisition Adaptor Class

	Author: Gus K Lott III, PhD
	May 10, 2010
	
	Neurobiological Instrumentation Engineer
	Howard Hughes Medical Institute - Janelia Farm Research Campus
	lottg@janelia.hhmi.org

*/

#include "TreadmillAdaptor.h"
#include "TreadmillPropListener.h"
#include "TreadmillSourceListener.h"

#include <stdio.h>


//Class Constructor
TreadmillAdaptor::TreadmillAdaptor(imaqkit::IEngine* engine, imaqkit::IDeviceInfo* deviceInfo, const char* formatName):imaqkit::IAdaptor(engine){

	//setup listeners
	_enginePropContainer = getEngine()->getAdaptorPropContainer();
	_enginePropContainer->addListener("SelectedSourceName", new TreadmillSourceListener(this));

	imaqkit::IPropContainer* adaptorPropContainer = getEngine()->getAdaptorPropContainer();
	int numDeviceProps = adaptorPropContainer->getNumberProps();
	const char **devicePropNames = new const char*[numDeviceProps];
	adaptorPropContainer->getPropNames(devicePropNames);

	for (int i = 0; i < numDeviceProps; i++){

         // Get the property information object.
         imaqkit::IPropInfo* propInfo = adaptorPropContainer->getIPropInfo(devicePropNames[i]);

         // Check to see whether the property is device-specific. Do not create
         // create property listeners for non device-specific properties such
         // as 'Parent' and 'Tag'.
         if (propInfo->isPropertyDeviceSpecific()) {
             adaptorPropContainer->addListener(devicePropNames[i], new TreadmillPropListener(this));
         }
     }
	delete [] devicePropNames;
	_grabSection = imaqkit::createCriticalSection();


	
	//props = engine->getAdaptorPropContainer();
	capflag = false;
	motionVideo = 0;
	PacketsPerFrame = 200;

	//imaqkit::adaptorWarn("TreadmillAdaptor:debug",formatName);

	devID = deviceInfo->getDeviceID();

	//imaqkit::adaptorWarn("TreadmillAdaptor:debug","In Constructor");
	


}

//Class Destructor
TreadmillAdaptor::~TreadmillAdaptor(){
	//When the C1394Camera Object is destroyed, it cleans itself up (Stops acquisition and frees resources)
}



//Device Driver information functions
const char* TreadmillAdaptor::getDriverDescription() const{
	return "Optical Flow Meter";
}
const char* TreadmillAdaptor::getDriverVersion() const{
	return "1.0";
}

//Image data information functions
int TreadmillAdaptor::getMaxWidth() const { 
	return 60;
}

int TreadmillAdaptor::getMaxHeight() const { 
	return 200;
}

int TreadmillAdaptor::getNumberOfBands() const { 
	//Image Acquisition toolbox software only supports image data with 1 or 3 bands
	return 1; }

imaqkit::frametypes::FRAMETYPE TreadmillAdaptor::getFrameType() const {
	return imaqkit::frametypes::MONO8;
}

//Image acquisition functions
bool TreadmillAdaptor::openDevice() { 

	//If device is already open, return true.
	if (isOpen()) return true;

	//imaqkit::adaptorWarn("TreadmillAdaptor:debug","Constructing Acquisition Thread");
	//Create acquisition thread to poll serial interface
	_acquireThread = CreateThread(NULL,0,acquireThread,this,0,&_acquireThreadID);
	if ( _acquireThread == NULL ){closeDevice();return false;}
	
	//Wait for thread to create message queue.
	while(PostThreadMessage(_acquireThreadID, WM_USER+1,0,0) == 0) 
		Sleep(1);

	return true; 
}

bool TreadmillAdaptor::closeDevice() { 
	//If the device is not open, return.
	if(!isOpen()) return true;

	//Terminate and close the acquisition thread.
	if(_acquireThread){
		// Send WM_QUIT message to thread
		PostThreadMessage(_acquireThreadID, WM_QUIT, 0,0);

		//Give the thread a chance to finish
		WaitForSingleObject(_acquireThread, 1000);

		//Close thread handle
		CloseHandle(_acquireThread);
		_acquireThread = NULL;
	}
	capflag = false;

	return true; 
}
bool TreadmillAdaptor::startCapture() { 
	//Check if device is already acquiring frames
	if (isAcquiring()) return false;

	//Send start message to acquisition thread
	PostThreadMessage(_acquireThreadID, WM_USER,0,0);
	capflag = true;

	return true; 
}

bool TreadmillAdaptor::stopCapture() { 
	//If the device is not acquiring data, return
	if (!isOpen()) return true;

	capflag = false;  //Message to thread to stop
	Sleep(40);
	return true; 
}


