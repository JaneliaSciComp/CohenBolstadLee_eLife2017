/* Definition of the Treadmill interface for acquisition from a given camera
	This is the Image Acquisition Adaptor Class

	Author: Gus K Lott III, PhD
	May 10, 2010
	
	Neurobiological Instrumentation Engineer
	Howard Hughes Medical Institute - Janelia Farm Research Campus
	lottg@janelia.hhmi.org
*/

#ifndef __TREADMILL_ADAPTOR_HEADER__
#define __TREADMILL_ADAPTOR_HEADER__

#include "mwadaptorimaq.h"
#include <windows.h>
#include "ftd2xx.h"

class TreadmillAdaptor : public imaqkit::IAdaptor {
	
public:

	//Constructor and Destructor
	TreadmillAdaptor(imaqkit::IEngine* engine, imaqkit::IDeviceInfo* deviceInfo, const char* formatName);
	virtual ~TreadmillAdaptor();

	//Adaptor and Image Information Functions
	virtual const char* getDriverDescription() const;
	virtual const char* getDriverVersion() const;
	virtual int getMaxWidth() const;
	virtual int getMaxHeight() const;
	virtual int getNumberOfBands() const;
	virtual imaqkit::frametypes::FRAMETYPE getFrameType() const;

	//Image Acquisition Functions
	virtual bool openDevice();
	virtual bool closeDevice();
	virtual bool startCapture();
	virtual bool stopCapture();

	//Customized Properties
	bool capflag;
	int motionVideo;
	int devID;
	int PacketsPerFrame;
	imaqkit::IPropContainer * props;  //for updates

private:

	imaqkit::IPropContainer* _enginePropContainer;
	imaqkit::IDeviceInfo* _di;
	imaqkit::ICriticalSection* _grabSection;

	HANDLE _acquireThread; //Thread variable
	DWORD _acquireThreadID; //Thread ID returned by Windows
	static DWORD WINAPI acquireThread(void* param); //Declaration of acquisition thread function

};


#endif