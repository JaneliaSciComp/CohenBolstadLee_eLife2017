/*
	Janelia Optical Flow Meter DLL Interface Functions, Enumerate FTDI devices, Define Properties

	Author: Gus K Lott III, PhD
	May 10, 2010

	Neurobiological Instrumentation Engineer
	Howard Hughes Medical Institute - Janelia Farm Research Campus
	lottg@janelia.hhmi.org

*/
#include "mwadaptorimaq.h"
#include <windows.h>
#include "ftd2xx.h"
#include "TreadmillAdaptor.h"


void initializeAdaptor(){

}

void getAvailHW(imaqkit::IHardwareInfo* hardwareInfo){

	/*
		For each device you want to make available through your adaptor, you must
		create an IDeviceInfo object and then store the object in the IHardwareInfo
		object.  For each format supported by a device, you must create an
		IDeviceFormat object and then store the object in the IDeviceInfo object.

		IHardwareInfo -> IDeviceInfo (contains Adaptor data) -> IDeviceFormat (contains Adaptor data)
	*/

	imaqkit::IDeviceInfo * deviceInfo;
	imaqkit::IDeviceFormat * deviceFormat;
	char buf[512];
	DWORD numDevs;
	
	FT_ListDevices(&numDevs,NULL,FT_LIST_NUMBER_ONLY);


	//For each device: Device ID, Device name, supported formats, if camera files are supported

	//Steps:
	//1: Determine which devices are available through the SDK
	//2: For each device found, create an IDeviceInfo object
	//	2a: For each format supported by the device, create an IDeviceFormat Object
	//	2b: Add each device format object that you create to the IDeviceInfo object
	//3: Add the IDeviceInfo object to the IHardwareInfo object passed to this function
	bool gflag=true;
	for (unsigned int i = 0; i<numDevs; i++){
		sprintf(buf,"Janelia Optical Motion Detection System &d",i);
		deviceInfo = hardwareInfo->createDeviceInfo(i,buf);
		
		deviceFormat=deviceInfo->createDeviceFormat(i,"Default");
		deviceInfo->addDeviceFormat(deviceFormat,true);

		hardwareInfo->addDevice(deviceInfo);
	}


}

void getDeviceAttributes(const imaqkit::IDeviceInfo* deviceInfo, 
						 const char* sourceType, 
						 imaqkit::IPropFactory* devicePropFact,
						 imaqkit::IVideoSourceInfo* sourceContainer,
						 imaqkit::ITriggerInfo* hwTriggerInfo){

    void * hProp;
	int devID = deviceInfo->getDeviceID();


	sourceContainer->addAdaptorSource("TreadmillDefault",0);

	hProp = devicePropFact->createIntProperty("motionVideo",0,1,0);
	devicePropFact->setPropReadOnly(hProp,imaqkit::propreadonly::WHILE_RUNNING);
	devicePropFact->addProperty(hProp);

	hProp = devicePropFact->createIntProperty("PacketsPerFrame",0,200,200);
	devicePropFact->setPropReadOnly(hProp,imaqkit::propreadonly::WHILE_RUNNING);
	devicePropFact->addProperty(hProp);

		

}

imaqkit::IAdaptor* createInstance(imaqkit::IEngine* engine, imaqkit::IDeviceInfo* deviceInfo, char* formatName){
	
	//instantiate a dcamAdaptor object and pass it back to Matlab
	imaqkit::IAdaptor* adaptor = new TreadmillAdaptor(engine,deviceInfo,formatName);
	return adaptor;
}



void uninitializeAdaptor(){

}
