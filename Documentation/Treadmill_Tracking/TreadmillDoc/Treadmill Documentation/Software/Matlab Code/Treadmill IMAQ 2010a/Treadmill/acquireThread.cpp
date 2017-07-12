/* 	This file contains only the acquire thread that is spawned to poll for data 
		from the FTDI treadmill interface and hand that data back to MATLAB

	Author: Gus K Lott III, PhD
	May 10, 2010
	
	Neurobiological Instrumentation Engineer
	Howard Hughes Medical Institute - Janelia Farm Research Campus
	lottg@janelia.hhmi.org

*/
#include "TreadmillAdaptor.h"

DWORD WINAPI TreadmillAdaptor::acquireThread(void* param){

	TreadmillAdaptor* adaptor = reinterpret_cast<TreadmillAdaptor*>(param);
	MSG msg;
	unsigned int imWidth,imHeight;
	unsigned char * rBuffer;  //Data buffer to read from treadmill

	//FTDI Variables
	FT_STATUS ftStatus = FT_OK;
	FT_HANDLE ftHandle;
	unsigned char wBuffer[1024];
	DWORD txBytes = 0;
	DWORD rxBytes;
	
	
	//Switch on type of data flow requested
	switch (adaptor->motionVideo){
		case 0://Motion Data Stream (packets arranged into columns)
			imHeight = adaptor->PacketsPerFrame; //Default 200
			imWidth = 12; //Basic 4kHz motion & Status Stream is 12 bytes per sample
			break;
		case 1://Video stream
			imWidth = 60; //Two 30x30 cameras
			imHeight = 30;
			break;
	}
	//imaqkit::adaptorWarn("TreadmillAdaptor:debug","imHeight = %d, imWidth = %d",imHeight,imWidth);
	rBuffer = new unsigned char[imWidth*imHeight];  //Allocate memory for reading
	

	//Configure and Connect to Treadmill serial interface
	ftStatus |= FT_Open(adaptor->devID,&ftHandle);
	ftStatus |= FT_ResetDevice(ftHandle);
	ftStatus |= FT_SetTimeouts(ftHandle,2000,2000);
	ftStatus |= FT_SetDataCharacteristics(ftHandle,FT_BITS_8,FT_STOP_BITS_1,FT_PARITY_NONE);
	ftStatus |= FT_SetFlowControl(ftHandle,FT_FLOW_NONE,NULL,NULL);
	ftStatus |= FT_SetBaudRate(ftHandle,1250000);  //1.25MBaud Communication rate
	if (ftStatus!=FT_OK) imaqkit::adaptorWarn("TreadmillAdaptor:debug","error during port configuration");

	//Stop any existing data stream from the treadmill
	wBuffer[0]=254;
	wBuffer[1]=0;
	FT_Write(ftHandle,wBuffer,2,&txBytes);
	Sleep(20);

	//Set to x/y outputs
	wBuffer[0]=246;
	wBuffer[1]=1;
	FT_Write(ftHandle,wBuffer,2,&txBytes);
	Sleep(20);

	FT_Purge(ftHandle,FT_PURGE_RX|FT_PURGE_TX);


	// Set the thread priority. 
    //SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_TIME_CRITICAL);
	//imaqkit::adaptorWarn("TreadmillAdaptor:debug","Entry to Acquisition Thread - width %i height %i",imWidth,imHeight);

	//Will spin in this while loop until a trigger is issued to start acquisition (immediate or manual)
	//Will respond to thread messages sent via "PostThreadMessage" function
	// Quit message, 0, will terminate this loop
	while(GetMessage(&msg,NULL,0,0) > 0){
		switch(msg.message){
			case WM_USER:
				//The Frame Acquisition Loop code goes here.
				//imaqkit::adaptorWarn("TreadmillAdaptor:debug","In Acquisition Case for WM_USER");
				
				//Start the Camera Object
				FT_Purge(ftHandle,FT_PURGE_RX|FT_PURGE_TX);
				switch (adaptor->motionVideo){
					case 0: //Motion Data @ High Speed
						wBuffer[0]=255;
						wBuffer[1]=0;
						break;
					case 1: //Video Data @ 20Hz
						wBuffer[0]=251;
						wBuffer[1]=0;
						break;
				}
				FT_Write(ftHandle,wBuffer,2,&txBytes);
				
				//Check if a frame needs to be acquired
				while(adaptor->isAcquisitionNotComplete() & adaptor->capflag){

					
					//imaqkit::adaptorWarn("TreadmillAdaptor:debug","In Acquisition Loop");
					//Insert code to pull frame into a buffer (rBuffer)
					FT_Read(ftHandle,rBuffer,imWidth*imHeight,&rxBytes);
					
					if (rxBytes != imWidth*imHeight) {
						imaqkit::adaptorWarn("TreadmillAdaptor:debug","ERROR: %d bytes Acquired, expected %d, motionVideo %d",rxBytes,imWidth*imHeight,adaptor->motionVideo);	
						break;
					}
					
					//At this point, you have the data in "rBuffer" variable.
					//High speed closed loop algorithms can be implemented here
					//Reducing the adaptor's "PacketsPerFrame" property will increase the speed at which this loop executes
					//	Default is 200 packetsPerFrame, this corresponds to 20Hz at 4kHz motion capture rate (good for video update for a user).
					//One could even bypass matlab all together and just close the loop here and skip the "sendframe" 
					//	to decrease loop time by elliminating data transfer, but those savings may be minimal
					//Also, ThreadPriority function call above may be uncommented and Matlab may be increased to real-time priority
					//	to increase responsivity (or this can all be exported to a stand-alone console program for super fast closed loop)


					if (adaptor->isSendFrame()){
						//get frame type & dimensions
						imaqkit::frametypes::FRAMETYPE frameType = adaptor->getFrameType();
						imaqkit::IAdaptorFrame* frame = adaptor->getEngine()->makeFrame(frameType,imWidth,imHeight); //Get a frame object
						
						frame->setImage(rBuffer,imWidth,imHeight,0,0); //Copy data from buffer into frame object
						frame->setTime(imaqkit::getCurrentTime()); //Set image's timestamp
						adaptor->getEngine()->receiveFrame(frame); //Send frame object to engine.

					} //isSendFrame

					//Increment the frame count
					adaptor->incrementFrameCount();
				}//While Frame Acq Loop
				
				//Stop Acquisition
				wBuffer[0]=254;
				wBuffer[1]=0;
				FT_Write(ftHandle,wBuffer,2,&txBytes);
				Sleep(20);

				FT_Purge(ftHandle,FT_PURGE_RX);

				break;
		}//While message is not WM_QUIT = 0
	}
	

	FT_Close(ftHandle);
	//imaqkit::adaptorWarn("TreadmillAdaptor:debug","Closed Device");
	delete rBuffer;

	//imaqkit::adaptorWarn("TreadmillAdaptor:debug","Leaving Acquisition Thread");
	return 0;
}
