/* TreadmillDemo.cpp 

	This Console Application is a Demonstration tool for Connecting 
	to a Lott/Jayaraman Treadmill Device via the FTDI D2xx API.

	This application allows the user to specify, manually, a device 
	ID (if there are more than one).  and then streams and parses 
	stream data.

	Written by Gus K. Lott III, PhD
	May 18, 2010
	lottg@janelia.hhmi.org

*/

//Comment this code and add display of accumulated x/y motion

#include "stdafx.h"
#include <windows.h>
#include "ftd2xx.h"



int main(int argc, char* argv[])
{
	//Camera Data Bins
	int dx[2],dy[2],squal[2];
	float shutter[2];

	//FTDI Variables
	FT_STATUS ftStatus = FT_OK;
	FT_HANDLE ftHandle;
	unsigned char wBuffer[1024];
	unsigned char rBuffer[2400];
	DWORD txBytes = 0;
	DWORD rxBytes;
	DWORD numDevs, devID; 
	

	//Detect the number of FTDI devices connected
	// You may have multiple devices using FTDI interfaces
	//  This is commonly the case with RS232-USB adaptors
	//  In this case, you'll have to determine who's who by trial and error
	FT_CreateDeviceInfoList(&numDevs);
	//Case of no devices plugged in
	if (numDevs == 0){
		printf("No FTDI Devices Found");
		Sleep(2000);
		return 0;
	}
	//if only one FTDI device found, assume it's a treadmill
	if (numDevs == 1){
		printf("1 Device Detected, Conntecting to Dev:0");
		devID = 0;
	}else{ //Allow user to specify which device ID to use
		printf("%d Devices Detected, Enter device ID (0-%d): ",numDevs,numDevs-1);
		scanf("%d",&devID);
		printf("\n\nConntecting to Dev:%d",devID);
	}
	printf("\n\n");
	Sleep(1000);

	//Configure and Connect to Treadmill serial interface
	ftStatus |= FT_Open(devID,&ftHandle);
	ftStatus |= FT_ResetDevice(ftHandle);
	ftStatus |= FT_SetTimeouts(ftHandle,2000,2000);
	ftStatus |= FT_SetDataCharacteristics(ftHandle,FT_BITS_8,FT_STOP_BITS_1,FT_PARITY_NONE);
	ftStatus |= FT_SetFlowControl(ftHandle,FT_FLOW_NONE,NULL,NULL);
	ftStatus |= FT_SetBaudRate(ftHandle,1250000);  //1.25MBaud Communication rate
	if (ftStatus!=FT_OK) { printf("Error connecting to FTDI interface\n"); Sleep(1000); return 0; }

	//Stop any existing data stream from the treadmill
	wBuffer[0]=254;
	wBuffer[1]=0;
	FT_Write(ftHandle,wBuffer,2,&txBytes);
	Sleep(20);

	FT_Purge(ftHandle,FT_PURGE_RX|FT_PURGE_TX);

	//Start Motion Data @ High Speed (4kHz)
	wBuffer[0]=255;
	wBuffer[1]=0;
	FT_Write(ftHandle,wBuffer,2,&txBytes);

	
	//poll device at 20Hz until canceled by user (ctrl+c)
	while(1){
		//Read 200 packets of data (12 bytes per packet)
		FT_Read(ftHandle,rBuffer,200*12,&rxBytes);

		if (rxBytes!=2400){
			printf("Bad Read\n");
			Sleep(1);
			continue;
		}

		//Accumulate Motion Data for this 20Hz chunk
		dx[0]=0; dx[1]=0; dy[0]=0; dy[1]=0;
		for (int i = 0; i<2400; i+=12){
			dx[0] += ((int)rBuffer[i+2])-128;
			dy[0] += ((int)rBuffer[i+3])-128;
			dx[1] += ((int)rBuffer[i+4])-128;
			dy[1] += ((int)rBuffer[i+5])-128;
		}
		squal[0] = (int)rBuffer[6];
		squal[1] = (int)rBuffer[7];
		shutter[0] = (((float)(rBuffer[8]-1))*256.0 + (float)rBuffer[9])/24.0;
		shutter[1] = (((float)(rBuffer[10]-1))*256.0 + (float)rBuffer[11])/24.0;

		printf("%d bytes received\n",rxBytes);
		printf("\tdx0 = %+3d",dx[0]);
		printf("\tdy0 = %+3d\n",dy[0]);
		printf("\tdx1 = %+3d",dx[1]);
		printf("\tdy1 = %+3d\n",dy[1]);
		printf("\tsqual0 = %d",squal[0]);
		printf("\tsqual1 = %d\n",squal[1]);
		printf("\tShutter0 = %.2fus",shutter[0]);
		printf("\tShutter1 = %.2fus\n",shutter[1]);
		Sleep(1);
	}

	//Stop Acquisition
	wBuffer[0]=254;
	wBuffer[1]=0;
	FT_Write(ftHandle,wBuffer,2,&txBytes);

	//Close serial interface
	FT_Close(ftHandle);

	return 0;
}

