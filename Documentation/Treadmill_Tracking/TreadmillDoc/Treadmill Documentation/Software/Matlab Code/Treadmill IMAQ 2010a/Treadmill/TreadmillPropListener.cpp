/* 
	Notify function is called when the user changes a property using the "set" function in Matlab

	Author: Gus K Lott III, PhD
	May 10, 2010
	
	Neurobiological Instrumentation Engineer
	Howard Hughes Medical Institute - Janelia Farm Research Campus
	lottg@janelia.hhmi.org

*/
#include "TreadmillPropListener.h"


void TreadmillPropListener::notify(imaqkit::IPropInfo* propertyInfo, void* newValue){

	if (newValue) {
		_propInfo = propertyInfo;

		switch(_propInfo->getPropertyStorageType()){
			case imaqkit::propertytypes::DOUBLE:
				_lastDoubleValue = *reinterpret_cast<double*>(newValue);
				break;
			case imaqkit::propertytypes::INT:
                _lastIntValue = *reinterpret_cast<int*>(newValue);
                break;
			case imaqkit::propertytypes::STRING:
                _lastStrValue = reinterpret_cast<char*>(newValue);
                break;
			case imaqkit::propertytypes::INT_ARRAY:
                _lastIntArrayValue = reinterpret_cast<int*>(newValue);
		}
	}

	const char * propName = propertyInfo->getPropertyName();
	int propID = 0;

	//Determine the name of the property
	if (!strcmp("motionVideo",propName)) propID = 1;
	if (!strcmp("PacketsPerFrame",propName)) propID = 2;
		
	//Feed property into adaptor object
	switch (propID){
		case 1://motionVideo
			_parent->motionVideo = _lastIntValue!=0;
			break;
		case 2:
			_parent->PacketsPerFrame = _lastIntValue;
			break;
	}


	//imaqkit::adaptorWarn("TreadmillPropListener:debug","In Property listener. PropertyID: %s\n",propertyInfo->getPropertyName());
}
