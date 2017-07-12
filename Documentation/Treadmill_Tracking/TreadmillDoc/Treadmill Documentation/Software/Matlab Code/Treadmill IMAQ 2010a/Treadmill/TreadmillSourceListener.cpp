/*
	Author: Gus K Lott III, PhD
	May 10, 2010
	
	Neurobiological Instrumentation Engineer
	Howard Hughes Medical Institute - Janelia Farm Research Campus
	lottg@janelia.hhmi.org

*/
#include "TreadmillSourceListener.h"

void TreadmillSourceListener::notify(imaqkit::IPropInfo *propertyInfo, void *newValue) {

	if (newValue){
		_source = *static_cast<const int*>(newValue);

		if (_parent->isOpen()) {
			applyValue();
		}
	}
	

}

void TreadmillSourceListener::applyValue(void){

	bool wasAcquiring = _parent->isAcquiring();
	if(wasAcquiring){
		_parent->stop();
	}

	if(wasAcquiring) {
		_parent->restart();
	}

}