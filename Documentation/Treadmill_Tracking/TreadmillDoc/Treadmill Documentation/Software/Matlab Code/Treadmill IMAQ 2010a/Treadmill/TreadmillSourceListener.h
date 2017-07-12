/* 	
	Author: Gus K Lott III, PhD
	May 10, 2010
	
	Neurobiological Instrumentation Engineer
	Howard Hughes Medical Institute - Janelia Farm Research Campus
	lottg@janelia.hhmi.org
*/
#ifndef __TREADMILL_SOURCE_LISTENER_HEADER__
#define __TREADMILL_SOURCE_LISTENER_HEADER__

#include "mwadaptorimaq.h"  
#include "TreadmillAdaptor.h"


class TreadmillSourceListener : public imaqkit::IPropPostSetListener
{
public:
	TreadmillSourceListener(TreadmillAdaptor* parent) : _parent(parent) {};
	virtual ~TreadmillSourceListener(void){};
	virtual void notify(imaqkit::IPropInfo* propertyInfo, void* newValue);

private:
	virtual void applyValue(void);
	TreadmillAdaptor* _parent;
	int _source;

};

#endif