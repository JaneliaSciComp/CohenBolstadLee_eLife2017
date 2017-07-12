/* Property Listener Header (called when properties are set)

	Author: Gus K Lott III, PhD
	May 10, 2010
	
	Neurobiological Instrumentation Engineer
	Howard Hughes Medical Institute - Janelia Farm Research Campus
	lottg@janelia.hhmi.org
*/


#ifndef __TREADMILL_PROP_LISTENER_HEADER__
#define __TREADMILL_PROP_LISTENER_HEADER__

#include "mwadaptorimaq.h"  
#include "TreadmillAdaptor.h"


class TreadmillPropListener : public imaqkit::IPropPostSetListener{
public:
	TreadmillPropListener(TreadmillAdaptor* parent): _parent(parent) {};
    virtual ~TreadmillPropListener() {};
	virtual void notify(imaqkit::IPropInfo* propertyInfo, void* newValue);

private:
	//virtual void applyValue(void);
	TreadmillAdaptor* _parent;
	imaqkit::IPropInfo* _propInfo;
	int _lastIntValue;
	double _lastDoubleValue;
	char * _lastStrValue;
	int * _lastIntArrayValue;
};

#endif