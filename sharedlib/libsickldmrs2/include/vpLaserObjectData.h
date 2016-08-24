/*
 * vpLaserObjectData.h
 *
 *  Created on: 18 de ago de 2016
 *      Author: luan
 */

#ifndef VPLASEROBJECTDATA_H_
#define VPLASEROBJECTDATA_H_

#include "vpObject.h"

class vpLaserObjectData
{
public:

	/* construtor default */
	vpLaserObjectData(){
		startTimestamp = 0;
		numObjects = 0;
	}

	/* construtor de cópia*/
	vpLaserObjectData(const vpLaserObjectData &objectData) {
		startTimestamp = objectData.startTimestamp;
		numObjects = objectData.numObjects;
		objectList = objectData.objectList;
	}

	/* destrutor */
	virtual ~vpLaserObjectData() {};

	inline void addObject(const vpObject o){
		objectList.push_back(o);
	}

	inline void clearObjectList() {
		objectList.clear();
	}

	inline std::vector<vpObject> getObjectList(){
		return objectList;
	}

	inline unsigned short getNumObjects() const {
		return numObjects;
	}

	inline void setNumObjects(unsigned short numObjects) {
		this->numObjects = numObjects;
	}

	inline double getStartTimestamp() const {
		return startTimestamp;
	}

	inline void setStartTimestamp(double startTimestamp) {
		this->startTimestamp = startTimestamp;
	}

private:
	double startTimestamp;
	unsigned short numObjects;
	std::vector<vpObject> objectList;
};





#endif /* SHAREDLIB_LIBSICKLDMRS2_INCLUDE_VPLASEROBJECTDATA_H_ */
