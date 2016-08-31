/*
 * vpObject.h
 *
 *  Created on: 18 de ago de 2016
 *      Author: luan
 */

#ifndef SHAREDLIB_LIBSICKLDMRS2_INCLUDE_VPOBJECT_H_
#define SHAREDLIB_LIBSICKLDMRS2_INCLUDE_VPOBJECT_H_

#include <vector>

typedef struct _point_2d {
	short x_pos;
	short y_pos;
} point_2d;

typedef struct _size_2d {
	unsigned short int x_size;
	unsigned short int y_size;
} size_2d;

class vpObject
{
public:

	/* construtor */
	vpObject(){
		objectID = 0;
		objectAge = 0;
		objectPredictionAge = 0;
		relativeTimestamp = 0;
//		referencePoint = 0;
//		referencePointSigma = 0;
//		closestPoint = 0;
//		boundingBoxCenter = 0;
//		boundingBoxSize = 0;
//		objectBoxCenter = 0;
//		objectBoxSize = 0;
		objectBoxOrientation = 0;
//		absoluteVelocity = 0;
//		absoluteVelocitySigma = 0;
//		relativeVelocity = 0;
		numContourPoints = 0;

	}

	/* construtor de cópia */
	vpObject(const vpObject &object){
		objectID = object.objectID;
		objectAge = object.objectAge;
		objectPredictionAge = object.objectPredictionAge;
		relativeTimestamp = object.relativeTimestamp;
		referencePoint = object.referencePoint;
		referencePointSigma = object.referencePointSigma;
		closestPoint = object.closestPoint;
		boundingBoxCenter = object.boundingBoxCenter;
		boundingBoxSize = object.boundingBoxSize;
		objectBoxCenter = object.objectBoxCenter;
		objectBoxSize = object.objectBoxSize;
		objectBoxOrientation = object.objectBoxOrientation;
		absoluteVelocity = object.absoluteVelocity;
		absoluteVelocitySigma = object.absoluteVelocitySigma;
		relativeVelocity = object.relativeVelocity;
		numContourPoints = object.numContourPoints;
		listContourPoints = object.listContourPoints;
	}

	/* destrutor */
	virtual ~vpObject(){};

	inline void addContourPoint(point_2d p){
		listContourPoints.push_back(p);
	}

	inline void clearListContourPoints() {
		listContourPoints.clear();
	}

	inline std::vector<point_2d> getListContourPoints(){
		return listContourPoints;
	}

	inline point_2d getAbsoluteVelocity() const {
		return absoluteVelocity;
	}

	inline void setAbsoluteVelocity(point_2d absoluteVelocity) {
		this->absoluteVelocity = absoluteVelocity;
	}

	inline size_2d getAbsoluteVelocitySigma() const {
		return absoluteVelocitySigma;
	}

	void setAbsoluteVelocitySigma(size_2d absoluteVelocitySigma) {
		this->absoluteVelocitySigma = absoluteVelocitySigma;
	}

	inline point_2d getBoundingBoxCenter() const {
		return boundingBoxCenter;
	}

	inline void setBoundingBoxCenter(point_2d boundingBoxCenter) {
		this->boundingBoxCenter = boundingBoxCenter;
	}

	inline size_2d getBoundingBoxSize() const {
		return boundingBoxSize;
	}

	inline void setBoundingBoxSize(size_2d boundingBoxSize) {
		this->boundingBoxSize = boundingBoxSize;
	}

	inline point_2d getClosestPoint() const {
		return closestPoint;
	}

	inline void setClosestPoint(point_2d closestPoint) {
		this->closestPoint = closestPoint;
	}

	inline unsigned short int getNumContourPoints() const {
		return numContourPoints;
	}

	inline void setNumContourPoints(unsigned short int numContourPoints) {
		this->numContourPoints = numContourPoints;
	}

	inline unsigned short int getObjectAge() const {
		return objectAge;
	}

	inline void setObjectAge(unsigned short int objectAge) {
		this->objectAge = objectAge;
	}

	inline point_2d getObjectBoxCenter() const {
		return objectBoxCenter;
	}

	inline void setObjectBoxCenter(point_2d objectBoxCenter) {
		this->objectBoxCenter = objectBoxCenter;
	}

	inline short getObjectBoxOrientation() const {
		return objectBoxOrientation;
	}

	inline void setObjectBoxOrientation(short objectBoxOrientation) {
		this->objectBoxOrientation = objectBoxOrientation;
	}

	inline size_2d getObjectBoxSize() const {
		return objectBoxSize;
	}

	inline void setObjectBoxSize(size_2d objectBoxSize) {
		this->objectBoxSize = objectBoxSize;
	}

	inline unsigned short int getObjectId() const {
		return objectID;
	}

	inline void setObjectId(unsigned short int objectId) {
		objectID = objectId;
	}

	inline unsigned short int getObjectPredictionAge() const {
		return objectPredictionAge;
	}

	inline void setObjectPredictionAge(unsigned short int objectPredictionAge) {
		this->objectPredictionAge = objectPredictionAge;
	}

	inline point_2d getReferencePoint() const {
		return referencePoint;
	}

	inline void setReferencePoint(point_2d referencePoint) {
		this->referencePoint = referencePoint;
	}

	inline point_2d getReferencePointSigma() const {
		return referencePointSigma;
	}

	inline void setReferencePointSigma(point_2d referencePointSigma) {
		this->referencePointSigma = referencePointSigma;
	}

	inline unsigned short int getRelativeTimestamp() const {
		return relativeTimestamp;
	}

	inline void setRelativeTimestamp(unsigned short int relativeTimestamp) {
		this->relativeTimestamp = relativeTimestamp;
	}

	inline point_2d getRelativeVelocity() const {
		return relativeVelocity;
	}

	inline void setRelativeVelocity(point_2d relativeVelocity) {
		this->relativeVelocity = relativeVelocity;
	}

	inline unsigned short int getClassification() const {
		return classification;
	}

	inline void setClassification(unsigned short int classification) {
		this->classification = classification;
	}

private:
	unsigned short int objectID;
	unsigned short int objectAge;
	unsigned short int objectPredictionAge;
	unsigned short int relativeTimestamp;
	point_2d referencePoint;
	point_2d referencePointSigma;
	point_2d closestPoint;
	point_2d boundingBoxCenter;
	size_2d boundingBoxSize;
	point_2d objectBoxCenter;
	size_2d objectBoxSize;
	short objectBoxOrientation;
	point_2d absoluteVelocity;
	size_2d absoluteVelocitySigma;
	point_2d relativeVelocity;
	unsigned short int classification;
	unsigned short int numContourPoints;
	std::vector<point_2d> listContourPoints;
};




#endif /* SHAREDLIB_LIBSICKLDMRS2_INCLUDE_VPOBJECT_H_ */
