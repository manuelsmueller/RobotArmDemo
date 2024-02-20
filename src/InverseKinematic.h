/*
 * InverseKinematic.h
 *
 *  Created on: 26.11.2022
 *      Author: manuel
 */

#ifndef INVERSEKINEMATIC_H_
#define INVERSEKINEMATIC_H_

#include <cmath>
#include "collider.h"

class InverseKinematic {

private:
	InverseKinematic();
	static InverseKinematic *ik;
public:
	static InverseKinematic* get_instance();
	virtual ~InverseKinematic();


	// constants that define the lengths and size properties of the relevant arm segments and joints:
	const float lastSegmentMid = 4.05f, lastSegmentLen = lastSegmentMid + 3.632f, gripperBaseLen = 9.7f;  // 4.332f, 7.75f
	const float len0 = 14.7f + 4.6f, len1 = 3.3f, len2 = 15.5f, len3 = 13.5f, len4 = lastSegmentLen + gripperBaseLen;

	// constants that define the movement ranges of the 5 robot arm joints in degrees:
	const float qLowerLimits[5] = { -160.0f, -64.0f, -141.5f, -101.0f, -165.2f + 3.5f };
	const float qUpperLimits[5] = { 160.0f, 84.9f, 144.5f, 93.5f, 150.5f + 3.5f };

	const float stepSize = 2.0f;

	// necessary variables for calculating the inverse kinematics:
	float innerVector[2], offset1[2], offset2[2], offset3[2], offset4[2], innerRadius, innerAngle, innerRadius2, innerAngle2, rad_ik;

	float pointRadius, x_diff, y_diff, pointAngle, x_in, y_in, x_mid, y_mid, x_joint, y_joint;
	bool inside[3], outside[5];

	float jointDistance;

	float* q_ik = new float[5]; // array for storing the calculated joint angles, that describe a robot pose clearly

	float circleCentersX[4], circleCentersY[4], circleRadius[4], circleRangesLower[4], circleRangesUpper[4];

	float intersectAngles[4], angleCircle1, angleCircle2, angleDiff1, angleDiff2, startAngle, endAngle, upperBorder, lowerBorder;
	int intersectCounter;
	bool firstAdded, firstIteration;

	//-------------------------------------
	// function headers
	//-------------------------------------
	void initInverseKin();
	bool isThirdJointValid(float x, float y);
	bool isSecondJointValid(float x, float y, bool includeThird, float theta);
	bool doesThetaExist(float xCoord, float yCoord, float zCoord);
	void writeQValues(float* qVal);

	//-------------------------------------
	// inline functions
	//-------------------------------------

	// increaseSize() increases the size of a pointer array [oldList] with the length [oldSize] by the value specified in [diff] and returns the new pointer:
	template<class Type>
	inline Type* increaseSize(Type* oldList, int oldSize, int diff) {
	    Type* tmp = new Type[oldSize + diff];
	    for (int i = 0; i < oldSize; i++) {
	        tmp[i] = oldList[i];
	    }
	    delete[] oldList;
	    return tmp;
	}

	// cutOff() cuts down a pointer array [oldList] with the length [oldSize] to the length [newSize] by removing the last elements:
	template<class Type>
	inline Type* cutOff(Type* oldList, int oldSize, int newSize) {
	    if (newSize < oldSize) {
	        Type* tmp = new Type[newSize];
	        for (int i = 0; i < newSize; i++) {
	            tmp[i] = oldList[i];
	        }
	        delete[] oldList;
	        return tmp;
	    }
	    else {
	        return oldList;
	    }
	}

	// insertAtBegin() increases the size of an array [list] by one, shifting all elements by one to the end, leaving the first
	// index free for a new element:
	template<class Type>
	inline Type* insertAtBegin(Type* list, int size) {
	    Type* tmp = new Type[size + 1];
	    for (int i = 1; i <= size; i++) {
	        tmp[i] = list[i - 1];
	    }
	    delete[] list;
	    return tmp;
	}

	// sortArray() sorts an array [list] with length [size] and returns the sorted list, starting with the biggest value:
	template<class Type>
	inline Type* sortArray(Type* list, int size) {
	    Type* tmp = new Type[size];
	    bool* proc = new bool[size];
	    for (int i = 0; i < size; i++) {
	        proc[i] = false;
	    }

	    float maxVal;
	    int maxIndex;
	    for (int i = 0; i < size; i++) {
	        maxVal = -1000.0f;
	        maxIndex = -1;
	        for (int j = 0; j < size; j++) {
	            if (list[j] > maxVal && proc[j] == false) {
	                maxVal = list[j];
	                maxIndex = j;
	            }
	        }
	        tmp[i] = maxVal;
	        proc[maxIndex] = true;
	    }
	    delete[] list;
	    delete[] proc;
	    return tmp;
	}

};

#endif /* INVERSEKINEMATIC_H_ */
