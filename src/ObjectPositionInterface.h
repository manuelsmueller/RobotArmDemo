/*
 * ObjectPositionInterface.h
 *
 *  Created on: 07.01.2023
 *      Author: manuel
 */

#ifndef OBJECTPOSITIONINTERFACE_H_
#define OBJECTPOSITIONINTERFACE_H_

#include <iostream>
#include "collider.h"

class ObjectPositionInterface {

public:
	static ObjectPositionInterface convert_from_edge(int index, collider::Edge e, float angle);

	ObjectPositionInterface();
	ObjectPositionInterface(int index, float x, float y, float z, float angle);
	virtual ~ObjectPositionInterface();

	void print();

	void init(int index, float x, float y, float z, float angle);

	int index;
	float x;
	float y;
	float z;
	float angle;
};

#endif /* OBJECTPOSITIONINTERFACE_H_ */
