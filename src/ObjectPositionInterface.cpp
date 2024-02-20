/*
 * ObjectPositionInterface.cpp
 *
 *  Created on: 07.01.2023
 *      Author: manuel
 */

#include "ObjectPositionInterface.h"

ObjectPositionInterface::ObjectPositionInterface() {
	index=0;
	x=-21.0f;
	y=21.0f;
	z=0;
	angle=0;
}

ObjectPositionInterface::ObjectPositionInterface(int index, float x, float y, float z, float angle) {
	init( index,  x,  y,  z,  angle);
}

/*
 * initilizes index,x,y,z,angle.
 */
void ObjectPositionInterface::init(int index, float x, float y, float z, float angle){
	this->index=index;
	this->x=x;
	this->y=y;
	this->z=z;
	this->angle=angle;
}

ObjectPositionInterface::~ObjectPositionInterface() {
	// TODO Auto-generated destructor stub
}

void ObjectPositionInterface::print(){
	std::cout<<"obj pos = ("<<index<<"," <<x<<","<<y<<","<<z<<","<<angle<<")"<<std::endl;
}


ObjectPositionInterface ObjectPositionInterface::convert_from_edge(int index, collider::Edge e, float angle){
	ObjectPositionInterface opi;
	opi.init(index, e.x, e.y, e.z, angle);

	return opi;
}
