/*
 * collider.cpp
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#include "collider.h"

collider::collider() {
	// TODO Auto-generated constructor stub
	pattern_indices=0;
	points_stat=0;
	facesTimes3=0;
	patternCount=0;

	// convex structure!
	x_stat=0;
	y_stat=0;
	z_stat=0;

	x_coll=0;
	y_coll=0;
	z_coll=0;

	ignoredPoints=0;
	firstSurfacePoint = -1;

	connections=0;
	Edge color;

	x_pattern_stat=0;
	y_pattern_stat=0;
	z_pattern_stat=0;

	x_pattern_tf=0;
	y_pattern_tf=0;
	z_pattern_tf=0;



	offsets[3]={0};
	angles[3]={0};
	maxSize=0;
	limits[6]={0};
}

collider::~collider() {
	// TODO Auto-generated destructor stub
}



