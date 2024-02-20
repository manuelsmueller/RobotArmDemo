/*
 * MiscModels.h
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#ifndef MISCMODELS_H_
#define MISCMODELS_H_

#include "collider.h"
#include <cmath>

class MiscModels {
public:
	MiscModels();
	virtual ~MiscModels();
	static void init_safetyCollider(collider* safetyCollider);
	static void init_workpiceMesh(collider* workpieceMesh);
	static void calibrate_gripper(float *objectHeights, float* objectOffsets,
			float* objectGripDepths, float  gripperInstallationOffset,
			int transportableObjectsCount);
	static void init_light(float &lengthLight, float* lightDir );
};

#endif /* MISCMODELS_H_ */
