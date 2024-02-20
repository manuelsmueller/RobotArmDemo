/*
 * MiscModels.cpp
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#include "MiscModels.h"

MiscModels::MiscModels() {

}

MiscModels::~MiscModels() {
}

/*
 *
 */
void MiscModels::init_safetyCollider(collider* safetyCollider) {
	safetyCollider->points_stat = 9;
	safetyCollider->x_stat = new float[safetyCollider->points_stat] { 0, 1.0f,
			1.0f, -1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f };
	safetyCollider->y_stat = new float[safetyCollider->points_stat] { 0, 1.0f,
			-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f };
	safetyCollider->z_stat = new float[safetyCollider->points_stat] { 0, -1.0f,
			-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
}





void MiscModels::calibrate_gripper(float *objectHeights, float* objectOffsets,
		float* objectGripDepths, float  gripperInstallationOffset,
		int transportableObjectsCount) {
	objectHeights[0] = 1.5f;//Workpiece ... values in cm
	objectHeights[1] = 1.5f;
	objectHeights[2] = 1.5f;
	objectHeights[3] = 1.5f;//Workpiece
	objectHeights[4] = 3.5f;//Monitoring Tools
	objectHeights[5] = 3.5f;//Monitoring Tools
	objectOffsets[0] = 1.5f;
	objectOffsets[1] = 1.5f;
	objectOffsets[2] = 1.5f;
	objectOffsets[3] = 1.5f;
	objectOffsets[4] = 3.5f;
	objectOffsets[5] = 3.5f;
	objectGripDepths[0] = 2.5f;
	objectGripDepths[1] = 2.5f;
	objectGripDepths[2] = 2.5f;
	objectGripDepths[3] = 2.5f;
	objectGripDepths[4] = 1.6f;
	objectGripDepths[5] = 1.6f;
	for (int i = 0; i < transportableObjectsCount; i++) {
		objectGripDepths[i] += gripperInstallationOffset;
	}
}

void MiscModels::init_light(float &lengthLight, float* lightDir ) {
	// setup the light for the scene
	lengthLight = sqrt(
			lightDir[0] * lightDir[0] + lightDir[1] * lightDir[1]
					+ lightDir[2] * lightDir[2]);
	lightDir[0] = -lightDir[0] / lengthLight;
	lightDir[1] = -lightDir[1] / lengthLight;
	lightDir[2] = -lightDir[2] / lengthLight;
}


