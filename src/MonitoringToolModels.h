/*
 * MonitoringToolModels.h
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#ifndef MONITORINGTOOLMODELS_H_
#define MONITORINGTOOLMODELS_H_

#include <iostream>
#include "collider.h"
#include "GlobalVariables.h"
#include "InverseKinematic.h"
#include "Constants.h"
#include "CollisionDetection.h"
//#include "HelperFunctions.h"

class MonitoringToolModels {
private:
	static MonitoringToolModels *mt;

	int visiblePatternsMT1[objectsWithPatternsCount], visiblePatternsMT2[objectsWithPatternsCount];

public:
	MonitoringToolModels();
	virtual ~MonitoringToolModels();

	static MonitoringToolModels* get_instance();

	static void init_safetyCollider4MT1(collider* safetyCollidersForMTs);
	static void init_safetyCollider4MT2(collider* safetyCollidersForMTs);
	static void init_monitoringTool2(collider* colliders);
	static void init_monitoringTool1(collider* colliders);

	void updateMonitoredSpaceJustPlot();
	void updateMonitoredSpace(bool init, bool plot);

	bool checkForFreeSight(float startX, float startY, float startZ, float endX, float endY, float endZ, int obstacleIndex, int mtIndex, float* camVectors);
	int isObjectVisible(int colIndex, int mtIndex);
	bool isPatternVisible(int patternIndex, int mtIndex, bool detectionMode);
//	void updateMonitoredSpace(bool init, bool plot, collider* colliders, float* camVectorsMT1, float* camVectorsMT2);

};

#endif /* MONITORINGTOOLMODELS_H_ */
