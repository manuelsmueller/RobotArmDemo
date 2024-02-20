/*
 * Adversaries.h
 *
 *  Created on: 05.12.2022
 *      Author: manuel
 */

#ifndef ADVERSARIES_H_
#define ADVERSARIES_H_

#include "MonitoringToolModels.h"
#include "CollisionDetection.h"
#include "InverseKinematic.h"
#include "HelperFunctions.h"
#include "GlobalVariables.h"
#include "PathPlanner.h"
#include "serverML.h"

class Adversaries {
private:
	bool obstacleCollision[3];
	float obstacleDistsTemp[3], obstacleDistsMin[3];
	int minInterIndices[3], minSegmentIndices[3];
	float oldXPos, oldYPos, oldAngleVal;

	float*  qValuesNext = new float[5];
	float*  qValuesFollowingNext = new float[5];

	GlobalVariables *gv;
	PathPlanner *pp;


	static Adversaries* adv;
	Adversaries();

public:
	virtual ~Adversaries();
	static Adversaries* get_instance();

	void obs_adversary(bool firstCall, bool lastCall, int obsIndex, bool prognosis);
	void obs_preparation(int transportedWorkpiece);

	void path_adversary(bool firstCall, bool lastCall, int transportedWorkpiece, bool prognosis);

	void loc_adversary(float action_space_dist, float action_space_angle, int transportedWorkpiece, bool prognosis);

};

#endif /* ADVERSARIES_H_ */
