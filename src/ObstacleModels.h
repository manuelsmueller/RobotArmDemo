/*
 * Obstacles.h
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#ifndef OBSTACLEMODELS_H_
#define OBSTACLEMODELS_H_

#include "collider.h"

class ObstacleModels {
private:
//	static const float patternSize = 2.5f;
	 static constexpr float patternSize=2.5f;
public:
	ObstacleModels();
	virtual ~ObstacleModels();

	static void init_distortedObstacle3(collider* distortedObstacles);
	static void init_obstacle3(collider* colliders);
	static void init_distortedObstacle2(collider* distortedObstacles);
	static void init_obstacle2(collider* colliders);
	static void init_distoredObstacle1(collider * distortedObstacles);
	static void init_obstacle1(collider* colliders);
};

#endif /* OBSTACLEMODELS_H_ */
