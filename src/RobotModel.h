/*
 * RobotModel.h
 *
 *  Created on: 20.11.2022
 *      Author: manuel
 */

#ifndef ROBOTMODEL_H_
#define ROBOTMODEL_H_

#include "collider.h"

class RobotModel {
public:
	RobotModel();
	virtual ~RobotModel();
	static void initialize_colliders_of_robot(collider* colliders);
};



#endif /* ROBOTMODEL_H_ */
