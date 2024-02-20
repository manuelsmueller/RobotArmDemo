/*
 * SimInit.h
 *
 *  Created on: 24.12.2022
 *      Author: manuel
 */

#ifndef SIMINIT_H_
#define SIMINIT_H_

#include <iostream>
#include <random>
#include <string>

#include "InverseKinematic.h"
#include "GlobalVariables.h"
#include "PathPlanner.h"
#include "ConvHull3d.h"
#include "FrequentOps.h"
#include "CollisionDetection.h"

#include "collider.h"
#include "ObjectPositionInterface.h"

class SimInit {
	private:
	static SimInit* si;

//	collider::Edge* ObjectPositions;
//	float * ObjectAngle;
	ObjectPositionInterface* obj;

	SimInit();
	void position_object_after_transport(GlobalVariables *gv, HelperFunctions *hf, InverseKinematic *ik,
			CollisionDetection *cd);
	void initially_place_object(GlobalVariables *gv, CollisionDetection *cd, float x, float y);

	void block4(CollisionDetection *cd, GlobalVariables *gv, HelperFunctions *hf);
	void block5(GlobalVariables *gv, HelperFunctions *hf, CollisionDetection *cd);
	void block6(GlobalVariables *gv);

	void reposition_workpiece_to_new_position(default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	void apply_workpiece_position(int index);
	void apply_random_workpiece_position();

	void print_calc_path_input(float* fStart, float* fEnd, collider::Edge eStart,collider::Edge eEnd) {
		std::cout << "calcPath: [";
		for (int i = 0; i < 5; i++) {
			std::cout << fStart[i] << ",";
		}
		std::cout << "],[";
		for (int i = 0; i < 5; i++) {
			std::cout << fEnd[i] << ",";
		}
		std::cout << "], start(" << eStart.x << ","
				<< eStart.y << ")," << "end(" << eEnd.x
				<< "," << eEnd.y << ")" << std::endl;
	}

public:
	static SimInit* get_instance();
//	SimInit();
	virtual ~SimInit();

	ObjectPositionInterface* get_workpiecePositions();

	void sim_control_simple_add_workpiece_on_init(
			bool &repaint,bool boWrite, bool boWaitForSpace,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	void set_workpiece_position(int index, float x, float y, float z, float angle);
	void set_workpiece_position(ObjectPositionInterface obj);

};

#endif /* SIMINIT_H_ */
