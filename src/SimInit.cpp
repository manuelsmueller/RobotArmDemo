/*
 * SimInit.cpp
 *
 *  Created on: 24.12.2022
 *      Author: manuel
 */

#include "SimInit.h"

SimInit::SimInit() {
	GlobalVariables *gv=GlobalVariables::get_instance();
//	ObjectPositions = new collider::Edge[gv->transportableObjectsCount];
//	ObjectAngle = new float[gv->transportableObjectsCount];
	obj= new ObjectPositionInterface[gv->transportableObjectsCount];
}

SimInit::~SimInit() {
	delete [] obj;
}

/*
 * singelton pattern
 */
SimInit* SimInit::si = 0;
SimInit* SimInit::get_instance(){
	static bool isInit=false;

	if(!isInit){
		si  = new SimInit();
		isInit=true;
	}
	return si;
}

/*
 * this function place the workpieces and MTs initially so that the robot can pick it up and move it to
 * specified destination.
 *
 * Note: x, y must be in range of the robot!
 */
void SimInit::initially_place_object(GlobalVariables *gv, CollisionDetection *cd, float x, float y) {
	//select parameters of the object to transport
	gv->objectToTransport = gv->transportableObjects[gv->initPhase - 1];
	gv->objectHeight = gv->objectHeights[gv->initPhase - 1];
	gv->objectOffset = gv->objectOffsets[gv->initPhase - 1];
	gv->objectGripDepth = gv->objectGripDepths[gv->initPhase - 1];


	cd->recalculateCollider(&gv->colliders[gv->objectToTransport], x,
			y, gv->objectOffset, 0, 0, 0);

	gv->objectDimension.x = gv->objectDimensions[gv->initPhase - 1].x + 0.5f;
	gv->objectDimension.y = gv->objectDimensions[gv->initPhase - 1].y + 0.5f;
	gv->objectDimension.z = gv->objectDimensions[gv->initPhase - 1].z + 0.5f;
	cd->updateColliderSize(gv->safetyCollider, gv->objectDimension.x,
			gv->objectDimension.y, gv->objectDimension.z);

	gv->pathStartPos.x = gv->colliders[gv->objectToTransport].offsets[0];
	gv->pathStartPos.y = gv->colliders[gv->objectToTransport].offsets[1];
	gv->pathStartPos.z = gv->objectOffset + gv->objectHeight + gv->objectGripDepth;
	gv->startObjectAngle = gv->colliders[gv->objectToTransport].angles[0]* 180.0f / M_PI;
}

void SimInit::position_object_after_transport(GlobalVariables *gv, HelperFunctions *hf, InverseKinematic *ik,
		CollisionDetection *cd) {
	gv->tempCounter = gv->totalPlatePoints;
	for (int i = 0; i < gv->transportableObjects[gv->initPhase - 2]; i++) {
		gv->tempCounter += gv->colliders[i].facesTimes3;
	}
	hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[gv->transportableObjects[gv->initPhase - 2]],
			gv->tempCounter);
	// Workpieces
	if (gv->transportableObjects[gv->initPhase - 2] < 12) {
		gv->tempCounter = 0;
		for (int i = 8; i < gv->transportableObjects[gv->initPhase - 2]; i++) {
			gv->tempCounter += gv->colliders[i].patternCount * 4;
		}
		hf->calcEdgesForPatterns(gv->object_pattern_edges,
				&gv->colliders[gv->transportableObjects[gv->initPhase - 2]],
				gv->tempCounter);
		if (gv->transportableObjects[gv->initPhase - 2] == 8)
			hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 1);

		if (gv->transportableObjects[gv->initPhase - 2] == 9)
			hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 2);

		if (gv->transportableObjects[gv->initPhase - 2] == 10)
			hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 3);

		if (gv->transportableObjects[gv->initPhase - 2] == 11)
			hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 4);
	} // MT1
	else if (gv->transportableObjects[gv->initPhase - 2] == 15) {
		hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
	} // MT2 + finalization
	else if (gv->transportableObjects[gv->initPhase - 2] == 16) {
		hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
		gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
				gv->numPointsPlannedPath, 0);
		gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
				gv->numPointsPlannedPath, 0);
		gv->numPointsPlannedPath = 0;
		cd->recalculateCollider(gv->workpieceMesh, 0.0f, 0.0f, 10.0e5, 0, 0, 0);
		hf->calcEdgesForMesh(gv->workpiece_mesh_edges, gv->workpieceMesh, 0);
	}
}

void SimInit::block4(CollisionDetection *cd, GlobalVariables *gv, HelperFunctions *hf) {
	cd->overrideCollider(gv->workpieceMesh,
			&gv->colliders[gv->objectToTransport]);
	cd->recalculateCollider(gv->workpieceMesh, gv->pathEndPos.x,
			gv->pathEndPos.y, gv->objectOffset + 0.01f,
			gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f);
	hf->calcEdgesForMesh(gv->workpiece_mesh_edges, gv->workpieceMesh, 0);
	gv->tempCounter = gv->totalPlatePoints;
	for (int i = 0; i < gv->objectToTransport; i++) {
		gv->tempCounter += gv->colliders[i].facesTimes3;
	}
	hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[gv->objectToTransport], gv->tempCounter);
}

void SimInit::block5(GlobalVariables *gv, HelperFunctions *hf, CollisionDetection *cd) {
	gv->tempCounter = 0;
	for (int i = 8; i < gv->objectToTransport; i++) {
		gv->tempCounter += gv->colliders[i].patternCount * 4;
	}
	hf->calcEdgesForPatterns(gv->object_pattern_edges,
			&gv->colliders[gv->objectToTransport], gv->tempCounter);
	if (gv->objectToTransport == 8) {
		hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 1);
		cd->recalculateCollider(&gv->colliders[8], gv->pathEndPos.x,
				gv->pathEndPos.y, gv->objectOffset,
				gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true, false,
				nullptr, true, cd->workpieceArrowPosition1);
	}
	if (gv->objectToTransport == 9) {
		hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 2);
		cd->recalculateCollider(&gv->colliders[9], gv->pathEndPos.x,
				gv->pathEndPos.y, gv->objectOffset,
				gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true, false,
				nullptr, true, cd->workpieceArrowPosition2);
	}
	if (gv->objectToTransport == 10) {
		hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 3);
		cd->recalculateCollider(&gv->colliders[10], gv->pathEndPos.x,
				gv->pathEndPos.y, gv->objectOffset,
				gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true, false,
				nullptr, true, cd->workpieceArrowPosition3);
	}
	if (gv->objectToTransport == 11) {
		hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 4);
		cd->recalculateCollider(&gv->colliders[11], gv->pathEndPos.x,
				gv->pathEndPos.y, gv->objectOffset,
				gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true, false,
				nullptr, true, cd->workpieceArrowPosition4);
	}
}

void SimInit::block6(GlobalVariables *gv) {
	for (int i = 0; i < gv->transportableObjectsCount; i++) {
		gv->initPositionsX[i] =
				gv->colliders[gv->transportableObjects[i]].offsets[0];
		gv->initPositionsY[i] =
				gv->colliders[gv->transportableObjects[i]].offsets[1];
		gv->initPositionsAngles[i] =
				gv->colliders[gv->transportableObjects[i]].angles[0];
	}
	gv->realGripperWidth = gv->grippingWidthOpen;
	gv->initConfirmation = false;
	gv->initProcessed = true;
	gv->initPreviouslyProcessed = true;
	gv->modeActive = false;
	gv->initPhase = 0;
}


/*
 * This function manages the positioning of the objects step by step.
 *
 * The algorithm produces a trajectory (i.e. list of joint positions).
 * This trajectory is then stored in gv->pathDataForKuka.
 *
 * This parameters are most important:
 * - gv->initPhase: specifies which workpiece or MT shall be transported (1..4=WP; 5+6=MT)
 * - gv->pathEndPos: specifies where the object will be transported to
 * - gv->endObjectAngle: specifies the target angle
 * - gv->pathDataForKuka: stores joints for Kuka to be transported in comma-separated string.
 *   Format is: "normal", <list of joints>, gripper value
 */
void SimInit::sim_control_simple_add_workpiece_on_init(
		bool &repaint,bool boWrite, bool boWaitForSpace,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{

	GlobalVariables *gv=GlobalVariables::get_instance();
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();

	float x =  21.0f;//21.0f;
	float y = -21.0f;

	if (gv->initPhase > 1) {
		position_object_after_transport(gv, hf, ik, cd);
	}

	if (gv->initPhase <= gv->transportableObjectsCount)
	{
		initially_place_object(gv, cd, x, y);

		if (ik->doesThetaExist(gv->pathStartPos.x, gv->pathStartPos.y,gv->pathStartPos.z))
		{
			ik->writeQValues(gv->qValuesObjectStart);

			if (abs(gv->qValuesObjectStart[1] + gv->qValuesObjectStart[2]
							+ gv->qValuesObjectStart[3] - 180.0f) >= cd->epsilon3) {
				cout << "Init. error: start theta value not 180 degree!"<< endl;
			}
		} else {
			cout << "Init. error: start point kinematics not calculable!"
					<< endl;
		}

		bool boFirst=boWrite;
		do {
			if(boFirst){
				apply_workpiece_position(gv->initPhase-1);
				boFirst=false;
			}
			else if(boWrite){
				std::cout<<"Err@SimInit: object position not reachable! Try random..."<<std::endl;
				apply_random_workpiece_position();
			}else{
				//read random workpiece position
				apply_random_workpiece_position();
			}
			std::cout<<gv->initPhase<< ". Workpiece position= (x="<<gv->pathEndPos.x<<",y="<<gv->pathEndPos.y
					<<", z="<<gv->pathEndPos.z
					<<",rot ="<<gv->endObjectAngle<<")"<<std::endl;

			reposition_workpiece_to_new_position(generator1,distribution1,
					generator2,distribution2,
					generator3,distribution3);
		} while (!gv->objectPosReachable);

		//if (gv->processInitWithRealRobot) { 
		if (true) {
			gv->pathDataForKuka = "normal,";
			for (int i = 0; i < gv->firstSegSize; i++) {
				gv->pathDataForKuka += hf->to_string_with_precision(
						gv->q0ValsFirstSeg[i], 4) + ","
						+ hf->to_string_with_precision(gv->q1ValsFirstSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q2ValsFirstSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q3ValsFirstSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q4ValsFirstSeg[i], 4)
						+ ",";
			}
			gv->pathDataForKuka += hf->to_string_with_precision(
					gv->grippingWidthOpen, 4);

//			gv->initWaitForFinish = true;
			gv->finishSimInit = true;
		}


		block4(cd, gv, hf);

		// Workpiece
		if (gv->objectToTransport < 12) {
			block5(gv, hf, cd);
		}
		// MT1
		else if (gv->objectToTransport == 15) {
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
			cd->recalculateCollider(&gv->colliders[15], gv->pathEndPos.x,
					gv->pathEndPos.y, gv->objectOffset,
					gv->endObjectAngle / 180.0f * M_PI, 0, 0, false, true,
					gv->camVectorsMT1);
		}
		// MT2
		else if (gv->objectToTransport == 16) {
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
			cd->recalculateCollider(&gv->colliders[16], gv->pathEndPos.x,
					gv->pathEndPos.y, gv->objectOffset,
					gv->endObjectAngle / 180.0f * M_PI, 0, 0, false, true,
					gv->camVectorsMT2);
		}
	}

	if (gv->initPhase == gv->transportableObjectsCount + 1) {
		block6(gv);
	} else {
		gv->initPhase++;
	}

	if(boWaitForSpace){
	   gv->initSpacePressed = false;
	}

	repaint = true;
}

void SimInit::set_workpiece_position(ObjectPositionInterface obj){
	GlobalVariables *gv=GlobalVariables::get_instance();
	if(obj.index >= 0 && obj.index < gv->transportableObjectsCount){

		this->obj[obj.index] = obj;
	}else{
		std::cout<<"Err@set_object_position: Index out of bounds!"<<endl;
	}
}

/*
 * sets the object position with index and coordinates.
 * deprecated. Use ObjectPositionInterface instead!!!
 */
void SimInit::set_workpiece_position(int index, float x, float y, float z, float angle){
	GlobalVariables *gv=GlobalVariables::get_instance();
	if(index >= 0 && index < gv->transportableObjectsCount){
		obj[index].x = x;
		obj[index].y = y;
		obj[index].z = z;
		obj[index].angle = angle;
		obj[index].index = index;
	}else{
		std::cout<<"Err@set_workpiece_position: Index out of bounds!"<<endl;
	}
}

void SimInit::apply_workpiece_position(int index){
	GlobalVariables *gv=GlobalVariables::get_instance();

	if(index >= 0 && index < gv->transportableObjectsCount){
		collider::Edge e;
		e.x = obj[index].x;
		e.y = obj[index].y;
		e.z = obj[index].z;

		gv->pathEndPos = e;
		gv->endObjectAngle = obj[index].angle;

		// revealed to be missing although
		gv->pathEndPos.z = gv->objectOffset + gv->objectHeight+ gv->objectGripDepth;
	}else{
		std::cout<<"Err@apply_object_position: Index out of bounds!"<<endl;
	}
}


void SimInit::apply_random_workpiece_position(){
	GlobalVariables *gv=GlobalVariables::get_instance();
	CollisionDetection *cd =CollisionDetection::get_instance();
	gv->pathEndPos.x = ((float) ((rand())) / RAND_MAX) * 68.0f - 34.0f;
	gv->pathEndPos.y = ((float) ((rand())) / RAND_MAX) * 46.5f - 12.5f;
	gv->pathEndPos.z = gv->objectOffset + gv->objectHeight+ gv->objectGripDepth;
	gv->endObjectAngle = ((float) ((rand())) / RAND_MAX)* (360.0f - cd->epsilon4);

	// save used workpiece position;
	set_workpiece_position(gv->initPhase-1, gv->pathEndPos.x, gv->pathEndPos.y, gv->pathEndPos.z, gv->endObjectAngle);
}


ObjectPositionInterface* SimInit::get_workpiecePositions(){
	return obj;
}

/*
	 * This function handels the actual repositioning.
	 * To this end it simulates a transportation from the start position
	 * that is initially set to (x=21,y=-21,rot =0)
	 * to the end position which is defined in pathEndPos.
	 * Rotation is defined in endObjectAngle.
	 *
	 */
	void SimInit::reposition_workpiece_to_new_position(default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3) {
		GlobalVariables *gv = GlobalVariables::get_instance();
		CollisionDetection *cd = CollisionDetection::get_instance();
		InverseKinematic *ik = InverseKinematic::get_instance();
		HelperFunctions *hf = HelperFunctions::get_instance();
		FrequentOps *fo = FrequentOps::get_instance();
		PathPlanner *pp = PathPlanner::get_instance();
		gv->objectPosReachable = false;
		// find out whether the distance to 0/0 is larger than 1.5² + 2.5²
		// NOTE: sqrt not neccessary at this point!
		if (sqrt(
				(gv->pathEndPos.x - 21.0f) * (gv->pathEndPos.x - 21.0f)
				+ (gv->pathEndPos.y + 21.0f)
				* (gv->pathEndPos.y + 21.0f))
				> 2.0f * sqrt(1.5f * 1.5f + 2.5f * 2.5f) + cd->epsilon3) {
			//find out, whether the end point is in range of the robot
			if (ik->doesThetaExist(gv->pathEndPos.x, gv->pathEndPos.y,
					gv->pathEndPos.z))
			{
				// add end point to trajectory.
				ik->writeQValues(gv->qValuesObjectEnd);

				// sum of joints adds up to 180°
				if (abs(gv->qValuesObjectEnd[1] + gv->qValuesObjectEnd[2]
						+ gv->qValuesObjectEnd[3] - 180.0f)	< cd->epsilon3)
				{
					gv->qValuesObjectStart[4] = gv->startObjectAngle
							- (90.0f - gv->qValuesObjectStart[0]);
					gv->qValuesObjectEnd[4] = gv->endObjectAngle
							- (90.0f - gv->qValuesObjectEnd[0]);
					fo->gripAngleCalculation(gv->initPhase - 1);

					//check limits for start and and point
					if (   gv->qValuesObjectStart[4] > ik->qUpperLimits[4]
						|| gv->qValuesObjectStart[4] < ik->qLowerLimits[4]
						|| gv->qValuesObjectEnd[4] > ik->qUpperLimits[4]
						|| gv->qValuesObjectEnd[4] < ik->qLowerLimits[4]
					    ) {
						gv->objectPosReachable = false;
					} else {
						// limits okay....
						gv->transportPhase = 0;
						gv->grippingWidth = gv->grippingWidthOpen;
						gv->planned_path_edges = ik->cutOff(
								gv->planned_path_edges,
								gv->numPointsPlannedPath, 0);
						gv->planned_path_vertices = ik->cutOff(
								gv->planned_path_vertices,
								gv->numPointsPlannedPath, 0);
						gv->numPointsPlannedPath = 0;

						// calculate path of robot to the obstacle
						gv->objectPosReachable = pp->calcPath(
								gv->qValuesObjectStart, gv->qValuesStandby,
								&gv->pathStartPos, &gv->standbyPos, true,
								generator1, distribution1, generator2,
								distribution2, generator3, distribution3);

						if (gv->objectPosReachable) {
							// generate path to
							pp->optimizePath();
							fo->drawPathOnly(2);
							if (gv->firstSegSize != 0) {
								gv->q0ValsFirstSeg = ik->cutOff(
										gv->q0ValsFirstSeg, gv->firstSegSize,
										0);
								gv->q1ValsFirstSeg = ik->cutOff(
										gv->q1ValsFirstSeg, gv->firstSegSize,
										0);
								gv->q2ValsFirstSeg = ik->cutOff(
										gv->q2ValsFirstSeg, gv->firstSegSize,
										0);
								gv->q3ValsFirstSeg = ik->cutOff(
										gv->q3ValsFirstSeg, gv->firstSegSize,
										0);
								gv->q4ValsFirstSeg = ik->cutOff(
										gv->q4ValsFirstSeg, gv->firstSegSize,
										0);
							}
							gv->q0ValsFirstSeg = ik->increaseSize(
									gv->q0ValsFirstSeg, 0, 1);
							gv->q1ValsFirstSeg = ik->increaseSize(
									gv->q1ValsFirstSeg, 0, 1);
							gv->q2ValsFirstSeg = ik->increaseSize(
									gv->q2ValsFirstSeg, 0, 1);
							gv->q3ValsFirstSeg = ik->increaseSize(
									gv->q3ValsFirstSeg, 0, 1);
							gv->q4ValsFirstSeg = ik->increaseSize(
									gv->q4ValsFirstSeg, 0, 1);
							gv->q0ValsFirstSeg[0] = pp->nodeList[1].nodeQVal[0];
							gv->q1ValsFirstSeg[0] = pp->nodeList[1].nodeQVal[1];
							gv->q2ValsFirstSeg[0] = pp->nodeList[1].nodeQVal[2];
							gv->q3ValsFirstSeg[0] = pp->nodeList[1].nodeQVal[3];
							gv->q4ValsFirstSeg[0] = pp->nodeList[1].nodeQVal[4];
							gv->firstSegSize = 1;
							pp->nodeIndex = 1;
							while (pp->nodeIndex != 0) {
								pp->nodeIndex =
										pp->nodeList[pp->nodeIndex].previous;
								gv->q0ValsFirstSeg = ik->increaseSize(
										gv->q0ValsFirstSeg, gv->firstSegSize,
										1);
								gv->q1ValsFirstSeg = ik->increaseSize(
										gv->q1ValsFirstSeg, gv->firstSegSize,
										1);
								gv->q2ValsFirstSeg = ik->increaseSize(
										gv->q2ValsFirstSeg, gv->firstSegSize,
										1);
								gv->q3ValsFirstSeg = ik->increaseSize(
										gv->q3ValsFirstSeg, gv->firstSegSize,
										1);
								gv->q4ValsFirstSeg = ik->increaseSize(
										gv->q4ValsFirstSeg, gv->firstSegSize,
										1);
								gv->q0ValsFirstSeg[gv->firstSegSize] =
										pp->nodeList[pp->nodeIndex].nodeQVal[0];
								gv->q1ValsFirstSeg[gv->firstSegSize] =
										pp->nodeList[pp->nodeIndex].nodeQVal[1];
								gv->q2ValsFirstSeg[gv->firstSegSize] =
										pp->nodeList[pp->nodeIndex].nodeQVal[2];
								gv->q3ValsFirstSeg[gv->firstSegSize] =
										pp->nodeList[pp->nodeIndex].nodeQVal[3];
								gv->q4ValsFirstSeg[gv->firstSegSize] =
										pp->nodeList[pp->nodeIndex].nodeQVal[4];
								gv->firstSegSize++;
							}
							gv->transportPhase = 1;
							gv->grippingWidth = gv->grippingWidthFixed;


							// path of obstacle to its destination
							gv->objectPosReachable = pp->calcPath(
									gv->qValuesObjectEnd,
									gv->qValuesObjectStart, &gv->pathEndPos,
									&gv->pathStartPos, true, generator1,
									distribution1, generator2, distribution2,
									generator3, distribution3);

							if (gv->objectPosReachable) {
								pp->optimizePath();
								fo->drawPathOnly(1);
								if (gv->secondSegSize != 0) {
									gv->secondSegLengths = ik->cutOff(
											gv->secondSegLengths,
											gv->secondSegSize - 1, 0);
									gv->q0ValsSecondSeg = ik->cutOff(
											gv->q0ValsSecondSeg,
											gv->secondSegSize, 0);
									gv->q1ValsSecondSeg = ik->cutOff(
											gv->q1ValsSecondSeg,
											gv->secondSegSize, 0);
									gv->q2ValsSecondSeg = ik->cutOff(
											gv->q2ValsSecondSeg,
											gv->secondSegSize, 0);
									gv->q3ValsSecondSeg = ik->cutOff(
											gv->q3ValsSecondSeg,
											gv->secondSegSize, 0);
									gv->q4ValsSecondSeg = ik->cutOff(
											gv->q4ValsSecondSeg,
											gv->secondSegSize, 0);
								}
								gv->q0ValsSecondSeg = ik->increaseSize(
										gv->q0ValsSecondSeg, 0, 1);
								gv->q1ValsSecondSeg = ik->increaseSize(
										gv->q1ValsSecondSeg, 0, 1);
								gv->q2ValsSecondSeg = ik->increaseSize(
										gv->q2ValsSecondSeg, 0, 1);
								gv->q3ValsSecondSeg = ik->increaseSize(
										gv->q3ValsSecondSeg, 0, 1);
								gv->q4ValsSecondSeg = ik->increaseSize(
										gv->q4ValsSecondSeg, 0, 1);
								gv->q0ValsSecondSeg[0] =
										pp->nodeList[1].nodeQVal[0];
								gv->q1ValsSecondSeg[0] =
										pp->nodeList[1].nodeQVal[1];
								gv->q2ValsSecondSeg[0] =
										pp->nodeList[1].nodeQVal[2];
								gv->q3ValsSecondSeg[0] =
										pp->nodeList[1].nodeQVal[3];
								gv->q4ValsSecondSeg[0] =
										pp->nodeList[1].nodeQVal[4];
								gv->secondSegSize = 1;
								pp->nodeIndex = 1;
								while (pp->nodeIndex != 0) {
									pp->nodeIndex =
											pp->nodeList[pp->nodeIndex].previous;
									gv->q0ValsSecondSeg = ik->increaseSize(
											gv->q0ValsSecondSeg,
											gv->secondSegSize, 1);
									gv->q1ValsSecondSeg = ik->increaseSize(
											gv->q1ValsSecondSeg,
											gv->secondSegSize, 1);
									gv->q2ValsSecondSeg = ik->increaseSize(
											gv->q2ValsSecondSeg,
											gv->secondSegSize, 1);
									gv->q3ValsSecondSeg = ik->increaseSize(
											gv->q3ValsSecondSeg,
											gv->secondSegSize, 1);
									gv->q4ValsSecondSeg = ik->increaseSize(
											gv->q4ValsSecondSeg,
											gv->secondSegSize, 1);
									gv->q0ValsSecondSeg[gv->secondSegSize] =
											pp->nodeList[pp->nodeIndex].nodeQVal[0];
									gv->q1ValsSecondSeg[gv->secondSegSize] =
											pp->nodeList[pp->nodeIndex].nodeQVal[1];
									gv->q2ValsSecondSeg[gv->secondSegSize] =
											pp->nodeList[pp->nodeIndex].nodeQVal[2];
									gv->q3ValsSecondSeg[gv->secondSegSize] =
											pp->nodeList[pp->nodeIndex].nodeQVal[3];
									gv->q4ValsSecondSeg[gv->secondSegSize] =
											pp->nodeList[pp->nodeIndex].nodeQVal[4];
									gv->secondSegSize++;
								}
								for (int i = 1; i < gv->secondSegSize; i++) {
									gv->secondSegLengths = ik->increaseSize(
											gv->secondSegLengths, i - 1, 1);
								}
								cd->recalculateCollider(
										&gv->colliders[gv->objectToTransport],
										gv->pathEndPos.x, gv->pathEndPos.y,
										gv->objectOffset,
										gv->endObjectAngle / 180.0f * M_PI,
										0.0f, 0.0f);
								gv->transportPhase = 2;
								gv->grippingWidth = gv->grippingWidthOpen;

								//move robot to initial position
								gv->objectPosReachable = pp->calcPath(
										gv->qValuesStandby,
										gv->qValuesObjectEnd, &gv->standbyPos,
										&gv->pathEndPos, false, generator1,
										distribution1, generator2,
										distribution2, generator3,
										distribution3);

								if (gv->objectPosReachable) {
									pp->optimizePath();
									fo->drawPathOnly(0);
									if (gv->thirdSegSize != 0) {
										gv->q0ValsThirdSeg = ik->cutOff(
												gv->q0ValsThirdSeg,
												gv->thirdSegSize, 0);
										gv->q1ValsThirdSeg = ik->cutOff(
												gv->q1ValsThirdSeg,
												gv->thirdSegSize, 0);
										gv->q2ValsThirdSeg = ik->cutOff(
												gv->q2ValsThirdSeg,
												gv->thirdSegSize, 0);
										gv->q3ValsThirdSeg = ik->cutOff(
												gv->q3ValsThirdSeg,
												gv->thirdSegSize, 0);
										gv->q4ValsThirdSeg = ik->cutOff(
												gv->q4ValsThirdSeg,
												gv->thirdSegSize, 0);
									}
									gv->q0ValsThirdSeg = ik->increaseSize(
											gv->q0ValsThirdSeg, 0, 1);
									gv->q1ValsThirdSeg = ik->increaseSize(
											gv->q1ValsThirdSeg, 0, 1);
									gv->q2ValsThirdSeg = ik->increaseSize(
											gv->q2ValsThirdSeg, 0, 1);
									gv->q3ValsThirdSeg = ik->increaseSize(
											gv->q3ValsThirdSeg, 0, 1);
									gv->q4ValsThirdSeg = ik->increaseSize(
											gv->q4ValsThirdSeg, 0, 1);
									gv->q0ValsThirdSeg[0] =
											pp->nodeList[1].nodeQVal[0];
									gv->q1ValsThirdSeg[0] =
											pp->nodeList[1].nodeQVal[1];
									gv->q2ValsThirdSeg[0] =
											pp->nodeList[1].nodeQVal[2];
									gv->q3ValsThirdSeg[0] =
											pp->nodeList[1].nodeQVal[3];
									gv->q4ValsThirdSeg[0] =
											pp->nodeList[1].nodeQVal[4];
									gv->thirdSegSize = 1;
									pp->nodeIndex = 1;
									while (pp->nodeIndex != 0) {
										pp->nodeIndex =
												pp->nodeList[pp->nodeIndex].previous;
										gv->q0ValsThirdSeg = ik->increaseSize(
												gv->q0ValsThirdSeg,
												gv->thirdSegSize, 1);
										gv->q1ValsThirdSeg = ik->increaseSize(
												gv->q1ValsThirdSeg,
												gv->thirdSegSize, 1);
										gv->q2ValsThirdSeg = ik->increaseSize(
												gv->q2ValsThirdSeg,
												gv->thirdSegSize, 1);
										gv->q3ValsThirdSeg = ik->increaseSize(
												gv->q3ValsThirdSeg,
												gv->thirdSegSize, 1);
										gv->q4ValsThirdSeg = ik->increaseSize(
												gv->q4ValsThirdSeg,
												gv->thirdSegSize, 1);
										gv->q0ValsThirdSeg[gv->thirdSegSize] =
												pp->nodeList[pp->nodeIndex].nodeQVal[0];
										gv->q1ValsThirdSeg[gv->thirdSegSize] =
												pp->nodeList[pp->nodeIndex].nodeQVal[1];
										gv->q2ValsThirdSeg[gv->thirdSegSize] =
												pp->nodeList[pp->nodeIndex].nodeQVal[2];
										gv->q3ValsThirdSeg[gv->thirdSegSize] =
												pp->nodeList[pp->nodeIndex].nodeQVal[3];
										gv->q4ValsThirdSeg[gv->thirdSegSize] =
												pp->nodeList[pp->nodeIndex].nodeQVal[4];
										gv->thirdSegSize++;
									}
									if (gv->objectToTransport == 8) {
										cd->recalculateCollider(
												&gv->colliders[8],
												gv->pathStartPos.x,
												gv->pathStartPos.y,
												gv->objectOffset,
												gv->startObjectAngle
												/ 180.0f* M_PI, 0.0f,
												0.0f, true, false, nullptr,
												true,
												cd->workpieceArrowPosition1);
									} else if (gv->objectToTransport == 9) {
										cd->recalculateCollider(
												&gv->colliders[9],
												gv->pathStartPos.x,
												gv->pathStartPos.y,
												gv->objectOffset,
												gv->startObjectAngle
												/ 180.0f* M_PI, 0.0f,
												0.0f, true, false, nullptr,
												true,
												cd->workpieceArrowPosition2);
									} else if (gv->objectToTransport == 10) {
										cd->recalculateCollider(
												&gv->colliders[10],
												gv->pathStartPos.x,
												gv->pathStartPos.y,
												gv->objectOffset,
												gv->startObjectAngle
												/ 180.0f* M_PI, 0.0f,
												0.0f, true, false, nullptr,
												true,
												cd->workpieceArrowPosition3);
									} else if (gv->objectToTransport == 11) {
										cd->recalculateCollider(
												&gv->colliders[11],
												gv->pathStartPos.x,
												gv->pathStartPos.y,
												gv->objectOffset,
												gv->startObjectAngle
												/ 180.0f* M_PI, 0.0f,
												0.0f, true, false, nullptr,
												true,
												cd->workpieceArrowPosition4);
									} else if (gv->objectToTransport == 15) {
										cd->recalculateCollider(
												&gv->colliders[15],
												gv->pathStartPos.x,
												gv->pathStartPos.y,
												gv->objectOffset,
												gv->startObjectAngle
												/ 180.0f* M_PI, 0, 0,
												false, true, gv->camVectorsMT1);
									} else if (gv->objectToTransport == 16) {
										cd->recalculateCollider(
												&gv->colliders[16],
												gv->pathStartPos.x,
												gv->pathStartPos.y,
												gv->objectOffset,
												gv->startObjectAngle
												/ 180.0f* M_PI, 0, 0,
												false, true, gv->camVectorsMT2);
									}
								}
							} else {
								std::cout
								<< "dbg@SimInit: !gv->objectPosReachable"
								<< std::endl;
							}
							gv->transportPhase = 0;
						}else{
							std::cout<<"E: Init pos must agree!"<<endl;
						}
					}
				}else{
					std::cout<<"E: Sum of joints must add to 180°!"<<endl;
				}
			}else{
				std::cout<<"E: out of range!"<<endl;
			}
		}else{
			std::cout<<"E: too close to socket!?"<<endl;
		}
	}
