/*
 * RealRobotExecution.cpp
 *
 *  Created on: 22.01.2023
 *      Author: manuel
 */

#include "RealRobotExecution.h"

RealRobotExecution::RealRobotExecution() {
	// TODO Auto-generated constructor stub

}

RealRobotExecution::~RealRobotExecution() {
	// TODO Auto-generated destructor stub
}

/*
 * singelton pattern
 */
RealRobotExecution* RealRobotExecution::rre = 0;
RealRobotExecution* RealRobotExecution::get_instance(){
	static bool isInit=false;

//	InverseKinematic *ik;
	if(!isInit){
		rre  = new RealRobotExecution();
		isInit=true;
	}
	return rre;
}


/*
 * this function asks whether an action should be executed on real robot.
 */
void RealRobotExecution::sim_control_ask_real_execution(){
	GlobalVariables *gv=GlobalVariables::get_instance();
	HelperFunctions *hf=HelperFunctions::get_instance();

	std::cout << "\nExecute on real robot anyway? (press y/n)" << std::endl;
	char c_in = hf->wait_for_user_input_yn();

	if (c_in == 'y') {
		gv->synMode = 3;
		gv->asynMode = 3;
		gv->realExecPhase = 0;
	}
	else {
		std::cout << "No execution on real robot!" << std::endl;
		gv->modeActive = false;
	}
}


void RealRobotExecution::sim_control_physical_mode(){
	MonitoringToolModels *mt=MonitoringToolModels::get_instance();
	GlobalVariables *gv= GlobalVariables::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();

	if (gv->onlyAskMTsAfterInit && gv->initPreviouslyProcessed) {
		gv->initPreviouslyProcessed = false;
//#if defined(WIN32)
		sendMessageMT1("state_request");
		sendMessageMT2("state_request");
		thread mt1Thread(receiveMessageMT1);
		thread mt2Thread( receiveMessageMT2);

		mt1Thread.join();
		mt2Thread.join();
//#endif
		gv->stateMessageMT1saved = gv->stateMessageMT1;
		gv->stateMessageMT2saved = gv->stateMessageMT2;
	}
	else if (!gv->onlyAskMTsAfterInit) {
//#if defined(WIN32)
		sendMessageMT1("state_request");
		sendMessageMT2("state_request");
		thread mt1Thread( receiveMessageMT1);
		thread mt2Thread( receiveMessageMT2);

		mt1Thread.join();
		mt2Thread.join();
//#endif
	}
	else {
		gv->stateMessageMT1 = gv->stateMessageMT1saved;
		gv->stateMessageMT2 = gv->stateMessageMT2saved;
	}

	cout << "Detected init. state from MT1: " << gv->stateMessageMT1 << endl;
	cout << "Detected init. state from MT2: " << gv->stateMessageMT2 << endl;

	vector<string> splitList1;
	vector<string> splitList2;
	std::stringstream string_stream_1;
	std::stringstream string_stream_2;
	string splitted;

	string_stream_1.str(gv->stateMessageMT1);
	while (getline(string_stream_1, splitted, ',')) {
		splitList1.push_back(splitted);
	}

	string_stream_2.str(gv->stateMessageMT2);
	while (getline(string_stream_2, splitted, ',')) {
		splitList2.push_back(splitted);
	}

	fo->resetDetectedPatterns();
	for (int i = 0; i < totalPatternCount; i++) {
		fo->detectedPatternIndicesForMT1[i] = false;
		fo->detectedPatternIndicesForMT2[i] = false;
	}

	if (splitList1.size() > 0) {
		for (int i = 0; i < stoi(splitList1[splitList1.size() - 1]) * 5; i += 5) {
			fo->detectedPatternIndicesForMT1[stoi(splitList1[i])] = true;
			fo->distortedRelativePosition[0] = stof(splitList1[i + 1]);
			fo->distortedRelativePosition[1] = stof(splitList1[i + 2]);
			fo->distortedRelativePosition[2] = stof(splitList1[i + 3]);
			fo->distortedRelativePosition[3] = stof(splitList1[i + 4]);
			fo->calcPosFromMTData(&gv->colliders[15], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], stoi(splitList1[i]));
			fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList1[i])] - 8);
		}
	}

	if (splitList2.size() > 0) {
		for (int i = 0; i < stoi(splitList2[splitList2.size() - 1]) * 5; i += 5) {
			fo->detectedPatternIndicesForMT2[stoi(splitList2[i])] = true;
			fo->distortedRelativePosition[0] = stof(splitList2[i + 1]);
			fo->distortedRelativePosition[1] = stof(splitList2[i + 2]);
			fo->distortedRelativePosition[2] = stof(splitList2[i + 3]);
			fo->distortedRelativePosition[3] = stof(splitList2[i + 4]);
			fo->calcPosFromMTData(&gv->colliders[16], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], stoi(splitList2[i]));
			fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList2[i])] - 8);
		}
	}
	fo->evaluateDetections(true);
}


void RealRobotExecution::sim_control_transportPhase1(bool &repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();
	PathPlanner *pp=PathPlanner::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();

	gv->grippingWidth = gv->grippingWidthFixed;

	// std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	gv->objectPosReachable = false;
	for (int i = 0; i < 10; i++) {
		if (pp->calcPath(gv->qValuesObjectEnd, gv->qValuesObjectStart, &gv->pathEndPos, &gv->pathStartPos, false, generator1,distribution1, generator2,distribution2,generator3,distribution3)) {
			pp->optimizePath();
			gv->objectPosReachable = true;
			break;
		}
	}
	// std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	// std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

	if (gv->objectPosReachable) {
		pp->optimizePath();
		cout << "Training info: second path planning successful" << endl;

		fo->drawPathAndCalculation();

		gv->adv_dev_x = 0.0f; gv->adv_dev_y = 0.0f; gv->adv_dev_z = 0.0f;

		repaint = true;
		gv->virtSim = true;
	}
	else {
		cout << "Training info: second path planning failed -> abort..." << endl;
		gv->transportPhase = 0;
		gv->modeActive = false;
	}
}

void RealRobotExecution::sim_control_transportPhas2(
		bool &repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3){
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();
	PathPlanner *pp=PathPlanner::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();

	gv->grippingWidth = gv->grippingWidthOpen;

	gv->objectPosReachable = false;
	for (int i = 0; i < 10; i++) {
		if (pp->calcPath(gv->qValuesStandby, gv->qValuesObjectEnd, &gv->standbyPos, &gv->pathEndPos, false, generator1,distribution1, generator2,distribution2,generator3,distribution3)) {
			gv->objectPosReachable = true;
			break;
		}
	}

	if (gv->objectPosReachable) {
		pp->optimizePath();
		cout << "Training info: third gv->path planning successful" << endl;

		fo->drawPathAndCalculation();

		gv->adv_dev_x = 0.0f; gv->adv_dev_y = 0.0f; gv->adv_dev_z = 0.0f;

		repaint = true;
		gv->virtSim = true;
	}
	else {
		cout << "Training info: third gv->path planning failed -> abort..." << endl;
		gv->transportPhase = 0;
		gv->modeActive = false;
	}

}

bool RealRobotExecution::real_execution_phase0(bool repaint)
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	MonitoringToolModels *mt = MonitoringToolModels::get_instance();
	CollisionDetection * cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

	gv->userAbort = false;
	gv->collision = false;
	gv->grippingWidth = 0;
	hf->storePathRelatedData(0);
	for (int i = 0; i < 5; i++) {
		gv->qValuesCurrent[i] = gv->qValuesStandby[i];
	}
	gv->q0Sin = sin(gv->qValuesCurrent[0] / 180.0f * M_PI);
	gv->q0Cos = cos(gv->qValuesCurrent[0] / 180.0f * M_PI);
	gv->q1Sin = sin(gv->qValuesCurrent[1] / 180.0f * M_PI);
	gv->q1Cos = cos(gv->qValuesCurrent[1] / 180.0f * M_PI);
	gv->q12Sin = sin(
			(gv->qValuesCurrent[1] + gv->qValuesCurrent[2]) / 180.0f * M_PI);
	gv->q12Cos = cos(
			(gv->qValuesCurrent[1] + gv->qValuesCurrent[2]) / 180.0f * M_PI);
	gv->q123Sin = sin(
			(gv->qValuesCurrent[1] + gv->qValuesCurrent[2]
					+ gv->qValuesCurrent[3]) / 180.0f * M_PI);
	gv->q123Cos = cos(
			(gv->qValuesCurrent[1] + gv->qValuesCurrent[2]
					+ gv->qValuesCurrent[3]) / 180.0f * M_PI);
	cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f,
			(-gv->qValuesCurrent[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
	cd->recalculateCollider(&gv->colliders[4],
			(ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Sin
					+ 4.5f * gv->q0Cos,
			(ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Cos
					- 4.5f * gv->q0Sin, ik->len0 + ik->len2 / 2.0f * gv->q1Cos,
			(-gv->qValuesCurrent[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesCurrent[1]) / 180.0f * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[5],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin)
					* gv->q0Sin - 4.155f * gv->q0Cos,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin)
					* gv->q0Cos + 4.155f * gv->q0Sin,
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 / 2.0f * gv->q12Cos,
			(-gv->qValuesCurrent[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]) / 180 * M_PI,
			0);
	cd->recalculateCollider(&gv->colliders[6],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ ik->lastSegmentMid * gv->q123Sin) * gv->q0Sin,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ ik->lastSegmentMid * gv->q123Sin) * gv->q0Cos,
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos
					+ ik->lastSegmentMid * gv->q123Cos,
			(-gv->qValuesCurrent[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]
					- gv->qValuesCurrent[3]) / 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[7],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f)
							* gv->q123Sin) * gv->q0Sin,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f)
							* gv->q123Sin) * gv->q0Cos,
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos
					+ (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f)
							* gv->q123Cos,
			(-gv->qValuesCurrent[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]
					- gv->qValuesCurrent[3]) / 180 * M_PI,
			gv->qValuesCurrent[4] / 180.0f * M_PI);
	cd->updateMatricesTransport(
			(-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]
					- gv->qValuesCurrent[3]) / 180.0f * M_PI,
			(gv->qValuesCurrent[4] + 0) / 180.0f * M_PI,
			gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
	cd->rotationMatrix(gv->q0Cos * (gv->grippingWidth / 2.0f + 0.45f),
			-gv->q0Sin * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
	gv->grippingDiffVector[0] = cd->xVal_cd;
	gv->grippingDiffVector[1] = cd->yVal_cd;
	gv->grippingDiffVector[2] = cd->zVal_cd;
	cd->recalculateColliderTransport(&gv->grippers[0],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + 2.25f + gv->gripperInstallationOffset)
							* gv->q123Sin) * gv->q0Sin
					+ gv->grippingDiffVector[0],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + gv->gripperInstallationOffset + 2.25f)
							* gv->q123Sin) * gv->q0Cos
					+ gv->grippingDiffVector[1],
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos
					+ (ik->len4 + gv->gripperInstallationOffset + 2.25f)
							* gv->q123Cos + gv->grippingDiffVector[2],
			(-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]
					- gv->qValuesCurrent[3]) / 180.0f * M_PI,
			gv->qValuesCurrent[4] / 180.0f * M_PI);
	cd->recalculateColliderTransport(&gv->grippers[1],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + 2.25f + gv->gripperInstallationOffset)
							* gv->q123Sin) * gv->q0Sin
					- gv->grippingDiffVector[0],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + gv->gripperInstallationOffset + 2.25f)
							* gv->q123Sin) * gv->q0Cos
					- gv->grippingDiffVector[1],
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos
					+ (ik->len4 + gv->gripperInstallationOffset + 2.25f)
							* gv->q123Cos - gv->grippingDiffVector[2],
			(-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]
					- gv->qValuesCurrent[3]) / 180.0f * M_PI,
			gv->qValuesCurrent[4] / 180.0f * M_PI);
	gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
			gv->numPointsPlannedPath, 0);
	gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
			gv->numPointsPlannedPath, 0);
	gv->numPointsPlannedPath = 0;
	gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath,
			0);
	gv->real_path_vertices = ik->cutOff(gv->real_path_vertices,
			gv->numPointsRealPath, 0);
	gv->numPointsRealPath = 0;
	for (int i = 0; i < gv->transportableObjectsCount; i++) {
		if (i < gv->transportableObjectsCount - 2) {
			cd->recalculateCollider(&gv->colliders[gv->transportableObjects[i]],
					gv->initPositionsX[i], gv->initPositionsY[i],
					gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0,
					true);
		} else if (gv->transportableObjects[i] == 15) {
			cd->recalculateCollider(&gv->colliders[15], gv->initPositionsX[i],
					gv->initPositionsY[i], gv->objectOffsets[i],
					gv->initPositionsAngles[i], 0, 0, false, true,
					gv->camVectorsMT1);
		} else if (gv->transportableObjects[i] == 16) {
			cd->recalculateCollider(&gv->colliders[16], gv->initPositionsX[i],
					gv->initPositionsY[i], gv->objectOffsets[i],
					gv->initPositionsAngles[i], 0, 0, false, true,
					gv->camVectorsMT2);
		}
	}
	for (int i = 12; i <= 14; i++) {
		cd->recalculateCollider(&gv->colliders[i], 0.0f, 0.0f, 10.0e5, 0, 0, 0,
				true);
	}
	gv->tempCounter = gv->totalPlatePoints;
	for (int i = 0; i < 3; i++) {
		gv->tempCounter += gv->colliders[i].facesTimes3;
	}
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[3], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[4], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[5], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[6], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[7], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[8], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[9], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[10], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[11], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[12], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[13], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[14], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[15], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[16], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->grippers[0], gv->tempCounter);
	hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[1],
			gv->tempCounter);
	hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
	hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges,
			&gv->colliders[8], 0);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges,
			&gv->colliders[9], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges,
			&gv->colliders[10], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges,
			&gv->colliders[11], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges,
			&gv->colliders[12], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges,
			&gv->colliders[13], gv->tempCounter);
	hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[14],
			gv->tempCounter);
	gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges, gv->numPointsMTrays, 0);
	gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices, gv->numPointsMTrays,
			0);
	gv->numPointsMTrays = 0;
	if (gv->virtualMode) {
		cd->recalculateCollider(&gv->colliders[12],
				gv->groundTruthObstaclePositions[0],
				gv->groundTruthObstaclePositions[1], 4.0f,
				gv->groundTruthObstaclePositions[2], 0, 0, true);
		cd->recalculateCollider(&gv->colliders[13],
				gv->groundTruthObstaclePositions[3],
				gv->groundTruthObstaclePositions[4], 3.0f,
				gv->groundTruthObstaclePositions[5], 0, 0, true);
		cd->recalculateCollider(&gv->colliders[14],
				gv->groundTruthObstaclePositions[6],
				gv->groundTruthObstaclePositions[7], 6.0f,
				gv->groundTruthObstaclePositions[8], 0, 0, true);
		gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges,
				&gv->colliders[12], 0);
		gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges,
				&gv->colliders[13], gv->tempCounter);
		hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[14],
				gv->tempCounter);
		fo->resetDetectedPatterns();
		for (int i = 0; i < totalPatternCount; i++) {
			if (mt->isPatternVisible(i, 1, true)) {
				hf->getMTPositionData(1);
				fo->calcPosFromMTData(&gv->colliders[15],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], i);
				fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
			if (mt->isPatternVisible(i, 2, true)) {
				hf->getMTPositionData(2);
				fo->calcPosFromMTData(&gv->colliders[16],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], i);
				fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
		}
		fo->evaluateDetections(false);
	} else {
		vector<string> splitList1;
		vector<string> splitList2;
		std::stringstream string_stream_1;
		std::stringstream string_stream_2;
		string splitted;
		string_stream_1.str(gv->stateMessageMT1);
		while (getline(string_stream_1, splitted, ',')) {
			splitList1.push_back(splitted);
		}
		string_stream_2.str(gv->stateMessageMT2);
		while (getline(string_stream_2, splitted, ',')) {
			splitList2.push_back(splitted);
		}
		fo->resetDetectedPatterns();
		if (splitList1.size() > 0) {
			for (int i = 0; i < stoi(splitList1[splitList1.size() - 1]) * 5;
					i += 5) {
				fo->distortedRelativePosition[0] = stof(splitList1[i + 1]);
				fo->distortedRelativePosition[1] = stof(splitList1[i + 2]);
				fo->distortedRelativePosition[2] = stof(splitList1[i + 3]);
				fo->distortedRelativePosition[3] = stof(splitList1[i + 4]);
				fo->calcPosFromMTData(&gv->colliders[15],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], stoi(splitList1[i]));
				fo->insertDetectedPattern(
						objectIndicesFromPattern[stoi(splitList1[i])] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
		}
		if (splitList2.size() > 0) {
			for (int i = 0; i < stoi(splitList2[splitList2.size() - 1]) * 5;
					i += 5) {
				fo->distortedRelativePosition[0] = stof(splitList2[i + 1]);
				fo->distortedRelativePosition[1] = stof(splitList2[i + 2]);
				fo->distortedRelativePosition[2] = stof(splitList2[i + 3]);
				fo->distortedRelativePosition[3] = stof(splitList2[i + 4]);
				fo->calcPosFromMTData(&gv->colliders[16],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], stoi(splitList2[i]));
				fo->insertDetectedPattern(
						objectIndicesFromPattern[stoi(splitList2[i])] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
		}
		fo->evaluateDetections(false);
	}
	if (fo->detectionCounts[4] == 0) {
		cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f, 10.0e5,
				0, 0, 0);
		gv->currentDetectedObsPosX[0] = 0.0f;
		gv->currentDetectedObsPosY[0] = 0.0f;
		gv->currentDetectedObsPosA[0] = 0.0f;
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[0],
				fo->evaluatedDetections[4].x, fo->evaluatedDetections[4].y,
				4.0f, fo->evaluatedDetections[4].z, 0, 0);
		gv->currentDetectedObsPosX[0] = fo->evaluatedDetections[4].x;
		gv->currentDetectedObsPosY[0] = fo->evaluatedDetections[4].y;
		gv->currentDetectedObsPosA[0] = fo->evaluatedDetections[4].z;
	}
	if (fo->detectionCounts[5] == 0) {
		cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f, 10.0e5,
				0, 0, 0);
		gv->currentDetectedObsPosX[1] = 0.0f;
		gv->currentDetectedObsPosY[1] = 0.0f;
		gv->currentDetectedObsPosA[1] = 0.0f;
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[1],
				fo->evaluatedDetections[5].x, fo->evaluatedDetections[5].y,
				3.0f, fo->evaluatedDetections[5].z, 0, 0);
		gv->currentDetectedObsPosX[1] = fo->evaluatedDetections[5].x;
		gv->currentDetectedObsPosY[1] = fo->evaluatedDetections[5].y;
		gv->currentDetectedObsPosA[1] = fo->evaluatedDetections[5].z;
	}
	if (fo->detectionCounts[6] == 0) {
		cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f, 10.0e5,
				0, 0, 0);
		gv->currentDetectedObsPosX[2] = 0.0f;
		gv->currentDetectedObsPosY[2] = 0.0f;
		gv->currentDetectedObsPosA[2] = 0.0f;
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[2],
				fo->evaluatedDetections[6].x, fo->evaluatedDetections[6].y,
				6.0f, fo->evaluatedDetections[6].z, 0, 0);
		gv->currentDetectedObsPosX[2] = fo->evaluatedDetections[6].x;
		gv->currentDetectedObsPosY[2] = fo->evaluatedDetections[6].y;
		gv->currentDetectedObsPosA[2] = fo->evaluatedDetections[6].z;
	}
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
			&gv->distortedObstacles[0], 0);
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
			&gv->distortedObstacles[1], gv->tempCounter);
	hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2],
			gv->tempCounter);
	if (gv->MT1posChanged) {
		gv->realExecPhase = 1;
		cd->overrideCollider(gv->workpieceMesh, &gv->colliders[15]);
		cd->recalculateCollider(gv->workpieceMesh, gv->mtPositionsX[0],
				gv->mtPositionsY[0], gv->objectOffsets[4],
				gv->mtPositionsAngles[0], 0.0f, 0.0f);
		hf->calcEdgesForMesh(gv->workpiece_mesh_edges, gv->workpieceMesh, 0);
	} else {
		std::cout << "\nReal exec. info: MT 1 can stay at the current position"
				<< std::endl;
		if (gv->MT2posChanged) {
			gv->realExecPhase = 4;
			cd->overrideCollider(gv->workpieceMesh, &gv->colliders[16]);
			cd->recalculateCollider(gv->workpieceMesh, gv->mtPositionsX[1],
					gv->mtPositionsY[1], gv->objectOffsets[5],
					gv->mtPositionsAngles[1], 0.0f, 0.0f);
			hf->calcEdgesForMesh(gv->workpiece_mesh_edges, gv->workpieceMesh,
					0);
		} else {
			gv->realExecPhase = 7;
			gv->realExecSubPhase = 0;
			std::cout << "Real exec. info: MT 2 can stay at the current position"
					<< std::endl;
		}
	}
	repaint = true;
	return repaint;
}


bool RealRobotExecution::position_monitoring_tools1(bool repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	CollisionDetection * cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

	// position monitoring tools
	// -------------------------
	std::cout << "\nReal exec. info: MT 1 has to be moved to the position "
			<< gv->mtPositionsX[0] << "cm (x)  " << gv->mtPositionsY[0]
			<< "cm (y)  " << gv->mtPositionsAngles[0] * 180.0f / M_PI
			<< " degree (angle)" << std::endl;

	if (fo->checkMTTransport(1, generator1, distribution1, generator2,
			distribution2, generator3, distribution3)) {
		cout
				<< "Real exec. info: MT 1 can be moved by the robot arm himself. Execute (press y/n)?"
				<< endl;
		char c_in = hf->wait_for_user_input_yn();
		if (c_in == 'y') {
			gv->realExecPhase = 2;
			gv->realExecSubPhase = 0;
			gv->MT1transported = true;
		} else {
			cout
					<< "Real exec. info: no automatic repositioning. Manually move the MT to the shown position..."
					<< endl;
			gv->realExecPhase = 3;
			gv->MT1transported = false;
		}
	} else {
		cout
				<< "Real exec. info: MT 1 cannot be moved by the robot arm. Manually move the MT to the shown position..."
				<< endl;
		gv->realExecPhase = 3;
		gv->MT1transported = false;
	}

	if (gv->realExecPhase == 3) {
		cd->recalculateCollider(&gv->colliders[15], gv->mtPositionsX[0],
				gv->mtPositionsY[0], gv->objectOffsets[4],
				gv->mtPositionsAngles[0], 0, 0, false, true, gv->camVectorsMT1);
		if (gv->virtualMode) {
			cout << "Real exe. info: press space to continue..." << endl;

			char c_in = hf->wait_for_user_input_space();

			gv->MT1collision = false;
			for (int i = 8; i <= 16; i++) {
				if (i != 15) {
					if (cd->checkForCollision(&gv->colliders[15],
							&gv->colliders[i])) {
						gv->MT1collision = true;
						break;
					}
				}
			}
			if (!gv->MT1collision
					&& cd->checkForCollision(&gv->colliders[15],
							&gv->colliders[1])) {
				gv->MT1collision = true;
			}
		} else {
			std::cout
					<< "Real exe. info: MT 1 repositioning collision-free possible (press y/n)?"
					<< std::endl;
			char c_in = hf->wait_for_user_input_yn();

			if (c_in == 'y') {
				gv->MT1collision = false;
			} else {
				gv->MT1collision = true;
			}
		}
		if (gv->MT1collision) {
			cout
					<< "Real exec. info: real transport failed, MT 1 collided with environment"
					<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			gv->transportPhase = 0;
			repaint = false;
		} else {
			gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges,
					gv->numPointsMTrays, 0);
			gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices,
					gv->numPointsMTrays, 0);
			gv->numPointsMTrays = 0;
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
			gv->tempCounter = gv->totalPlatePoints;
			for (int i = 0; i < 15; i++) {
				gv->tempCounter += gv->colliders[i].facesTimes3;
			}
			hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[15],
					gv->tempCounter);
			gv->initPositionsX[4] = gv->mtPositionsX[0];
			gv->initPositionsY[4] = gv->mtPositionsY[0];
			gv->initPositionsAngles[4] = gv->mtPositionsAngles[0];
			repaint = true;
		}
	}
	return repaint;
}

bool RealRobotExecution::execute_real_phase2(bool repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	CollisionDetection * cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();
	MonitoringToolModels *mt = MonitoringToolModels::get_instance();

	if (gv->realExecSubPhase == 0) {
		gv->transportPhase = 0;
		gv->grippingWidth = gv->grippingWidthOpen;
		gv->objectPosReachable = false;
		for (int i = 0; i < 10; i++) {
			if (pp->calcPath(gv->qValuesObjectStart, gv->qValuesStandby,
					&gv->pathStartPos, &gv->standbyPos, false, generator1,
					distribution1, generator2, distribution2, generator3,
					distribution3)) {
				gv->objectPosReachable = true;
				break;
			}
		}
		if (gv->objectPosReachable) {
			gv->atStandby = false;
			pp->optimizePath();
			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
					gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
					gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;
			gv->real_path_edges = ik->cutOff(gv->real_path_edges,
					gv->numPointsRealPath, 0);
			gv->real_path_vertices = ik->cutOff(gv->real_path_vertices,
					gv->numPointsRealPath, 0);
			gv->numPointsRealPath = 0;
			fo->drawPathOnly(2);
			gv->pathLastPos = collider::Edge { 0.0f, 0.0f, 0.0f };
			fo->writeIntoTempSegments();
			gv->pathDataForKuka = "normal,";
			for (int i = 0; i < gv->tempSegSize; i++) {
				gv->pathDataForKuka += hf->to_string_with_precision(
						gv->q0ValsTempSeg[i], 4) + ","
						+ hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4)
						+ ",";
			}
			gv->pathDataForKuka += hf->to_string_with_precision(
					gv->grippingWidth, 4);
			gv->realGripperWidth = gv->grippingWidthOpen;
			gv->realExecSubPhase++;
			repaint = true;
		} else {
			cout
					<< "Real exec. info: first gv->path planning and real MT transport failed!"
					<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			repaint = false;
		}
	} else if (gv->realExecSubPhase == 2) {
		gv->transportPhase = 1;
		gv->grippingWidth = gv->grippingWidthFixed;
		gv->objectPosReachable = false;
		for (int i = 0; i < 10; i++) {
			if (pp->calcPath(gv->qValuesObjectEnd, gv->qValuesObjectStart,
					&gv->pathEndPos, &gv->pathStartPos, false, generator1,
					distribution1, generator2, distribution2, generator3,
					distribution3)) {
				gv->objectPosReachable = true;
				break;
			}
		}
		if (gv->objectPosReachable) {
			gv->currentlyTransporting = true;
			cd->recalculateCollider(gv->safetyCollider, 0.0f, 0.0f, 10.0e5, 0,
					0, 0);
			hf->calcEdgesForMesh(gv->safety_col_edges, gv->safetyCollider, 0);
			gv->MT1currentlyActive = false;
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
			pp->optimizePath();
			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
					gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
					gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;
			gv->real_path_edges = ik->cutOff(gv->real_path_edges,
					gv->numPointsRealPath, 0);
			gv->real_path_vertices = ik->cutOff(gv->real_path_vertices,
					gv->numPointsRealPath, 0);
			gv->numPointsRealPath = 0;
			fo->drawPathOnly(2);
			gv->pathLastPos = collider::Edge { 0.0f, 0.0f, 0.0f };
			fo->writeIntoTempSegments();
			gv->pathDataForKuka = "normal,";
			for (int i = 0; i < gv->tempSegSize; i++) {
				gv->pathDataForKuka += hf->to_string_with_precision(
						gv->q0ValsTempSeg[i], 4) + ","
						+ hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4)
						+ ",";
			}
			gv->pathDataForKuka += hf->to_string_with_precision(
					gv->grippingWidth, 4);
			gv->realGripperWidth = gv->grippingWidthFixed;
			gv->realExecSubPhase++;
			repaint = true;
		} else {
			gv->robotPoseAtAbort[0] = gv->pathStartPos.x;
			gv->robotPoseAtAbort[1] = gv->pathStartPos.y;
			gv->robotPoseAtAbort[2] = gv->pathStartPos.z;
			gv->robotPoseAtAbort[3] = gv->qValuesObjectStart[0];
			gv->robotPoseAtAbort[4] = gv->qValuesObjectStart[1];
			gv->robotPoseAtAbort[5] = gv->qValuesObjectStart[2];
			gv->robotPoseAtAbort[6] = gv->qValuesObjectStart[3];
			gv->robotPoseAtAbort[7] = gv->qValuesObjectStart[4];
			cout
					<< "Real exec. info: second gv->path planning and real MT transport failed!"
					<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			gv->transportPhase = 0;
			repaint = false;
		}
	} else if (gv->realExecSubPhase == 4) {
		gv->transportPhase = 2;
		gv->grippingWidth = gv->grippingWidthOpen;
		gv->objectPosReachable = false;
		for (int i = 0; i < 10; i++) {
			if (pp->calcPath(gv->qValuesStandby, gv->qValuesObjectEnd,
					&gv->standbyPos, &gv->pathEndPos, false, generator1,
					distribution1, generator2, distribution2, generator3,
					distribution3)) {
				gv->objectPosReachable = true;
				break;
			}
		}
		if (gv->objectPosReachable) {
			gv->currentlyTransporting = false;
			gv->MT1currentlyActive = true;
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
			pp->optimizePath();
			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
					gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
					gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;
			gv->real_path_edges = ik->cutOff(gv->real_path_edges,
					gv->numPointsRealPath, 0);
			gv->real_path_vertices = ik->cutOff(gv->real_path_vertices,
					gv->numPointsRealPath, 0);
			gv->numPointsRealPath = 0;
			fo->drawPathOnly(2);
			gv->pathLastPos = collider::Edge { 0.0f, 0.0f, 0.0f };
			fo->writeIntoTempSegments();
			gv->pathDataForKuka = "normal,";
			for (int i = 0; i < gv->tempSegSize; i++) {
				gv->pathDataForKuka += hf->to_string_with_precision(
						gv->q0ValsTempSeg[i], 4) + ","
						+ hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4)
						+ ",";
			}
			gv->pathDataForKuka += hf->to_string_with_precision(
					gv->grippingWidth, 4);
			gv->realGripperWidth = gv->grippingWidthOpen;
			gv->realExecSubPhase++;
			repaint = true;
		} else {
			gv->robotPoseAtAbort[0] = gv->pathEndPos.x;
			gv->robotPoseAtAbort[1] = gv->pathEndPos.y;
			gv->robotPoseAtAbort[2] = gv->pathEndPos.z;
			gv->robotPoseAtAbort[3] = gv->qValuesObjectEnd[0];
			gv->robotPoseAtAbort[4] = gv->qValuesObjectEnd[1];
			gv->robotPoseAtAbort[5] = gv->qValuesObjectEnd[2];
			gv->robotPoseAtAbort[6] = gv->qValuesObjectEnd[3];
			gv->robotPoseAtAbort[7] = gv->qValuesObjectEnd[4];
			hf->storePathRelatedData(1);
			gv->MT1currentlyActiveTemp = gv->MT1currentlyActive;
			gv->MT2currentlyActiveTemp = gv->MT2currentlyActive;
			cout
					<< "Real exec. info: third gv->path planning and real MT transport failed!"
					<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			gv->transportPhase = 0;
			repaint = false;
		}
	} else if (gv->realExecSubPhase == 6) {
		cout << "Real exec. info: MT transport was successful" << endl;
		gv->atStandby = true;
		gv->realExecPhase = 3;
	} else {
		if (gv->virtualMode) {
		} else {
		}
		vector<string> splitList;
		std::stringstream string_stream;
		string splitted;
		string_stream.str(gv->stateMessageSTATE);
		while (getline(string_stream, splitted, ',')) {
			splitList.push_back(splitted);
		}
		gv->realNodeIndex = stoi(splitList[splitList.size() - 1]);
		hf->syncVisualization(splitList);
		if (pp->nodeIndex != 1) {
			gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges,
					gv->numPointsMTrays, 0);
			gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices,
					gv->numPointsMTrays, 0);
			gv->numPointsMTrays = 0;
			if (gv->virtualMode) {
				fo->resetDetectedPatternsAlt();
				for (int i = 0; i < totalPatternCount; i++) {
					if (mt->isPatternVisible(i, 1, true)) {
						hf->getMTPositionData(1);
						fo->calcPosFromMTData(&gv->colliders[15],
								fo->distortedRelativePosition[0],
								fo->distortedRelativePosition[1],
								fo->distortedRelativePosition[2],
								fo->distortedRelativePosition[3], i);
						fo->insertDetectedPatternAlt(
								objectIndicesFromPattern[i] - 8);
						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
								gv->numPointsMTrays, 2);
						gv->mt_rays_vertices = ik->increaseSize(
								gv->mt_rays_vertices, gv->numPointsMTrays, 2);
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								fo->distortedPatternCenter[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								fo->distortedPatternCenter[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								gv->camVectorsMT1[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								gv->camVectorsMT1[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								gv->camVectorsMT1[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
					if (mt->isPatternVisible(i, 2, true)) {
						hf->getMTPositionData(2);
						fo->calcPosFromMTData(&gv->colliders[16],
								fo->distortedRelativePosition[0],
								fo->distortedRelativePosition[1],
								fo->distortedRelativePosition[2],
								fo->distortedRelativePosition[3], i);
						fo->insertDetectedPatternAlt(
								objectIndicesFromPattern[i] - 8);
						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
								gv->numPointsMTrays, 2);
						gv->mt_rays_vertices = ik->increaseSize(
								gv->mt_rays_vertices, gv->numPointsMTrays, 2);
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								fo->distortedPatternCenter[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								fo->distortedPatternCenter[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								gv->camVectorsMT2[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								gv->camVectorsMT2[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								gv->camVectorsMT2[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
				}
				fo->evaluateDetectionsAlt();
			} else {
				fo->resetDetectedPatternsAlt();
				if (gv->MT1currentlyActive) {
					vector<string> splitList1;
					std::stringstream string_stream_1;
					string_stream_1.str(gv->stateMessageMT1);
					while (getline(string_stream_1, splitted, ',')) {
						splitList1.push_back(splitted);
					}
					if (splitList1.size() > 0) {
						for (int i = 0;
								i < stoi(splitList1[splitList1.size() - 1]) * 5;
								i += 5) {
							fo->distortedRelativePosition[0] = stof(
									splitList1[i + 1]);
							fo->distortedRelativePosition[1] = stof(
									splitList1[i + 2]);
							fo->distortedRelativePosition[2] = stof(
									splitList1[i + 3]);
							fo->distortedRelativePosition[3] = stof(
									splitList1[i + 4]);
							fo->calcPosFromMTData(&gv->colliders[15],
									fo->distortedRelativePosition[0],
									fo->distortedRelativePosition[1],
									fo->distortedRelativePosition[2],
									fo->distortedRelativePosition[3],
									stoi(splitList1[i]));
							fo->insertDetectedPatternAlt(
									objectIndicesFromPattern[stoi(splitList1[i])]
											- 8);
							gv->mt_rays_edges = ik->increaseSize(
									gv->mt_rays_edges, gv->numPointsMTrays, 2);
							gv->mt_rays_vertices = ik->increaseSize(
									gv->mt_rays_vertices, gv->numPointsMTrays,
									2);
							gv->mt_rays_edges[gv->numPointsMTrays].x =
									fo->distortedPatternCenter[0];
							gv->mt_rays_edges[gv->numPointsMTrays].y =
									fo->distortedPatternCenter[1];
							gv->mt_rays_edges[gv->numPointsMTrays].z =
									fo->distortedPatternCenter[2];
							gv->mt_rays_vertices[gv->numPointsMTrays] =
									collider::Vertex { 0.0f, 0.0f, 0.0f,
											gv->otherColors[4].x,
											gv->otherColors[4].y,
											gv->otherColors[4].z, 1.0f };
							gv->numPointsMTrays++;
							gv->mt_rays_edges[gv->numPointsMTrays].x =
									gv->camVectorsMT1[0];
							gv->mt_rays_edges[gv->numPointsMTrays].y =
									gv->camVectorsMT1[1];
							gv->mt_rays_edges[gv->numPointsMTrays].z =
									gv->camVectorsMT1[2];
							gv->mt_rays_vertices[gv->numPointsMTrays] =
									collider::Vertex { 0.0f, 0.0f, 0.0f,
											gv->otherColors[4].x,
											gv->otherColors[4].y,
											gv->otherColors[4].z, 1.0f };
							gv->numPointsMTrays++;
						}
					}
				}
				vector<string> splitList2;
				std::stringstream string_stream_2;
				string_stream_2.str(gv->stateMessageMT2);
				while (getline(string_stream_2, splitted, ',')) {
					splitList2.push_back(splitted);
				}
				if (splitList2.size() > 0) {
					for (int i = 0;
							i < stoi(splitList2[splitList2.size() - 1]) * 5;
							i += 5) {
						fo->distortedRelativePosition[0] = stof(
								splitList2[i + 1]);
						fo->distortedRelativePosition[1] = stof(
								splitList2[i + 2]);
						fo->distortedRelativePosition[2] = stof(
								splitList2[i + 3]);
						fo->distortedRelativePosition[3] = stof(
								splitList2[i + 4]);
						fo->calcPosFromMTData(&gv->colliders[16],
								fo->distortedRelativePosition[0],
								fo->distortedRelativePosition[1],
								fo->distortedRelativePosition[2],
								fo->distortedRelativePosition[3],
								stoi(splitList2[i]));
						fo->insertDetectedPatternAlt(
								objectIndicesFromPattern[stoi(splitList2[i])]
										- 8);
						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
								gv->numPointsMTrays, 2);
						gv->mt_rays_vertices = ik->increaseSize(
								gv->mt_rays_vertices, gv->numPointsMTrays, 2);
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								fo->distortedPatternCenter[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								fo->distortedPatternCenter[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								gv->camVectorsMT2[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								gv->camVectorsMT2[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								gv->camVectorsMT2[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
				}
				fo->evaluateDetectionsAlt();
			}
			if (fo->detectionCounts[4] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f,
						10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[0] = 0.0f;
				gv->currentDetectedObsPosY[0] = 0.0f;
				gv->currentDetectedObsPosA[0] = 0.0f;
			} else {
				cd->recalculateCollider(&gv->distortedObstacles[0],
						fo->evaluatedDetections[4].x,
						fo->evaluatedDetections[4].y, 4.0f,
						fo->evaluatedDetections[4].z, 0, 0);
				gv->currentDetectedObsPosX[0] = fo->evaluatedDetections[4].x;
				gv->currentDetectedObsPosY[0] = fo->evaluatedDetections[4].y;
				gv->currentDetectedObsPosA[0] = fo->evaluatedDetections[4].z;
			}
			if (fo->detectionCounts[5] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f,
						10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[1] = 0.0f;
				gv->currentDetectedObsPosY[1] = 0.0f;
				gv->currentDetectedObsPosA[1] = 0.0f;
			} else {
				cd->recalculateCollider(&gv->distortedObstacles[1],
						fo->evaluatedDetections[5].x,
						fo->evaluatedDetections[5].y, 3.0f,
						fo->evaluatedDetections[5].z, 0, 0);
				gv->currentDetectedObsPosX[1] = fo->evaluatedDetections[5].x;
				gv->currentDetectedObsPosY[1] = fo->evaluatedDetections[5].y;
				gv->currentDetectedObsPosA[1] = fo->evaluatedDetections[5].z;
			}
			if (fo->detectionCounts[6] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f,
						10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[2] = 0.0f;
				gv->currentDetectedObsPosY[2] = 0.0f;
				gv->currentDetectedObsPosA[2] = 0.0f;
			} else {
				cd->recalculateCollider(&gv->distortedObstacles[2],
						fo->evaluatedDetections[6].x,
						fo->evaluatedDetections[6].y, 6.0f,
						fo->evaluatedDetections[6].z, 0, 0);
				gv->currentDetectedObsPosX[2] = fo->evaluatedDetections[6].x;
				gv->currentDetectedObsPosY[2] = fo->evaluatedDetections[6].y;
				gv->currentDetectedObsPosA[2] = fo->evaluatedDetections[6].z;
			}
			gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[0], 0);
			gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[1], gv->tempCounter);
			hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[2], gv->tempCounter);
			if (gv->realNodeIndex == gv->tempSegSize) {
				gv->syncSim = false;
				if (gv->realExecSubPhase == 3) {
					gv->initPositionsX[4] = gv->mtPositionsX[0];
					gv->initPositionsY[4] = gv->mtPositionsY[0];
					gv->initPositionsAngles[4] = gv->mtPositionsAngles[0];
					cd->recalculateCollider(&gv->colliders[15],
							gv->mtPositionsX[0], gv->mtPositionsY[0],
							gv->objectOffsets[4], gv->mtPositionsAngles[0], 0,
							0, false, true, gv->camVectorsMT1);
					gv->tempCounter = gv->totalPlatePoints;
					for (int i = 0; i < 15; i++) {
						gv->tempCounter += gv->colliders[i].facesTimes3;
					}
					hf->calcEdgesForConvexHull(gv->object_edges,
							&gv->colliders[15], gv->tempCounter);
				}
				gv->realExecSubPhase++;
			} else if (gv->realNodeIndex > 0) {
				if (!gv->syncSim)
					gv->syncSim = true;
			}

			repaint = true;
		} else
			gv->syncSim = false;
	}

	return repaint;
}





bool RealRobotExecution::real_exec_phase3(bool repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{

	GlobalVariables *gv=GlobalVariables::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	MonitoringToolModels *mt = MonitoringToolModels::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	CollisionDetection * cd = CollisionDetection::get_instance();
	Protagonists *prot =Protagonists::get_instance();




	if (gv->MT1transported) {
		gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges, gv->numPointsMTrays,
				0);
		gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices,
				gv->numPointsMTrays, 0);
		gv->numPointsMTrays = 0;
	}
	if (gv->virtualMode) {
		for (int i = 0; i < totalPatternCount; i++) {
			if (mt->isPatternVisible(i, 1, true)) {
				hf->getMTPositionData(1);
				fo->calcPosFromMTData(&gv->colliders[15],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], i);
				fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
			if (mt->isPatternVisible(i, 2, true)) {
				hf->getMTPositionData(2);
				fo->calcPosFromMTData(&gv->colliders[16],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], i);
				// fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
		}
		fo->evaluateDetections(false);
	} else {
		vector<string> splitList1;
		vector<string> splitList2;
		std::stringstream string_stream_1;
		std::stringstream string_stream_2;
		string splitted;
		string_stream_1.str(gv->stateMessageMT1);
		while (getline(string_stream_1, splitted, ',')) {
			splitList1.push_back(splitted);
		}
		string_stream_2.str(gv->stateMessageMT2);
		while (getline(string_stream_2, splitted, ',')) {
			splitList2.push_back(splitted);
		}
		if (splitList1.size() > 0) {
			for (int i = 0; i < stoi(splitList1[splitList1.size() - 1]) * 5;
					i += 5) {
				fo->distortedRelativePosition[0] = stof(splitList1[i + 1]);
				fo->distortedRelativePosition[1] = stof(splitList1[i + 2]);
				fo->distortedRelativePosition[2] = stof(splitList1[i + 3]);
				fo->distortedRelativePosition[3] = stof(splitList1[i + 4]);
				fo->calcPosFromMTData(&gv->colliders[15],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], stoi(splitList1[i]));
				fo->insertDetectedPattern(
						objectIndicesFromPattern[stoi(splitList1[i])] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
		}
		if (splitList2.size() > 0) {
			for (int i = 0; i < stoi(splitList2[splitList2.size() - 1]) * 5;
					i += 5) {
				fo->distortedRelativePosition[0] = stof(splitList2[i + 1]);
				fo->distortedRelativePosition[1] = stof(splitList2[i + 2]);
				fo->distortedRelativePosition[2] = stof(splitList2[i + 3]);
				fo->distortedRelativePosition[3] = stof(splitList2[i + 4]);
				fo->calcPosFromMTData(&gv->colliders[16],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], stoi(splitList2[i]));
				// fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList2[i])] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
						0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
						gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
		}
		fo->evaluateDetections(false);
	}
	if (fo->detectionCounts[4] == 0) {
		cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f, 10.0e5,
				0, 0, 0);
		gv->currentDetectedObsPosX[0] = 0.0f;
		gv->currentDetectedObsPosY[0] = 0.0f;
		gv->currentDetectedObsPosA[0] = 0.0f;
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[0],
				fo->evaluatedDetections[4].x, fo->evaluatedDetections[4].y,
				4.0f, fo->evaluatedDetections[4].z, 0, 0);
		gv->currentDetectedObsPosX[0] = fo->evaluatedDetections[4].x;
		gv->currentDetectedObsPosY[0] = fo->evaluatedDetections[4].y;
		gv->currentDetectedObsPosA[0] = fo->evaluatedDetections[4].z;
	}
	if (fo->detectionCounts[5] == 0) {
		cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f, 10.0e5,
				0, 0, 0);
		gv->currentDetectedObsPosX[1] = 0.0f;
		gv->currentDetectedObsPosY[1] = 0.0f;
		gv->currentDetectedObsPosA[1] = 0.0f;
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[1],
				fo->evaluatedDetections[5].x, fo->evaluatedDetections[5].y,
				3.0f, fo->evaluatedDetections[5].z, 0, 0);
		gv->currentDetectedObsPosX[1] = fo->evaluatedDetections[5].x;
		gv->currentDetectedObsPosY[1] = fo->evaluatedDetections[5].y;
		gv->currentDetectedObsPosA[1] = fo->evaluatedDetections[5].z;
	}
	if (fo->detectionCounts[6] == 0) {
		cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f, 10.0e5,
				0, 0, 0);
		gv->currentDetectedObsPosX[2] = 0.0f;
		gv->currentDetectedObsPosY[2] = 0.0f;
		gv->currentDetectedObsPosA[2] = 0.0f;
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[2],
				fo->evaluatedDetections[6].x, fo->evaluatedDetections[6].y,
				6.0f, fo->evaluatedDetections[6].z, 0, 0);
		gv->currentDetectedObsPosX[2] = fo->evaluatedDetections[6].x;
		gv->currentDetectedObsPosY[2] = fo->evaluatedDetections[6].y;
		gv->currentDetectedObsPosA[2] = fo->evaluatedDetections[6].z;
	}
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
			&gv->distortedObstacles[0], 0);
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
			&gv->distortedObstacles[1], gv->tempCounter);
	hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2],
			gv->tempCounter);
	if (gv->MT2posChanged) {
		gv->realExecPhase = 4;
		gv->oldObstaclePositions[0] = gv->colliders[12].offsets[0];
		gv->oldObstaclePositions[1] = gv->colliders[12].offsets[1];
		gv->oldObstaclePositions[2] = gv->colliders[12].angles[0];
		gv->oldObstaclePositions[3] = gv->colliders[13].offsets[0];
		gv->oldObstaclePositions[4] = gv->colliders[13].offsets[1];
		gv->oldObstaclePositions[5] = gv->colliders[13].angles[0];
		gv->oldObstaclePositions[6] = gv->colliders[14].offsets[0];
		gv->oldObstaclePositions[7] = gv->colliders[14].offsets[1];
		gv->oldObstaclePositions[8] = gv->colliders[14].angles[0];
		gv->oldMTPos[0] = gv->colliders[16].offsets[0];
		gv->oldMTPos[1] = gv->colliders[16].offsets[1];
		gv->oldMTPos[2] = gv->colliders[16].angles[0];
		srand(1);
		generator1.seed(1);
		prot->mt_preparation(2, generator1, distribution1, generator2,
				distribution2, generator3, distribution3);
		srand((unsigned) (time(NULL)));
		generator1.seed((unsigned) (time(NULL)));
		cd->recalculateCollider(&gv->colliders[12], gv->oldObstaclePositions[0],
				gv->oldObstaclePositions[1], 4.0f, gv->oldObstaclePositions[2],
				0, 0, true);
		cd->recalculateCollider(&gv->colliders[13], gv->oldObstaclePositions[3],
				gv->oldObstaclePositions[4], 3.0f, gv->oldObstaclePositions[5],
				0, 0, true);
		cd->recalculateCollider(&gv->colliders[14], gv->oldObstaclePositions[6],
				gv->oldObstaclePositions[7], 6.0f, gv->oldObstaclePositions[8],
				0, 0, true);
		cd->recalculateCollider(&gv->colliders[16], gv->oldMTPos[0],
				gv->oldMTPos[1], gv->objectOffsets[5], gv->oldMTPos[2], 0, 0,
				false, true, gv->camVectorsMT2);
		prot->mt_protagonist(2, gv->objectIndex, true);
		gv->mtPositionsX[1] = gv->tempMTPos[0];
		gv->mtPositionsY[1] = gv->tempMTPos[1];
		gv->mtPositionsAngles[1] = gv->tempMTPos[2];
		cd->overrideCollider(gv->workpieceMesh, &gv->colliders[16]);
		cd->recalculateCollider(gv->workpieceMesh, gv->mtPositionsX[1],
				gv->mtPositionsY[1], gv->objectOffsets[5],
				gv->mtPositionsAngles[1], 0.0f, 0.0f);
		hf->calcEdgesForMesh(gv->workpiece_mesh_edges, gv->workpieceMesh, 0);
	} else {
		cout << "Real exec. info: MT 2 can stay at the current position"
				<< endl;
		gv->realExecPhase = 7;
		gv->realExecSubPhase = 0;
	}
	repaint = true;
	return repaint;
}

bool RealRobotExecution::real_exec_phase4(bool repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3)
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	CollisionDetection * cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

	cout << "Real exec. info: MT 2 has to be moved to the position "
			<< gv->mtPositionsX[1] << "cm (x)  " << gv->mtPositionsY[1]
																	 << "cm (y)  " << gv->mtPositionsAngles[1] * 180.0f / M_PI
																	 << " degree (angle)" << endl;
	gv->mtPositionsX[1] = 25.0f;
	gv->mtPositionsY[1] = 0.0f;
	gv->mtPositionsAngles[1] = M_PI / 2;
	if (fo->checkMTTransport(2, generator1, distribution1, generator2,
			distribution2, generator3, distribution3)) {
		cout
		<< "Real exec. info: MT 2 can be moved by the robot arm himself. Execute (press y/n)?"
		<< endl;
		char c_in = '-';
		SDL_Event event_sub;
		do {
			while (SDL_PollEvent(&event_sub)) {
				if (event_sub.type == SDL_KEYUP) {
					if (event_sub.key.keysym.sym == SDLK_y) {
						c_in = 'y';
					} else if (event_sub.key.keysym.sym == SDLK_n) {
						c_in = 'n';
					}
				}
			}
		} while (c_in == '-');
		if (c_in == 'y') {
			gv->realExecPhase = 5;
			gv->realExecSubPhase = 0;
			gv->MT2transported = true;
		} else {
			cout
			<< "Real exec. info: no automatic repositioning. Manually move the MT to the shown position..."
			<< endl;
			gv->realExecPhase = 6;
			gv->MT2transported = false;
		}
	} else {
		cout
		<< "Real exec. info: MT 2 cannot be moved by the robot arm. Manually move the MT to the shown position..."
		<< endl;
		gv->realExecPhase = 6;
		gv->MT2transported = false;
	}
	if (gv->realExecPhase == 6) {
		cd->recalculateCollider(&gv->colliders[16], gv->mtPositionsX[1],
				gv->mtPositionsY[1], gv->objectOffsets[5],
				gv->mtPositionsAngles[1], 0, 0, false, true, gv->camVectorsMT2);
		if (gv->virtualMode) {
			cout << "Real exe. info: press space to continue..." << endl;
			char c_in = '-';
			SDL_Event event_sub;
			do {
				while (SDL_PollEvent(&event_sub)) {
					if (event_sub.type == SDL_KEYUP) {
						if (event_sub.key.keysym.sym == SDLK_SPACE) {
							c_in = 's';
						}
					}
				}
			} while (c_in == '-');
			gv->MT2collision = false;
			for (int i = 8; i <= 15; i++) {
				if (cd->checkForCollision(&gv->colliders[16],
						&gv->colliders[i])) {
					gv->MT2collision = true;
					break;
				}
			}
			if (!gv->MT2collision
					&& cd->checkForCollision(&gv->colliders[16],
							&gv->colliders[1])) {
				gv->MT2collision = true;
			}
		} else {
			cout
			<< "Real exe. info: MT 2 repositioning gv->collision-free possible (press y/n)?"
			<< endl;
			char c_in = '-';
			SDL_Event event_sub;
			do {
				while (SDL_PollEvent(&event_sub)) {
					if (event_sub.type == SDL_KEYUP) {
						if (event_sub.key.keysym.sym == SDLK_y) {
							c_in = 'y';
						} else if (event_sub.key.keysym.sym == SDLK_n) {
							c_in = 'n';
						}
					}
				}
			} while (c_in == '-');
			if (c_in == 'y') {
				gv->MT2collision = false;
			} else {
				gv->MT2collision = true;
			}
		}
		if (gv->MT2collision) {
			cout
			<< "Real exec. info: real transport failed, MT 2 collided with environment"
			<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			gv->transportPhase = 0;
			repaint = false;
		} else {
			gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges,
					gv->numPointsMTrays, 0);
			gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices,
					gv->numPointsMTrays, 0);
			gv->numPointsMTrays = 0;
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
			gv->tempCounter = gv->totalPlatePoints;
			for (int i = 0; i < 16; i++) {
				gv->tempCounter += gv->colliders[i].facesTimes3;
			}
			hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[16],
					gv->tempCounter);
			gv->initPositionsX[5] = gv->mtPositionsX[1];
			gv->initPositionsY[5] = gv->mtPositionsY[1];
			gv->initPositionsAngles[5] = gv->mtPositionsAngles[1];
			repaint = true;
		}
	}
	return repaint;
}

bool RealRobotExecution::real_exec_phase5(bool repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	CollisionDetection * cd = CollisionDetection::get_instance();
	MonitoringToolModels *mt=MonitoringToolModels::get_instance();


	if (gv->realExecSubPhase == 0) {
		gv->transportPhase = 0;
		gv->grippingWidth = gv->grippingWidthOpen;
		gv->objectPosReachable = false;
		for (int i = 0; i < 10; i++) {
			if (pp->calcPath(gv->qValuesObjectStart, gv->qValuesStandby,
					&gv->pathStartPos, &gv->standbyPos, false, generator1,
					distribution1, generator2, distribution2, generator3,
					distribution3)) {
				gv->objectPosReachable = true;
				break;
			}
		}
		if (gv->objectPosReachable) {
			gv->atStandby = false;
			pp->optimizePath();
			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
					gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
					gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;
			gv->real_path_edges = ik->cutOff(gv->real_path_edges,
					gv->numPointsRealPath, 0);
			gv->real_path_vertices = ik->cutOff(gv->real_path_vertices,
					gv->numPointsRealPath, 0);
			gv->numPointsRealPath = 0;
			fo->drawPathOnly(2);
			gv->pathLastPos = collider::Edge { 0.0f, 0.0f, 0.0f };
			fo->writeIntoTempSegments();
			gv->pathDataForKuka = "normal,";
			for (int i = 0; i < gv->tempSegSize; i++) {
				gv->pathDataForKuka += hf->to_string_with_precision(
						gv->q0ValsTempSeg[i], 4) + ","
						+ hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4)
						+ ",";
			}
			gv->pathDataForKuka += hf->to_string_with_precision(
					gv->grippingWidth, 4);
			gv->realGripperWidth = gv->grippingWidthOpen;
			gv->realExecSubPhase++;
			repaint = true;
		} else {
			cout
					<< "Real exec. info: first gv->path planning and real MT transport failed!"
					<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			repaint = false;
		}
	} else if (gv->realExecSubPhase == 2) {
		gv->transportPhase = 1;
		gv->grippingWidth = gv->grippingWidthFixed;
		gv->objectPosReachable = false;
		for (int i = 0; i < 10; i++) {
			if (pp->calcPath(gv->qValuesObjectEnd, gv->qValuesObjectStart,
					&gv->pathEndPos, &gv->pathStartPos, false, generator1,
					distribution1, generator2, distribution2, generator3,
					distribution3)) {
				gv->objectPosReachable = true;
				break;
			}
		}
		if (gv->objectPosReachable) {
			gv->currentlyTransporting = true;
			cd->recalculateCollider(gv->safetyCollider, 0.0f, 0.0f, 10.0e5, 0,
					0, 0);
			hf->calcEdgesForMesh(gv->safety_col_edges, gv->safetyCollider, 0);
			gv->MT2currentlyActive = false;
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
			pp->optimizePath();
			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
					gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
					gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;
			gv->real_path_edges = ik->cutOff(gv->real_path_edges,
					gv->numPointsRealPath, 0);
			gv->real_path_vertices = ik->cutOff(gv->real_path_vertices,
					gv->numPointsRealPath, 0);
			gv->numPointsRealPath = 0;
			fo->drawPathOnly(2);
			gv->pathLastPos = collider::Edge { 0.0f, 0.0f, 0.0f };
			fo->writeIntoTempSegments();
			gv->pathDataForKuka = "normal,";
			for (int i = 0; i < gv->tempSegSize; i++) {
				gv->pathDataForKuka += hf->to_string_with_precision(
						gv->q0ValsTempSeg[i], 4) + ","
						+ hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4)
						+ ",";
			}
			gv->pathDataForKuka += hf->to_string_with_precision(
					gv->grippingWidth, 4);
			gv->realGripperWidth = gv->grippingWidthFixed;
			gv->realExecSubPhase++;
			repaint = true;
		} else {
			gv->robotPoseAtAbort[0] = gv->pathStartPos.x;
			gv->robotPoseAtAbort[1] = gv->pathStartPos.y;
			gv->robotPoseAtAbort[2] = gv->pathStartPos.z;
			gv->robotPoseAtAbort[3] = gv->qValuesObjectStart[0];
			gv->robotPoseAtAbort[4] = gv->qValuesObjectStart[1];
			gv->robotPoseAtAbort[5] = gv->qValuesObjectStart[2];
			gv->robotPoseAtAbort[6] = gv->qValuesObjectStart[3];
			gv->robotPoseAtAbort[7] = gv->qValuesObjectStart[4];
			cout
					<< "Real exec. info: second gv->path planning and real MT transport failed!"
					<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			gv->transportPhase = 0;
			repaint = false;
		}
	} else if (gv->realExecSubPhase == 4) {
		gv->transportPhase = 2;
		gv->grippingWidth = gv->grippingWidthOpen;
		gv->objectPosReachable = false;
		for (int i = 0; i < 10; i++) {
			if (pp->calcPath(gv->qValuesStandby, gv->qValuesObjectEnd,
					&gv->standbyPos, &gv->pathEndPos, false, generator1,
					distribution1, generator2, distribution2, generator3,
					distribution3)) {
				gv->objectPosReachable = true;
				break;
			}
		}
		if (gv->objectPosReachable) {
			gv->currentlyTransporting = false;
			gv->MT2currentlyActive = true;
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
			pp->optimizePath();
			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
					gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
					gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;
			gv->real_path_edges = ik->cutOff(gv->real_path_edges,
					gv->numPointsRealPath, 0);
			gv->real_path_vertices = ik->cutOff(gv->real_path_vertices,
					gv->numPointsRealPath, 0);
			gv->numPointsRealPath = 0;
			fo->drawPathOnly(2);
			gv->pathLastPos = collider::Edge { 0.0f, 0.0f, 0.0f };
			fo->writeIntoTempSegments();
			gv->pathDataForKuka = "normal,";
			for (int i = 0; i < gv->tempSegSize; i++) {
				gv->pathDataForKuka += hf->to_string_with_precision(
						gv->q0ValsTempSeg[i], 4) + ","
						+ hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4)
						+ ","
						+ hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4)
						+ ",";
			}
			gv->pathDataForKuka += hf->to_string_with_precision(
					gv->grippingWidth, 4);
			gv->realGripperWidth = gv->grippingWidthOpen;
			gv->realExecSubPhase++;
			repaint = true;
		} else {
			gv->robotPoseAtAbort[0] = gv->pathEndPos.x;
			gv->robotPoseAtAbort[1] = gv->pathEndPos.y;
			gv->robotPoseAtAbort[2] = gv->pathEndPos.z;
			gv->robotPoseAtAbort[3] = gv->qValuesObjectEnd[0];
			gv->robotPoseAtAbort[4] = gv->qValuesObjectEnd[1];
			gv->robotPoseAtAbort[5] = gv->qValuesObjectEnd[2];
			gv->robotPoseAtAbort[6] = gv->qValuesObjectEnd[3];
			gv->robotPoseAtAbort[7] = gv->qValuesObjectEnd[4];
			hf->storePathRelatedData(1);
			gv->MT1currentlyActiveTemp = gv->MT1currentlyActive;
			gv->MT2currentlyActiveTemp = gv->MT2currentlyActive;
			cout
					<< "Real exec. info: third gv->path planning and real MT transport failed!"
					<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			gv->transportPhase = 0;
			repaint = false;
		}
	} else if (gv->realExecSubPhase == 6) {
		cout << "Real exec. info: MT transport was successful" << endl;
		gv->atStandby = true;
		gv->realExecPhase = 6;
	} else {
		if (gv->virtualMode) {
		} else {
		}
		vector<string> splitList;
		std::stringstream string_stream;
		string splitted;
		string_stream.str(gv->stateMessageSTATE);
		while (getline(string_stream, splitted, ',')) {
			splitList.push_back(splitted);
		}
		gv->realNodeIndex = stoi(splitList[splitList.size() - 1]);
		hf->syncVisualization(splitList);
		if (pp->nodeIndex != 1) {
			gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges,
					gv->numPointsMTrays, 0);
			gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices,
					gv->numPointsMTrays, 0);
			gv->numPointsMTrays = 0;
			if (gv->virtualMode) {
				fo->resetDetectedPatternsAlt();
				for (int i = 0; i < totalPatternCount; i++) {
					if (mt->isPatternVisible(i, 1, true)) {
						hf->getMTPositionData(1);
						fo->calcPosFromMTData(&gv->colliders[15],
								fo->distortedRelativePosition[0],
								fo->distortedRelativePosition[1],
								fo->distortedRelativePosition[2],
								fo->distortedRelativePosition[3], i);
						fo->insertDetectedPatternAlt(
								objectIndicesFromPattern[i] - 8);
						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
								gv->numPointsMTrays, 2);
						gv->mt_rays_vertices = ik->increaseSize(
								gv->mt_rays_vertices, gv->numPointsMTrays, 2);
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								fo->distortedPatternCenter[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								fo->distortedPatternCenter[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								gv->camVectorsMT1[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								gv->camVectorsMT1[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								gv->camVectorsMT1[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
					if (mt->isPatternVisible(i, 2, true)) {
						hf->getMTPositionData(2);
						fo->calcPosFromMTData(&gv->colliders[16],
								fo->distortedRelativePosition[0],
								fo->distortedRelativePosition[1],
								fo->distortedRelativePosition[2],
								fo->distortedRelativePosition[3], i);
						fo->insertDetectedPatternAlt(
								objectIndicesFromPattern[i] - 8);
						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
								gv->numPointsMTrays, 2);
						gv->mt_rays_vertices = ik->increaseSize(
								gv->mt_rays_vertices, gv->numPointsMTrays, 2);
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								fo->distortedPatternCenter[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								fo->distortedPatternCenter[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								gv->camVectorsMT2[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								gv->camVectorsMT2[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								gv->camVectorsMT2[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
				}
				fo->evaluateDetectionsAlt();
			} else {
				fo->resetDetectedPatternsAlt();
				vector<string> splitList1;
				std::stringstream string_stream_1;
				string_stream_1.str(gv->stateMessageMT1);
				while (getline(string_stream_1, splitted, ',')) {
					splitList1.push_back(splitted);
				}
				if (splitList1.size() > 0) {
					for (int i = 0;
							i < stoi(splitList1[splitList1.size() - 1]) * 5;
							i += 5) {
						fo->distortedRelativePosition[0] = stof(
								splitList1[i + 1]);
						fo->distortedRelativePosition[1] = stof(
								splitList1[i + 2]);
						fo->distortedRelativePosition[2] = stof(
								splitList1[i + 3]);
						fo->distortedRelativePosition[3] = stof(
								splitList1[i + 4]);
						fo->calcPosFromMTData(&gv->colliders[15],
								fo->distortedRelativePosition[0],
								fo->distortedRelativePosition[1],
								fo->distortedRelativePosition[2],
								fo->distortedRelativePosition[3],
								stoi(splitList1[i]));
						fo->insertDetectedPatternAlt(
								objectIndicesFromPattern[stoi(splitList1[i])]
										- 8);
						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
								gv->numPointsMTrays, 2);
						gv->mt_rays_vertices = ik->increaseSize(
								gv->mt_rays_vertices, gv->numPointsMTrays, 2);
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								fo->distortedPatternCenter[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								fo->distortedPatternCenter[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								gv->camVectorsMT1[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								gv->camVectorsMT1[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								gv->camVectorsMT1[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
										gv->otherColors[4].x,
										gv->otherColors[4].y,
										gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
				}
				if (gv->MT2currentlyActive) {
					vector<string> splitList2;
					std::stringstream string_stream_2;
					string_stream_2.str(gv->stateMessageMT2);
					while (getline(string_stream_2, splitted, ',')) {
						splitList2.push_back(splitted);
					}
					if (splitList2.size() > 0) {
						for (int i = 0;
								i < stoi(splitList2[splitList2.size() - 1]) * 5;
								i += 5) {
							fo->distortedRelativePosition[0] = stof(
									splitList2[i + 1]);
							fo->distortedRelativePosition[1] = stof(
									splitList2[i + 2]);
							fo->distortedRelativePosition[2] = stof(
									splitList2[i + 3]);
							fo->distortedRelativePosition[3] = stof(
									splitList2[i + 4]);
							fo->calcPosFromMTData(&gv->colliders[16],
									fo->distortedRelativePosition[0],
									fo->distortedRelativePosition[1],
									fo->distortedRelativePosition[2],
									fo->distortedRelativePosition[3],
									stoi(splitList2[i]));
							fo->insertDetectedPatternAlt(
									objectIndicesFromPattern[stoi(splitList2[i])]
											- 8);
							gv->mt_rays_edges = ik->increaseSize(
									gv->mt_rays_edges, gv->numPointsMTrays, 2);
							gv->mt_rays_vertices = ik->increaseSize(
									gv->mt_rays_vertices, gv->numPointsMTrays,
									2);
							gv->mt_rays_edges[gv->numPointsMTrays].x =
									fo->distortedPatternCenter[0];
							gv->mt_rays_edges[gv->numPointsMTrays].y =
									fo->distortedPatternCenter[1];
							gv->mt_rays_edges[gv->numPointsMTrays].z =
									fo->distortedPatternCenter[2];
							gv->mt_rays_vertices[gv->numPointsMTrays] =
									collider::Vertex { 0.0f, 0.0f, 0.0f,
											gv->otherColors[4].x,
											gv->otherColors[4].y,
											gv->otherColors[4].z, 1.0f };
							gv->numPointsMTrays++;
							gv->mt_rays_edges[gv->numPointsMTrays].x =
									gv->camVectorsMT2[0];
							gv->mt_rays_edges[gv->numPointsMTrays].y =
									gv->camVectorsMT2[1];
							gv->mt_rays_edges[gv->numPointsMTrays].z =
									gv->camVectorsMT2[2];
							gv->mt_rays_vertices[gv->numPointsMTrays] =
									collider::Vertex { 0.0f, 0.0f, 0.0f,
											gv->otherColors[4].x,
											gv->otherColors[4].y,
											gv->otherColors[4].z, 1.0f };
							gv->numPointsMTrays++;
						}
					}
				}
				fo->evaluateDetectionsAlt();
			}
			if (fo->detectionCounts[4] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f,
						10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[0] = 0.0f;
				gv->currentDetectedObsPosY[0] = 0.0f;
				gv->currentDetectedObsPosA[0] = 0.0f;
			} else {
				cd->recalculateCollider(&gv->distortedObstacles[0],
						fo->evaluatedDetections[4].x,
						fo->evaluatedDetections[4].y, 4.0f,
						fo->evaluatedDetections[4].z, 0, 0);
				gv->currentDetectedObsPosX[0] = fo->evaluatedDetections[4].x;
				gv->currentDetectedObsPosY[0] = fo->evaluatedDetections[4].y;
				gv->currentDetectedObsPosA[0] = fo->evaluatedDetections[4].z;
			}
			if (fo->detectionCounts[5] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f,
						10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[1] = 0.0f;
				gv->currentDetectedObsPosY[1] = 0.0f;
				gv->currentDetectedObsPosA[1] = 0.0f;
			} else {
				cd->recalculateCollider(&gv->distortedObstacles[1],
						fo->evaluatedDetections[5].x,
						fo->evaluatedDetections[5].y, 3.0f,
						fo->evaluatedDetections[5].z, 0, 0);
				gv->currentDetectedObsPosX[1] = fo->evaluatedDetections[5].x;
				gv->currentDetectedObsPosY[1] = fo->evaluatedDetections[5].y;
				gv->currentDetectedObsPosA[1] = fo->evaluatedDetections[5].z;
			}
			if (fo->detectionCounts[6] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f,
						10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[2] = 0.0f;
				gv->currentDetectedObsPosY[2] = 0.0f;
				gv->currentDetectedObsPosA[2] = 0.0f;
			} else {
				cd->recalculateCollider(&gv->distortedObstacles[2],
						fo->evaluatedDetections[6].x,
						fo->evaluatedDetections[6].y, 6.0f,
						fo->evaluatedDetections[6].z, 0, 0);
				gv->currentDetectedObsPosX[2] = fo->evaluatedDetections[6].x;
				gv->currentDetectedObsPosY[2] = fo->evaluatedDetections[6].y;
				gv->currentDetectedObsPosA[2] = fo->evaluatedDetections[6].z;
			}
			gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[0], 0);
			gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[1], gv->tempCounter);
			hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[2], gv->tempCounter);
			if (gv->realNodeIndex == gv->tempSegSize) {
				gv->syncSim = false;
				if (gv->realExecSubPhase == 3) {
					gv->initPositionsX[5] = gv->mtPositionsX[1];
					gv->initPositionsY[5] = gv->mtPositionsY[1];
					gv->initPositionsAngles[5] = gv->mtPositionsAngles[1];
					cd->recalculateCollider(&gv->colliders[16],
							gv->mtPositionsX[1], gv->mtPositionsY[1],
							gv->objectOffsets[5], gv->mtPositionsAngles[1], 0,
							0, false, true, gv->camVectorsMT2);
					gv->tempCounter = gv->totalPlatePoints;
					for (int i = 0; i < 16; i++) {
						gv->tempCounter += gv->colliders[i].facesTimes3;
					}
					hf->calcEdgesForConvexHull(gv->object_edges,
							&gv->colliders[16], gv->tempCounter);
				}
				gv->realExecSubPhase++;
			} else if (gv->realNodeIndex > 0) {
				if (!gv->syncSim)
					gv->syncSim = true;
			}

			repaint = true;
		} else
			gv->syncSim = false;
	}

	return repaint;
}

bool RealRobotExecution::real_exec_phase6(bool repaint)
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	MonitoringToolModels *mt=MonitoringToolModels::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	CollisionDetection * cd = CollisionDetection::get_instance();



	if (gv->MT2transported) {
		gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges, gv->numPointsMTrays,
				0);
		gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices,
				gv->numPointsMTrays, 0);
		gv->numPointsMTrays = 0;
	}
	if (gv->virtualMode) {
		for (int i = 0; i < totalPatternCount; i++) {
			if (mt->isPatternVisible(i, 1, true)) {
				hf->getMTPositionData(1);
				fo->calcPosFromMTData(&gv->colliders[15],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], i);
				// fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
					0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
					gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
					0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
					gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
			if (mt->isPatternVisible(i, 2, true)) {
				hf->getMTPositionData(2);
				fo->calcPosFromMTData(&gv->colliders[16],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], i);
				fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
					0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
					gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
					0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
					gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
		}
		fo->evaluateDetections(false);
	} else {
		vector<string> splitList1;
		vector<string> splitList2;
		std::stringstream string_stream_1;
		std::stringstream string_stream_2;
		string splitted;
		string_stream_1.str(gv->stateMessageMT1);
		while (getline(string_stream_1, splitted, ',')) {
			splitList1.push_back(splitted);
		}
		string_stream_2.str(gv->stateMessageMT2);
		while (getline(string_stream_2, splitted, ',')) {
			splitList2.push_back(splitted);
		}
		if (splitList1.size() > 0) {
			for (int i = 0; i < stoi(splitList1[splitList1.size() - 1]) * 5;
					i += 5) {
				fo->distortedRelativePosition[0] = stof(splitList1[i + 1]);
				fo->distortedRelativePosition[1] = stof(splitList1[i + 2]);
				fo->distortedRelativePosition[2] = stof(splitList1[i + 3]);
				fo->distortedRelativePosition[3] = stof(splitList1[i + 4]);
				fo->calcPosFromMTData(&gv->colliders[15],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], stoi(splitList1[i]));
				// fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList1[i])] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
					0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
					gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
					0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
					gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
		}
		if (splitList2.size() > 0) {
			for (int i = 0; i < stoi(splitList2[splitList2.size() - 1]) * 5;
					i += 5) {
				fo->distortedRelativePosition[0] = stof(splitList2[i + 1]);
				fo->distortedRelativePosition[1] = stof(splitList2[i + 2]);
				fo->distortedRelativePosition[2] = stof(splitList2[i + 3]);
				fo->distortedRelativePosition[3] = stof(splitList2[i + 4]);
				fo->calcPosFromMTData(&gv->colliders[16],
						fo->distortedRelativePosition[0],
						fo->distortedRelativePosition[1],
						fo->distortedRelativePosition[2],
						fo->distortedRelativePosition[3], stoi(splitList2[i]));
				fo->insertDetectedPattern(
						objectIndicesFromPattern[stoi(splitList2[i])] - 8);
				gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
						gv->numPointsMTrays, 2);
				gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices,
						gv->numPointsMTrays, 2);
				gv->mt_rays_edges[gv->numPointsMTrays].x =
						fo->distortedPatternCenter[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y =
						fo->distortedPatternCenter[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z =
						fo->distortedPatternCenter[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
					0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
					gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
				gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0];
				gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1];
				gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
				gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex {
					0.0f, 0.0f, 0.0f, gv->otherColors[4].x,
					gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
				gv->numPointsMTrays++;
			}
		}
		fo->evaluateDetections(false);
	}
	if (fo->detectionCounts[4] == 0) {
		cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f, 10.0e5,
				0, 0, 0);
		gv->currentDetectedObsPosX[0] = 0.0f;
		gv->currentDetectedObsPosY[0] = 0.0f;
		gv->currentDetectedObsPosA[0] = 0.0f;
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[0],
				fo->evaluatedDetections[4].x, fo->evaluatedDetections[4].y,
				4.0f, fo->evaluatedDetections[4].z, 0, 0);
		gv->currentDetectedObsPosX[0] = fo->evaluatedDetections[4].x;
		gv->currentDetectedObsPosY[0] = fo->evaluatedDetections[4].y;
		gv->currentDetectedObsPosA[0] = fo->evaluatedDetections[4].z;
	}
	if (fo->detectionCounts[5] == 0) {
		cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f, 10.0e5,
				0, 0, 0);
		gv->currentDetectedObsPosX[1] = 0.0f;
		gv->currentDetectedObsPosY[1] = 0.0f;
		gv->currentDetectedObsPosA[1] = 0.0f;
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[1],
				fo->evaluatedDetections[5].x, fo->evaluatedDetections[5].y,
				3.0f, fo->evaluatedDetections[5].z, 0, 0);
		gv->currentDetectedObsPosX[1] = fo->evaluatedDetections[5].x;
		gv->currentDetectedObsPosY[1] = fo->evaluatedDetections[5].y;
		gv->currentDetectedObsPosA[1] = fo->evaluatedDetections[5].z;
	}
	if (fo->detectionCounts[6] == 0) {
		cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f, 10.0e5,
				0, 0, 0);
		gv->currentDetectedObsPosX[2] = 0.0f;
		gv->currentDetectedObsPosY[2] = 0.0f;
		gv->currentDetectedObsPosA[2] = 0.0f;
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[2],
				fo->evaluatedDetections[6].x, fo->evaluatedDetections[6].y,
				6.0f, fo->evaluatedDetections[6].z, 0, 0);
		gv->currentDetectedObsPosX[2] = fo->evaluatedDetections[6].x;
		gv->currentDetectedObsPosY[2] = fo->evaluatedDetections[6].y;
		gv->currentDetectedObsPosA[2] = fo->evaluatedDetections[6].z;
	}
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
			&gv->distortedObstacles[0], 0);
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
			&gv->distortedObstacles[1], gv->tempCounter);
	hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2],
			gv->tempCounter);
	gv->realExecPhase = 7;
	gv->realExecSubPhase = 0;
	repaint = true;
	return repaint;
}

bool RealRobotExecution::real_exec_phase7(bool repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3)
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	CollisionDetection * cd = CollisionDetection::get_instance();
	MonitoringToolModels *mt=MonitoringToolModels::get_instance();

	if (gv->realExecSubPhase == 0) {
		hf->reloadPathRelatedData(0);
		gv->transportPhase = 0;
		gv->grippingWidth = gv->grippingWidthOpen;
		gv->objectPosReachable = false;
		for (int i = 0; i < 10; i++) {
			if (pp->calcPath(gv->qValuesObjectStart, gv->qValuesStandby,
					&gv->pathStartPos, &gv->standbyPos, false, generator1,
					distribution1, generator2, distribution2, generator3,
					distribution3)) {
				gv->objectPosReachable = true;
				break;
			}
		}
		if (gv->objectPosReachable) {
			gv->atStandby = false;
			pp->optimizePath();
			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
					gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
					gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;
			gv->real_path_edges = ik->cutOff(gv->real_path_edges,
					gv->numPointsRealPath, 0);
			gv->real_path_vertices = ik->cutOff(gv->real_path_vertices,
					gv->numPointsRealPath, 0);
			gv->numPointsRealPath = 0;
			fo->drawPathOnly(2);
			gv->pathLastPos = collider::Edge { 0.0f, 0.0f, 0.0f };
			fo->writeIntoTempSegments();
			gv->pathDataForKuka = "normal,";
			for (int i = 0; i < gv->tempSegSize; i++) {
				gv->pathDataForKuka += hf->to_string_with_precision(
						gv->q0ValsTempSeg[i], 4) + ","
								+ hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4)
								+ ","
								+ hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4)
								+ ","
								+ hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4)
								+ ","
								+ hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4)
								+ ",";
			}
			gv->pathDataForKuka += hf->to_string_with_precision(
					gv->grippingWidth, 4);
			gv->realGripperWidth = gv->grippingWidthOpen;
			gv->realExecSubPhase++;
			repaint = true;
		} else {
			cout
			<< "Real exec. info: first gv->path planning and real transport failed!"
			<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			repaint = false;
		}
	} else if (gv->realExecSubPhase == 2) {
		gv->transportPhase = 1;
		gv->grippingWidth = gv->grippingWidthFixed;
		gv->objectPosReachable = false;
		for (int i = 0; i < 10; i++) {
			if (pp->calcPath(gv->qValuesObjectEnd, gv->qValuesObjectStart,
					&gv->pathEndPos, &gv->pathStartPos, false, generator1,
					distribution1, generator2, distribution2, generator3,
					distribution3)) {
				gv->objectPosReachable = true;
				break;
			}
		}
		if (gv->objectPosReachable) {
			gv->currentlyTransporting = true;
			cd->recalculateCollider(gv->safetyCollider, 0.0f, 0.0f, 10.0e5, 0,
					0, 0);
			hf->calcEdgesForMesh(gv->safety_col_edges, gv->safetyCollider, 0);
			pp->optimizePath();
			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
					gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
					gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;
			gv->real_path_edges = ik->cutOff(gv->real_path_edges,
					gv->numPointsRealPath, 0);
			gv->real_path_vertices = ik->cutOff(gv->real_path_vertices,
					gv->numPointsRealPath, 0);
			gv->numPointsRealPath = 0;
			fo->drawPathOnly(2);
			gv->pathLastPos = collider::Edge { 0.0f, 0.0f, 0.0f };
			fo->writeIntoTempSegments();
			gv->pathDataForKuka = "normal,";
			for (int i = 0; i < gv->tempSegSize; i++) {
				gv->pathDataForKuka += hf->to_string_with_precision(
						gv->q0ValsTempSeg[i], 4) + ","
								+ hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4)
								+ ","
								+ hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4)
								+ ","
								+ hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4)
								+ ","
								+ hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4)
								+ ",";
			}
			gv->pathDataForKuka += hf->to_string_with_precision(
					gv->grippingWidth, 4);
			gv->realGripperWidth = gv->grippingWidthFixed;
			gv->realExecSubPhase++;
			repaint = true;
		} else {
			gv->robotPoseAtAbort[0] = gv->pathStartPos.x;
			gv->robotPoseAtAbort[1] = gv->pathStartPos.y;
			gv->robotPoseAtAbort[2] = gv->pathStartPos.z;
			gv->robotPoseAtAbort[3] = gv->qValuesObjectStart[0];
			gv->robotPoseAtAbort[4] = gv->qValuesObjectStart[1];
			gv->robotPoseAtAbort[5] = gv->qValuesObjectStart[2];
			gv->robotPoseAtAbort[6] = gv->qValuesObjectStart[3];
			gv->robotPoseAtAbort[7] = gv->qValuesObjectStart[4];
			cout
			<< "Real exec. info: second gv->path planning and real transport failed!"
			<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			gv->transportPhase = 0;
			repaint = false;
		}
	} else if (gv->realExecSubPhase == 4) {
		gv->transportPhase = 2;
		gv->grippingWidth = gv->grippingWidthOpen;
		gv->objectPosReachable = false;
		for (int i = 0; i < 10; i++) {
			if (pp->calcPath(gv->qValuesStandby, gv->qValuesObjectEnd,
					&gv->standbyPos, &gv->pathEndPos, false, generator1,
					distribution1, generator2, distribution2, generator3,
					distribution3)) {
				gv->objectPosReachable = true;
				break;
			}
		}
		if (gv->objectPosReachable) {
			gv->currentlyTransporting = false;
			pp->optimizePath();
			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
					gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
					gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;
			gv->real_path_edges = ik->cutOff(gv->real_path_edges,
					gv->numPointsRealPath, 0);
			gv->real_path_vertices = ik->cutOff(gv->real_path_vertices,
					gv->numPointsRealPath, 0);
			gv->numPointsRealPath = 0;
			fo->drawPathOnly(2);
			gv->pathLastPos = collider::Edge { 0.0f, 0.0f, 0.0f };
			fo->writeIntoTempSegments();
			gv->pathDataForKuka = "normal,";
			for (int i = 0; i < gv->tempSegSize; i++) {
				gv->pathDataForKuka += hf->to_string_with_precision(
						gv->q0ValsTempSeg[i], 4) + ","
								+ hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4)
								+ ","
								+ hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4)
								+ ","
								+ hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4)
								+ ","
								+ hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4)
								+ ",";
			}
			gv->pathDataForKuka += hf->to_string_with_precision(
					gv->grippingWidth, 4);
			gv->realGripperWidth = gv->grippingWidthOpen;
			gv->realExecSubPhase++;
			repaint = true;
		} else {
			gv->robotPoseAtAbort[0] = gv->pathEndPos.x;
			gv->robotPoseAtAbort[1] = gv->pathEndPos.y;
			gv->robotPoseAtAbort[2] = gv->pathEndPos.z;
			gv->robotPoseAtAbort[3] = gv->qValuesObjectEnd[0];
			gv->robotPoseAtAbort[4] = gv->qValuesObjectEnd[1];
			gv->robotPoseAtAbort[5] = gv->qValuesObjectEnd[2];
			gv->robotPoseAtAbort[6] = gv->qValuesObjectEnd[3];
			gv->robotPoseAtAbort[7] = gv->qValuesObjectEnd[4];
			hf->storePathRelatedData(1);
			gv->MT1currentlyActiveTemp = gv->MT1currentlyActive;
			gv->MT2currentlyActiveTemp = gv->MT2currentlyActive;
			cout
			<< "Real exec. info: third gv->path planning and real transport failed!"
			<< endl;
			gv->modeActive = false;
			gv->realExecPhase = -1;
			gv->transportPhase = 0;
			repaint = false;
		}
	} else if (gv->realExecSubPhase == 6) {
		cout << "Real exec. info: end reached, transport finished!" << endl;
		gv->atStandby = true;
		gv->modeActive = false;
		gv->realExecPhase = -1;
		gv->transportPhase = 0;
	} else {
		if (gv->virtualMode) {
		} else
			hf->requestRealSystemState();

		vector<string> splitList;
		std::stringstream string_stream;
		string splitted;
		string_stream.str(gv->stateMessageSTATE);
		while (getline(string_stream, splitted, ',')) {
			splitList.push_back(splitted);
		}
		gv->realNodeIndex = stoi(splitList[splitList.size() - 1]);
		hf->syncVisualization(splitList);
		if (pp->nodeIndex != 1) {
			gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges,
					gv->numPointsMTrays, 0);
			gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices,
					gv->numPointsMTrays, 0);
			gv->numPointsMTrays = 0;
			if (gv->virtualMode) {
				for (int i = 0; i <= 6; i += 3) {
					gv->groundTruthObstaclePositions[i] += ((rand()
							/ (float) (RAND_MAX)) - 0.5f) / 2.5f;
					gv->groundTruthObstaclePositions[i + 1] += ((rand()
							/ (float) (RAND_MAX)) - 0.5f) / 2.5f;
					if (gv->groundTruthObstaclePositions[i] > 55.0f)
						gv->groundTruthObstaclePositions[i] = 55.0f;

					if (gv->groundTruthObstaclePositions[i] < -55.0f)
						gv->groundTruthObstaclePositions[i] = -55.0f;

					if (gv->groundTruthObstaclePositions[i + 1] > 55.0f)
						gv->groundTruthObstaclePositions[i + 1] = 55.0f;

					if (gv->groundTruthObstaclePositions[i + 1] < -12.5f)
						gv->groundTruthObstaclePositions[i + 1] = -12.5f;
				}
				cd->recalculateCollider(&gv->colliders[12],
						gv->groundTruthObstaclePositions[0],
						gv->groundTruthObstaclePositions[1], 4.0f,
						gv->groundTruthObstaclePositions[2], 0, 0, true);
				cd->recalculateCollider(&gv->colliders[13],
						gv->groundTruthObstaclePositions[3],
						gv->groundTruthObstaclePositions[4], 3.0f,
						gv->groundTruthObstaclePositions[5], 0, 0, true);
				cd->recalculateCollider(&gv->colliders[14],
						gv->groundTruthObstaclePositions[6],
						gv->groundTruthObstaclePositions[7], 6.0f,
						gv->groundTruthObstaclePositions[8], 0, 0, true);
				gv->tempCounter = hf->calcEdgesForMesh(
						gv->ground_truth_obs_edges, &gv->colliders[12], 0);
				gv->tempCounter = hf->calcEdgesForMesh(
						gv->ground_truth_obs_edges, &gv->colliders[13],
						gv->tempCounter);
				hf->calcEdgesForMesh(gv->ground_truth_obs_edges,
						&gv->colliders[14], gv->tempCounter);
				fo->resetDetectedPatterns();
				for (int i = 0; i < totalPatternCount; i++) {
					if (mt->isPatternVisible(i, 1, true)) {
						hf->getMTPositionData(1);
						fo->calcPosFromMTData(&gv->colliders[15],
								fo->distortedRelativePosition[0],
								fo->distortedRelativePosition[1],
								fo->distortedRelativePosition[2],
								fo->distortedRelativePosition[3], i);
						fo->insertDetectedPattern(
								objectIndicesFromPattern[i] - 8);
						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
								gv->numPointsMTrays, 2);
						gv->mt_rays_vertices = ik->increaseSize(
								gv->mt_rays_vertices, gv->numPointsMTrays, 2);
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								fo->distortedPatternCenter[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								fo->distortedPatternCenter[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
							gv->otherColors[4].x,
							gv->otherColors[4].y,
							gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								gv->camVectorsMT1[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								gv->camVectorsMT1[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								gv->camVectorsMT1[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
							gv->otherColors[4].x,
							gv->otherColors[4].y,
							gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
					if (mt->isPatternVisible(i, 2, true)) {
						hf->getMTPositionData(2);
						fo->calcPosFromMTData(&gv->colliders[16],
								fo->distortedRelativePosition[0],
								fo->distortedRelativePosition[1],
								fo->distortedRelativePosition[2],
								fo->distortedRelativePosition[3], i);
						fo->insertDetectedPattern(
								objectIndicesFromPattern[i] - 8);
						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
								gv->numPointsMTrays, 2);
						gv->mt_rays_vertices = ik->increaseSize(
								gv->mt_rays_vertices, gv->numPointsMTrays, 2);
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								fo->distortedPatternCenter[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								fo->distortedPatternCenter[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
							gv->otherColors[4].x,
							gv->otherColors[4].y,
							gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								gv->camVectorsMT2[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								gv->camVectorsMT2[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								gv->camVectorsMT2[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
							gv->otherColors[4].x,
							gv->otherColors[4].y,
							gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
				}
				fo->evaluateDetections(false);
			} else {
				vector<string> splitList1;
				vector<string> splitList2;
				std::stringstream string_stream_1;
				std::stringstream string_stream_2;
				string_stream_1.str(gv->stateMessageMT1);
				while (getline(string_stream_1, splitted, ',')) {
					splitList1.push_back(splitted);
				}
				string_stream_2.str(gv->stateMessageMT2);
				while (getline(string_stream_2, splitted, ',')) {
					splitList2.push_back(splitted);
				}
				fo->resetDetectedPatterns();
				if (splitList1.size() > 0) {
					for (int i = 0;
							i < stoi(splitList1[splitList1.size() - 1]) * 5;
							i += 5) {
						fo->distortedRelativePosition[0] = stof(
								splitList1[i + 1]);
						fo->distortedRelativePosition[1] = stof(
								splitList1[i + 2]);
						fo->distortedRelativePosition[2] = stof(
								splitList1[i + 3]);
						fo->distortedRelativePosition[3] = stof(
								splitList1[i + 4]);
						fo->calcPosFromMTData(&gv->colliders[15],
								fo->distortedRelativePosition[0],
								fo->distortedRelativePosition[1],
								fo->distortedRelativePosition[2],
								fo->distortedRelativePosition[3],
								stoi(splitList1[i]));
						fo->insertDetectedPattern(
								objectIndicesFromPattern[stoi(splitList1[i])]
														 - 8);
						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
								gv->numPointsMTrays, 2);
						gv->mt_rays_vertices = ik->increaseSize(
								gv->mt_rays_vertices, gv->numPointsMTrays, 2);
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								fo->distortedPatternCenter[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								fo->distortedPatternCenter[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
							gv->otherColors[4].x,
							gv->otherColors[4].y,
							gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								gv->camVectorsMT1[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								gv->camVectorsMT1[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								gv->camVectorsMT1[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
							gv->otherColors[4].x,
							gv->otherColors[4].y,
							gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
				}
				if (splitList2.size() > 0) {
					for (int i = 0;
							i < stoi(splitList2[splitList2.size() - 1]) * 5;
							i += 5) {
						fo->distortedRelativePosition[0] = stof(
								splitList2[i + 1]);
						fo->distortedRelativePosition[1] = stof(
								splitList2[i + 2]);
						fo->distortedRelativePosition[2] = stof(
								splitList2[i + 3]);
						fo->distortedRelativePosition[3] = stof(
								splitList2[i + 4]);
						fo->calcPosFromMTData(&gv->colliders[16],
								fo->distortedRelativePosition[0],
								fo->distortedRelativePosition[1],
								fo->distortedRelativePosition[2],
								fo->distortedRelativePosition[3],
								stoi(splitList2[i]));
						fo->insertDetectedPattern(
								objectIndicesFromPattern[stoi(splitList2[i])]
														 - 8);
						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges,
								gv->numPointsMTrays, 2);
						gv->mt_rays_vertices = ik->increaseSize(
								gv->mt_rays_vertices, gv->numPointsMTrays, 2);
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								fo->distortedPatternCenter[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								fo->distortedPatternCenter[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
							gv->otherColors[4].x,
							gv->otherColors[4].y,
							gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
						gv->mt_rays_edges[gv->numPointsMTrays].x =
								gv->camVectorsMT2[0];
						gv->mt_rays_edges[gv->numPointsMTrays].y =
								gv->camVectorsMT2[1];
						gv->mt_rays_edges[gv->numPointsMTrays].z =
								gv->camVectorsMT2[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] =
								collider::Vertex { 0.0f, 0.0f, 0.0f,
							gv->otherColors[4].x,
							gv->otherColors[4].y,
							gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
				}
				fo->evaluateDetections(false);
			}
			if (fo->detectionCounts[4] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f,
						10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[0] = 0.0f;
				gv->currentDetectedObsPosY[0] = 0.0f;
				gv->currentDetectedObsPosA[0] = 0.0f;
			} else {
				cd->recalculateCollider(&gv->distortedObstacles[0],
						fo->evaluatedDetections[4].x,
						fo->evaluatedDetections[4].y, 4.0f,
						fo->evaluatedDetections[4].z, 0, 0);
				gv->currentDetectedObsPosX[0] = fo->evaluatedDetections[4].x;
				gv->currentDetectedObsPosY[0] = fo->evaluatedDetections[4].y;
				gv->currentDetectedObsPosA[0] = fo->evaluatedDetections[4].z;
			}
			if (fo->detectionCounts[5] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f,
						10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[1] = 0.0f;
				gv->currentDetectedObsPosY[1] = 0.0f;
				gv->currentDetectedObsPosA[1] = 0.0f;
			} else {
				cd->recalculateCollider(&gv->distortedObstacles[1],
						fo->evaluatedDetections[5].x,
						fo->evaluatedDetections[5].y, 3.0f,
						fo->evaluatedDetections[5].z, 0, 0);
				gv->currentDetectedObsPosX[1] = fo->evaluatedDetections[5].x;
				gv->currentDetectedObsPosY[1] = fo->evaluatedDetections[5].y;
				gv->currentDetectedObsPosA[1] = fo->evaluatedDetections[5].z;
			}
			if (fo->detectionCounts[6] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f,
						10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[2] = 0.0f;
				gv->currentDetectedObsPosY[2] = 0.0f;
				gv->currentDetectedObsPosA[2] = 0.0f;
			} else {
				cd->recalculateCollider(&gv->distortedObstacles[2],
						fo->evaluatedDetections[6].x,
						fo->evaluatedDetections[6].y, 6.0f,
						fo->evaluatedDetections[6].z, 0, 0);
				gv->currentDetectedObsPosX[2] = fo->evaluatedDetections[6].x;
				gv->currentDetectedObsPosY[2] = fo->evaluatedDetections[6].y;
				gv->currentDetectedObsPosA[2] = fo->evaluatedDetections[6].z;
			}
			gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[0], 0);
			gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[1], gv->tempCounter);
			hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[2], gv->tempCounter);
			if (gv->realNodeIndex == gv->tempSegSize) {
				gv->syncSim = false;
				if (gv->realExecSubPhase == 3) {
					gv->initPositionsX[gv->objectToTransport - 8] =
							gv->pathEndPos.x;
					gv->initPositionsY[gv->objectToTransport - 8] =
							gv->pathEndPos.y;
					gv->initPositionsAngles[gv->objectToTransport - 8] =
							gv->endObjectAngle / 180.0f * M_PI;
					cd->recalculateCollider(
							&gv->colliders[gv->objectToTransport],
							gv->pathEndPos.x, gv->pathEndPos.y,
							gv->objectOffset,
							gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f,
							true);
					gv->tempCounter = 0;
					for (int i = 8; i < gv->objectToTransport; i++) {
						gv->tempCounter += gv->colliders[i].patternCount * 4;
					}
					hf->calcEdgesForPatterns(gv->object_pattern_edges,
							&gv->colliders[gv->objectToTransport],
							gv->tempCounter);
					gv->tempCounter = gv->totalPlatePoints;
					for (int i = 0; i < gv->objectToTransport; i++) {
						gv->tempCounter += gv->colliders[i].facesTimes3;
					}
					hf->calcEdgesForConvexHull(gv->object_edges,
							&gv->colliders[gv->objectToTransport],
							gv->tempCounter);
				}
				gv->realExecSubPhase++;
			} else if (gv->realNodeIndex > 0) {
				if (!gv->syncSim)
					gv->syncSim = true;
			}

			repaint = true;
		} else
			gv->syncSim = false;
	}

	return repaint;
}
