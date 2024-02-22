/*
 * SimulationControl.cpp
 *
 *  Created on: 18.12.2022
 *      Author: manuel
 */

#include "SimulationControl.h"

SimulationControl::SimulationControl() {

}

SimulationControl::~SimulationControl() {
	// TODO Auto-generated destructor stub
}

/*
 * singelton pattern
 */
SimulationControl* SimulationControl::sc = 0;
SimulationControl* SimulationControl::get_instance(){
	static bool isInit=false;

//	InverseKinematic *ik;
	if(!isInit){
		sc  = new SimulationControl();
		isInit=true;
	}
	return sc;
}

/*
 * this function controls the training process of the agents in the simulation.
 * TODO: this funciton is still too complex. Revice it!!!
 */
void SimulationControl::train_agents(bool &repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik=InverseKinematic::get_instance(); Adversaries *adv=Adversaries::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance(); Protagonists *prot=Protagonists::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance(); FrequentOps *fo=FrequentOps::get_instance();
MonitoringToolModels *mt = MonitoringToolModels::get_instance(); PathPlanner *pp = PathPlanner::get_instance();
	gv->MT1currentlyActive = true;
	gv->MT2currentlyActive = true;
	gv->collision = false;
	gv->modeActive = true;
	gv->grippingWidth = 0.0f;
	gv->q0Sin = sin(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q0Cos = cos(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q1Sin = sin(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q1Cos = cos(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q12Sin = sin(
			(gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q12Cos = cos(
			(gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q123Sin = sin(
			(gv->qValuesStandby[1] + gv->qValuesStandby[2]
					+ gv->qValuesStandby[3]) / 180.0f * M_PI);
	gv->q123Cos = cos(
			(gv->qValuesStandby[1] + gv->qValuesStandby[2]
					+ gv->qValuesStandby[3]) / 180.0f * M_PI);
	cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f,
			(-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
	cd->recalculateCollider(&gv->colliders[4],
			(ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Sin
					+ 4.5f * gv->q0Cos,
			(ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Cos
					- 4.5f * gv->q0Sin, ik->len0 + ik->len2 / 2.0f * gv->q1Cos,
			(-gv->qValuesStandby[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesStandby[1]) / 180.0f * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[5],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin)
					* gv->q0Sin - 4.155f * gv->q0Cos,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin)
					* gv->q0Cos + 4.155f * gv->q0Sin,
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 / 2.0f * gv->q12Cos,
			(-gv->qValuesStandby[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesStandby[1] - gv->qValuesStandby[2]) / 180 * M_PI,
			0);
	cd->recalculateCollider(&gv->colliders[6],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ ik->lastSegmentMid * gv->q123Sin) * gv->q0Sin,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ ik->lastSegmentMid * gv->q123Sin) * gv->q0Cos,
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos
					+ ik->lastSegmentMid * gv->q123Cos,
			(-gv->qValuesStandby[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesStandby[1] - gv->qValuesStandby[2]
					- gv->qValuesStandby[3]) / 180 * M_PI, 0);
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
			(-gv->qValuesStandby[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesStandby[1] - gv->qValuesStandby[2]
					- gv->qValuesStandby[3]) / 180 * M_PI,
			gv->qValuesStandby[4] / 180.0f * M_PI);
	cd->updateMatricesTransport(
			(-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2]
					- gv->qValuesStandby[3]) / 180.0f * M_PI,
			(gv->qValuesStandby[4] + 0) / 180.0f * M_PI,
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
			(-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2]
					- gv->qValuesStandby[3]) / 180.0f * M_PI,
			gv->qValuesStandby[4] / 180.0f * M_PI);
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
			(-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2]
					- gv->qValuesStandby[3]) / 180.0f * M_PI,
			gv->qValuesStandby[4] / 180.0f * M_PI);
	cout << endl;
	cout << "Training info: training begin...";
//#if defined(WIN32)
	 sendMessageML("train");
	 receiveMessageML();
//#endif
	cout << " ...end" << endl;
	cout << "---------------------------------------" << endl;
	gv->activeAgents[0] = true;
	gv->activeAgents[1] = true;
	gv->randomAgent = (rand() % 13);
	if (gv->randomAgent < 4) {
		gv->activeAgents[2] = true;
		gv->activeAgents[3] = false;
		gv->activeAgents[4] = false;
	} else if (gv->randomAgent < 11) {
		gv->activeAgents[2] = false;
		gv->activeAgents[3] = true;
		gv->activeAgents[4] = false;
	} else {
		gv->activeAgents[2] = false;
		gv->activeAgents[3] = false;
		gv->activeAgents[4] = true;
	}

	do {
		do {
			gv->collision_init = false;
			cd->recalculateCollider(&gv->colliders[8],
					((float) (rand()) / RAND_MAX) * 68.0f - 34.0f,
					((float) (rand()) / RAND_MAX) * 46.5f - 12.5f,
					gv->objectOffsets[0],
					(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX),
					0.0f, 0.0f, true);
			if (cd->checkForCollision(&gv->colliders[8], &gv->colliders[1])) {
				gv->collision_init = true;
			}
		} while (gv->collision_init);
		do {
			gv->collision_init = false;
			cd->recalculateCollider(&gv->colliders[9],
					((float) (rand()) / RAND_MAX) * 68.0f - 34.0f,
					((float) (rand()) / RAND_MAX) * 46.5f - 12.5f,
					gv->objectOffsets[1],
					(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX),
					0.0f, 0.0f, true);
			if (cd->checkForCollision(&gv->colliders[9], &gv->colliders[8])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[9],
							&gv->colliders[1])) {
				gv->collision_init = true;
			}
		} while (gv->collision_init);
		do {
			gv->collision_init = false;
			cd->recalculateCollider(&gv->colliders[10],
					((float) (rand()) / RAND_MAX) * 68.0f - 34.0f,
					((float) (rand()) / RAND_MAX) * 46.5f - 12.5f,
					gv->objectOffsets[2],
					(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX),
					0.0f, 0.0f, true);
			if (cd->checkForCollision(&gv->colliders[10], &gv->colliders[8])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[10],
							&gv->colliders[9])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[10],
							&gv->colliders[1])) {
				gv->collision_init = true;
			}
		} while (gv->collision_init);
		do {
			gv->collision_init = false;
			cd->recalculateCollider(&gv->colliders[11],
					((float) (rand()) / RAND_MAX) * 68.0f - 34.0f,
					((float) (rand()) / RAND_MAX) * 46.5f - 12.5f,
					gv->objectOffsets[3],
					(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX),
					0.0f, 0.0f, true);
			if (cd->checkForCollision(&gv->colliders[11], &gv->colliders[8])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[11],
							&gv->colliders[9])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[11],
							&gv->colliders[10])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[11],
							&gv->colliders[1])) {
				gv->collision_init = true;
			}
		} while (gv->collision_init);
		do {
			gv->collision_init = false;
			cd->recalculateCollider(&gv->colliders[12],
					((float) (rand()) / RAND_MAX) * 110.0f - 55.0f,
					((float) (rand()) / RAND_MAX) * 67.5f - 12.5f, 4.0f,
					(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX),
					0, 0, true);
			if (cd->checkForCollision(&gv->colliders[12], &gv->colliders[8])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[12],
							&gv->colliders[9])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[12],
							&gv->colliders[10])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[12],
							&gv->colliders[11])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[12],
							&gv->colliders[1])) {
				gv->collision_init = true;
			}
		} while (gv->collision_init);
		do {
			gv->collision_init = false;
			cd->recalculateCollider(&gv->colliders[13],
					((float) (rand()) / RAND_MAX) * 110.0f - 55.0f,
					((float) (rand()) / RAND_MAX) * 67.5f - 12.5f, 3.0f,
					(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX),
					0, 0, true);
			if (cd->checkForCollision(&gv->colliders[13], &gv->colliders[8])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[13],
							&gv->colliders[9])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[13],
							&gv->colliders[10])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[13],
							&gv->colliders[11])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[13],
							&gv->colliders[12])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[13],
							&gv->colliders[1])) {
				gv->collision_init = true;
			}
		} while (gv->collision_init);
		do {
			gv->collision_init = false;
			cd->recalculateCollider(&gv->colliders[14],
					((float) (rand()) / RAND_MAX) * 110.0f - 55.0f,
					((float) (rand()) / RAND_MAX) * 67.5f - 12.5f, 6.0f,
					(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX),
					0, 0, true);
			if (cd->checkForCollision(&gv->colliders[14], &gv->colliders[8])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[14],
							&gv->colliders[9])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[14],
							&gv->colliders[10])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[14],
							&gv->colliders[11])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[14],
							&gv->colliders[12])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[14],
							&gv->colliders[13])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[14],
							&gv->colliders[1])) {
				gv->collision_init = true;
			}
		} while (gv->collision_init);
		do {
			gv->collision_init = false;
			cd->recalculateCollider(&gv->colliders[15],
					((float) (rand()) / RAND_MAX) * 68.0f - 34.0f,
					((float) (rand()) / RAND_MAX) * 46.5f - 12.5f,
					gv->objectOffsets[4],
					(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX),
					0, 0, false, true, gv->camVectorsMT1);
			if (cd->checkForCollision(&gv->colliders[15], &gv->colliders[8])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[15],
							&gv->colliders[9])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[15],
							&gv->colliders[10])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[15],
							&gv->colliders[11])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[15],
							&gv->colliders[12])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[15],
							&gv->colliders[13])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[15],
							&gv->colliders[14])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[15],
							&gv->colliders[1])) {
				gv->collision_init = true;
			}
		} while (gv->collision_init);
		do {
			gv->collision_init = false;
			cd->recalculateCollider(&gv->colliders[16],
					((float) (rand()) / RAND_MAX) * 68.0f - 34.0f,
					((float) (rand()) / RAND_MAX) * 46.5f - 12.5f,
					gv->objectOffsets[5],
					(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX),
					0, 0, false, true, gv->camVectorsMT2);
			if (cd->checkForCollision(&gv->colliders[16], &gv->colliders[8])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[16],
							&gv->colliders[9])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[16],
							&gv->colliders[10])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[16],
							&gv->colliders[11])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[16],
							&gv->colliders[12])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[16],
							&gv->colliders[13])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[16],
							&gv->colliders[14])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[16],
							&gv->colliders[15])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init
					&& cd->checkForCollision(&gv->colliders[16],
							&gv->colliders[1])) {
				gv->collision_init = true;
			}
		} while (gv->collision_init);
		gv->initIterations = 0;
		do {
			gv->randomWorkspace = (rand() % 4);
			gv->objectToTransport =
					gv->transportableObjects[gv->randomWorkspace];
			gv->objectOffset = gv->objectOffsets[gv->randomWorkspace];
			gv->objectHeight = gv->objectHeights[gv->randomWorkspace];
			gv->objectGripDepth = gv->objectGripDepths[gv->randomWorkspace];
			gv->objectDimension.x = gv->objectDimensions[gv->randomWorkspace].x
					+ 0.25f;
			gv->objectDimension.y = gv->objectDimensions[gv->randomWorkspace].y
					+ 0.25f;
			gv->objectDimension.z = gv->objectDimensions[gv->randomWorkspace].z
					+ 0.25f;
			gv->objectIndex = gv->randomWorkspace;
			gv->pathStartPos.x =
					gv->colliders[gv->objectToTransport].offsets[0];
			gv->pathStartPos.y =
					gv->colliders[gv->objectToTransport].offsets[1];
			gv->pathStartPos.z = gv->objectOffset + gv->objectHeight
					+ gv->objectGripDepth;
			gv->startObjectAngle =
					gv->colliders[gv->objectToTransport].angles[0]
							* 180.0f/ M_PI;
			gv->pathEndPos.x = ((float) (rand()) / RAND_MAX) * 68.0f - 34.0f;
			gv->pathEndPos.y = ((float) (rand()) / RAND_MAX) * 46.5f - 12.5f;
			gv->pathEndPos.z = gv->objectOffset + gv->objectHeight
					+ gv->objectGripDepth;
			gv->endObjectAngle = ((float) (rand()) / RAND_MAX)
					* (360.0f - cd->epsilon4);
			gv->objectPosReachable = false;
			if (ik->doesThetaExist(gv->pathStartPos.x, gv->pathStartPos.y,
					gv->pathStartPos.z)) {
				ik->writeQValues(gv->qValuesObjectStart);
				if (abs(
						gv->qValuesObjectStart[1] + gv->qValuesObjectStart[2]
								+ gv->qValuesObjectStart[3] - 180.0f)
						< cd->epsilon3) {
					if (ik->doesThetaExist(gv->pathEndPos.x, gv->pathEndPos.y,
							gv->pathEndPos.z)) {
						ik->writeQValues(gv->qValuesObjectEnd);
						if (abs(
								gv->qValuesObjectEnd[1]
										+ gv->qValuesObjectEnd[2]
										+ gv->qValuesObjectEnd[3] - 180.0f)
								< cd->epsilon3) {
							gv->qValuesObjectStart[4] = gv->startObjectAngle
									- (90.0f - gv->qValuesObjectStart[0]);
							gv->qValuesObjectEnd[4] = gv->endObjectAngle
									- (90.0f - gv->qValuesObjectEnd[0]);
							fo->gripAngleCalculation(gv->randomWorkspace);
							if (gv->qValuesObjectStart[4] > ik->qUpperLimits[4]
									|| gv->qValuesObjectStart[4]
											< ik->qLowerLimits[4]
									|| gv->qValuesObjectEnd[4]
											> ik->qUpperLimits[4]
									|| gv->qValuesObjectEnd[4]
											< ik->qLowerLimits[4]) {
								gv->objectPosReachable = false;
							} else {
								prot->safety_area_protagonist(gv->objectIndex,
										false);
								cd->updateColliderSize(gv->safetyCollider,
										gv->objectDimensionTemp.x,
										gv->objectDimensionTemp.y,
										gv->objectDimensionTemp.z);
								cd->updateColliderSize(
										&gv->adversaryCollider[3],
										gv->objectDimension.x - 0.5f,
										gv->objectDimension.y - 0.5f,
										gv->objectDimension.z - 0.5f);
								cd->updateColliderSize(
										&gv->safetyCollidersForMTs[0],
										gv->objectDimension.x + 0.5f,
										gv->objectDimension.y + 0.5f,
										gv->objectDimension.z);
								cd->updateColliderSize(
										&gv->safetyCollidersForMTs[1],
										gv->objectDimension.x + 0.5f,
										gv->objectDimension.y + 0.5f,
										gv->objectDimension.z);
								cd->recalculateCollider(
										&gv->safetyCollidersForMTs[0],
										gv->pathStartPos.x, gv->pathStartPos.y,
										gv->objectOffset,
										gv->startObjectAngle / 180.0f * M_PI,
										0.0f, 0.0f);
								cd->recalculateCollider(
										&gv->safetyCollidersForMTs[1],
										gv->pathEndPos.x, gv->pathEndPos.y,
										gv->objectOffset,
										gv->endObjectAngle / 180.0f * M_PI,
										0.0f, 0.0f);
								for (int i = 0; i < 9; i++) {
									gv->distortions[i] = 0.0f;
								}
								if (gv->activeAgents[2]) {
									adv->loc_adversary(
											gv->actionSpaceDistances[rand()
													% gv->actionSpaceCount],
											gv->actionSpaceAngles[rand()
													% gv->actionSpaceCount],
											gv->randomWorkspace, false);
								}
								cd->recalculateCollider(
										&gv->distortedObstacles[0],
										gv->colliders[12].offsets[0]
												+ gv->distortions[0],
										gv->colliders[12].offsets[1]
												+ gv->distortions[1], 4.0f,
										gv->colliders[12].angles[0]
												+ gv->distortions[2], 0, 0);
								cd->recalculateCollider(
										&gv->distortedObstacles[1],
										gv->colliders[13].offsets[0]
												+ gv->distortions[3],
										gv->colliders[13].offsets[1]
												+ gv->distortions[4], 3.0f,
										gv->colliders[13].angles[0]
												+ gv->distortions[5], 0, 0);
								cd->recalculateCollider(
										&gv->distortedObstacles[2],
										gv->colliders[14].offsets[0]
												+ gv->distortions[6],
										gv->colliders[14].offsets[1]
												+ gv->distortions[7], 6.0f,
										gv->colliders[14].angles[0]
												+ gv->distortions[8], 0, 0);
								mt->updateMonitoredSpace(true, false);
								gv->grippingWidth = gv->grippingWidthOpen;
								gv->objectPosReachable = pp->calcPath(
										gv->qValuesObjectStart,
										gv->qValuesStandby, &gv->pathStartPos,
										&gv->standbyPos, false, generator1,
										distribution1, generator2,
										distribution2, generator3,
										distribution3);
								if (gv->objectPosReachable) {
									gv->transportPhase = 1;
									gv->grippingWidth = gv->grippingWidthFixed;
									gv->objectPosReachable = pp->calcPath(
											gv->qValuesObjectEnd,
											gv->qValuesObjectStart,
											&gv->pathEndPos, &gv->pathStartPos,
											false, generator1, distribution1,
											generator2, distribution2,
											generator3, distribution3);
									if (gv->objectPosReachable) {
										pp->optimizePath();
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
											gv->q0ValsSecondSeg =
													ik->increaseSize(
															gv->q0ValsSecondSeg,
															gv->secondSegSize,
															1);
											gv->q1ValsSecondSeg =
													ik->increaseSize(
															gv->q1ValsSecondSeg,
															gv->secondSegSize,
															1);
											gv->q2ValsSecondSeg =
													ik->increaseSize(
															gv->q2ValsSecondSeg,
															gv->secondSegSize,
															1);
											gv->q3ValsSecondSeg =
													ik->increaseSize(
															gv->q3ValsSecondSeg,
															gv->secondSegSize,
															1);
											gv->q4ValsSecondSeg =
													ik->increaseSize(
															gv->q4ValsSecondSeg,
															gv->secondSegSize,
															1);
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
										gv->secondSegLengthTotal = 0.0f;
										for (int i = 1; i < gv->secondSegSize;
												i++) {
											gv->secondSegLengths =
													ik->increaseSize(
															gv->secondSegLengths,
															i - 1, 1);
											gv->secondSegLengths[i - 1] =
													gv->secondSegLengthTotal
															+ max(
																	abs(
																			gv->q0ValsSecondSeg[i]
																					- gv->q0ValsSecondSeg[i
																							- 1]),
																	max(
																			abs(
																					gv->q1ValsSecondSeg[i]
																							- gv->q1ValsSecondSeg[i
																									- 1]),
																			max(
																					abs(
																							gv->q2ValsSecondSeg[i]
																									- gv->q2ValsSecondSeg[i
																											- 1]),
																					abs(
																							gv->q3ValsSecondSeg[i]
																									- gv->q3ValsSecondSeg[i
																											- 1]))));
											gv->secondSegLengthTotal =
													gv->secondSegLengths[i - 1];
										}
										cd->recalculateCollider(
												&gv->colliders[gv->objectToTransport],
												gv->pathEndPos.x,
												gv->pathEndPos.y,
												gv->objectOffset,
												gv->endObjectAngle
														/ 180.0f* M_PI, 0.0f,
												0.0f);
										gv->transportPhase = 2;
										gv->grippingWidth =
												gv->grippingWidthOpen;
										gv->objectPosReachable = pp->calcPath(
												gv->qValuesStandby,
												gv->qValuesObjectEnd,
												&gv->standbyPos,
												&gv->pathEndPos, false,
												generator1, distribution1,
												generator2, distribution2,
												generator3, distribution3);
									}
									cd->recalculateCollider(
											&gv->colliders[gv->objectToTransport],
											gv->pathStartPos.x,
											gv->pathStartPos.y,
											gv->objectOffset,
											gv->startObjectAngle / 180.0f * M_PI,
											0.0f, 0.0f);
									gv->transportPhase = 0;
								}
							}
						}
					}
				}
			}
			gv->initIterations++;
			if (gv->initIterations == 21)
				break;
		} while (!gv->objectPosReachable);
	} while (gv->initIterations == 21);
	gv->oldObstaclePositions[0] = gv->colliders[12].offsets[0];
	gv->oldObstaclePositions[1] = gv->colliders[12].offsets[1];
	gv->oldObstaclePositions[2] = gv->colliders[12].angles[0];
	gv->oldObstaclePositions[3] = gv->colliders[13].offsets[0];
	gv->oldObstaclePositions[4] = gv->colliders[13].offsets[1];
	gv->oldObstaclePositions[5] = gv->colliders[13].angles[0];
	gv->oldObstaclePositions[6] = gv->colliders[14].offsets[0];
	gv->oldObstaclePositions[7] = gv->colliders[14].offsets[1];
	gv->oldObstaclePositions[8] = gv->colliders[14].angles[0];
	gv->MT1collision = false;
	gv->MT2collision = false;
	gv->MT1posChanged = false;
	gv->MT2posChanged = false;
	gv->oldMTPos[0] = gv->colliders[15].offsets[0];
	gv->oldMTPos[1] = gv->colliders[15].offsets[1];
	gv->oldMTPos[2] = gv->colliders[15].angles[0];
	srand(1);
	prot->mt_preparation(1, generator1, distribution1, generator2,
			distribution2, generator3, distribution3);
	srand((unsigned) (time(NULL)));
	cd->recalculateCollider(&gv->colliders[12], gv->oldObstaclePositions[0],
			gv->oldObstaclePositions[1], 4.0f, gv->oldObstaclePositions[2], 0,
			0, true);
	cd->recalculateCollider(&gv->colliders[13], gv->oldObstaclePositions[3],
			gv->oldObstaclePositions[4], 3.0f, gv->oldObstaclePositions[5], 0,
			0, true);
	cd->recalculateCollider(&gv->colliders[14], gv->oldObstaclePositions[6],
			gv->oldObstaclePositions[7], 6.0f, gv->oldObstaclePositions[8], 0,
			0, true);
	if (gv->oldViewVal >= gv->maxViewVal * 0.75f) {
		cout << "Training procedure: keep MT 1 position -> ";
		cd->recalculateCollider(&gv->colliders[15], gv->oldMTPos[0],
				gv->oldMTPos[1], gv->objectOffsets[4], gv->oldMTPos[2], 0, 0,
				false, true, gv->camVectorsMT1);
	} else {
		cout << "Training procedure: MT 1 moved to new position -> ";
		prot->mt_protagonist(1, gv->objectIndex, false);
		gv->MT1posChanged = true;
		cd->recalculateCollider(&gv->colliders[15], gv->tempMTPos[0],
				gv->tempMTPos[1], gv->objectOffsets[4], gv->tempMTPos[2], 0, 0,
				false, true, gv->camVectorsMT1);
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
		if (gv->MT1collision) {
//#if defined(WIN32)
			 sendMessageML("mt1_reward,-1000");
			 receiveMessageML();
//#endif
		}
		mt->updateMonitoredSpace(false, false);
		for (int i = 0; i < 3; i++) {
			if ((abs(gv->lastDetectedObsPosX[i]) > cd->epsilon3
					|| abs(gv->lastDetectedObsPosY[i]) > cd->epsilon3)
					&& (abs(gv->currentDetectedObsPosX[i]) < cd->epsilon3
							&& abs(gv->currentDetectedObsPosY[i]) < cd->epsilon3)) {
				gv->currentDetectedObsPosX[i] = gv->lastDetectedObsPosX[i];
				gv->currentDetectedObsPosY[i] = gv->lastDetectedObsPosY[i];
				gv->currentDetectedObsPosA[i] = gv->lastDetectedObsPosA[i];
			}
		}
	}
	if (!gv->MT1collision) {
		if (gv->maxViewVal >= 29.0f) {
			cout << "MT 2 is not needed and not moved -> ";
		} else {
			cout << "MT 2 moved to new position -> ";
			gv->oldMTPos[0] = gv->colliders[16].offsets[0];
			gv->oldMTPos[1] = gv->colliders[16].offsets[1];
			gv->oldMTPos[2] = gv->colliders[16].angles[0];
			srand(1);
			prot->mt_preparation(2, generator1, distribution1, generator2,
					distribution2, generator3, distribution3);
			srand((unsigned) (time(NULL)));
			cd->recalculateCollider(&gv->colliders[12],
					gv->oldObstaclePositions[0], gv->oldObstaclePositions[1],
					4.0f, gv->oldObstaclePositions[2], 0, 0, true);
			cd->recalculateCollider(&gv->colliders[13],
					gv->oldObstaclePositions[3], gv->oldObstaclePositions[4],
					3.0f, gv->oldObstaclePositions[5], 0, 0, true);
			cd->recalculateCollider(&gv->colliders[14],
					gv->oldObstaclePositions[6], gv->oldObstaclePositions[7],
					6.0f, gv->oldObstaclePositions[8], 0, 0, true);
			prot->mt_protagonist(2, gv->objectIndex, false);
			gv->MT2posChanged = true;
			cd->recalculateCollider(&gv->colliders[16], gv->tempMTPos[0],
					gv->tempMTPos[1], gv->objectOffsets[4], gv->tempMTPos[2], 0,
					0, false, true, gv->camVectorsMT2);
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
			if (gv->MT2collision) {
//#if defined(WIN32)
				 sendMessageML("mt2_reward,-1000");
				 receiveMessageML();
//#endif
			}
			mt->updateMonitoredSpace(false, false);
			for (int i = 0; i < 3; i++) {
				if ((abs(gv->lastDetectedObsPosX[i]) > cd->epsilon3
						|| abs(gv->lastDetectedObsPosY[i]) > cd->epsilon3)
						&& (abs(gv->currentDetectedObsPosX[i]) <= cd->epsilon3
								&& abs(gv->currentDetectedObsPosY[i])
										<= cd->epsilon3)) {
					gv->currentDetectedObsPosX[i] = gv->lastDetectedObsPosX[i];
					gv->currentDetectedObsPosY[i] = gv->lastDetectedObsPosY[i];
					gv->currentDetectedObsPosA[i] = gv->lastDetectedObsPosA[i];
				}
			}
		}
	}
	if (!gv->MT1collision && !gv->MT2collision) {
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
			pp->optimizePath();
			fo->drawPathAndCalculation();
			mt->updateMonitoredSpaceJustPlot();
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
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
			hf->calcEdgesForPatterns(gv->object_pattern_edges,
					&gv->colliders[14], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[0], 0);
			gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[1], gv->tempCounter);
			hf->calcEdgesForMesh(gv->distorted_obs_edges,
					&gv->distortedObstacles[2], gv->tempCounter);
			gv->adv_dev_x = 0.0f;
			gv->adv_dev_y = 0.0f;
			gv->adv_dev_z = 0.0f;
			repaint = true;
			gv->virtSim = true;
			cout << "first gv->path planning was successful" << endl;
		} else {
			cout << "first gv->path planning failed -> abort..." << endl;
			gv->modeActive = false;
		}
	} else {
		cout << "MT 1/MT 2 reposition ended with gv->collision -> abort..."
				<< endl;
		gv->modeActive = false;
		repaint = false;
	}
//	return repaint;
}



/*
 * this function controls the actions, the simulation should take when a key is released.
 *
 * This function defines the mode change.
 * t = training
 * i = init
 * s = simulation
 * m = manual control
 * SPACE = ???
 * p = pause
 * 1 = view perception of MT1
 * 2 = view perception of MT2
 * k = save marker position
 * l = load marker postition
 *
 * The respective state variables are hold in GlobalVariables.
 */
bool SimulationControl::sim_control_keyup(SDL_Event event, bool repaint_old, bool boWaitForSpace){
	bool repaint=repaint_old;

	CollisionDetection *cd = CollisionDetection::get_instance();;
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();
	if (event.key.keysym.sym == SDLK_t) {
		gv->asynMode = 0;
		gv->initConfirmation = false;
		gv->simConfirmation = false;
		gv->manualConfirmation = false;
		cout << "Info: changing operation mode to training" << endl;
	}
	else if (event.key.keysym.sym == SDLK_i) {
		if (gv->atStandby) {
			gv->asynMode = 1;
			gv->initConfirmation = true;
			gv->simConfirmation = false;
			gv->manualConfirmation = false;

			gv->initSpacePressed = false;
			cout << "Info: changing operation mode to initialization" << endl;
		}
		else cout << "Warning: move the real robot back to the standby position first before switching to init. mode!" << endl;
	}
	else if (event.key.keysym.sym == SDLK_s) {
		if (gv->atStandby) {
			if (!gv->initProcessed) cout << "Warning: initialization was not processed yet. Do this first!" << endl;
			else {
				gv->asynMode = 2;
				gv->initConfirmation = false;
				gv->simConfirmation = true;
				gv->manualConfirmation = false;
				cout << "Info: changing operation mode to simulation" << endl;
			}
		}
		else cout << "Warning: move the real robot back to the standby position first before switching to simulation mode!" << endl;
	}
	else if (event.key.keysym.sym == SDLK_m) {
		if (!gv->initProcessed) cout << "Warning: initialization was not processed yet. Do this first!" << endl;
		else {
			gv->asynMode = 4;
			gv->initConfirmation = false;
			gv->simConfirmation = false;
			gv->manualConfirmation = true;
			cout << "Info: changing operation mode to manual control" << endl;
		}
	}

	if (event.key.keysym.sym == SDLK_SPACE) {
		if (gv->synMode == 1) gv->initSpacePressed = true;
	}
	if (event.key.keysym.sym == SDLK_p) {
		if (gv->virtSim) gv->paused = !gv->paused;
	}

	if (event.key.keysym.sym == SDLK_1) {
		if (abs(gv->colliders[15].offsets[2]) < 100.0f && gv->MT1currentlyActive) {
			gv->camPos[0] = gv->camVectorsMT1[0];
			gv->camPos[1] = gv->camVectorsMT1[1];
			gv->camPos[2] = gv->camVectorsMT1[2];
			gv->camAngles[0] = gv->colliders[15].angles[0];
			gv->camAngles[1] = cd->nickAngleMT / 180.0f * M_PI;
			hf->updateTransformationMatrix();
			repaint = true;
		}
	}
	else if (event.key.keysym.sym == SDLK_2) {
		if (abs(gv->colliders[16].offsets[2]) < 100.0f && gv->MT2currentlyActive) {
			gv->camPos[0] = gv->camVectorsMT2[0];
			gv->camPos[1] = gv->camVectorsMT2[1];
			gv->camPos[2] = gv->camVectorsMT2[2];
			gv->camAngles[0] = gv->colliders[16].angles[0];
			gv->camAngles[1] = cd->nickAngleMT / 180.0f * M_PI;
			hf->updateTransformationMatrix();
			repaint = true;
		}
	}

	if (event.key.keysym.sym == SDLK_k && gv->selectionMarkerMode) {
		gv->savedPos.x = gv->targetPos.x;
		gv->savedPos.y = gv->targetPos.y;
		gv->savedPos.z = gv->targetPos.z;
		gv->savedRot = gv->rotationValue;
		cout << "savedPos.x: "<< gv->savedPos.x << endl;
		cout << "savedPos.y: "<< gv->savedPos.y << endl;
		cout << "savedPos.z: "<< gv->savedPos.z << endl;
		cout << "savedPos.rot: "<< gv->savedRot << endl;
		cout << "Info: marker position saved" << endl;

	}
	else if (event.key.keysym.sym == SDLK_l && gv->selectionMarkerMode) {
		gv->targetPos = { gv->savedPos.x, gv->savedPos.y, gv->savedPos.z };
		gv->rotationValue = gv->savedRot;
		cd->recalculateCollider(&gv->colliders[0], gv->targetPos.x, gv->targetPos.y, gv->targetPos.z, 0.0f, 0.0f, 0.0f);
		hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[0], gv->totalPlatePoints);
		repaint = true;
		cout << "LoadedPos.x: "<< gv->targetPos.x << endl;
		cout << "LoadedPos.y: "<< gv->targetPos.y << endl;
		cout << "LoadedPos.z: "<< gv->targetPos.z << endl;
		cout << "LoadedPos.rot: "<< gv->rotationValue<< endl;
		cout << "Info: marker position loaded" << endl;
	}
	return repaint;
}



bool SimulationControl::sim_control_close() {
	bool close=true;
	GlobalVariables *gv = GlobalVariables::get_instance();
//#if defined(WIN32)
	shutdownML();
//#endif
	if (!gv->virtualMode) {
//#if defined(WIN32)
		shutdownMT1();
		shutdownMT2();
//#endif
	}
//#if defined(WIN32)
	shutdownPATH();
	shutdownSTATE();
//#endif
	return close;
}

/*
 * this function handels the mouse ctrl.
 *
 */
void SimulationControl::sim_control_mouse_ctrl(SDL_Event event, SDL_Window* window,
		bool &repaint,
		bool &forwards,
		bool &backwards,
		bool &left,
		bool &right,
		bool &up,
		bool &down,
		bool &rotationDown,
		bool &rotationUp){

	CollisionDetection *cd = CollisionDetection::get_instance();;
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();
	if (event.type == SDL_KEYUP) {
		if (event.key.keysym.sym == SDLK_f) hf->increaseSimSpeed();

		if (event.key.keysym.sym == SDLK_r && gv->selectionMarkerMode) {
			hf->resetSelectionMarkerPos();
			repaint = true;
		}
		else if (event.key.keysym.sym == SDLK_r && !gv->selectionMarkerMode) {
			hf->resetCamPos();
			repaint = true;
		}

		if (event.key.keysym.sym == SDLK_UP) forwards = false;
		if (event.key.keysym.sym == SDLK_RIGHT) right = false;
		if (event.key.keysym.sym == SDLK_DOWN) backwards = false;
		if (event.key.keysym.sym == SDLK_LEFT) left = false;

		if (event.key.keysym.sym == 1073741914) down = false;
		if (event.key.keysym.sym == 1073741920) up = false;
	}

	if (event.type == SDL_KEYDOWN) {
		if (event.key.keysym.sym == SDLK_UP) forwards = true;
		if (event.key.keysym.sym == SDLK_RIGHT) right = true;
		if (event.key.keysym.sym == SDLK_DOWN) backwards = true;
		if (event.key.keysym.sym == SDLK_LEFT) left = true;

		if (event.key.keysym.sym == 1073741914) down = true;
		if (event.key.keysym.sym == 1073741920) up = true;
	}

	if (event.type == SDL_MOUSEWHEEL) {
		if (event.wheel.y < 0) rotationDown = true;
		else if (event.wheel.y > 0) rotationUp = true;
	}

	if (event.type == SDL_MOUSEBUTTONUP) {
		if (event.button.button == SDL_BUTTON_MIDDLE && gv->selectionMarkerMode) {
			if (hf->processConfirmButtonPress()){
				// cout << "debug@SC l. "<<__LINE__<<endl;
				repaint = true;
			}
		}
		else if (event.button.button == SDL_BUTTON_MIDDLE && gv->syncSim) {
			gv->userAbort = true;
		}

		if (event.button.button == SDL_BUTTON_RIGHT && gv->simTargetSelection) {
			hf->selectionMarkerModeSwitch();
			repaint = true;
		}
	}

	if (event.type == SDL_MOUSEMOTION) {
		gv->axisXold = event.motion.x;
		gv->axisYold = event.motion.y;

		int winXPos, winYPos;
		SDL_GetWindowPosition(window, &winXPos, &winYPos);
		winXPos += 400;
		winYPos += 400;

		gv->axisX = gv->axisXold - 400;
		gv->axisY = gv->axisYold - 400;

		gv->axisX *= 2500;
		gv->axisY *= 2500;
//TODO: implement in Linux
//					SetCursorPos(winXPos, winYPos);
	}

}

/*
 * This function handles the control of the simulation using joystick.
 * (X) = button 2 = "reset"
 * (Y) = change Simulation Speed.
 * (B) = ?
 * (A) = ?
 * Start = button 5= Enter/Leaf Control Mode
 * joystick left button = button 9 = select workpiece.
 */
void SimulationControl::sim_control_joystick(SDL_Event event,
		bool &repaint,
		bool &forwards,
		bool &backwards,
		bool &left,
		bool &right,
		bool &up,
		bool &down,
		bool &rotationDown,
		bool &rotationUp){
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();



	if (event.type == SDL_JOYBUTTONDOWN) {
		if (event.jbutton.button == 4) {
			rotationUp = true;
//			// cout << "debug: SimulationControl " << __LINE__ <<endl;
			// cout << " debug: SDL_JOYBUTTONDOWN, button = 4" <<endl;
		}
		if (event.jbutton.button == 5) {
			up = true;
//			// cout << "debug: SimulationControl " << __LINE__ <<endl;
			cout << " debug: SDL_JOYBUTTONDOWN, button = 5" <<endl;
		}
	}

	if (event.type == SDL_JOYBUTTONUP) {
		// cout << "debug: SimCtrl l." << __LINE__ <<" event ="<< (int)event.jbutton.button  <<endl;

		if (event.jbutton.button == 4) {
			rotationUp = false;
//			// cout << "debug: SimulationControl " << __LINE__ <<endl;
			cout << " debug: SDL_JOYBUTTONUP, button = 4" <<endl;
		}
		if (event.jbutton.button == 5){
			up = false;
//			// cout << "debug: SimulationControl " << __LINE__ <<endl;
			cout << " : SDL_JOYBUTTONUP, button = 5" <<endl;
		}

		if (event.jbutton.button == 2 && gv->selectionMarkerMode) {
			// cout << "debug: SimulationControl " << __LINE__ <<endl;
			cout << " debug: event.jbutton.button == 2 && gv->selectionMarkerMode" <<endl;

			hf->resetSelectionMarkerPos();
			repaint = true;
		}
		else if (event.jbutton.button == 2 && !gv->selectionMarkerMode) {
			// cout << "debug: SimulationControl " << __LINE__ <<endl;
			cout << " debug: event.jbutton.button == 2 && !gv->selectionMarkerMode" <<endl;

			hf->resetCamPos();
			repaint = true;
		}else if(event.jbutton.button == 2 ){
			// cout << "debug: SimulationControl " << __LINE__ <<endl;
			cout << " debug: event.jbutton.button == 2" <<endl;
		}

		if (event.jbutton.button == 9 && gv->selectionMarkerMode) {
			// cout << "debug: SimulationControl " << __LINE__ <<endl;
			if (hf->processConfirmButtonPress()){
				// cout << "debug@SC l. "<<__LINE__<<endl;
				repaint = true;
				// cout << "debug: SimulationControl " << __LINE__ <<endl;
				cout << " debug: event.jbutton.button == 9 && gv->selectionMarkerMode" <<endl;
			}

		}
		else if (event.jbutton.button == 9 && gv->syncSim){
			gv->userAbort = true;
			// cout << "debug: SimulationControl " << __LINE__ <<endl;
			cout << " debug: event.jbutton.button == 9 && gv->syncSim" <<endl;
		}
		else if(event.jbutton.button == 9){
			// cout << "debug: SimulationControl " << __LINE__ <<endl;
			cout << " event.jbutton.button == 9" <<endl;
		}

		if (event.jbutton.button == 3) {
			hf->increaseSimSpeed();
			cout << " event.jbutton.button == 3" <<endl;
		}

		if (event.jbutton.button == 7 && gv->simTargetSelection) {
			hf->selectionMarkerModeSwitch();
			repaint = true;
			cout << " event.jbutton.button == 7 && gv->simTargetSelection" <<endl;
		}else if(event.jbutton.button == 7 ){
			// cout << "debug: SimulationControl " << __LINE__ <<endl;
		}
	}

	if (event.type == SDL_JOYAXISMOTION) {
		if (event.jaxis.axis == 2)
		{
			gv->axisX = event.jaxis.value;
			if (abs(gv->axisX) < 1500) gv->axisX = 0;
		}

		if (event.jaxis.axis == 3)
		{
			gv->axisY = event.jaxis.value;
			if (abs(gv->axisY) < 1500) gv->axisY = 0;
		}

		if (event.jaxis.axis == 4)
		{
			if (event.jaxis.value > -32000) rotationDown = true;
			else rotationDown = false;
		}

		if (event.jaxis.axis == 5)
		{
			if (event.jaxis.value > -32000) down = true;
			else down = false;
		}
	}

	if (event.type == SDL_JOYHATMOTION) {
		if (event.jhat.value == 1) forwards = true;
		else if (event.jhat.value == 2) right = true;
		else if (event.jhat.value == 4) backwards = true;
		else if (event.jhat.value == 8) left = true;
		else {
			forwards = false; backwards = false; right = false; left = false;
		}
	}
}


void SimulationControl::sim_control_take_action_to_user_input(
		bool &repaint,
		bool &forwards,
		bool &backwards,
		bool &left,
		bool &right,
		bool &up,
		bool &down,
		bool &rotationDown,
		bool &rotationUp
){
	HelperFunctions *hf = HelperFunctions::get_instance();
	CollisionDetection *cd = CollisionDetection::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();
	if (gv->axisX != 0 || gv->axisY != 0) {
		if (!gv->selectionMarkerMode) {
			gv->camAngles[0] -= ((float)gv->axisX) / 32768.0f * 0.015f;
			gv->camAngles[1] -= ((float)gv->axisY) / 32768.0f * 0.015f;

			if (gv->camAngles[0] >= 2 * M_PI) {
				gv->camAngles[0] -= 2 * M_PI;
			}
			else if (gv->camAngles[0] < 0) {
				gv->camAngles[0] += 2 * M_PI;
			}

			if (gv->camAngles[1] > M_PI / 2) gv->camAngles[1] = M_PI / 2;
			if (gv->camAngles[1] < -M_PI / 2) gv->camAngles[1] = -M_PI / 2;
			hf->updateTransformationMatrix();
			repaint = true;
		}
	}

	if (gv->selectionMarkerMode == false) {
		if (forwards) {
			gv->camPos[0] += 0.25f * cos(gv->camAngles[0]);
			gv->camPos[1] += 0.25f * sin(gv->camAngles[0]);
			repaint = true;
		}
		else if (backwards) {
			gv->camPos[0] -= 0.25f * cos(gv->camAngles[0]);
			gv->camPos[1] -= 0.25f * sin(gv->camAngles[0]);
			repaint = true;
		}
		else if (right) {
			gv->camPos[0] += gv->camHorDir[0] * 0.25f;
			gv->camPos[1] += gv->camHorDir[1] * 0.25f;
			repaint = true;
		}
		else if (left) {
			gv->camPos[0] -= gv->camHorDir[0] * 0.25f;
			gv->camPos[1] -= gv->camHorDir[1] * 0.25f;
			repaint = true;
		}
		if (up) {
			gv->camPos[2] += 0.25f;

			if (gv->camPos[2] > 75) {
				gv->camPos[2] = 75;
			}
			repaint = true;
		}
		else if (down) {
			gv->camPos[2] -= 0.25f;

			if (gv->camPos[2] < 0) {
				gv->camPos[2] = 0;
			}
			repaint = true;
		}
	}
	else {
		if (forwards) {
			gv->targetPos.x += 0.25f * cos(gv->camAngles[0]);
			gv->targetPos.y += 0.25f * sin(gv->camAngles[0]);

			if (gv->targetPos.x > 45.0f) {
				gv->targetPos.x = 45.0f;
			}

			if (gv->targetPos.x < -45.0f) {
				gv->targetPos.x = -45.0f;
			}

			if (gv->targetPos.y > 45.0f) {
				gv->targetPos.y = 45.0f;
			}

			if (gv->targetPos.y < -12.5f) {
				gv->targetPos.y = -12.5f;
			}

			cd->recalculateCollider(&gv->colliders[0], gv->targetPos.x, gv->targetPos.y, gv->targetPos.z, 0.0f, 0.0f, 0.0f);
			hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[0], gv->totalPlatePoints);
			repaint = true;
		}
		else if (backwards) {
			gv->targetPos.x -= 0.25f * cos(gv->camAngles[0]);
			gv->targetPos.y -= 0.25f * sin(gv->camAngles[0]);

			if (gv->targetPos.x > 45.0f) {
				gv->targetPos.x = 45.0f;
			}

			if (gv->targetPos.x < -45.0f) {
				gv->targetPos.x = -45.0f;
			}

			if (gv->targetPos.y > 45.0f) {
				gv->targetPos.y = 45.0f;
			}

			if (gv->targetPos.y < -12.5f) {
				gv->targetPos.y = -12.5f;
			}

			cd->recalculateCollider(&gv->colliders[0], gv->targetPos.x, gv->targetPos.y, gv->targetPos.z, 0.0f, 0.0f, 0.0f);
			hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[0], gv->totalPlatePoints);
			repaint = true;
		}
		else if (right) {
			gv->targetPos.x += gv->camHorDir[0] * 0.25f;
			gv->targetPos.y += gv->camHorDir[1] * 0.25f;

			if (gv->targetPos.x > 45.0f) {
				gv->targetPos.x = 45.0f;
			}

			if (gv->targetPos.x < -45.0f) {
				gv->targetPos.x = -45.0f;
			}

			if (gv->targetPos.y > 45.0f) {
				gv->targetPos.y = 45.0f;
			}

			if (gv->targetPos.y < -12.5f) {
				gv->targetPos.y = -12.5f;
			}

			cd->recalculateCollider(&gv->colliders[0], gv->targetPos.x, gv->targetPos.y, gv->targetPos.z, 0.0f, 0.0f, 0.0f);
			hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[0], gv->totalPlatePoints);
			repaint = true;
		}
		else if (left) {
			gv->targetPos.x -= gv->camHorDir[0] * 0.25f;
			gv->targetPos.y -= gv->camHorDir[1] * 0.25f;

			if (gv->targetPos.x > 45.0f) {
				gv->targetPos.x = 45.0f;
			}

			if (gv->targetPos.x < -45.0f) {
				gv->targetPos.x = -45.0f;
			}

			if (gv->targetPos.y > 45.0f) {
				gv->targetPos.y = 45.0f;
			}

			if (gv->targetPos.y < -12.5f) {
				gv->targetPos.y = -12.5f;
			}

			cd->recalculateCollider(&gv->colliders[0], gv->targetPos.x, gv->targetPos.y, gv->targetPos.z, 0.0f, 0.0f, 0.0f);
			hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[0], gv->totalPlatePoints);
			repaint = true;
		}
		if (up) {
			gv->targetPos.z += 0.25f;

			if (gv->targetPos.z > 25.0f) {
				gv->targetPos.z = 25.0f;
			}

			cd->recalculateCollider(&gv->colliders[0], gv->targetPos.x, gv->targetPos.y, gv->targetPos.z, 0.0f, 0.0f, 0.0f);
			hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[0], gv->totalPlatePoints);
			repaint = true;
		}
		else if (down) {
			gv->targetPos.z -= 0.25f;

			if (gv->targetPos.z < 0.0f) {
				gv->targetPos.z = 0.0f;
			}
			cd->recalculateCollider(&gv->colliders[0], gv->targetPos.x, gv->targetPos.y, gv->targetPos.z, 0.0f, 0.0f, 0.0f);
			hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[0], gv->totalPlatePoints);
			repaint = true;
		}
		if (rotationUp) {
			gv->rotationValue += 0.2f;
			if (gv->rotationValue >= 360.0f) gv->rotationValue = 360 - 0.2f;
		}
		else if (rotationDown) {
			gv->rotationValue -= 0.2f;
			if (gv->rotationValue < 0.0f) gv->rotationValue = 0.0f;
		}
	}
}


/*
 * Todo: find proper description!!!
 */
void SimulationControl::sim_control_wait_for_completed_init(bool boWaitForUserInput){
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();

	gv->initWaitForFinish = false;

	while (true) {
//#if defined(WIN32)
		//  cout << "debug: simctrl " << __LINE__ <<endl;
		 sendMessageSTATE("state_request");
		 receiveMessageSTATE();
//#endif
		vector<string> splitList;
		std::stringstream string_stream;
		string splitted;

		string_stream.str(gv->stateMessageSTATE);
		while (getline(string_stream, splitted, ',')) {
			splitList.push_back(splitted);
		}

		if (stoi(splitList[splitList.size() - 1]) == gv->firstSegSize) {
			break;
		}
	}

	cout << "Init. info: press space to continue after the object to transport is placed like shown between the grippers..." << endl;
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

	gv->pathDataForKuka = "normal,";
	for (int i = 0; i < gv->secondSegSize; i++) {
		gv->pathDataForKuka += hf->to_string_with_precision(gv->q0ValsSecondSeg[i], 4) + "," + hf->to_string_with_precision(gv->q1ValsSecondSeg[i], 4) + "," + hf->to_string_with_precision(gv->q2ValsSecondSeg[i], 4) + "," + hf->to_string_with_precision(gv->q3ValsSecondSeg[i], 4) + "," + hf->to_string_with_precision(gv->q4ValsSecondSeg[i], 4) + ",";
	}
	gv->pathDataForKuka += hf->to_string_with_precision(gv->grippingWidthFixed, 4);
//#if defined(WIN32)
	// cout << "debug: simctrl " << __LINE__ <<endl;
	//  sendMessagePATH(gv->pathDataForKuka);
	//  receiveMessagePATH();
//#endif

	while (true) {
//#if defined(WIN32)
		//  cout << "debug: simctrl " << __LINE__ <<endl;
		//  sendMessageSTATE("state_request");
		//  receiveMessageSTATE();
//#endif

		vector<string> splitList;
		std::stringstream string_stream;
		string splitted;

		string_stream.str(gv->stateMessageSTATE);
		while (getline(string_stream, splitted, ',')) {
			splitList.push_back(splitted);
		}

		if (stoi(splitList[splitList.size() - 1]) == gv->secondSegSize) {
			break;
		}
	}

	gv->pathDataForKuka = "normal,";
	for (int i = 0; i < gv->thirdSegSize; i++) {
		gv->pathDataForKuka += hf->to_string_with_precision(gv->q0ValsThirdSeg[i], 4) + "," + hf->to_string_with_precision(gv->q1ValsThirdSeg[i], 4) + "," + hf->to_string_with_precision(gv->q2ValsThirdSeg[i], 4) + "," + hf->to_string_with_precision(gv->q3ValsThirdSeg[i], 4) + "," + hf->to_string_with_precision(gv->q4ValsThirdSeg[i], 4) + ",";
	}
	gv->pathDataForKuka += hf->to_string_with_precision(gv->grippingWidthOpen, 4);
//#if defined(WIN32)
	// cout << "debug: simctrl " << __LINE__ <<endl;
	//  sendMessagePATH(gv->pathDataForKuka);
	//  receiveMessagePATH();
//#endif

	while (true) {
//#if defined(WIN32)
		//  cout << "debug: simctrl " << __LINE__ <<endl;
		//  sendMessageSTATE("state_request");
		//  receiveMessageSTATE();
//#endif

		vector<string> splitList;
		std::stringstream string_stream;
		string splitted;

		string_stream.str(gv->stateMessageSTATE);
		while (getline(string_stream, splitted, ',')) {
			splitList.push_back(splitted);
		}

		if (stoi(splitList[splitList.size() - 1]) == gv->thirdSegSize) {
			cout << "Init. info: physical robot movement finished" << endl;
			break;
		}
	}

}

//
//void SimulationControl::sim_control_transportPhase1(bool &repaint,
//		default_random_engine generator1,
//		normal_distribution<float> distribution1,
//		default_random_engine generator2,
//		normal_distribution<float> distribution2,
//		default_random_engine generator3,
//		normal_distribution<float> distribution3)
//{
//	HelperFunctions *hf = HelperFunctions::get_instance();
//	GlobalVariables *gv = GlobalVariables::get_instance();
//	PathPlanner *pp=PathPlanner::get_instance();
//	FrequentOps *fo=FrequentOps::get_instance();
//
//	gv->grippingWidth = gv->grippingWidthFixed;
//
//	// std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//	gv->objectPosReachable = false;
//	for (int i = 0; i < 10; i++) {
//		if (pp->calcPath(gv->qValuesObjectEnd, gv->qValuesObjectStart, &gv->pathEndPos, &gv->pathStartPos, false, generator1,distribution1, generator2,distribution2,generator3,distribution3)) {
//			pp->optimizePath();
//			gv->objectPosReachable = true;
//			break;
//		}
//	}
//	// std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//	// std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
//
//	if (gv->objectPosReachable) {
//		pp->optimizePath();
//		cout << "Training info: second path planning successful" << endl;
//
//		fo->drawPathAndCalculation();
//
//		gv->adv_dev_x = 0.0f; gv->adv_dev_y = 0.0f; gv->adv_dev_z = 0.0f;
//
//		repaint = true;
//		gv->virtSim = true;
//	}
//	else {
//		cout << "Training info: second path planning failed -> abort..." << endl;
//		gv->transportPhase = 0;
//		gv->modeActive = false;
//	}
//}

//
//void SimulationControl::sim_control_transportPhas2(
//		bool &repaint,
//		default_random_engine generator1,
//		normal_distribution<float> distribution1,
//		default_random_engine generator2,
//		normal_distribution<float> distribution2,
//		default_random_engine generator3,
//		normal_distribution<float> distribution3){
//	HelperFunctions *hf = HelperFunctions::get_instance();
//	GlobalVariables *gv = GlobalVariables::get_instance();
//	PathPlanner *pp=PathPlanner::get_instance();
//	FrequentOps *fo=FrequentOps::get_instance();
//
//	gv->grippingWidth = gv->grippingWidthOpen;
//
//	gv->objectPosReachable = false;
//	for (int i = 0; i < 10; i++) {
//		if (pp->calcPath(gv->qValuesStandby, gv->qValuesObjectEnd, &gv->standbyPos, &gv->pathEndPos, false, generator1,distribution1, generator2,distribution2,generator3,distribution3)) {
//			gv->objectPosReachable = true;
//			break;
//		}
//	}
//
//	if (gv->objectPosReachable) {
//		pp->optimizePath();
//		cout << "Training info: third gv->path planning successful" << endl;
//
//		fo->drawPathAndCalculation();
//
//		gv->adv_dev_x = 0.0f; gv->adv_dev_y = 0.0f; gv->adv_dev_z = 0.0f;
//
//		repaint = true;
//		gv->virtSim = true;
//	}
//	else {
//		cout << "Training info: third gv->path planning failed -> abort..." << endl;
//		gv->transportPhase = 0;
//		gv->modeActive = false;
//	}
//
//}


void SimulationControl::sim_control_enter_seed(
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{
	cout << "Init. info: enter seed value (press 1 (use 98), press 2 (use 99), press 3 (use 100), press 4 (use 101), press 5 (use 102))" << endl;;
	unsigned int seed;

	char c_in = '-';
	SDL_Event event_sub;
	do {
		while (SDL_PollEvent(&event_sub)) {
			if (event_sub.type == SDL_KEYUP) {
				if (event_sub.key.keysym.sym == SDLK_1) {
					c_in = '1';
				}
				else if (event_sub.key.keysym.sym == SDLK_2) {
					c_in = '2';
				}
				else if (event_sub.key.keysym.sym == SDLK_3) {
					c_in = '3';
				}
				else if (event_sub.key.keysym.sym == SDLK_4) {
					c_in = '4';
				}
				else if (event_sub.key.keysym.sym == SDLK_5) {
					c_in = '5';
				}
			}
		}
	} while (c_in == '-');

	if (c_in == '1') seed = 98;
	else if (c_in == '2') seed = 99;
	else if (c_in == '3') seed = 100;
	else if (c_in == '4') seed = 101;
	else if (c_in == '5') seed = 102;

	cout << "Init. info: selected seed: " << to_string(seed) << endl;

	srand(seed);
	generator1.seed(seed);
	generator2.seed(seed);
	generator3.seed(seed);

	cout << "Init. info: first seed values: " << rand() << " " << rand() << " " << rand() << endl;

}

/*
 * TODO: discribe what this function actually does. It is not clear to me. Is the title well-chosen???
 */
void SimulationControl::sim_control_reorganize_after_init(
		bool &repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	MonitoringToolModels *mt = MonitoringToolModels::get_instance();
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

	if (!gv->MT1currentlyActive || !gv->MT2currentlyActive) cout << "Error: a MT is not active!" << endl;

	if (gv->advancedControlMode) {
		sc->sim_control_enter_seed(
				generator1,distribution1,
				generator2,distribution2,
				generator3,distribution3);
	}

	gv->modeActive = true;
	gv->collision = false;
	gv->grippingWidth = 0.0f;

	gv->q0Sin = sin(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q0Cos = cos(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q1Sin = sin(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q1Cos = cos(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q12Sin = sin((gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q12Cos = cos((gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q123Sin = sin((gv->qValuesStandby[1] + gv->qValuesStandby[2] + gv->qValuesStandby[3]) / 180.0f * M_PI);
	gv->q123Cos = cos((gv->qValuesStandby[1] + gv->qValuesStandby[2] + gv->qValuesStandby[3]) / 180.0f * M_PI);

	cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
	cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Sin + 4.5f * gv->q0Cos, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Cos - 4.5f * gv->q0Sin, ik->len0 + ik->len2 / 2.0f * gv->q1Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1]) / 180.0f * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Sin - 4.155f * gv->q0Cos, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Cos + 4.155f * gv->q0Sin, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 / 2.0f * gv->q12Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2]) / 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->lastSegmentMid * gv->q123Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180 * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);

	cd->updateMatricesTransport((-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, (gv->qValuesStandby[4] + 0) / 180.0f * M_PI, gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
	cd->rotationMatrix(gv->q0Cos * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
	gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

	cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin) * gv->q0Cos + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos + gv->grippingDiffVector[2], (-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);
	cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin) * gv->q0Cos - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos - gv->grippingDiffVector[2], (-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);

	gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
	gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
	gv->numPointsPlannedPath = 0;

	gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
	gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
	gv->numPointsRealPath = 0;

	cd->recalculateCollider(&gv->colliders[8], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true, false, nullptr, true, cd->workpieceArrowPosition1);
	hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 1);
	cd->recalculateCollider(&gv->colliders[9], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true, false, nullptr, true, cd->workpieceArrowPosition2);
	hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 2);
	cd->recalculateCollider(&gv->colliders[10], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true, false, nullptr, true, cd->workpieceArrowPosition3);
	hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 3);
	cd->recalculateCollider(&gv->colliders[11], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true, false, nullptr, true, cd->workpieceArrowPosition4);
	hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 4);

	for (int i = 12; i <= 14; i++) {
		cd->recalculateCollider(&gv->colliders[i], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
	}

	cd->recalculateCollider(&gv->colliders[15], 0.0f, 0.0f, 10.0e5, 0, 0, 0, false, true, gv->camVectorsMT1);
	hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
	cd->recalculateCollider(&gv->colliders[16], 0.0f, 0.0f, 10.0e5, 0, 0, 0, false, true, gv->camVectorsMT2);
	hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);

	for (int i = 0; i < 9; i++) {
		gv->distortions[i] = 0.0f;
	}

	cd->recalculateCollider(&gv->distortedObstacles[0], gv->colliders[12].offsets[0] + gv->distortions[0], gv->colliders[12].offsets[1] + gv->distortions[1], gv->colliders[12].offsets[2], gv->colliders[12].angles[0] + gv->distortions[2], 0, 0);
	cd->recalculateCollider(&gv->distortedObstacles[1], gv->colliders[13].offsets[0] + gv->distortions[3], gv->colliders[13].offsets[1] + gv->distortions[4], gv->colliders[13].offsets[2], gv->colliders[13].angles[0] + gv->distortions[5], 0, 0);
	cd->recalculateCollider(&gv->distortedObstacles[2], gv->colliders[14].offsets[0] + gv->distortions[6], gv->colliders[14].offsets[1] + gv->distortions[7], gv->colliders[14].offsets[2], gv->colliders[14].angles[0] + gv->distortions[8], 0, 0);

	cd->overrideCollider(gv->workpieceMesh, &gv->colliders[8]);
	cd->recalculateCollider(gv->workpieceMesh, 0.0f, 0.0f, 10.0e5, 0, 0, 0);

	mt->updateMonitoredSpace(true, true);

	gv->tempCounter = gv->totalPlatePoints;
	for (int i = 0; i < 3; i++) {
		gv->tempCounter += gv->colliders[i].facesTimes3;
	}
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[3], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[4], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[5], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[6], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[7], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[8], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[9], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[10], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[11], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[12], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[13], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[14], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[15], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[16], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[0], gv->tempCounter);
	hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[1], gv->tempCounter);

	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[8], 0);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[9], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[10], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[11], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[12], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[13], gv->tempCounter);
	hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[14], gv->tempCounter);

	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[0], 0);
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[1], gv->tempCounter);
	hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2], gv->tempCounter);

	hf->calcEdgesForMesh(gv->workpiece_mesh_edges, gv->workpieceMesh, 0);
	gv->initPhase = 1;
	repaint = true;
}


void SimulationControl::sim_control_add_obj_during_initialization(
		bool &repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3) {

	GlobalVariables *gv=GlobalVariables::get_instance();
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();

	if (gv->initPhase > 1) {

		gv->tempCounter = gv->totalPlatePoints;
		for (int i = 0; i < gv->transportableObjects[gv->initPhase - 2]; i++) {
			gv->tempCounter += gv->colliders[i].facesTimes3;
		}
		hf->calcEdgesForConvexHull(gv->object_edges,
				&gv->colliders[gv->transportableObjects[gv->initPhase - 2]],
				gv->tempCounter);
		if (gv->transportableObjects[gv->initPhase - 2] < 12) {
			gv->tempCounter = 0;
			for (int i = 8; i < gv->transportableObjects[gv->initPhase - 2];
					i++) {
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
		} else if (gv->transportableObjects[gv->initPhase - 2] == 15)
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
		else if (gv->transportableObjects[gv->initPhase - 2] == 16) {
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges,
					gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices,
					gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;
			cd->recalculateCollider(gv->workpieceMesh, 0.0f, 0.0f, 10.0e5, 0, 0,
					0);
			hf->calcEdgesForMesh(gv->workpiece_mesh_edges, gv->workpieceMesh,
					0);
		}
	}

	if (gv->initPhase <= gv->transportableObjectsCount) {


		gv->objectToTransport = gv->transportableObjects[gv->initPhase - 1];
		gv->objectHeight = gv->objectHeights[gv->initPhase - 1];
		gv->objectOffset = gv->objectOffsets[gv->initPhase - 1];
		gv->objectGripDepth = gv->objectGripDepths[gv->initPhase - 1];
		cd->recalculateCollider(&gv->colliders[gv->objectToTransport], 21.0f,
				-21.0f, gv->objectOffset, 0, 0, 0);
		gv->objectDimension.x = gv->objectDimensions[gv->initPhase - 1].x
				+ 0.5f;
		gv->objectDimension.y = gv->objectDimensions[gv->initPhase - 1].y
				+ 0.5f;
		gv->objectDimension.z = gv->objectDimensions[gv->initPhase - 1].z
				+ 0.5f;
		cd->updateColliderSize(gv->safetyCollider, gv->objectDimension.x,
				gv->objectDimension.y, gv->objectDimension.z);
		gv->pathStartPos.x = gv->colliders[gv->objectToTransport].offsets[0];
		gv->pathStartPos.y = gv->colliders[gv->objectToTransport].offsets[1];
		gv->pathStartPos.z = gv->objectOffset + gv->objectHeight
				+ gv->objectGripDepth;
		gv->startObjectAngle = gv->colliders[gv->objectToTransport].angles[0]
				* 180.0f / M_PI;
		if (ik->doesThetaExist(gv->pathStartPos.x, gv->pathStartPos.y,
				gv->pathStartPos.z))
		{


			ik->writeQValues(gv->qValuesObjectStart);



			if (abs(
					gv->qValuesObjectStart[1] + gv->qValuesObjectStart[2]
							+ gv->qValuesObjectStart[3] - 180.0f)
					>= cd->epsilon3) {
				cout << "Init. error: start theta value not 180 degree!"
						<< endl;
			}
		} else {
			cout << "Init. error: start point kinematics not calculable!"
					<< endl;
		}



		do {
			gv->objectPosReachable = false;
			gv->pathEndPos.x = ((float) (rand()) / RAND_MAX) * 68.0f - 34.0f;
			gv->pathEndPos.y = ((float) (rand()) / RAND_MAX) * 46.5f - 12.5f;
			gv->pathEndPos.z = gv->objectOffset + gv->objectHeight
					+ gv->objectGripDepth;
			gv->endObjectAngle = ((float) (rand()) / RAND_MAX)
					* (360.0f - cd->epsilon4);
			// Size of MT!!
			if (sqrt(
					(gv->pathEndPos.x - 21.0f) * (gv->pathEndPos.x - 21.0f)
							+ (gv->pathEndPos.y + 21.0f)
									* (gv->pathEndPos.y + 21.0f))
					> 2.0f * sqrt(1.5f * 1.5f + 2.5f * 2.5f) + cd->epsilon3)
			{


				if (ik->doesThetaExist(gv->pathEndPos.x, gv->pathEndPos.y,
						gv->pathEndPos.z))
				{


					ik->writeQValues(gv->qValuesObjectEnd);
					if (abs(
							gv->qValuesObjectEnd[1] + gv->qValuesObjectEnd[2]
									+ gv->qValuesObjectEnd[3] - 180.0f)
							< cd->epsilon3)
					{


						gv->qValuesObjectStart[4] = gv->startObjectAngle
								- (90.0f - gv->qValuesObjectStart[0]);



						gv->qValuesObjectEnd[4] = gv->endObjectAngle
								- (90.0f - gv->qValuesObjectEnd[0]);



						fo->gripAngleCalculation(gv->initPhase - 1);



						if (gv->qValuesObjectStart[4] > ik->qUpperLimits[4]
								|| gv->qValuesObjectStart[4]
										< ik->qLowerLimits[4]
								|| gv->qValuesObjectEnd[4] > ik->qUpperLimits[4]
								|| gv->qValuesObjectEnd[4]
										< ik->qLowerLimits[4])
						{

							gv->objectPosReachable = false;
						} else {



							gv->transportPhase = 0;
							gv->grippingWidth = gv->grippingWidthOpen;
							gv->planned_path_edges = ik->cutOff(
									gv->planned_path_edges,
									gv->numPointsPlannedPath, 0);
							gv->planned_path_vertices = ik->cutOff(
									gv->planned_path_vertices,
									gv->numPointsPlannedPath, 0);
							gv->numPointsPlannedPath = 0;
							gv->objectPosReachable = pp->calcPath(
									gv->qValuesObjectStart, gv->qValuesStandby,
									&gv->pathStartPos, &gv->standbyPos, true,
									generator1, distribution1, generator2,
									distribution2, generator3, distribution3);
							if (gv->objectPosReachable)
							{


								pp->optimizePath();
								fo->drawPathOnly(2);
								if (gv->firstSegSize != 0)
								{


									gv->q0ValsFirstSeg = ik->cutOff(
											gv->q0ValsFirstSeg,
											gv->firstSegSize, 0);
									gv->q1ValsFirstSeg = ik->cutOff(
											gv->q1ValsFirstSeg,
											gv->firstSegSize, 0);
									gv->q2ValsFirstSeg = ik->cutOff(
											gv->q2ValsFirstSeg,
											gv->firstSegSize, 0);
									gv->q3ValsFirstSeg = ik->cutOff(
											gv->q3ValsFirstSeg,
											gv->firstSegSize, 0);
									gv->q4ValsFirstSeg = ik->cutOff(
											gv->q4ValsFirstSeg,
											gv->firstSegSize, 0);
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
								gv->q0ValsFirstSeg[0] =
										pp->nodeList[1].nodeQVal[0];
								gv->q1ValsFirstSeg[0] =
										pp->nodeList[1].nodeQVal[1];
								gv->q2ValsFirstSeg[0] =
										pp->nodeList[1].nodeQVal[2];
								gv->q3ValsFirstSeg[0] =
										pp->nodeList[1].nodeQVal[3];
								gv->q4ValsFirstSeg[0] =
										pp->nodeList[1].nodeQVal[4];
								gv->firstSegSize = 1;
								pp->nodeIndex = 1;
								while (pp->nodeIndex != 0)
								{



									pp->nodeIndex =
											pp->nodeList[pp->nodeIndex].previous;
									gv->q0ValsFirstSeg = ik->increaseSize(
											gv->q0ValsFirstSeg,
											gv->firstSegSize, 1);
									gv->q1ValsFirstSeg = ik->increaseSize(
											gv->q1ValsFirstSeg,
											gv->firstSegSize, 1);
									gv->q2ValsFirstSeg = ik->increaseSize(
											gv->q2ValsFirstSeg,
											gv->firstSegSize, 1);
									gv->q3ValsFirstSeg = ik->increaseSize(
											gv->q3ValsFirstSeg,
											gv->firstSegSize, 1);
									gv->q4ValsFirstSeg = ik->increaseSize(
											gv->q4ValsFirstSeg,
											gv->firstSegSize, 1);
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
								gv->objectPosReachable = pp->calcPath(
										gv->qValuesObjectEnd,
										gv->qValuesObjectStart, &gv->pathEndPos,
										&gv->pathStartPos, true, generator1,
										distribution1, generator2,
										distribution2, generator3,
										distribution3);
								//TODO: discover use of second line!!!
//								gv->objectPosReachable = pp->calcPath(gv->qValuesObjectEnd, gv->qValuesObjectStart, &gv->pathEndPos, &gv->pathStartPos, true);
//								gv->objectPosReachable = pp->calcPath(
//										gv->qValuesObjectEnd,
//										gv->qValuesObjectStart, &gv->pathEndPos,
//										&gv->pathStartPos, true, generator1,
//										distribution1, generator2,
//										distribution2, generator3,
//										distribution3);


								if (gv->objectPosReachable)
								{


									pp->optimizePath();
									fo->drawPathOnly(1);
									if (gv->secondSegSize != 0)
									{



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

									while (pp->nodeIndex != 0)
									{


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

									for (int i = 1; i < gv->secondSegSize;
											i++)
									{


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
									gv->objectPosReachable = pp->calcPath(
											gv->qValuesStandby,
											gv->qValuesObjectEnd,
											&gv->standbyPos, &gv->pathEndPos,
											false, generator1, distribution1,
											generator2, distribution2,
											generator3, distribution3);

									if (gv->objectPosReachable)
									{


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
											gv->q0ValsThirdSeg =
													ik->increaseSize(
															gv->q0ValsThirdSeg,
															gv->thirdSegSize,
															1);
											gv->q1ValsThirdSeg =
													ik->increaseSize(
															gv->q1ValsThirdSeg,
															gv->thirdSegSize,
															1);
											gv->q2ValsThirdSeg =
													ik->increaseSize(
															gv->q2ValsThirdSeg,
															gv->thirdSegSize,
															1);
											gv->q3ValsThirdSeg =
													ik->increaseSize(
															gv->q3ValsThirdSeg,
															gv->thirdSegSize,
															1);
											gv->q4ValsThirdSeg =
													ik->increaseSize(
															gv->q4ValsThirdSeg,
															gv->thirdSegSize,
															1);
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
															/ 180.0f * M_PI,
													0.0f, 0.0f, true, false,
													nullptr, true,
													cd->workpieceArrowPosition1);
										} else if (gv->objectToTransport == 9) {
											cd->recalculateCollider(
													&gv->colliders[9],
													gv->pathStartPos.x,
													gv->pathStartPos.y,
													gv->objectOffset,
													gv->startObjectAngle
															/ 180.0f * M_PI,
													0.0f, 0.0f, true, false,
													nullptr, true,
													cd->workpieceArrowPosition2);
										} else if (gv->objectToTransport
												== 10) {
											cd->recalculateCollider(
													&gv->colliders[10],
													gv->pathStartPos.x,
													gv->pathStartPos.y,
													gv->objectOffset,
													gv->startObjectAngle
															/ 180.0f * M_PI,
													0.0f, 0.0f, true, false,
													nullptr, true,
													cd->workpieceArrowPosition3);
										} else if (gv->objectToTransport
												== 11) {
											cd->recalculateCollider(
													&gv->colliders[11],
													gv->pathStartPos.x,
													gv->pathStartPos.y,
													gv->objectOffset,
													gv->startObjectAngle
															/ 180.0f * M_PI,
													0.0f, 0.0f, true, false,
													nullptr, true,
													cd->workpieceArrowPosition4);
										} else if (gv->objectToTransport
												== 15) {
											cd->recalculateCollider(
													&gv->colliders[15],
													gv->pathStartPos.x,
													gv->pathStartPos.y,
													gv->objectOffset,
													gv->startObjectAngle
															/ 180.0f * M_PI, 0,
													0, false, true,
													gv->camVectorsMT1);
										} else if (gv->objectToTransport
												== 16) {
											cd->recalculateCollider(
													&gv->colliders[16],
													gv->pathStartPos.x,
													gv->pathStartPos.y,
													gv->objectOffset,
													gv->startObjectAngle
															/ 180.0f * M_PI, 0,
													0, false, true,
													gv->camVectorsMT2);
										}
									}
								}
								gv->transportPhase = 0;
							}
						}
					}
				}
			}
		} while (!gv->objectPosReachable);



		if (gv->processInitWithRealRobot) {

//#if defined(WIN32)
			//  cout << "debug: simctrl " << __LINE__ <<endl;
			//  sendMessagePATH("zero");
			//  receiveMessagePATH();
//#endif
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
//#if defined(WIN32)
			// cout << "debug: simctrl " << __LINE__ <<endl;
			//  sendMessagePATH(gv->pathDataForKuka);
			//  receiveMessagePATH();
//#endif
			gv->initWaitForFinish = true;
		}



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
		if (gv->objectToTransport < 12) {


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
						gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true,
						false, nullptr, true, cd->workpieceArrowPosition1);
			}
			if (gv->objectToTransport == 9) {
				hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 2);
				cd->recalculateCollider(&gv->colliders[9], gv->pathEndPos.x,
						gv->pathEndPos.y, gv->objectOffset,
						gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true,
						false, nullptr, true, cd->workpieceArrowPosition2);
			}
			if (gv->objectToTransport == 10) {
				hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 3);
				cd->recalculateCollider(&gv->colliders[10], gv->pathEndPos.x,
						gv->pathEndPos.y, gv->objectOffset,
						gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true,
						false, nullptr, true, cd->workpieceArrowPosition3);
			}
			if (gv->objectToTransport == 11) {
				hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 4);
				cd->recalculateCollider(&gv->colliders[11], gv->pathEndPos.x,
						gv->pathEndPos.y, gv->objectOffset,
						gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true,
						false, nullptr, true, cd->workpieceArrowPosition4);
			}
		} else if (gv->objectToTransport == 15) {
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
			cd->recalculateCollider(&gv->colliders[15], gv->pathEndPos.x,
					gv->pathEndPos.y, gv->objectOffset,
					gv->endObjectAngle / 180.0f * M_PI, 0, 0, false, true,
					gv->camVectorsMT1);
		} else if (gv->objectToTransport == 16) {
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
			cd->recalculateCollider(&gv->colliders[16], gv->pathEndPos.x,
					gv->pathEndPos.y, gv->objectOffset,
					gv->endObjectAngle / 180.0f * M_PI, 0, 0, false, true,
					gv->camVectorsMT2);
		}
	}

	if (gv->initPhase == gv->transportableObjectsCount + 1) {
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
		if (gv->advancedControlMode) {
			srand((unsigned) (time(NULL)));
			generator1.seed((unsigned) (time(NULL)));
			generator2.seed((unsigned) (time(NULL)));
			generator3.seed((unsigned) (time(NULL)));
		}
		gv->initPhase = 0;
	} else {
		gv->initPhase++;
	}
	gv->initSpacePressed = false;
	repaint = true;
}

void SimulationControl::sim_control_print_prepare_models(){
	GlobalVariables *gv=GlobalVariables::get_instance();
	cout << endl;
	cout << "Sim. info: preparing models...";
//#if defined(WIN32)
	 sendMessageML("prepare_models");
			gv->receivedAvailableAgents =  receiveMessageML();
//#endif
	cout << " ...finished" << endl;
	cout << "------------------------------------------" << endl;
}

/*
 * Todo: find out, what this function does.
 * -it seems to be some calculations with the object positions/robot positions.
 * Depending on what the function does, it should be moved to a different module!
 */
void SimulationControl::sim_control_do_something2(){
	GlobalVariables *gv=GlobalVariables::get_instance();
	CollisionDetection *cd=CollisionDetection::get_instance();
	InverseKinematic *ik=InverseKinematic::get_instance();

	// cout << "debug: simControl l." << __LINE__ <<endl;

	gv->q0Sin = sin(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q0Cos = cos(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q1Sin = sin(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q1Cos = cos(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q12Sin = sin((gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q12Cos = cos((gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q123Sin = sin((gv->qValuesStandby[1] + gv->qValuesStandby[2] + gv->qValuesStandby[3]) / 180.0f * M_PI);
	gv->q123Cos = cos((gv->qValuesStandby[1] + gv->qValuesStandby[2] + gv->qValuesStandby[3]) / 180.0f * M_PI);

	cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
	cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Sin + 4.5f * gv->q0Cos, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Cos - 4.5f * gv->q0Sin, ik->len0 + ik->len2 / 2.0f * gv->q1Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1]) / 180.0f * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Sin - 4.155f * gv->q0Cos, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Cos + 4.155f * gv->q0Sin, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 / 2.0f * gv->q12Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2]) / 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->lastSegmentMid * gv->q123Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180 * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);

	cd->updateMatricesTransport((-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, (gv->qValuesStandby[4] + 0) / 180.0f * M_PI, gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
	cd->rotationMatrix(gv->q0Cos * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
	gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

	cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin) * gv->q0Cos + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos + gv->grippingDiffVector[2], (-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);
	cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin) * gv->q0Cos - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos - gv->grippingDiffVector[2], (-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);

	gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
	gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
	gv->numPointsPlannedPath = 0;

	gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
	gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
	gv->numPointsRealPath = 0;

	for (int i = 0; i < gv->transportableObjectsCount; i++) {
		if (i < gv->transportableObjectsCount - 2) {
			cd->recalculateCollider(&gv->colliders[gv->transportableObjects[i]], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, true);
		}
		else if (gv->transportableObjects[i] == 15) {
			cd->recalculateCollider(&gv->colliders[15], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, false, true, gv->camVectorsMT1);
		}
		else  if (gv->transportableObjects[i] == 16) {
			cd->recalculateCollider(&gv->colliders[16], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, false, true, gv->camVectorsMT2);
		}
	}
}

/*
 *
 */
void SimulationControl::sim_control_virtual_mode1(){

	MonitoringToolModels *mt=MonitoringToolModels::get_instance();
	GlobalVariables *gv= GlobalVariables::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	CollisionDetection *cd=CollisionDetection::get_instance();
	HelperFunctions *hf=HelperFunctions::get_instance();

	// cout << "debug: SimCtrl. " << __LINE__ <<endl;

	do {
		gv->collision_init = false;
		cd->recalculateCollider(&gv->colliders[12], ((float)rand() / RAND_MAX) * 110.0f - 55.0f, ((float)rand() / RAND_MAX) * 67.5f - 12.5f, 4.0f, (2 * M_PI - cd->epsilon5) * ((float)rand() / RAND_MAX), 0, 0, true);

		if (cd->checkForCollision(&gv->colliders[12], &gv->colliders[8])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[9])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[10])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[11])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[1])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[15])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[16])) {
			gv->collision_init = true;
		}
	} while (gv->collision_init);

	do {
		gv->collision_init = false;
		cd->recalculateCollider(&gv->colliders[13], ((float)rand() / RAND_MAX) * 110.0f - 55.0f, ((float)rand() / RAND_MAX) * 67.5f - 12.5f, 3.0f, (2 * M_PI - cd->epsilon5) * ((float)rand() / RAND_MAX), 0, 0, true);

		if (cd->checkForCollision(&gv->colliders[13], &gv->colliders[8])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[9])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[10])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[11])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[12])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[1])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[15])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[16])) {
			gv->collision_init = true;
		}
	} while (gv->collision_init);

	do {
		gv->collision_init = false;
		cd->recalculateCollider(&gv->colliders[14], ((float)rand() / RAND_MAX) * 110.0f - 55.0f, ((float)rand() / RAND_MAX) * 67.5f - 12.5f, 6.0f, (2 * M_PI - cd->epsilon5) * ((float)rand() / RAND_MAX), 0, 0, true);

		if (cd->checkForCollision(&gv->colliders[14], &gv->colliders[8])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[9])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[10])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[11])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[12])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[13])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[1])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[15])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[16])) {
			gv->collision_init = true;
		}
	} while (gv->collision_init);

	gv->groundTruthObstaclePositions[0] = gv->colliders[12].offsets[0]; gv->groundTruthObstaclePositions[1] = gv->colliders[12].offsets[1]; gv->groundTruthObstaclePositions[2] = gv->colliders[12].angles[0];
	gv->groundTruthObstaclePositions[3] = gv->colliders[13].offsets[0]; gv->groundTruthObstaclePositions[4] = gv->colliders[13].offsets[1]; gv->groundTruthObstaclePositions[5] = gv->colliders[13].angles[0];
	gv->groundTruthObstaclePositions[6] = gv->colliders[14].offsets[0]; gv->groundTruthObstaclePositions[7] = gv->colliders[14].offsets[1]; gv->groundTruthObstaclePositions[8] = gv->colliders[14].angles[0];

	gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[12], 0);
	gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[13], gv->tempCounter);
	hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[14], gv->tempCounter);

	fo->resetDetectedPatterns();
	for (int i = 0; i < totalPatternCount; i++) {
		if (mt->isPatternVisible(i, 1, true)) {
			fo->detectedPatternIndicesForMT1[i] = true;
			hf->getMTPositionData(1);
			fo->calcPosFromMTData(&gv->colliders[15], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], i);
			fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);
		}
		else fo->detectedPatternIndicesForMT1[i] = false;

		if (mt->isPatternVisible(i, 2, true)) {
			fo->detectedPatternIndicesForMT2[i] = true;
			hf->getMTPositionData(2);
			fo->calcPosFromMTData(&gv->colliders[16], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], i);
			fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);
		}
		else fo->detectedPatternIndicesForMT2[i] = false;
	}
	fo->evaluateDetections(true);

}


/*
 * this function selects, which agents should be (de) activated.
 */
void SimulationControl::sim_control_select_agents(){
GlobalVariables *gv=GlobalVariables::get_instance();
	cout << "\nUse which agents? (press 1 (use gv->path), 2 (use obs), 3 (use gv->path & obs))" << endl;
	char c_in = '-';
	SDL_Event event_sub;
	do {
		while (SDL_PollEvent(&event_sub)) {
			if (event_sub.type == SDL_KEYUP) {
				if (event_sub.key.keysym.sym == SDLK_1) {
					c_in = 'p';
				}
				else if (event_sub.key.keysym.sym == SDLK_2) {
					c_in = 'o';
				}
				else if (event_sub.key.keysym.sym == SDLK_3) {
					c_in = 'b';
				}
			}
		}
	} while (c_in == '-');

	if (c_in == 'p') {
		gv->activeAgents[3] = true;
		gv->activeAgents[4] = false;
		gv->agentMode = 1;
		cout << "Sim. info: path active" << endl;
	}
	else if (c_in == 'o') {
		gv->activeAgents[3] = false;
		gv->activeAgents[4] = true;
		gv->agentMode = 2;
		cout << "Sim info: obs active" << endl;
	}
	else if (c_in == 'b') {
		gv->activeAgents[3] = true;
		gv->activeAgents[4] = true;
		gv->agentMode = 3;
		cout << "Sim info: both active" << endl;
	}

}


/*
 * This function validates the detection of Objects (obstacles + workpieces).
 * It results in the flags gv->valids[0]..gv->valids[2] for valid position of obstacle
 * and gv->collision_init for no collision on initialization.
 */
void SimulationControl::validate_obstacle_positions(){

	GlobalVariables *gv=GlobalVariables::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	PathPlanner *pp=PathPlanner::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	CollisionDetection *cd =CollisionDetection::get_instance();
	MonitoringToolModels *mt=MonitoringToolModels::get_instance();

	// validate the detections of MTs
    // -------------------------------
	ObjectPositionInterface opi;
	ObjectPositionInterface opi2;

	opi= ObjectPositionInterface::convert_from_edge(4,fo->evaluatedDetections[4],0.0f);
	opi2.init(4,gv->colliders[12].offsets[0],gv->colliders[12].offsets[1],gv->colliders[12].offsets[2],0.0f);
	std::cout<<"dbg@simCtrl "<<__LINE__<<": current action space dist= "<<fo->currentActionSpaceDist<<std::endl;
	opi.print();
	opi2.print();

	// obstacle 1 distance to evaluatedDetections[4]
	const double d4=sqrt(
			(gv->colliders[12].offsets[0] - fo->evaluatedDetections[4].x) * (gv->colliders[12].offsets[0] - fo->evaluatedDetections[4].x)
		  + (gv->colliders[12].offsets[1] - fo->evaluatedDetections[4].y) * (gv->colliders[12].offsets[1] - fo->evaluatedDetections[4].y)
		  );

	if (fo->detectionCounts[4] > 0
			&& d4 <= fo->currentActionSpaceDist
			&& gv->colliders[12].offsets[0] >= -55.0f && gv->colliders[12].offsets[0] <= 55.0f // obstacle 1 x on monitored space
			&& gv->colliders[12].offsets[1] >= -12.5f && gv->colliders[12].offsets[1] <= 55.0f)// obstacle 1 y on monitored space
	{
		gv->valids[0] = true;
	}else if (fo->detectionCounts[4] == 0) {
		gv->valids[0] = true;
	}else{
		//do nothing
	}
	// obstacle 2 distance to evaluatedDetections[5]
	opi= ObjectPositionInterface::convert_from_edge(4,fo->evaluatedDetections[5],0.0f);
	opi2.init(5,gv->colliders[13].offsets[0],gv->colliders[13].offsets[1],gv->colliders[13].offsets[2],0.0f);
	std::cout<<"dbg@simCtrl "<<__LINE__<<": current action space dist= "<<fo->currentActionSpaceDist<<std::endl;
	opi.print();
	opi2.print();

	const double d5=sqrt(
			(gv->colliders[13].offsets[0] - fo->evaluatedDetections[5].x) * (gv->colliders[13].offsets[0] - fo->evaluatedDetections[5].x)
		  + (gv->colliders[13].offsets[1] - fo->evaluatedDetections[5].y) * (gv->colliders[13].offsets[1] - fo->evaluatedDetections[5].y)
		  );

	if (fo->detectionCounts[5] > 0 && d5  <= fo->currentActionSpaceDist
			&& gv->colliders[13].offsets[0] >= -55.0f
			&& gv->colliders[13].offsets[0] <= 55.0f
			&& gv->colliders[13].offsets[1] >= -12.5f
			&& gv->colliders[13].offsets[1] <= 55.0f) {
		gv->valids[1] = true;
	}else if (fo->detectionCounts[5] == 0) {
		gv->valids[1] = true;
	}else{
		//do nothing
	}
	// obstacle 3 distance to evaluatedDetections[6]
	opi= ObjectPositionInterface::convert_from_edge(6,fo->evaluatedDetections[6],0.0f);
	opi2.init(6,gv->colliders[14].offsets[0],gv->colliders[14].offsets[1],gv->colliders[14].offsets[2],0.0f);
	std::cout<<"dbg@simCtrl "<<__LINE__<<": current action space dist= "<<fo->currentActionSpaceDist<<std::endl;
	opi.print();
	opi2.print();

	const double d6= sqrt(
			(gv->colliders[14].offsets[0] - fo->evaluatedDetections[6].x) * (gv->colliders[14].offsets[0] - fo->evaluatedDetections[6].x)
		  + (gv->colliders[14].offsets[1] - fo->evaluatedDetections[6].y) * (gv->colliders[14].offsets[1] - fo->evaluatedDetections[6].y)
		  );
	if (fo->detectionCounts[6] > 0 && d6  <= fo->currentActionSpaceDist
			&& gv->colliders[14].offsets[0] >= -55.0f
			&& gv->colliders[14].offsets[0] <= 55.0f
			&& gv->colliders[14].offsets[1] >= -12.5f
			&& gv->colliders[14].offsets[1] <= 55.0f){
		gv->valids[2] = true;
	}else if (fo->detectionCounts[6] == 0) {
		gv->valids[2] = true;
	}else{
		//do nothing
	}

	// figure out whether initialization caused a collision
	if (gv->valids[0] && gv->valids[1] && gv->valids[2]) {
		for (int j = 12; j <= 14; j++) {
			if (cd->checkForCollision(&gv->colliders[j], &gv->colliders[8])) {
				gv->collision_init = true;
			}

			if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[9])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[10])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[11])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[15])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[16])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[1])) {
				gv->collision_init = true;
			}
		}

		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[13])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[14])) {
			gv->collision_init = true;
		}
		if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[14])) {
			gv->collision_init = true;
		}
	}
}


void SimulationControl::Position_Obstacles_from_MTs(FrequentOps *fo, CollisionDetection *cd, GlobalVariables *gv){

	// recalculate collider 12 (obstacle 1)
	if (fo->detectionCounts[4] > 0) {
		cd->recalculateCollider
		(&gv->colliders[12],
		  fo->evaluatedDetections[4].x + ((float) (rand()) / RAND_MAX) * fo->currentActionSpaceDist * 2.0f - fo->currentActionSpaceDist,
		  fo->evaluatedDetections[4].y + ((float) (rand()) / RAND_MAX) * fo->currentActionSpaceDist * 2.0f - fo->currentActionSpaceDist,
		  4.0f,
		  fo->evaluatedDetections[4].z + ((float) (rand()) / RAND_MAX) * fo->currentActionSpaceAng * 2.0f  - fo->currentActionSpaceAng,
						0, 0, true);
	}
	// recalculate collider 13 (obstacle 2)
	if (fo->detectionCounts[5] > 0) {
		cd->recalculateCollider(&gv->colliders[13],
				fo->evaluatedDetections[5].x
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceDist * 2.0f
						- fo->currentActionSpaceDist,
				fo->evaluatedDetections[5].y
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceDist * 2.0f
						- fo->currentActionSpaceDist, 3.0f,
				fo->evaluatedDetections[5].z
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceAng * 2.0f
						- fo->currentActionSpaceAng, 0, 0, true);
	}

	// recalculate collider 14 (obstacle 3)
	if (fo->detectionCounts[6] > 0) {
		cd->recalculateCollider(&gv->colliders[14],
				fo->evaluatedDetections[6].x
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceDist * 2.0f
						- fo->currentActionSpaceDist,
				fo->evaluatedDetections[6].y
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceDist * 2.0f
						- fo->currentActionSpaceDist, 6.0f,
				fo->evaluatedDetections[6].z
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceAng * 2.0f
						- fo->currentActionSpaceAng, 0, 0, true);
	}
}

/*
 * This function specifies the position of the obstacles.
 * @param index: index of obstacle in range 0..2
 * @param boOverride: defines whether the detections of the MTs should be override.
 * @param obstacle_positions: array of 3 object positions.
 */
void SimulationControl::Position_Obstacle_from_Ext(bool boOverride, ObjectPositionInterface obj )
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	CollisionDetection *cd =CollisionDetection::get_instance();

	float height=0.0f;
// override z or height respectively.
	if(obj.index==0){
		height=4.0f;
	}else if(obj.index == 1){
		height=3.0f;
	}else if(obj.index == 2){
		height=6.0f;
	}else{
		cout <<"E@SimulationControl l."<< __LINE__ <<"index out of bound!"<<endl;
	}


	if(boOverride){
		// cout <<"debug@SimulationControl l."<< __LINE__ <<endl;
		cd->recalculateCollider(&gv->colliders[12+obj.index],obj.x,obj.y,height,obj.angle, 0, 0,true);
	}else{
		// obstacle NOT detected by MT
		if (fo->detectionCounts[4+obj.index] <= 0) {
			cd->recalculateCollider(&gv->colliders[12+obj.index],obj.x,obj.y,height,obj.angle, 0, 0,true);
		}
	}
}


/*
 * This function specifies the position of the obstacles.
 *
 * Old and deprecated. Use ObjectPositionInterface instead!
 *
 * @param index: index of obstacle in range 0..2
 * @param boOverride: defines whether the detections of the MTs should be override.
 * @param obstacle_positions: array of 3 object positions.
 */
void SimulationControl::Position_Obstacle_from_Ext(unsigned int index, bool boOverride, float x1,float y1,float phi1 )
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	CollisionDetection *cd =CollisionDetection::get_instance();

	// cout <<"debug@SimulationControl l."<< __LINE__ <<endl;

	float height=0.0f;

	if(index==0){
		height=4.0f;
	}else if(index == 1){
		height=3.0f;
	}else if(index == 2){
		height=6.0f;
	}else{
		cout <<"E@SimulationControl l."<< __LINE__ <<"index out of bound!"<<endl;
	}


	if(boOverride){
		// cout <<"debug@SimulationControl l."<< __LINE__ <<endl;
		cd->recalculateCollider(&gv->colliders[12+index],x1,y1,height,phi1, 0, 0,true);
	}else{
		// obstacle 1 NOT detected by MT
		if (fo->detectionCounts[4+index] <= 0) {
			// cout <<"debug@SimulationControl l."<< __LINE__ <<endl;
			cd->recalculateCollider(&gv->colliders[12+index],x1,y1,height, phi1, 0, 0,true);
		}
	}
}


/*
 * This function determins the obstacle position randomly, if not detected by MT.
 * @param obj = array of 3 ObjectPositionInterface objects. Stores new object position.
 * @param override = ignore information of MTs.
 */
void SimulationControl::Position_Obstacles_Randomly(ObjectPositionInterface *obj, bool boOverride){
	GlobalVariables *gv=GlobalVariables::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	CollisionDetection *cd =CollisionDetection::get_instance();

	// recalculate collider 12 (obstacle 1)
	if (fo->detectionCounts[4] > 0 &&!boOverride) {
		// do nothing.
	} else {
		obj[0].x=((float) (rand()) / RAND_MAX) * 110.0f - 55.0f;
		obj[0].y=((float) (rand()) / RAND_MAX) * 67.5f - 12.5f;
		obj[0].z=4.0f;
		obj[0].angle=(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX);
		obj[0].index=0;
		cd->recalculateCollider(&gv->colliders[12], obj[0].x, obj[0].y, obj[0].z, obj[0].angle, 0, 0,true);
	}
	// recalculate collider 13 (obstacle 2)
	if (fo->detectionCounts[5] > 0 && !boOverride) {
		//do nothing
	} else {
		obj[1].x=((float) (rand()) / RAND_MAX) * 110.0f - 55.0f;
		obj[1].y=((float) (rand()) / RAND_MAX) * 67.5f - 12.5f;
		obj[1].z=3.0f;
		obj[1].angle=(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX);
		obj[1].index=1;

		cd->recalculateCollider(&gv->colliders[13],	obj[1].x, obj[1].y, obj[1].z, obj[1].angle, 0, 0, true);
	}

	// recalculate collider 14 (obstacle 3)
	if (fo->detectionCounts[6] > 0 && !boOverride) {
		//do nothing. Take value from MT
	} else {
		obj[2].x=((float) (rand()) / RAND_MAX) * 110.0f - 55.0f;
		obj[2].y=((float) (rand()) / RAND_MAX) * 67.5f - 12.5f;
		obj[2].z=6.0f;
		obj[2].angle=(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX);
		obj[2].index=2;

		cd->recalculateCollider(&gv->colliders[14],
				obj[2].x,
				obj[2].y,
				obj[2].z,
				obj[2].angle,
				0, 0,true);
	}
}

void SimulationControl::Position_Obstacles_Old(FrequentOps *fo,
		CollisionDetection *cd, GlobalVariables *gv)
{
	// recalculate collider 12 (obstacle 1)
	if (fo->detectionCounts[4] > 0) {
		cout << "debug: SimulationControl " << __LINE__ <<endl;

		cd->recalculateCollider
		(&gv->colliders[12],
		  fo->evaluatedDetections[4].x + ((float) (rand()) / RAND_MAX) * fo->currentActionSpaceDist * 2.0f - fo->currentActionSpaceDist,
		  fo->evaluatedDetections[4].y + ((float) (rand()) / RAND_MAX) * fo->currentActionSpaceDist * 2.0f - fo->currentActionSpaceDist,
		  4.0f,
		  fo->evaluatedDetections[4].z + ((float) (rand()) / RAND_MAX) * fo->currentActionSpaceAng * 2.0f  - fo->currentActionSpaceAng,
						0, 0, true);
	} else {
		cout << "debug: SimulationControl " << __LINE__ <<endl;
		cd->recalculateCollider(&gv->colliders[12],
				((float) (rand()) / RAND_MAX) * 110.0f - 55.0f,
				((float) (rand()) / RAND_MAX) * 67.5f - 12.5f, 4.0f,
				(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX), 0, 0,
				true);
	}
	// recalculate collider 13 (obstacle 2)
	if (fo->detectionCounts[5] > 0) {
		cout << "debug: SimulationControl " << __LINE__ <<endl;
		cd->recalculateCollider(&gv->colliders[13],
				fo->evaluatedDetections[5].x
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceDist * 2.0f
						- fo->currentActionSpaceDist,
				fo->evaluatedDetections[5].y
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceDist * 2.0f
						- fo->currentActionSpaceDist, 3.0f,
				fo->evaluatedDetections[5].z
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceAng * 2.0f
						- fo->currentActionSpaceAng, 0, 0, true);
	} else {
		cout << "debug: SimulationControl " << __LINE__ <<endl;
		cd->recalculateCollider(&gv->colliders[13],
				((float) (rand()) / RAND_MAX) * 110.0f - 55.0f,
				((float) (rand()) / RAND_MAX) * 67.5f - 12.5f, 3.0f,
				(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX), 0, 0,
				true);
	}

	// recalculate collider 14 (obstacle 3)
	if (fo->detectionCounts[6] > 0) {
		cout << "debug: SimulationControl " << __LINE__ <<endl;
		cd->recalculateCollider(&gv->colliders[14],
				fo->evaluatedDetections[6].x
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceDist * 2.0f
						- fo->currentActionSpaceDist,
				fo->evaluatedDetections[6].y
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceDist * 2.0f
						- fo->currentActionSpaceDist, 6.0f,
				fo->evaluatedDetections[6].z
						+ ((float) (rand()) / RAND_MAX)
								* fo->currentActionSpaceAng * 2.0f
						- fo->currentActionSpaceAng, 0, 0, true);
	} else {
		cout << "debug: SimulationControl " << __LINE__ <<endl;
		cd->recalculateCollider(&gv->colliders[14],
				((float) (rand()) / RAND_MAX) * 110.0f - 55.0f,
				((float) (rand()) / RAND_MAX) * 67.5f - 12.5f, 6.0f,
				(2 * M_PI - cd->epsilon5) * ((float) (rand()) / RAND_MAX), 0, 0,
				true);
	}
}


/*
 * this function positions the obstacles.
 * @param obj: Array with 3 elements spezifying the position of the obstacle.
 * @param write: tells whether a random position should be found and handed back or an existing should be written.
 * returns true if an error occurs.
 */
bool SimulationControl::sim_control_position_obstacles(ObjectPositionInterface* obj, bool boWrite){

	GlobalVariables *gv=GlobalVariables::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	PathPlanner *pp=PathPlanner::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	CollisionDetection *cd =CollisionDetection::get_instance();
	MonitoringToolModels *mt=MonitoringToolModels::get_instance();

	cout << "debug: SimulationControl " << __LINE__ <<endl;

	gv->simAttemptCounter = 0;

	bool boFirst=boWrite;
	bool boReturn=false;

	do {
		if(boFirst){
			Position_Obstacle_from_Ext(true, obj[0]); //testing
			Position_Obstacle_from_Ext(true, obj[1]);
			Position_Obstacle_from_Ext(true, obj[2]);
			boFirst=false;
		}
		else
		{
			if(boWrite)
			{
				boReturn=true;
				break;
			}
			Position_Obstacles_Randomly(obj,boWrite);

			//std::cout<<"----- obstacle positions ----"<<std::endl;
			//for(int i=0;i<3;i++){
			//	std::cout<<"("<<obj[i].index<<","<<obj[i].x<<","<<obj[i].y<<","<<obj[i].z<<","<<obj[i].angle<<")"<<std::endl;
			//}
			//std::cout<<"----- ----- ----"<<std::endl;
		}

		gv->valids[0] = false; gv->valids[1] = false; gv->valids[2] = false;
		gv->collision_init = false;

		validate_obstacle_positions();

		gv->simAttemptCounter++;
	} while ((!gv->valids[0] || !gv->valids[1] || !gv->valids[2] || gv->collision_init) && gv->simAttemptCounter < 5000);
	std::cout<<"----- FINAL obstacle positions ----"<<std::endl;
	for(int i=0;i<3;i++){
		std::cout<<"("<<obj[i].index<<","<<obj[i].x<<","<<obj[i].y<<","<<obj[i].z<<","<<obj[i].angle<<")"<<std::endl;
	}
	std::cout<<"----- ----- ----"<<std::endl;

	if (gv->simAttemptCounter >= 5000) {
		cout << "Sim. warning: timeout occurred during placements, obstacles are not validly placed or colliding!" << endl;
		cout << "Sim. warning: transport cannot be simulated for safe real execution!" << endl;
		gv->valid = false;
		return true;
	}

	gv->fittingPatternsTemp = 0;
	for (int j = 0; j < totalPatternCount; j++) {
		if (fo->detectedPatternIndicesForMT1[j] == mt->isPatternVisible(j, 1, false)) gv->fittingPatternsTemp++;
		if (fo->detectedPatternIndicesForMT2[j] == mt->isPatternVisible(j, 2, false)) gv->fittingPatternsTemp++;
	}

	if (gv->fittingPatternsTemp > gv->fittingPatternsMax) {
		gv->fittingPatternsMax = gv->fittingPatternsTemp;
		gv->finalObstaclePositions[0] = gv->colliders[12].offsets[0]; gv->finalObstaclePositions[1] = gv->colliders[12].offsets[1]; gv->finalObstaclePositions[2] = gv->colliders[12].angles[0];
		gv->finalObstaclePositions[3] = gv->colliders[13].offsets[0]; gv->finalObstaclePositions[4] = gv->colliders[13].offsets[1]; gv->finalObstaclePositions[5] = gv->colliders[13].angles[0];
		gv->finalObstaclePositions[6] = gv->colliders[14].offsets[0]; gv->finalObstaclePositions[7] = gv->colliders[14].offsets[1]; gv->finalObstaclePositions[8] = gv->colliders[14].angles[0];
	}

	return boReturn;
}

bool SimulationControl::sim_control_do_something3(){

	GlobalVariables *gv=GlobalVariables::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	PathPlanner *pp=PathPlanner::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	CollisionDetection *cd =CollisionDetection::get_instance();
	MonitoringToolModels *mt=MonitoringToolModels::get_instance();

	cout << "debug:SimulationControl " << __LINE__ <<endl;

	gv->simAttemptCounter = 0;

	do {
		gv->valids[0] = false; gv->valids[1] = false; gv->valids[2] = false;
		gv->collision_init = false;

		if (fo->detectionCounts[4] > 0) {
			cd->recalculateCollider(&gv->colliders[12], fo->evaluatedDetections[4].x + ((float)rand() / RAND_MAX) * fo->currentActionSpaceDist * 2.0f - fo->currentActionSpaceDist, fo->evaluatedDetections[4].y + ((float)rand() / RAND_MAX) * fo->currentActionSpaceDist * 2.0f - fo->currentActionSpaceDist, 4.0f, fo->evaluatedDetections[4].z + ((float)rand() / RAND_MAX) * fo->currentActionSpaceAng * 2.0f - fo->currentActionSpaceAng, 0, 0, true);
		}
		else {
			cd->recalculateCollider(&gv->colliders[12], ((float)rand() / RAND_MAX) * 110.0f - 55.0f, ((float)rand() / RAND_MAX) * 67.5f - 12.5f, 4.0f, (2 * M_PI - cd->epsilon5) * ((float)rand() / RAND_MAX), 0, 0, true);
		}

		if (fo->detectionCounts[5] > 0) {
			cd->recalculateCollider(&gv->colliders[13], fo->evaluatedDetections[5].x + ((float)rand() / RAND_MAX) * fo->currentActionSpaceDist * 2.0f - fo->currentActionSpaceDist, fo->evaluatedDetections[5].y + ((float)rand() / RAND_MAX) * fo->currentActionSpaceDist * 2.0f - fo->currentActionSpaceDist, 3.0f, fo->evaluatedDetections[5].z + ((float)rand() / RAND_MAX) * fo->currentActionSpaceAng * 2.0f - fo->currentActionSpaceAng, 0, 0, true);
		}
		else {
			cd->recalculateCollider(&gv->colliders[13], ((float)rand() / RAND_MAX) * 110.0f - 55.0f, ((float)rand() / RAND_MAX) * 67.5f - 12.5f, 3.0f, (2 * M_PI - cd->epsilon5) * ((float)rand() / RAND_MAX), 0, 0, true);
		}

		if (fo->detectionCounts[6] > 0) {
			cd->recalculateCollider(&gv->colliders[14], fo->evaluatedDetections[6].x + ((float)rand() / RAND_MAX) * fo->currentActionSpaceDist * 2.0f - fo->currentActionSpaceDist, fo->evaluatedDetections[6].y + ((float)rand() / RAND_MAX) * fo->currentActionSpaceDist * 2.0f - fo->currentActionSpaceDist, 6.0f, fo->evaluatedDetections[6].z + ((float)rand() / RAND_MAX) * fo->currentActionSpaceAng * 2.0f - fo->currentActionSpaceAng, 0, 0, true);
		}
		else {
			cd->recalculateCollider(&gv->colliders[14], ((float)rand() / RAND_MAX) * 110.0f - 55.0f, ((float)rand() / RAND_MAX) * 67.5f - 12.5f, 6.0f, (2 * M_PI - cd->epsilon5) * ((float)rand() / RAND_MAX), 0, 0, true);
		}

		if (fo->detectionCounts[4] > 0 && sqrt((gv->colliders[12].offsets[0] - fo->evaluatedDetections[4].x) * (gv->colliders[12].offsets[0] - fo->evaluatedDetections[4].x) + (gv->colliders[12].offsets[1] - fo->evaluatedDetections[4].y) * (gv->colliders[12].offsets[1] - fo->evaluatedDetections[4].y)) <= fo->currentActionSpaceDist && gv->colliders[12].offsets[0] >= -55.0f && gv->colliders[12].offsets[0] <= 55.0f && gv->colliders[12].offsets[1] >= -12.5f && gv->colliders[12].offsets[1] <= 55.0f) gv->valids[0] = true;
		else if (fo->detectionCounts[4] == 0) {
			gv->valids[0] = true;
		}

		if (fo->detectionCounts[5] > 0 && sqrt((gv->colliders[13].offsets[0] - fo->evaluatedDetections[5].x) * (gv->colliders[13].offsets[0] - fo->evaluatedDetections[5].x) + (gv->colliders[13].offsets[1] - fo->evaluatedDetections[5].y) * (gv->colliders[13].offsets[1] - fo->evaluatedDetections[5].y)) <= fo->currentActionSpaceDist && gv->colliders[13].offsets[0] >= -55.0f && gv->colliders[13].offsets[0] <= 55.0f && gv->colliders[13].offsets[1] >= -12.5f && gv->colliders[13].offsets[1] <= 55.0f) gv->valids[1] = true;
		else if (fo->detectionCounts[5] == 0) {
			gv->valids[1] = true;
		}

		if (fo->detectionCounts[6] > 0 && sqrt((gv->colliders[14].offsets[0] - fo->evaluatedDetections[6].x) * (gv->colliders[14].offsets[0] - fo->evaluatedDetections[6].x) + (gv->colliders[14].offsets[1] - fo->evaluatedDetections[6].y) * (gv->colliders[14].offsets[1] - fo->evaluatedDetections[6].y)) <= fo->currentActionSpaceDist && gv->colliders[14].offsets[0] >= -55.0f && gv->colliders[14].offsets[0] <= 55.0f && gv->colliders[14].offsets[1] >= -12.5f && gv->colliders[14].offsets[1] <= 55.0f) gv->valids[2] = true;
		else if (fo->detectionCounts[6] == 0) {
			gv->valids[2] = true;
		}

		if (gv->valids[0] && gv->valids[1] && gv->valids[2]) {
			for (int j = 12; j <= 14; j++) {
				if (cd->checkForCollision(&gv->colliders[j], &gv->colliders[8])) {
					gv->collision_init = true;
				}

				if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[9])) {
					gv->collision_init = true;
				}
				if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[10])) {
					gv->collision_init = true;
				}
				if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[11])) {
					gv->collision_init = true;
				}
				if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[15])) {
					gv->collision_init = true;
				}
				if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[16])) {
					gv->collision_init = true;
				}
				if (!gv->collision_init && cd->checkForCollision(&gv->colliders[j], &gv->colliders[1])) {
					gv->collision_init = true;
				}
			}

			if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[13])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[14])) {
				gv->collision_init = true;
			}
			if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[14])) {
				gv->collision_init = true;
			}
		}
		gv->simAttemptCounter++;
	} while ((!gv->valids[0] || !gv->valids[1] || !gv->valids[2] || gv->collision_init) && gv->simAttemptCounter < 5000);

	if (gv->simAttemptCounter >= 5000) {
		cout << "Sim. warning: timeout occurred during placements, obstacles are not validly placed or colliding!" << endl;
		cout << "Sim. warning: transport cannot be simulated for safe real execution!" << endl;
		gv->valid = false;
		return true;
	}

	gv->fittingPatternsTemp = 0;
	for (int j = 0; j < totalPatternCount; j++) {
		if (fo->detectedPatternIndicesForMT1[j] == mt->isPatternVisible(j, 1, false)) gv->fittingPatternsTemp++;
		if (fo->detectedPatternIndicesForMT2[j] == mt->isPatternVisible(j, 2, false)) gv->fittingPatternsTemp++;
	}

	if (gv->fittingPatternsTemp > gv->fittingPatternsMax) {
		gv->fittingPatternsMax = gv->fittingPatternsTemp;
		gv->finalObstaclePositions[0] = gv->colliders[12].offsets[0]; gv->finalObstaclePositions[1] = gv->colliders[12].offsets[1]; gv->finalObstaclePositions[2] = gv->colliders[12].angles[0];
		gv->finalObstaclePositions[3] = gv->colliders[13].offsets[0]; gv->finalObstaclePositions[4] = gv->colliders[13].offsets[1]; gv->finalObstaclePositions[5] = gv->colliders[13].angles[0];
		gv->finalObstaclePositions[6] = gv->colliders[14].offsets[0]; gv->finalObstaclePositions[7] = gv->colliders[14].offsets[1]; gv->finalObstaclePositions[8] = gv->colliders[14].angles[0];
	}

	return false;
}


/*
 * apply agents actions.
 */
void SimulationControl::sim_control_apply_agents_actions() {
	CollisionDetection *cd=CollisionDetection::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();

	if (fo->actionSpaceSamples == 0) {
		fo->currentActionSpaceDist = gv->actionSpaceDistances[1];
		fo->currentActionSpaceAng = gv->actionSpaceAngles[1];
	} else {
		fo->currentActionSpaceDist =
				gv->actionSpaceDistances[gv->actionSpaceCount - 1];
		fo->currentActionSpaceAng = gv->actionSpaceAngles[gv->actionSpaceCount
				- 1];
		for (int i = gv->actionSpaceCount - 2; i >= 0; i--) {
			if (gv->actionSpaceDistances[i] >= fo->actionSpaceDist) {
				fo->currentActionSpaceDist = gv->actionSpaceDistances[i];
			} else
				break;
		}
		for (int i = gv->actionSpaceCount - 2; i >= 0; i--) {
			if (gv->actionSpaceAngles[i] >= fo->actionSpaceAng) {
				fo->currentActionSpaceAng = gv->actionSpaceAngles[i];
			} else
				break;
		}
	}
}

/*
 * recalculate colliders
 */
void SimulationControl::sim_control_recalculate_colliders() {
	CollisionDetection *cd=CollisionDetection::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();

	if (gv->valid) {
		cd->recalculateCollider(&gv->colliders[12],
				gv->finalObstaclePositions[0], gv->finalObstaclePositions[1],
				4.0f, gv->finalObstaclePositions[2], 0, 0, true);
		cd->recalculateCollider(&gv->colliders[13],
				gv->finalObstaclePositions[3], gv->finalObstaclePositions[4],
				3.0f, gv->finalObstaclePositions[5], 0, 0, true);
		cd->recalculateCollider(&gv->colliders[14],
				gv->finalObstaclePositions[6], gv->finalObstaclePositions[7],
				6.0f, gv->finalObstaclePositions[8], 0, 0, true);
	} else {
		for (int i = 12; i <= 14; i++) {
			cd->recalculateCollider(&gv->colliders[i], 0.0f, 0.0f, 10.0e5, 0, 0,
					0, true);
		}
	}
	if (fo->detectionCounts[4] == 0) {
		if (!gv->valid) {
			cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f,
					10.0e5, 0, 0, 0);
		} else {
			cd->recalculateCollider(&gv->distortedObstacles[0],
					gv->finalObstaclePositions[0],
					gv->finalObstaclePositions[1], 4.0f,
					gv->finalObstaclePositions[2], 0, 0);
		}
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[0],
				fo->evaluatedDetections[4].x, fo->evaluatedDetections[4].y,
				4.0f, fo->evaluatedDetections[4].z, 0, 0);
	}
	if (fo->detectionCounts[5] == 0) {
		if (!gv->valid) {
			cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f,
					10.0e5, 0, 0, 0);
		} else {
			cd->recalculateCollider(&gv->distortedObstacles[1],
					gv->finalObstaclePositions[3],
					gv->finalObstaclePositions[4], 3.0f,
					gv->finalObstaclePositions[5], 0, 0);
		}
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[1],
				fo->evaluatedDetections[5].x, fo->evaluatedDetections[5].y,
				3.0f, fo->evaluatedDetections[5].z, 0, 0);
	}
	if (fo->detectionCounts[6] == 0) {
		if (!gv->valid)
			cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f,
					10.0e5, 0, 0, 0);
		else
			cd->recalculateCollider(&gv->distortedObstacles[2],
					gv->finalObstaclePositions[6],
					gv->finalObstaclePositions[7], 6.0f,
					gv->finalObstaclePositions[8], 0, 0);
	} else {
		cd->recalculateCollider(&gv->distortedObstacles[2],
				fo->evaluatedDetections[6].x, fo->evaluatedDetections[6].y,
				6.0f, fo->evaluatedDetections[6].z, 0, 0);
	}
}


void SimulationControl::sim_control_calc_edges() {

	CollisionDetection *cd=CollisionDetection::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();
	HelperFunctions *hf=HelperFunctions::get_instance();

	cout << "debug: SimulationControl l." << __LINE__ <<endl;

	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
			&gv->distortedObstacles[0], 0);
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
			&gv->distortedObstacles[1], gv->tempCounter);
	hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2],
			gv->tempCounter);
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
	if (gv->valid) {
		gv->selectedWorkpieceIndex = -1;
		gv->selectedTargetPosition[0] = 10000.0f;
		gv->selectedTargetPosition[1] = 0.0f;
		gv->selectedTargetPosition[2] = 0.0f;
		gv->selectedTargetPosition[3] = 0.0f;
		gv->simTargetSelection = true;
	} else {
		if (gv->writeDataIntoFiles) {
			fstream file;
			file.open("transport_result.csv", std::ios_base::app);
			if (file) {
				if (gv->advancedControlMode)
					file
							<< to_string(gv->simCounter)
									+ ";obstacle_placement_error;"
									+ to_string(gv->agentMode) + "\n";
				else
					file
							<< to_string(gv->simCounter)
									+ ";obstacle_placement_error\n";
			} else {
				cout << "Error: writing into file not possible!" << endl;
			}
			file.close();
		}
		gv->modeActive = false;
		gv->simTransport = false;
		gv->simConfirmation = false;
	}
}

void SimulationControl::sim_control_do_something4(
		bool &repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{
	MonitoringToolModels *mt = MonitoringToolModels::get_instance();
	CollisionDetection *cd=CollisionDetection::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();
	HelperFunctions *hf=HelperFunctions::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	Protagonists *prot=Protagonists::get_instance();
	Adversaries *adv=Adversaries::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();
	gv->objectToTransport =
			gv->transportableObjects[gv->selectedWorkpieceIndex];
	gv->objectOffset = gv->objectOffsets[gv->selectedWorkpieceIndex];
	gv->objectHeight = gv->objectHeights[gv->selectedWorkpieceIndex];
	gv->objectGripDepth = gv->objectGripDepths[gv->selectedWorkpieceIndex];
	gv->objectDimension.x = gv->objectDimensions[gv->selectedWorkpieceIndex].x
			+ 0.25f;
	gv->objectDimension.y = gv->objectDimensions[gv->selectedWorkpieceIndex].y
			+ 0.25f;
	gv->objectDimension.z = gv->objectDimensions[gv->selectedWorkpieceIndex].z
			+ 0.25f;
	gv->objectIndex = gv->selectedWorkpieceIndex;
	gv->pathStartPos.x = gv->colliders[gv->objectToTransport].offsets[0];
	gv->pathStartPos.y = gv->colliders[gv->objectToTransport].offsets[1];
	gv->pathStartPos.z = gv->objectOffset + gv->objectHeight
			+ gv->objectGripDepth;
	gv->startObjectAngle = gv->colliders[gv->objectToTransport].angles[0]
			* 180.0f / M_PI;
	gv->pathEndPos.x = gv->selectedTargetPosition[0];
	gv->pathEndPos.y = gv->selectedTargetPosition[1];
	gv->pathEndPos.z = gv->objectOffset + gv->objectHeight
			+ gv->objectGripDepth;
	gv->endObjectAngle = gv->selectedTargetPosition[3];
	gv->objectPosReachable = false;


	// cout << "debug: simCtrl " << __LINE__ <<endl;

	if (ik->doesThetaExist(gv->pathStartPos.x, gv->pathStartPos.y,
			gv->pathStartPos.z)) {
		// cout << "debug: simCtrl " << __LINE__ <<endl;

		ik->writeQValues(gv->qValuesObjectStart);
		if (abs(
				gv->qValuesObjectStart[1] + gv->qValuesObjectStart[2]
						+ gv->qValuesObjectStart[3] - 180.0f) < cd->epsilon3) {
			if (ik->doesThetaExist(gv->pathEndPos.x, gv->pathEndPos.y,
					gv->pathEndPos.z)) {
				ik->writeQValues(gv->qValuesObjectEnd);
				if (abs(
						gv->qValuesObjectEnd[1] + gv->qValuesObjectEnd[2]
								+ gv->qValuesObjectEnd[3] - 180.0f)
						< cd->epsilon3) {
					gv->qValuesObjectStart[4] = gv->startObjectAngle
							- (90.0f - gv->qValuesObjectStart[0]);
					gv->qValuesObjectEnd[4] = gv->endObjectAngle
							- (90.0f - gv->qValuesObjectEnd[0]);
					fo->gripAngleCalculation(gv->selectedWorkpieceIndex);
					if (gv->qValuesObjectStart[4] > ik->qUpperLimits[4]
							|| gv->qValuesObjectStart[4] < ik->qLowerLimits[4]
							|| gv->qValuesObjectEnd[4] > ik->qUpperLimits[4]
							|| gv->qValuesObjectEnd[4] < ik->qLowerLimits[4]) {
						cout
								<< "Sim. info: angle limits of last joint violated (start or end)!"
								<< endl;
						gv->objectPosReachable = false;
					} else {
						if (gv->activeAgents[0]) {
							prot->safety_area_protagonist(gv->objectIndex,
									true);
							cd->updateColliderSize(gv->safetyCollider,
									gv->objectDimensionTemp.x,
									gv->objectDimensionTemp.y,
									gv->objectDimensionTemp.z);
						} else {
							cd->updateColliderSize(gv->safetyCollider, 0.75f,
									0.75f, 0.75f);
						}
						cd->updateColliderSize(&gv->adversaryCollider[3],
								gv->objectDimension.x - 0.5f,
								gv->objectDimension.y - 0.5f,
								gv->objectDimension.z - 0.5f);
						cd->updateColliderSize(&gv->safetyCollidersForMTs[0],
								gv->objectDimension.x + 0.5f,
								gv->objectDimension.y + 0.5f,
								gv->objectDimension.z);
						cd->updateColliderSize(&gv->safetyCollidersForMTs[1],
								gv->objectDimension.x + 0.5f,
								gv->objectDimension.y + 0.5f,
								gv->objectDimension.z);
						cd->recalculateCollider(&gv->safetyCollidersForMTs[0],
								gv->pathStartPos.x, gv->pathStartPos.y,
								gv->objectOffset,
								gv->startObjectAngle / 180.0f * M_PI, 0.0f,
								0.0f);
						cd->recalculateCollider(&gv->safetyCollidersForMTs[1],
								gv->pathEndPos.x, gv->pathEndPos.y,
								gv->objectOffset,
								gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f);
						for (int i = 0; i < 9; i++) {
							gv->distortions[i] = 0.0f;
						}
						if (gv->activeAgents[2]) {
							adv->loc_adversary(
									gv->actionSpaceDistances[rand()
											% gv->actionSpaceCount],
									gv->actionSpaceAngles[rand()
											% gv->actionSpaceCount],
									gv->selectedWorkpieceIndex, true);
						}
						cd->recalculateCollider(&gv->distortedObstacles[0],
								gv->colliders[12].offsets[0]
										+ gv->distortions[0],
								gv->colliders[12].offsets[1]
										+ gv->distortions[1], 4.0f,
								gv->colliders[12].angles[0]
										+ gv->distortions[2], 0, 0);
						cd->recalculateCollider(&gv->distortedObstacles[1],
								gv->colliders[13].offsets[0]
										+ gv->distortions[3],
								gv->colliders[13].offsets[1]
										+ gv->distortions[4], 3.0f,
								gv->colliders[13].angles[0]
										+ gv->distortions[5], 0, 0);
						cd->recalculateCollider(&gv->distortedObstacles[2],
								gv->colliders[14].offsets[0]
										+ gv->distortions[6],
								gv->colliders[14].offsets[1]
										+ gv->distortions[7], 6.0f,
								gv->colliders[14].angles[0]
										+ gv->distortions[8], 0, 0);
						gv->transportPhase = 0;
						gv->grippingWidth = gv->grippingWidthOpen;
						gv->objectPosReachable = pp->calcPath(
								gv->qValuesObjectStart, gv->qValuesStandby,
								&gv->pathStartPos, &gv->standbyPos, false,
								generator1, distribution1, generator2,
								distribution2, generator3, distribution3);
						if (gv->objectPosReachable) {
							gv->transportPhase = 1;
							gv->grippingWidth = gv->grippingWidthFixed;
							gv->objectPosReachable = pp->calcPath(
									gv->qValuesObjectEnd,
									gv->qValuesObjectStart, &gv->pathEndPos,
									&gv->pathStartPos, false, generator1,
									distribution1, generator2, distribution2,
									generator3, distribution3);
							if (gv->objectPosReachable) {
								pp->optimizePath();
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

								// cout << "debug: simCtrl " << __LINE__ <<endl;

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

								// cout << "debug: simCtrl " << __LINE__ <<endl;

								gv->secondSegLengthTotal = 0.0f;
								for (int i = 1; i < gv->secondSegSize; i++) {
									gv->secondSegLengths = ik->increaseSize(
											gv->secondSegLengths, i - 1, 1);
									gv->secondSegLengths[i - 1] =
											gv->secondSegLengthTotal
													+ max(
															abs(
																	gv->q0ValsSecondSeg[i]
																			- gv->q0ValsSecondSeg[i
																					- 1]),
															max(
																	abs(
																			gv->q1ValsSecondSeg[i]
																					- gv->q1ValsSecondSeg[i
																							- 1]),
																	max(
																			abs(
																					gv->q2ValsSecondSeg[i]
																							- gv->q2ValsSecondSeg[i
																									- 1]),
																			abs(
																					gv->q3ValsSecondSeg[i]
																							- gv->q3ValsSecondSeg[i
																									- 1]))));
									gv->secondSegLengthTotal =
											gv->secondSegLengths[i - 1];
								}
								cd->recalculateCollider(
										&gv->colliders[gv->objectToTransport],
										gv->pathEndPos.x, gv->pathEndPos.y,
										gv->objectOffset,
										gv->endObjectAngle / 180.0f * M_PI,
										0.0f, 0.0f);
								gv->grippingWidth = gv->grippingWidthOpen;
								gv->transportPhase = 2;
								gv->objectPosReachable = pp->calcPath(
										gv->qValuesStandby,
										gv->qValuesObjectEnd, &gv->standbyPos,
										&gv->pathEndPos, false, generator1,
										distribution1, generator2,
										distribution2, generator3,
										distribution3);
							}
							cd->recalculateCollider(
									&gv->colliders[gv->objectToTransport],
									gv->pathStartPos.x, gv->pathStartPos.y,
									gv->objectOffset,
									gv->startObjectAngle / 180.0f * M_PI, 0.0f,
									0.0f);
						}
						gv->transportPhase = 0;
					}
				} else {
					cout
							<< "Sim. info: target object position not reachable with necessary theta angle!"
							<< endl;
				}
			} else {
				cout << "Sim. info: target object position not reachable!"
						<< endl;
			}
		} else {
			cout
					<< "Sim. info: current object position not reachable with necessary theta angle!"
					<< endl;
		}
	} else {
		cout << "Sim. info: current object position not reachable!" << endl;
	}
	if (gv->objectPosReachable) {
		// cout << "debug: simCtrl " << __LINE__ <<endl;

		gv->oldObstaclePositions[0] = gv->colliders[12].offsets[0];
		gv->oldObstaclePositions[1] = gv->colliders[12].offsets[1];
		gv->oldObstaclePositions[2] = gv->colliders[12].angles[0];
		gv->oldObstaclePositions[3] = gv->colliders[13].offsets[0];
		gv->oldObstaclePositions[4] = gv->colliders[13].offsets[1];
		gv->oldObstaclePositions[5] = gv->colliders[13].angles[0];
		gv->oldObstaclePositions[6] = gv->colliders[14].offsets[0];
		gv->oldObstaclePositions[7] = gv->colliders[14].offsets[1];
		gv->oldObstaclePositions[8] = gv->colliders[14].angles[0];
		gv->MT1collision = false;
		gv->MT2collision = false;
		gv->MT1posChanged = false;
		gv->MT2posChanged = false;
		gv->oldMTPos[0] = gv->colliders[15].offsets[0];
		gv->oldMTPos[1] = gv->colliders[15].offsets[1];
		gv->oldMTPos[2] = gv->colliders[15].angles[0];

		// cout << "debug: simCtrl " << __LINE__ <<endl;

		if (!gv->advancedControlMode) {
			srand(1);
			generator1.seed(1);
		}

		// cout << "debug: simCtrl " << __LINE__ <<endl;
		prot->mt_preparation(1, generator1, distribution1, generator2,
				distribution2, generator3, distribution3);

		// cout << "debug: simCtrl " << __LINE__ <<endl;

		if (!gv->advancedControlMode) {
			srand((unsigned) (time(NULL)));
			generator1.seed((unsigned) (time(NULL)));
		}
		cd->recalculateCollider(&gv->colliders[12], gv->oldObstaclePositions[0],
				gv->oldObstaclePositions[1], 4.0f, gv->oldObstaclePositions[2],
				0, 0, true);
		cd->recalculateCollider(&gv->colliders[13], gv->oldObstaclePositions[3],
				gv->oldObstaclePositions[4], 3.0f, gv->oldObstaclePositions[5],
				0, 0, true);
		cd->recalculateCollider(&gv->colliders[14], gv->oldObstaclePositions[6],
				gv->oldObstaclePositions[7], 6.0f, gv->oldObstaclePositions[8],
				0, 0, true);
		if (gv->oldViewVal >= gv->maxViewVal * 0.75f) {
			cout << "Sim. procedure: keep MT 1 position -> ";
			cd->recalculateCollider(&gv->colliders[15], gv->oldMTPos[0],
					gv->oldMTPos[1], gv->objectOffsets[4], gv->oldMTPos[2], 0,
					0, false, true, gv->camVectorsMT1);
		} else {
			if (gv->activeAgents[1]) {
				cout << "Sim. procedure: MT 1 moved to new position -> ";
				prot->mt_protagonist(1, gv->objectIndex, true);
				gv->MT1posChanged = true;
				gv->mtPositionsX[0] = gv->tempMTPos[0];
				gv->mtPositionsY[0] = gv->tempMTPos[1];
				gv->mtPositionsAngles[0] = gv->tempMTPos[2];
				cd->recalculateCollider(&gv->colliders[15], gv->tempMTPos[0],
						gv->tempMTPos[1], gv->objectOffsets[4],
						gv->tempMTPos[2], 0, 0, false, true, gv->camVectorsMT1);
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
				mt->updateMonitoredSpace(false, false);
				for (int i = 0; i < 3; i++) {
					if ((abs(gv->lastDetectedObsPosX[i]) > cd->epsilon3
							|| abs(gv->lastDetectedObsPosY[i]) > cd->epsilon3)
							&& (abs(gv->currentDetectedObsPosX[i])
									< cd->epsilon3
									&& abs(gv->currentDetectedObsPosY[i])
											< cd->epsilon3)) {
						gv->currentDetectedObsPosX[i] =
								gv->lastDetectedObsPosX[i];
						gv->currentDetectedObsPosY[i] =
								gv->lastDetectedObsPosY[i];
						gv->currentDetectedObsPosA[i] =
								gv->lastDetectedObsPosA[i];
					}
				}
			} else {
				cout << "Sim. procedure: keep MT 1 position -> ";
				cd->recalculateCollider(&gv->colliders[15], gv->oldMTPos[0],
						gv->oldMTPos[1], gv->objectOffsets[4], gv->oldMTPos[2],
						0, 0, false, true, gv->camVectorsMT1);
			}
		}
		if (!gv->MT1collision) {
			if (gv->maxViewVal >= 29.0f) {
				cout << "MT 2 is not needed and not moved -> ";
			} else {
				gv->oldMTPos[0] = gv->colliders[16].offsets[0];
				gv->oldMTPos[1] = gv->colliders[16].offsets[1];
				gv->oldMTPos[2] = gv->colliders[16].angles[0];
				if (!gv->advancedControlMode) {
					srand(1);
					generator1.seed(1);
				}
				prot->mt_preparation(2, generator1, distribution1, generator2,
						distribution2, generator3, distribution3);
				if (!gv->advancedControlMode) {
					srand((unsigned) (time(NULL)));
					generator1.seed((unsigned) (time(NULL)));
				}
				cd->recalculateCollider(&gv->colliders[12],
						gv->oldObstaclePositions[0],
						gv->oldObstaclePositions[1], 4.0f,
						gv->oldObstaclePositions[2], 0, 0, true);
				cd->recalculateCollider(&gv->colliders[13],
						gv->oldObstaclePositions[3],
						gv->oldObstaclePositions[4], 3.0f,
						gv->oldObstaclePositions[5], 0, 0, true);
				cd->recalculateCollider(&gv->colliders[14],
						gv->oldObstaclePositions[6],
						gv->oldObstaclePositions[7], 6.0f,
						gv->oldObstaclePositions[8], 0, 0, true);
				if (gv->activeAgents[1]) {
					cout << "MT 2 moved to new position -> ";
					prot->mt_protagonist(2, gv->objectIndex, true);
					gv->MT2posChanged = true;
					gv->mtPositionsX[1] = gv->tempMTPos[0];
					gv->mtPositionsY[1] = gv->tempMTPos[1];
					gv->mtPositionsAngles[1] = gv->tempMTPos[2];
					cd->recalculateCollider(&gv->colliders[16],
							gv->tempMTPos[0], gv->tempMTPos[1],
							gv->objectOffsets[5], gv->tempMTPos[2], 0, 0, false,
							true, gv->camVectorsMT2);
				} else {
					cout << "MT 2 is not moved -> ";
					cd->recalculateCollider(&gv->colliders[16], gv->oldMTPos[0],
							gv->oldMTPos[1], gv->objectOffsets[5],
							gv->oldMTPos[2], 0, 0, false, true,
							gv->camVectorsMT2);
				}
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
				mt->updateMonitoredSpace(false, false);
				for (int i = 0; i < 3; i++) {
					if ((abs(gv->lastDetectedObsPosX[i]) > cd->epsilon3
							|| abs(gv->lastDetectedObsPosY[i]) > cd->epsilon3)
							&& (abs(gv->currentDetectedObsPosX[i])
									<= cd->epsilon3
									&& abs(gv->currentDetectedObsPosY[i])
											<= cd->epsilon3)) {
						gv->currentDetectedObsPosX[i] =
								gv->lastDetectedObsPosX[i];
						gv->currentDetectedObsPosY[i] =
								gv->lastDetectedObsPosY[i];
						gv->currentDetectedObsPosA[i] =
								gv->lastDetectedObsPosA[i];
					}
				}
			}
		}
		if (!gv->MT1collision && !gv->MT2collision) {
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
				pp->optimizePath();
				fo->drawPathAndCalculation();
				mt->updateMonitoredSpaceJustPlot();
				hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
				hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
				gv->tempCounter = gv->totalPlatePoints;
				for (int i = 0; i < 15; i++) {
					gv->tempCounter += gv->colliders[i].facesTimes3;
				}
				gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
						&gv->colliders[15], gv->tempCounter);
				gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
						&gv->colliders[16], gv->tempCounter);
				gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges,
						&gv->grippers[0], gv->tempCounter);
				hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[1],
						gv->tempCounter);
				gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
						&gv->distortedObstacles[0], 0);
				gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges,
						&gv->distortedObstacles[1], gv->tempCounter);
				hf->calcEdgesForMesh(gv->distorted_obs_edges,
						&gv->distortedObstacles[2], gv->tempCounter);
				gv->adv_dev_x = 0.0f;
				gv->adv_dev_y = 0.0f;
				gv->adv_dev_z = 0.0f;
				cout << "first gv->path planning was successful" << endl;
				repaint = true;
				gv->virtSim = true;
			} else {
				cout
						<< "first gv->path planning failed -> no safety evaluation possible!"
						<< endl;
				gv->simTransport = false;
				gv->simConfirmation = false;
				if (gv->writeDataIntoFiles) {
					fstream file;
					file.open("transport_result.csv", std::ios_base::app);
					if (file) {
						if (gv->advancedControlMode)
							file
									<< to_string(gv->simCounter)
											+ ";first_gv->path_plannig_failed;"
											+ to_string(gv->agentMode) + "\n";
						else
							file
									<< to_string(gv->simCounter)
											+ ";first_gv->path_plannig_failed\n";
					} else {
						cout << "Error: writing into file not possible!"
								<< endl;
					}
					file.close();
				}
				cout << "\nExecute on real robot anyway? (press y/n)" << endl;
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
					if(gv -> autoFlag == 4){
									gv -> autoFlag = 5;
									break;
								}
				} while (c_in == '-');
				if (c_in == 'y') {
					gv->synMode = 3;
					gv->asynMode = 3;
					gv->realExecPhase = 0;
				} else {
					cout << "Sim. info: no execution on real robot!" << endl;
					gv->modeActive = false;
				}
				if (gv->advancedControlMode) {
					srand((unsigned) (time(NULL)));
					generator1.seed((unsigned) (time(NULL)));
					generator2.seed((unsigned) (time(NULL)));
					generator3.seed((unsigned) (time(NULL)));
				}
			}
		} else {
			cout
					<< "MT 1/MT 2 reposition ended with gv->collision -> transport is NOT SAFE!"
					<< endl;
			gv->simTransport = false;
			gv->simConfirmation = false;
			if (gv->writeDataIntoFiles) {
				fstream file;
				file.open("transport_result.csv", std::ios_base::app);
				if (file) {
					if (gv->advancedControlMode)
						file
								<< to_string(gv->simCounter)
										+ ";mt_repositioning_gv->collision\n;"
										+ to_string(gv->agentMode) + "\n";
					else
						file
								<< to_string(gv->simCounter)
										+ ";mt_repositioning_gv->collision\n";
				} else {
					cout << "Error: writing into file not possible!" << endl;
				}
				file.close();
			}
			cout << "\nExecute on real robot anyway? (press y/n)" << endl;
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
				if(gv -> autoFlag == 4){
									gv -> autoFlag = 5;
									break;
								}
			} while (c_in == '-');
			if (c_in == 'y') {
				gv->synMode = 3;
				gv->asynMode = 3;
				gv->realExecPhase = 0;
			} else {
				cout << "Sim. info: no execution on real robot!" << endl;
				gv->modeActive = false;
			}
			for (int i = 0; i < 3; i++) {
				gv->currentDetectedObsPosX[i] = gv->tempDetectedObsPosX[i];
				gv->currentDetectedObsPosY[i] = gv->tempDetectedObsPosY[i];
			}
			if (gv->advancedControlMode) {
				srand((unsigned) (time(NULL)));
				generator1.seed((unsigned) (time(NULL)));
				generator2.seed((unsigned) (time(NULL)));
				generator3.seed((unsigned) (time(NULL)));
			}
		}
	} else
	 {
		// cout << "debug: simCtrl " << __LINE__ <<endl;
		cout
				<< "Sim. info: complete gv->path planning failed or start/end not reachable properly -> no safety evaluation possible!"
				<< endl;
		gv->modeActive = false;
		gv->simTransport = false;
		gv->simConfirmation = false;
		cout << "Sim. warning: transport cannot be executed on real robot!"
				<< endl;
		if(gv -> autoFlag == 4)
		{
			gv -> autoFlag = 5;
		}
		if (gv->writeDataIntoFiles) {
			fstream file;
			file.open("transport_result.csv", std::ios_base::app);
			if (file) {
				if (gv->advancedControlMode)
					file
							<< to_string(gv->simCounter)
									+ ";complete_gv->path_plannig_failed;"
									+ to_string(gv->agentMode) + "\n";
				else
					file
							<< to_string(gv->simCounter)
									+ ";complete_gv->path_plannig_failed\n";
			} else {
				cout << "Error: writing into file not possible!" << endl;
			}
			file.close();
		}
		if (gv->advancedControlMode) {
			srand((unsigned) (time(NULL)));
			generator1.seed((unsigned) (time(NULL)));
			generator2.seed((unsigned) (time(NULL)));
			generator3.seed((unsigned) (time(NULL)));
		}
	}
}

void SimulationControl::sim_control_sync_sim(default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3)
{
	MonitoringToolModels *mt = MonitoringToolModels::get_instance();
	CollisionDetection *cd=CollisionDetection::get_instance();
	FrequentOps *fo=FrequentOps::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();
	HelperFunctions *hf=HelperFunctions::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	Protagonists *prot=Protagonists::get_instance();
	Adversaries *adv=Adversaries::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();

	if (gv->userAbort) {
		gv->robotPoseAtAbort[0] = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Sin;
		gv->robotPoseAtAbort[1] = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Cos;
		gv->robotPoseAtAbort[2] = ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->len4 * gv->q123Cos;
		gv->robotPoseAtAbort[3] = gv->qValuesCurrent[0];
		gv->robotPoseAtAbort[4] = gv->qValuesCurrent[1];
		gv->robotPoseAtAbort[5] = gv->qValuesCurrent[2];
		gv->robotPoseAtAbort[6] = gv->qValuesCurrent[3];
		gv->robotPoseAtAbort[7] = gv->qValuesCurrent[4];
		if (gv->currentlyTransporting) {
			hf->storePathRelatedData(1);
			gv->MT1currentlyActiveTemp = gv->MT1currentlyActive; gv->MT2currentlyActiveTemp = gv->MT2currentlyActive;
		}

		cout << "Real exec. info: manual abort!" << endl;
//#if defined(WIN32)
		 cout << " " << __LINE__ <<endl;
		 sendMessageSTATE("abort_transport");
		 receiveMessageSTATE();
//#endif

		gv->syncSim = false;
		gv->modeActive = false;
		if (gv->synMode == 3) gv->realExecPhase = -1;
		else {
			gv->simTransport = false;
			gv->manualConfirmation = false;
		}
		gv->transportPhase = 0;
	}
	else {
		if (!fo->checkIfPathIsCollisionFree()) {
			gv->pathAltStartPos.x = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Sin;
			gv->pathAltStartPos.y = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Cos;
			gv->pathAltStartPos.z = ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->len4 * gv->q123Cos;
//#if defined(WIN32)
			//  cout << "debug: simctrl " << __LINE__ <<endl;
			 sendMessageSTATE("abort_transport");
			 receiveMessageSTATE();
//#endif
			gv->objectPosReachable = false;
			if (gv->transportPhase == 0) {
				for (int i = 0; i < 10; i++) {
					if (pp->calcPath(gv->qValuesObjectStart, gv->qValuesCurrent, &gv->pathStartPos, &gv->pathAltStartPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3)) {
						gv->objectPosReachable = true;
						break;
					}
				}
			}
			else if (gv->transportPhase == 1) {
				for (int i = 0; i < 10; i++) {
					if (pp->calcPath(gv->qValuesObjectEnd, gv->qValuesCurrent, &gv->pathEndPos, &gv->pathAltStartPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3)) {
						gv->objectPosReachable = true;
						break;
					}
				}
			}
			else {
				for (int i = 0; i < 10; i++) {
					if (pp->calcPath(gv->qValuesStandby, gv->qValuesCurrent, &gv->standbyPos, &gv->pathAltStartPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3)) {
						gv->objectPosReachable = true;
						break;
					}
				}
			}

			if (gv->objectPosReachable) {
				cout << "Real exec. info: gv->path gv->replanning and automatic abort necessary..." << endl;
				pp->optimizePath();

				gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
				gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
				gv->numPointsPlannedPath = 0;
				gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
				gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
				gv->numPointsRealPath = 0;

				fo->drawPathOnly(2);
				gv->pathLastPos = collider::Edge{ 0.0f, 0.0f, 0.0f };

				fo->writeIntoTempSegments();
				gv->pathDataForKuka = "normal,";
				for (int i = 0; i < gv->tempSegSize; i++) {
					gv->pathDataForKuka += hf->to_string_with_precision(gv->q0ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4) + ",";
				}
				gv->pathDataForKuka += hf->to_string_with_precision(gv->grippingWidth, 4);
//#if defined(WIN32)
				//  cout << "debug: simctrl " << __LINE__ <<endl;
				 sendMessagePATH(gv->pathDataForKuka);
				 receiveMessagePATH();
//#endif
			}
			else {
				gv->robotPoseAtAbort[0] = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Sin;
				gv->robotPoseAtAbort[1] = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Cos;
				gv->robotPoseAtAbort[2] = ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->len4 * gv->q123Cos;
				gv->robotPoseAtAbort[3] = gv->qValuesCurrent[0];
				gv->robotPoseAtAbort[4] = gv->qValuesCurrent[1];
				gv->robotPoseAtAbort[5] = gv->qValuesCurrent[2];
				gv->robotPoseAtAbort[6] = gv->qValuesCurrent[3];
				gv->robotPoseAtAbort[7] = gv->qValuesCurrent[4];
				if (gv->currentlyTransporting) {
					hf->storePathRelatedData(1);
					gv->MT1currentlyActiveTemp = gv->MT1currentlyActive; gv->MT2currentlyActiveTemp = gv->MT2currentlyActive;
				}

				cout << "Real exec. info: gv->path gv->replanning failed!" << endl;
				gv->syncSim = false;
				gv->modeActive = false;
				if (gv->synMode == 3) gv->realExecPhase = -1;
				else {
					gv->simTransport = false;
					gv->manualConfirmation = false;
				}
				gv->transportPhase = 0;
			}
		}
	}
}

/*
 * TODO: discribe what this function actually does. It is not clear to me. Is the title well-chosen???
 */
void SimulationControl::sim_control_clean_objects(bool &repaint)
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	MonitoringToolModels *mt = MonitoringToolModels::get_instance();
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

	if (!gv->MT1currentlyActive || !gv->MT2currentlyActive) cout << "Error: a MT is not active!" << endl;

	gv->modeActive = true;
	gv->collision = false;
	gv->grippingWidth = 0.0f;

	gv->q0Sin = sin(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q0Cos = cos(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q1Sin = sin(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q1Cos = cos(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q12Sin = sin((gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q12Cos = cos((gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q123Sin = sin((gv->qValuesStandby[1] + gv->qValuesStandby[2] + gv->qValuesStandby[3]) / 180.0f * M_PI);
	gv->q123Cos = cos((gv->qValuesStandby[1] + gv->qValuesStandby[2] + gv->qValuesStandby[3]) / 180.0f * M_PI);

	cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
	cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Sin + 4.5f * gv->q0Cos, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Cos - 4.5f * gv->q0Sin, ik->len0 + ik->len2 / 2.0f * gv->q1Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1]) / 180.0f * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Sin - 4.155f * gv->q0Cos, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Cos + 4.155f * gv->q0Sin, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 / 2.0f * gv->q12Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2]) / 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->lastSegmentMid * gv->q123Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180 * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);

	cd->updateMatricesTransport((-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, (gv->qValuesStandby[4] + 0) / 180.0f * M_PI, gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
	cd->rotationMatrix(gv->q0Cos * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
	gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

	cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin) * gv->q0Cos + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos + gv->grippingDiffVector[2], (-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);
	cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin) * gv->q0Cos - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos - gv->grippingDiffVector[2], (-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);

	gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
	gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
	gv->numPointsPlannedPath = 0;

	gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
	gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
	gv->numPointsRealPath = 0;

	cd->recalculateCollider(&gv->colliders[8], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true, false, nullptr, true, cd->workpieceArrowPosition1);
	hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 1);
	cd->recalculateCollider(&gv->colliders[9], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true, false, nullptr, true, cd->workpieceArrowPosition2);
	hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 2);
	cd->recalculateCollider(&gv->colliders[10], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true, false, nullptr, true, cd->workpieceArrowPosition3);
	hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 3);
	cd->recalculateCollider(&gv->colliders[11], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true, false, nullptr, true, cd->workpieceArrowPosition4);
	hf->calcEdgesForWorkpieceArrows(gv->workpiece_arrows_edges, 4);

	for (int i = 12; i <= 14; i++) {
		cd->recalculateCollider(&gv->colliders[i], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
	}

	cd->recalculateCollider(&gv->colliders[15], 0.0f, 0.0f, 10.0e5, 0, 0, 0, false, true, gv->camVectorsMT1);
	hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
	cd->recalculateCollider(&gv->colliders[16], 0.0f, 0.0f, 10.0e5, 0, 0, 0, false, true, gv->camVectorsMT2);
	hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);

	for (int i = 0; i < 9; i++) {
		gv->distortions[i] = 0.0f;
	}

	cd->recalculateCollider(&gv->distortedObstacles[0], gv->colliders[12].offsets[0] + gv->distortions[0], gv->colliders[12].offsets[1] + gv->distortions[1], gv->colliders[12].offsets[2], gv->colliders[12].angles[0] + gv->distortions[2], 0, 0);
	cd->recalculateCollider(&gv->distortedObstacles[1], gv->colliders[13].offsets[0] + gv->distortions[3], gv->colliders[13].offsets[1] + gv->distortions[4], gv->colliders[13].offsets[2], gv->colliders[13].angles[0] + gv->distortions[5], 0, 0);
	cd->recalculateCollider(&gv->distortedObstacles[2], gv->colliders[14].offsets[0] + gv->distortions[6], gv->colliders[14].offsets[1] + gv->distortions[7], gv->colliders[14].offsets[2], gv->colliders[14].angles[0] + gv->distortions[8], 0, 0);

	cd->overrideCollider(gv->workpieceMesh, &gv->colliders[8]);
	cd->recalculateCollider(gv->workpieceMesh, 0.0f, 0.0f, 10.0e5, 0, 0, 0);

	mt->updateMonitoredSpace(true, true);

	gv->tempCounter = gv->totalPlatePoints;
	for (int i = 0; i < 3; i++) {
		gv->tempCounter += gv->colliders[i].facesTimes3;
	}
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[3], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[4], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[5], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[6], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[7], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[8], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[9], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[10], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[11], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[12], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[13], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[14], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[15], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[16], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[0], gv->tempCounter);
	hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[1], gv->tempCounter);

	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[8], 0);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[9], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[10], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[11], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[12], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[13], gv->tempCounter);
	hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[14], gv->tempCounter);

	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[0], 0);
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[1], gv->tempCounter);
	hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2], gv->tempCounter);

	hf->calcEdgesForMesh(gv->workpiece_mesh_edges, gv->workpieceMesh, 0);
	gv->initPhase = 1;
	repaint = true;
}

/*
 * Todo: find out, what this function does.
 * -it seems to be some calculations with the object positions/robot positions.
 * Depending on what the function does, it should be moved to a different module!
 */
void SimulationControl::sim_control_apply_changes_from_init(){
	GlobalVariables *gv=GlobalVariables::get_instance();
	CollisionDetection *cd=CollisionDetection::get_instance();
	InverseKinematic *ik=InverseKinematic::get_instance();

	cout << "debug: simControl l." << __LINE__ <<endl;

	gv->q0Sin = sin(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q0Cos = cos(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q1Sin = sin(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q1Cos = cos(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q12Sin = sin((gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q12Cos = cos((gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q123Sin = sin((gv->qValuesStandby[1] + gv->qValuesStandby[2] + gv->qValuesStandby[3]) / 180.0f * M_PI);
	gv->q123Cos = cos((gv->qValuesStandby[1] + gv->qValuesStandby[2] + gv->qValuesStandby[3]) / 180.0f * M_PI);

	cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
	cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Sin + 4.5f * gv->q0Cos, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Cos - 4.5f * gv->q0Sin, ik->len0 + ik->len2 / 2.0f * gv->q1Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1]) / 180.0f * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Sin - 4.155f * gv->q0Cos, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Cos + 4.155f * gv->q0Sin, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 / 2.0f * gv->q12Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2]) / 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->lastSegmentMid * gv->q123Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos, (-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180 * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);

	cd->updateMatricesTransport((-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, (gv->qValuesStandby[4] + 0) / 180.0f * M_PI, gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
	cd->rotationMatrix(gv->q0Cos * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
	gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

	cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin) * gv->q0Cos + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos + gv->grippingDiffVector[2], (-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);
	cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin) * gv->q0Cos - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos - gv->grippingDiffVector[2], (-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3]) / 180.0f * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);

	gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
	gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
	gv->numPointsPlannedPath = 0;

	gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
	gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
	gv->numPointsRealPath = 0;

	for (int i = 0; i < gv->transportableObjectsCount; i++) {
		if (i < gv->transportableObjectsCount - 2) {
			cd->recalculateCollider(&gv->colliders[gv->transportableObjects[i]], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, true);
		}
		else if (gv->transportableObjects[i] == 15) {
			cd->recalculateCollider(&gv->colliders[15], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, false, true, gv->camVectorsMT1);
		}
		else  if (gv->transportableObjects[i] == 16) {
			cd->recalculateCollider(&gv->colliders[16], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, false, true, gv->camVectorsMT2);
		}
	}
}



bool SimulationControl::do_something5(bool repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3) {

	GlobalVariables *gv = GlobalVariables::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	RealRobotExecution *rre = RealRobotExecution::get_instance();
	MixedFunctions *mf = MixedFunctions::get_instance();
	// cout << "dbg@simCtrl " << __LINE__ << endl;
	//TODO: this seems to be the second path planning attempt. Where is the first one???
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
		pp->optimizePath();
		fo->drawPathAndCalculation();
		gv->adv_dev_x = 0.0f;
		gv->adv_dev_y = 0.0f;
		gv->adv_dev_z = 0.0f;
		repaint = true;
		gv->virtSim = true;
	} else {
		gv->transportPhase = 0;
		cout
				<< "Sim. info: second path planning failed -> no safety evaluation possible!"
				<< endl;
		gv->simTransport = false;
		gv->simConfirmation = false;
		mf->log_transport_result();
		rre->sim_control_ask_real_execution();
		if (gv->advancedControlMode) {
			srand((unsigned) (time(NULL)));
			generator1.seed((unsigned) (time(NULL)));
			generator2.seed((unsigned) (time(NULL)));
			generator3.seed((unsigned) (time(NULL)));
		}
	}
	return repaint;
}



bool SimulationControl::do_something6(bool repaint,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3) {

	GlobalVariables *gv = GlobalVariables::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	RealRobotExecution *rre = RealRobotExecution::get_instance();

	// cout << "debug: main " << __LINE__ << endl;
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
		pp->optimizePath();
		fo->drawPathAndCalculation();
		gv->adv_dev_x = 0.0f;
		gv->adv_dev_y = 0.0f;
		gv->adv_dev_z = 0.0f;
		repaint = true;
		gv->virtSim = true;
	} else {
		gv->transportPhase = 0;
		cout
				<< "Sim. info: third gv->path planning failed -> no safety evaluation possible!"
				<< endl;
		gv->simTransport = false;
		gv->simConfirmation = false;
		if (gv->writeDataIntoFiles) {
			fstream file;
			file.open("transport_result.csv", std::ios_base::app);
			if (file) {
				if (gv->advancedControlMode)
					file
							<< to_string(gv->simCounter)
									+ ";third_gv->path_plannig_failed;"
									+ to_string(gv->agentMode) + "\n";
				else
					file
							<< to_string(gv->simCounter)
									+ ";third_gv->path_plannig_failed\n";
			} else {
				cout << "Error: writing into file not possible!" << endl;
			}
			file.close();
		}

		rre->sim_control_ask_real_execution();

//		cout << "\nExecute on real robot anyway? (press y/n)" << endl;
//		char c_in = '-';
//		SDL_Event event_sub;
//		do {
//			while (SDL_PollEvent(&event_sub)) {
//				if (event_sub.type == SDL_KEYUP) {
//					if (event_sub.key.keysym.sym == SDLK_y) {
//						c_in = 'y';
//					} else if (event_sub.key.keysym.sym == SDLK_n) {
//						c_in = 'n';
//					}
//				}
//			}
//		} while (c_in == '-');
//		if (c_in == 'y') {
//			gv->synMode = 3;
//			gv->asynMode = 3;
//			gv->realExecPhase = 0;
//		} else {
//			cout << "No execution on real robot!" << endl;
//			gv->modeActive = false;
//		}
		if (gv->advancedControlMode) {
			srand((unsigned) (time(NULL)));
			generator1.seed((unsigned) (time(NULL)));
			generator2.seed((unsigned) (time(NULL)));
			generator3.seed((unsigned) (time(NULL)));
		}
	}
	return repaint;
}

void SimulationControl::transport_phase1()
{
	CollisionDetection *cd = CollisionDetection::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf=HelperFunctions::get_instance();

	cd->updateMatricesTransport(
			(-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]
					- gv->qValuesCurrent[3]) / 180.0f * M_PI,
			(gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI,
			gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
	cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + gv->objectHeight + gv->objectGripDepth)
							* gv->q123Sin) * gv->q0Sin,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + gv->objectHeight + gv->objectGripDepth)
							* gv->q123Sin) * gv->q0Cos,
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos
					+ (ik->len4 + gv->objectHeight + gv->objectGripDepth)
							* gv->q123Cos,
			(-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]
					- gv->qValuesCurrent[3]) / 180.0f * M_PI,
			(gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI,
			true);
	cd->recalculateColliderTransport(gv->safetyCollider,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + gv->objectHeight + gv->objectGripDepth)
							* gv->q123Sin) * gv->q0Sin,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + gv->objectHeight + gv->objectGripDepth)
							* gv->q123Sin) * gv->q0Cos,
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos
					+ (ik->len4 + gv->objectHeight + gv->objectGripDepth)
							* gv->q123Cos,
			(-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]
					- gv->qValuesCurrent[3]) / 180.0f * M_PI,
			(gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
	for (int i = 8; i < gv->objectToTransport; i++) {
		gv->tempCounter += gv->colliders[i].facesTimes3;
	}
	hf->calcEdgesForConvexHull(gv->object_edges,
			&gv->colliders[gv->objectToTransport], gv->tempCounter);
	hf->calcEdgesForMesh(gv->safety_col_edges, gv->safetyCollider, 0);
	gv->tempCounter = 0;
	for (int i = 8; i < gv->objectToTransport; i++) {
		gv->tempCounter += gv->colliders[i].patternCount * 4;
	}
	hf->calcEdgesForPatterns(gv->object_pattern_edges,
			&gv->colliders[gv->objectToTransport], gv->tempCounter);
	// Mit sich selbst:
	for (int i = 1; i <= 4; i++) {
		if (cd->checkForCollision(&gv->colliders[i], &gv->colliders[7])) {
			gv->collision = true;
			gv->collisionObsIndex = i;
		}
		if (cd->checkForCollision(&gv->colliders[i],
				&gv->colliders[gv->objectToTransport])) {
			gv->collision = true;
			gv->collisionObsIndex = i;
			break;
		}
		if (cd->checkForCollision(&gv->colliders[i], &gv->grippers[0])) {
			gv->collision = true;
			gv->collisionObsIndex = i;
			break;
		}
		if (cd->checkForCollision(&gv->colliders[i], &gv->grippers[1])) {
			gv->collision = true;
			gv->collisionObsIndex = i;
			break;
		}
	}
	// Mit Umgebung:
	if (!gv->collision) {
		for (int i = 5; i <= 7; i++) {
			for (int j = 8; j < gv->collidersCount; j++) {
				if (j != gv->objectToTransport) {
					if (cd->checkForCollision(&gv->colliders[i],
							&gv->colliders[j])) {
						gv->collision = true;
						cout<< "collision with5 :"<< j << endl;
						gv -> collisionObsIndex = j;
						break;
					}
				}
			}
			if (gv->collision)
				break;
		}
	}
	// Transportobjekt mit anderen Kl�tzchen:
	if (!gv->collision) {
		for (int i = 8; i < gv->collidersCount; i++) {
			if (i != gv->objectToTransport) {
				if (cd->checkForCollision(&gv->colliders[gv->objectToTransport],
						&gv->colliders[i])) {
					gv->collision = true;
					gv->collisionObsIndex = i;
					break;
				}
			}
		}
	}
	// Greifer mit anderen Kl�tzchen:
	if (!gv->collision) {
		for (int i = 8; i < gv->collidersCount; i++) {
			if (i != gv->objectToTransport) {
				if (cd->checkForCollision(&gv->grippers[0],
						&gv->colliders[i])) {
					gv->collision = true;
					cout<< "collision with6 :"<< i<< endl;
					gv -> collisionObsIndex = i;
					break;
				}
				if (cd->checkForCollision(&gv->grippers[1],
						&gv->colliders[i])) {
					gv->collision = true;
					cout<< "collision with7 :"<< i<< endl;
					gv -> collisionObsIndex = i;
					break;
				}
			}
		}
	}
	// Mit Boden!
	if (!gv->collision) {
		if (cd->checkForCollisionWithGround(&gv->colliders[7], -cd->epsilon2)
				|| cd->checkForCollisionWithGround(
						&gv->colliders[gv->objectToTransport], -cd->epsilon2)
				|| cd->checkForCollisionWithGround(&gv->grippers[0],
						-cd->epsilon2)
				|| cd->checkForCollisionWithGround(&gv->grippers[0],
						-cd->epsilon2)) {
			gv->collision = true;
		}
	}
}
