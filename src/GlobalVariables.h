/*
 * GlobalVariables.h
 *
 *  Created on: 26.11.2022
 *      Author: manuel
 *
 *  Note: This is a fundamental basis class. Be careful with moving stuff into it.
 *  Avoid dependancies to other classes.
 */

#ifndef GLOBALVARIABLES_H_
#define GLOBALVARIABLES_H_

#include "collider.h"//only exception!

#include <string>
#include <random>
#include <cmath>
#include "ObjectPositionInterface.h"

#include "serverMT1.h"
#include "serverMT2.h"
#include "serverPath.h"
#include "serverState.h"
#include "serverML.h"

using namespace std;

class GlobalVariables {

private:
	static GlobalVariables* gv;
	GlobalVariables();

public:
	virtual ~GlobalVariables();
	static GlobalVariables* get_instance();


	// ---------------------------------------------------------
	// ------------Global Variables                -------------
	// ---------------------------------------------------------
	const static int transportableObjectsCount = 6, totalObjectsCount = 9;
	const static int actionSpaceCount = 4;

	const static int collidersCount = 17;
	const static int totalPlatePoints = 2244;
	const static int innerPlatePoints = 1620;

	const bool writeDataIntoFiles = false;
	const bool virtualMode = true;
	const bool processInitWithRealRobot = true; // false
	const bool onlyAskMTsAfterInit = false;
	const bool advancedControlMode = false; //TODO: make this editable!
	const bool calculateDistanceMetric = false;

	const collider::Edge* pathColors = new collider::Edge[3]{ collider::Edge{1.0f, 0.0f, 0.0f}, collider::Edge{0.0f, 1.0f, 0.0f}, collider::Edge{0.0f, 0.0f, 1.0f} };
	const collider::Edge* otherColors = new collider::Edge[7]{ collider::Edge{0.3f, 0.3f, 0.3f}, collider::Edge{0.0f, 0.0f, 0.0f}, collider::Edge{1.0f, 0.0f, 0.0f}, collider::Edge{7.0f / 255.0f, 110.0f / 255.0f, 235.0f / 255.0f}, collider::Edge{0.2f, 0.2f, 0.2f}, collider::Edge{0.0f, 0.0f, 0.0f}, collider::Edge{0.0f, 0.0f, 0.0f} };
	const collider::Edge* objectDimensions = new collider::Edge[transportableObjectsCount]{ collider::Edge{ 2.5f, 1.15f, 1.5f }, collider::Edge{ 1.5f, 2.0f, 1.5f }, collider::Edge{ 1.5f, 1.75f, 1.5f }, collider::Edge{ 1.5f, 2.0f, 1.5f }, collider::Edge{ 2.5f, 1.5f, 3.5f }, collider::Edge{ 2.5f, 1.5f, 3.5f } };

	const float actionSpaceDistances[actionSpaceCount]{ 0.25f, 0.5f, 0.75f, 1.0f };
	const float actionSpaceAngles[actionSpaceCount]{ 4.0f / 180.0f * M_PI, 8.0f / 180.0f * M_PI, 12.0f / 180.0f * M_PI, 16.0f / 180.0f * M_PI };
	const float grippingWidthAdditional = 0.5f, grippingWidthMax = 4.1f, gripperInstallationOffset = 0.15f;
	ObjectPositionInterface obstacles[3];


	//float* camVectorsMT1 = new float[12]; float* camVectorsMT2 = new float[12];
	float* camVectorsMT1;
	float* camVectorsMT2;
	float* camPos    ;
	float* camAngles ;
	float* lightDir  ;
	float* camDir    ;
	float* camHorDir ;
	float* camVerDir ;
	float* displayPos;

	bool MT1currentlyActive, MT2currentlyActive, MT1currentlyActiveTemp, MT2currentlyActiveTemp, MT1collision, MT2collision, MT1posChanged, MT2posChanged, MT1transported, MT2transported;
	bool obsLocError[3];
	bool finishSimInit = false;

	int pointValidCounter;
	int simCounter;
	int timeCounter;
	int colorsCount, colorIndex;
	// flag to control the sequence of simulator automation
	int autoFlag = 0;
	int collisionObsIndex = -1;
	float light_intensity = 1.0;

	float oldObstaclePositions[9], newMTPos[3], tempMTPos[3], oldMTPos[3];
	float currentDetectedObsPosX[3], currentDetectedObsPosY[3], currentDetectedObsPosA[3];
	float lastDetectedObsPosX[3], lastDetectedObsPosY[3], lastDetectedObsPosA[3];
	float tempDetectedObsPosX[3], tempDetectedObsPosY[3];
	float surfaceNormal[3], surfaceO[3], surfaceV[3], surfaceV1[3], surfaceV2[3], surfaceDval, surfaceTval, surfaceA, surfaceB, surfaceN0, surfaceN1, surfaceN2, distInDir, distInHor, distInVert;
	float minTValueLine, minDistValueLine, minLinePointVector[3];
	float patternNx, patternNy, patternNz, patternAngle, centerRay[3], centerRayLen;

	float xDisp, yDisp, zDisp, xDiffDisp, yDiffDisp, zDiffDisp;
	float normX, normY, normZ, cross1X, cross1Y, cross1Z, cross2X, cross2Y, cross2Z, angleLight, lengthLight;
	float horAngle, nickAngle, rollAngle;

	float distortions[9]{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	float savedRot;

	float* qValuesCurrent     ;//= new float[5];
	float* qValuesTarget      ;//= new float[5];
	float* qValuesStandby     ;//= new float[5];
                              ;//
	float* qValuesDiff        ;//= new float[5];
	float* qValuesObjectStart ;//= new float[5];
	float* qValuesObjectEnd   ;//= new float[5];
	float* qValuesStartPoint  ;//= new float[5];
	float* qValuesEndPoint    ;//= new float[5];

	collider::Edge* colors; // = new collider::Edge[colorsCount]{ collider::Edge{ 0.7f, 0.7f, 0.7f }, collider::Edge{ 0.5f, 0.5f, 0.5f } };
	int* colorStartPoints ;


	collider* colliders            ;//= new collider[collidersCount];
	collider* grippers             ;//= new collider[2];
	collider* safetyCollider       ;//= new collider;
	collider* distortedObstacles   ;//= new collider[3];
	collider* adversaryCollider    ;//= new collider[4];
	collider* safetyCollidersForMTs;//= new collider[2];
	collider* workpieceMesh        ;//= new collider;

	collider::Edge standbyPos     ;
	collider::Edge savedPos       ;
                                  ;
	collider::Edge pathAltStartPos;
	collider::Edge targetPos      ;
	collider::Edge pathStartPos   ;
	collider::Edge pathEndPos     ;
	collider::Edge pathLastPos    ;

	bool virtSim, syncSim, paused, selectionMarkerMode;
	bool valids[3] , valid, collision, collision_init, objectPosReachable, freeSight, intersectingBoundingBox, aborting, replanning, objectInOff, found;

	int simSpeed;
	int fpsMax[6]{ 8, 20, 40, 60, 80, 160 };


	int transportPhase = 0, objectToTransport, objectIndex;
	const int transportableObjects[transportableObjectsCount]{ 8, 9, 10, 11, 15, 16 }, transportableObjectsGrip[transportableObjectsCount]{ 1, 2, 2, 2, 1, 1 };
	float objectHeights[transportableObjectsCount], objectOffsets[transportableObjectsCount], objectHeight, objectOffset, startObjectAngle, endObjectAngle, grippingDiffVector[3];
	float objectGripDepths[transportableObjectsCount], objectGripDepth;

	float grippingAngleDiff, grippingWidthOpen, grippingWidthFixed, grippingWidth = 0.0f;
	collider::Edge objectDimension, objectDimensionTemp;

	int randomWorkspace, randomObs, randomAgent = -1;

	int maxValueIndex, maxValueIndexTemp;
	float maxValue;
	float q0Sin, q0Cos, q1Sin, q1Cos, q12Sin, q12Cos, q123Sin, q123Cos;

	float* q0ValsFirstSeg ; float* q1ValsFirstSeg ; float* q2ValsFirstSeg ; float* q3ValsFirstSeg ; float* q4ValsFirstSeg ;
	float* q0ValsSecondSeg; float* q1ValsSecondSeg; float* q2ValsSecondSeg; float* q3ValsSecondSeg; float* q4ValsSecondSeg;
	float* q0ValsThirdSeg ; float* q1ValsThirdSeg ; float* q2ValsThirdSeg ; float* q3ValsThirdSeg ; float* q4ValsThirdSeg ;
	float* q0ValsTempSeg  ; float* q1ValsTempSeg  ; float* q2ValsTempSeg  ; float* q3ValsTempSeg  ; float* q4ValsTempSeg  ;
	int firstSegSize = 0, secondSegSize = 0, thirdSegSize = 0, tempSegSize = 0;
	float* secondSegLengths;
	float secondSegLengthTotal;

	float initPositionsX[6], initPositionsY[6], initPositionsAngles[6];
	float mtPositionsX[2], mtPositionsY[2], mtPositionsAngles[2];

	bool activeAgents[5]{ false, false, false, false, false }; // safety, MT, loc, path, obs
	int agentMode;
	float reward = 0.0f, reward1 = 0.0f, reward2 = 0.0f, reward3 = 0.0f;

	float adv_dev_x = 0.0f, adv_dev_y = 0.0f, adv_dev_z = 0.0f;
	int path_adv_counter = 0, obs_adv_counter = 0;

	float maxViewVal, tempViewVal, oldViewVal, vectorLenDir, vectorLenObj;
	float distVectorMT[2], distVectorMTlen, angleDiffMT, maxDot;
	float diffVectorSafety[3];
	int iMin, jMin, kMin, angleMin, iMax, jMax, kMax, angleMax, action, initIterations;
	float posCurrent[3], posTarget[3], posNext[3], posFollowingNext[3], minDistVector[3];
	float minAdvDist, tempAdvDist, advDirectionLength, tempAdvAngle;
	float locDiffX, locDiffY, locDiffAngle, locDiffTemp, locDiffMax, obsDiffTemp, obsDiffMin;
	float distToStartColl[3], distToEndColl[3];


	string guiString, messageToSendML, coreMessageML, receivedAnswerML, receivedAvailableAgents, stateMessageMT1, stateMessageMT2, stateMessagePATH, stateMessageSTATE, pathDataForKuka, stateMessageObs, stateSM;
	string stateMessageMT1saved, stateMessageMT2saved;

	collider::Vertex* object_vertices           ;//= new collider::Vertex[0];
	collider::Vertex* object_pattern_vertices   ;//= new collider::Vertex[0];
	collider::Vertex* planned_path_vertices     ;//= new collider::Vertex[0];
	collider::Vertex* real_path_vertices        ;//= new collider::Vertex[0];
	collider::Vertex* mt_viewfields_vertices    ;//= new collider::Vertex[32];
	collider::Vertex* mt_rays_vertices          ;//= new collider::Vertex[0];
	collider::Vertex* distorted_obs_vertices    ;//= new collider::Vertex[72 * 3];
	collider::Vertex* ground_truth_obs_vertices ;//= new collider::Vertex[72 * 3];
	collider::Vertex* safety_col_vertices       ;//= new collider::Vertex[72];
	collider::Vertex* workpiece_mesh_vertices   ;//= new collider::Vertex[72];
	collider::Vertex* workpiece_arrows_vertices ;//= new collider::Vertex[24];
                                                ;//
	collider::Edge* object_edges                ;//= new collider::Edge[0];
	collider::Edge* object_pattern_edges        ;//= new collider::Edge[0];
	collider::Edge* planned_path_edges          ;//= new collider::Edge[0];
	collider::Edge* real_path_edges             ;//= new collider::Edge[0];
	collider::Edge* mt_viewfields_edges         ;//= new collider::Edge[32];
	collider::Edge* mt_rays_edges               ;//= new collider::Edge[0];
	collider::Edge* distorted_obs_edges         ;//= new collider::Edge[72 * 3];
	collider::Edge* ground_truth_obs_edges      ;//= new collider::Edge[72 * 3];
	collider::Edge* safety_col_edges            ;//= new collider::Edge[72];
	collider::Edge* workpiece_mesh_edges        ;//= new collider::Edge[72];
	collider::Edge* workpiece_arrows_edges      ;//= new collider::Edge[24];

	uint32_t tempCounter, numPointsObject, numPointsPattern, numPointsPlannedPath, numPointsRealPath, numPointsMTrays;

	int asynMode;
	int synMode;
	bool modeActive, initSpacePressed, initProcessed, initConfirmation, initWaitForFinish, simConfirmation, manualConfirmation, initPreviouslyProcessed;
	int initPhase = 0, realExecPhase, realExecSubPhase, realNodeIndex, manualMode;

	int fittingPatternsMax, fittingPatternsTemp, simAttemptCounter;
	float finalObstaclePositions[9];
	float groundTruthObstaclePositions[9]{ 0.0f, -25.0f, 0.0f, 0.0f, -25.0f, 0.0f, 0.0f, -25.0f, 0.0f };

	int axisX, axisY, axisXold, axisYold;
	int selectedWorkpieceIndex;
	float selectedTargetPosition[4], rotationValue;
	bool simTargetSelection, simTransport, simAskUser;

	float pathInterpolationFactor, realGripperWidth;
	float storedPathRelatedData1[28], storedPathRelatedData2[28];
	bool userAbort, atStandby, currentlyTransporting, movingToStandby;
	float robotPoseAtAbort[8];

	float q0Sin_pp, q0Cos_pp, q1Sin_pp, q1Cos_pp, q12Sin_pp, q12Cos_pp, q123Sin_pp, q123Cos_pp;
};

#endif /* GLOBALVARIABLES_H_ */
