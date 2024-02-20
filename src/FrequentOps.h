/*
 * FrequentOps.h
 *
 *  Created on: 05.12.2022
 *      Author: manuel
 */

#ifndef FREQUENTOPS_H_
#define FREQUENTOPS_H_

#include <random>
#include "Constants.h"

#include "collider.h"
#include "GlobalVariables.h"
#include "InverseKinematic.h"
#include "HelperFunctions.h"

class FrequentOps {

private:

//	HelperFunctions *hf;
//	GlobalVariables *gv;

	static FrequentOps *fo;

//	int visiblePatternsMT1[objectsWithPatternsCount], visiblePatternsMT2[objectsWithPatternsCount];
	int interpolationStartPoint;



	
	float lastJointAngleMax, lastJointAngleStartTemps[4], lastJointAngleEndTemps[4];


	bool isTransportPossible, isPartialSegment;

	collider::Edge* firstWorkpieceDetections = new collider::Edge[0];
	collider::Edge* secondWorkpieceDetections = new collider::Edge[0];
	collider::Edge* thirdWorkpieceDetections = new collider::Edge[0];
	collider::Edge* fourthWorkpieceDetections = new collider::Edge[0];

	collider::Edge* firstObsDetections = new collider::Edge[0];
	collider::Edge* secondObsDetections = new collider::Edge[0];
	collider::Edge* thirdObsDetections = new collider::Edge[0];

	collider::Edge* firstWorkpieceDetections2 = new collider::Edge[0];
	collider::Edge* secondWorkpieceDetections2 = new collider::Edge[0];
	collider::Edge* thirdWorkpieceDetections2 = new collider::Edge[0];
	collider::Edge* fourthWorkpieceDetections2 = new collider::Edge[0];

	collider::Edge* firstObsDetections2 = new collider::Edge[0];
	collider::Edge* secondObsDetections2 = new collider::Edge[0];
	collider::Edge* thirdObsDetections2 = new collider::Edge[0];



public:
	float minDistancesForMetric[GlobalVariables::totalObjectsCount + 2];
	float randomObstalcePosition[2];
	float distortedRelativePosition[4], distortedAbsolutePosition[3], distortedPatternCenter[3];
	bool detectedPatternIndicesForMT1[totalPatternCount], detectedPatternIndicesForMT2[totalPatternCount];
	float relativeObjPositions[2], relativeObjectDir[2], absoluteObjectDir[2], targetDetectionAngle, actionSpaceDist = 0.0f, actionSpaceAng = 0.0f;
	float currentActionSpaceDist, currentActionSpaceAng;
	int detectionCounts[objectsWithPatternsCount]{ 0, 0, 0, 0, 0, 0, 0 }, detectionCounts2[objectsWithPatternsCount]{ 0, 0, 0, 0, 0, 0, 0 }, actionSpaceSamples;
	collider::Edge evaluatedDetections[objectsWithPatternsCount];

	FrequentOps();
	virtual ~FrequentOps();
	static FrequentOps* get_instance();

	void drawPathAndCalculation();

	void gripAngleCalculation(int gripDirections);

	void calcMetricValue();
	bool checkIfPathIsCollisionFree();
	void getRandomObstaclePosition(
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3
	);
	void drawPathOnly(int pathColorIndex);
	void calcPosFromMTData(collider* col, float patternPosX, float patternPosY, float patternPosZ, float patternAngle, int patternIndex) ;

	void resetDetectedPatterns() ;
	void resetDetectedPatternsAlt();
	void insertDetectedPattern(int objectIndex) ;
	void insertDetectedPatternAlt(int objectIndex);
	void evaluateDetections(bool calcActionSpace);
	void evaluateDetectionsAlt();
	void writeIntoTempSegments();
	bool checkMTTransport(int mtIndex,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3
	);
	bool checkNewTransport(
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3
	);
	bool checkUnfinishedTransport(
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3
	);
};

#endif /* FREQUENTOPS_H_ */
