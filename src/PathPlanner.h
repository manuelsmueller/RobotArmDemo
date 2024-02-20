/*
 * PathPlanner.h
 *
 *  Created on: 29.11.2022
 *      Author: manuel
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include "collider.h"
#include "GlobalVariables.h"
#include "MonitoringToolModels.h"
#include "CollisionDetection.h"
#include "InverseKinematic.h"

#include <random>

class PathPlanner {

private:
	PathPlanner();
	static PathPlanner *pp;
public:

	virtual ~PathPlanner();
	static PathPlanner* get_instance();

	bool collisionWithEnvTransport(bool startOrEnd, bool debug);
	bool collisionWithEnv(bool print);
	float getMinDistanceTo(collider* col, bool transporting, float currentMinDist);
	float getMaxZVal(float x, float y, float z);
	void initNodeList();
	void resetNodeList();
	bool verifyPath();
	void optimizePath();
//	bool calcPath(float* q_start, float* q_end, collider::Edge* startPoint, collider::Edge* endPoint, bool extended);
//	bool calcPath(float* q_start, float* q_end, collider::Edge* startPoint, collider::Edge* endPoint, bool extended, default_random_engine generator3, normal_distribution<float> distribution3);
	bool calcPath(float* q_start, float* q_end, collider::Edge* startPoint,
			collider::Edge* endPoint, bool extended,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);


	// struct for stroring node informations:
	struct Node {
	    float x;
	    float y;
	    float z;

	    // indices of neigbors:
	    int* neighbors;
	    int* fromOthers;
	    int otherCount;

	    // for the Dijkstra algorithm:
	    float dist;
	    int previous;
	    bool processed;

	    float* nodeQVal;

	    int* verifiedNeighs;
	    int* verifiedOthers;
	};

	const float samplingStepSize = 0.5f; // resolution for the check of the collision-freeness of edges in degrees
	const float simulationStepSize = 0.5f; // speed of the robot in degree per frame
	const int targetNodesCount = 1500; // target size of the node set
	int nodesCount = 0, realCount = 0, hitCount;
	const int neighborsCount = 10; // target neighbor count for each node
	const int othersCount = 35;

	Node* nodeList = new Node[targetNodesCount];

	float xRand_pp, yRand_pp, zRand_pp;
	float* qValRand = new float[5];
	float* qValStart = new float[5];
	float* qValEnd = new float[5];
	float* qValInter = new float[5];

	float* distanceList = new float[neighborsCount];

	bool inOthers, inNeigh, isPathValid, redoDijkstra, isFirstIteration, collWithEnv;

	int qSize, sampleCount, sampleCounts[3];
	int currentNode, nextNode;

	float nodeDistance, nodeDistanceTemp, maxJointDiff;
	int nodeIndex, nodeIndexTemp, nodeIndexStart, nodeIndexEnd, maxJointIndex, neighborNode;

	int minValueIndex_pp;
	float minValue_pp, alt_pp;

	float startPointAngle, endPointAngle, startPointRadius, endPointRadius, tempRadius, maxZVal, tempZVal;
	float currAngle, currRadius, currHeight, currFactor, currTVal;

	float angleLayer_nx, angleLayer_ny, angleLayer_nz, angleLayer_d, angleLayer_nlen, angleLayer_nlen2, angleLayer_factor;
	collider::Edge* startPoint; collider::Edge* endPoint;

	float tempMinDistance, tempMinDistance2;
};

#endif /* PATHPLANNER_H_ */
