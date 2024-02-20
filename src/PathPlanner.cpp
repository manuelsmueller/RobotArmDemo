/*
 * PathPlanner.cpp
 *
 *  Created on: 29.11.2022
 *      Author: manuel
 */

#include "PathPlanner.h"

PathPlanner::PathPlanner() {
	// TODO Auto-generated constructor stub

}

PathPlanner::~PathPlanner() {
	// TODO Auto-generated destructor stub
}

/*
 * singelton pattern
 */
PathPlanner* PathPlanner::pp = 0;
PathPlanner* PathPlanner::get_instance(){
	static bool isInit=false;

//	InverseKinematic *ik;
	if(!isInit){
		pp  = new PathPlanner();
		isInit=true;
	}
	return pp;
}

// methods for collision detection (collisionWithEnvTransport() during transportation, collisionWithEnv() for other cases).
// All combinations of gv->colliders between which collisions can occur are checked. Only known object positions are used:
bool PathPlanner::collisionWithEnvTransport(bool startOrEnd, bool debug) {
	CollisionDetection *cd = CollisionDetection::get_instance();
    GlobalVariables *gv=GlobalVariables::get_instance();
	for (int i = 1; i <= 4; i++) {
        if (cd->checkForCollision(&gv->colliders[i], &gv->colliders[7])) {
            if (debug) cout << "Coll: " << i << "/7" << endl;
            return true;
        }
        if (cd->checkForCollision(&gv->colliders[i], gv->safetyCollider)) {
            if (debug) cout << "Coll: " << i << "/safety" << endl;
            return true;
        }
        if (cd->checkForCollision(&gv->colliders[i], &gv->grippers[0])) {
            if (debug) cout << "Coll: " << i << "/gripper0" << endl;
            return true;
        }
        if (cd->checkForCollision(&gv->colliders[i], &gv->grippers[1])) {
            if (debug) cout << "Coll: " << i << "/gripper1" << endl;
            return true;
        }
    }

    for (int i = 5; i <= 7; i++) {
        for (int j = 8; j < gv->collidersCount; j++) {
            if (j >= 12 && j <= 14) {
                if (abs(gv->currentDetectedObsPosX[j - 12]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[j - 12]) > cd->epsilon3) {
                    if (cd->checkForCollision(&gv->colliders[i], &gv->distortedObstacles[j - 12])) {
                        if (debug) cout << "Coll: " << i << "/distorted" << j - 12 << endl;
                        return true;
                    }
                }
            }
            else if (j != gv->objectToTransport) {
                if (cd->checkForCollision(&gv->colliders[i], &gv->colliders[j])) {
                    if (debug) cout << "Coll: " << i << "/" << j << endl;
                    return true;
                }
            }
        }
    }

    for (int j = 8; j < gv->collidersCount; j++) {
        if (j >= 12 && j <= 14) {
            if (abs(gv->currentDetectedObsPosX[j - 12]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[j - 12]) > cd->epsilon3) {
                if (cd->checkForCollision(&gv->grippers[0], &gv->distortedObstacles[j - 12])) {
                    if (debug) cout << "Coll: gripper0/distorted" << j - 12 << endl;
                    return true;
                }
                if (cd->checkForCollision(&gv->grippers[1], &gv->distortedObstacles[j - 12])) {
                    if (debug) cout << "Coll: gripper1/distorted" << j - 12 << endl;
                    return true;
                }
            }
        }
        else if (j != gv->objectToTransport) {
            if (cd->checkForCollision(&gv->grippers[0], &gv->colliders[j])) {
                if (debug) cout << "Coll: gripper0/" << j - 12 << endl;
                return true;
            }
            if (cd->checkForCollision(&gv->grippers[1], &gv->colliders[j])) {
                if (debug) cout << "Coll: gripper1/" << j - 12 << endl;
                return true;
            }
        }
    }

    for (int j = 8; j < gv->collidersCount; j++) {
        if (j >= 12 && j <= 14) {
            if (abs(gv->currentDetectedObsPosX[j - 12]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[j - 12]) > cd->epsilon3) {
                if (cd->checkForCollision(gv->safetyCollider, &gv->distortedObstacles[j - 12])) {
                    if (debug) cout << "Coll: safety/distorted" << j - 12 << endl;
                    return true;
                }
            }
        }
        else if (j != gv->objectToTransport) {
            if (cd->checkForCollision(gv->safetyCollider, &gv->colliders[j])) {
                if (debug) cout << "Coll: safety/" << j << endl;
                return true;
            }
        }
    }

    if (cd->checkForCollisionWithGround(&gv->colliders[7], 0) || cd->checkForCollisionWithGround(&gv->grippers[0], 0) || cd->checkForCollisionWithGround(&gv->grippers[1], 0)) {
        if (debug) cout << "Coll: ground (gripper or end)" << endl;
        return true;
    }

    if (!startOrEnd) {
        if (cd->checkForCollisionWithGround(gv->safetyCollider, 0)) {
            if (debug) cout << "Coll: ground (safety)" << endl;
            return true;
        }
    }

    return false;
}

bool PathPlanner::collisionWithEnv(bool print) {
	GlobalVariables *gv=GlobalVariables::get_instance();
	CollisionDetection *cd = CollisionDetection::get_instance();
    for (int i = 1; i <= 4; i++) {
        if (cd->checkForCollision(&gv->colliders[i], &gv->colliders[7])) {
            if (print) cout << "Coll: " << i << "/7" << endl;
            return true;
        }
        if (cd->checkForCollision(&gv->colliders[i], &gv->grippers[0])) {
            if (print) cout << "Coll: " << i << "/gripper0" << endl;
            return true;
        }
        if (cd->checkForCollision(&gv->colliders[i], &gv->grippers[1])) {
            if (print) cout << "Coll: " << i << "/gripper1" << endl;
            return true;
        }
    }

    for (int i = 5; i <= 7; i++) {
        for (int j = 8; j < gv->collidersCount; j++) {
            if (j >= 12 && j <= 14) {
                if (abs(gv->currentDetectedObsPosX[j - 12]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[j - 12]) > cd->epsilon3) {
                    if (cd->checkForCollision(&gv->colliders[i], &gv->distortedObstacles[j - 12])) {
                        if (print) cout << "Coll: " << i << "/distorted" << j - 12 << endl;
                        return true;
                    }
                }
            }
            else {
                if (cd->checkForCollision(&gv->colliders[i], &gv->colliders[j])) {
                    if (print) cout << "Coll: " << i << "/" << j << endl;
                    return true;
                }
            }
        }
    }

    for (int j = 8; j < gv->collidersCount; j++) {
        if (j >= 12 && j <= 14) {
            if (abs(gv->currentDetectedObsPosX[j - 12]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[j - 12]) > cd->epsilon3) {
                if (cd->checkForCollision(&gv->grippers[0], &gv->distortedObstacles[j - 12])) {
                    if (print) cout << "Coll: gripper0/distorted" << j - 12 << endl;
                    return true;
                }
                if (cd->checkForCollision(&gv->grippers[1], &gv->distortedObstacles[j - 12])) {
                    if (print) cout << "Coll: gripper1/distorted" << j - 12 << endl;
                    return true;
                }
            }
        }
        else {
            if (cd->checkForCollision(&gv->grippers[0], &gv->colliders[j])) {
                if (print) cout << "Coll: gripper0/" << j << endl;
                return true;
            }
            if (cd->checkForCollision(&gv->grippers[1], &gv->colliders[j])) {
                if (print) cout << "Coll: gripper1/" << j << endl;
                return true;
            }
        }
    }

    if (cd->checkForCollisionWithGround(&gv->colliders[7], 0) || cd->checkForCollisionWithGround(&gv->grippers[0], 0) || cd->checkForCollisionWithGround(&gv->grippers[1], 0)) {
        if (print) cout << "Coll: ground (gripper or end)" << endl;
        return true;
    }

    return false;
}


float PathPlanner::getMinDistanceTo(collider* col, bool transporting, float currentMinDist) {
	GlobalVariables *gv=GlobalVariables::get_instance();
	CollisionDetection *cd = CollisionDetection::get_instance();

    tempMinDistance = currentMinDist;

    if (transporting) {
        if (cd->distApproximation(col, gv->safetyCollider) < tempMinDistance) {
            tempMinDistance2 = cd->gjk(col, gv->safetyCollider);
            if (tempMinDistance2 < tempMinDistance) tempMinDistance = tempMinDistance2;
        }
    }

    if (cd->distApproximation(col, &gv->grippers[0]) < tempMinDistance) {
        tempMinDistance2 = cd->gjk(col, &gv->grippers[0]);
        if (tempMinDistance2 < tempMinDistance) tempMinDistance = tempMinDistance2;
    }

    if (cd->distApproximation(col, &gv->grippers[1]) < tempMinDistance) {
        tempMinDistance2 = cd->gjk(col, &gv->grippers[1]);
        if (tempMinDistance2 < tempMinDistance) tempMinDistance = tempMinDistance2;
    }

    if (cd->distApproximation(col, &gv->colliders[7]) < tempMinDistance) {
        tempMinDistance2 = cd->gjk(col, &gv->colliders[7]);
        if (tempMinDistance2 < tempMinDistance) tempMinDistance = tempMinDistance2;
    }

    if (cd->distApproximation(col, &gv->colliders[6]) < tempMinDistance) {
        tempMinDistance2 = cd->gjk(col, &gv->colliders[6]);
        if (tempMinDistance2 < tempMinDistance) tempMinDistance = tempMinDistance2;
    }

    if (cd->distApproximation(col, &gv->colliders[5]) < tempMinDistance) {
        tempMinDistance2 = cd->gjk(col, &gv->colliders[5]);
        if (tempMinDistance2 < tempMinDistance) tempMinDistance = tempMinDistance2;
    }

    return tempMinDistance;
}


float PathPlanner::getMaxZVal(float x, float y, float z) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();

    maxZVal = z;
    if (gv->transportPhase == 1) {
        for (int i = 8; i < gv->collidersCount; i++) {
            if (i >= 12 && i <= 14) {
                if (abs(gv->currentDetectedObsPosX[i - 12]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[i - 12]) > cd->epsilon3) {
                    tempRadius = sqrt((x - gv->distortedObstacles[i - 12].offsets[0]) * (x - gv->distortedObstacles[i - 12].offsets[0]) + (y - gv->distortedObstacles[i - 12].offsets[1]) * (y - gv->distortedObstacles[i - 12].offsets[1]));
                    if (tempRadius <= gv->distortedObstacles[i - 12].maxSize) {
                        tempZVal = sqrt(gv->distortedObstacles[i - 12].maxSize * gv->distortedObstacles[i - 12].maxSize - (x - gv->distortedObstacles[i - 12].offsets[0]) * (x - gv->distortedObstacles[i - 12].offsets[0]) - (y - gv->distortedObstacles[i - 12].offsets[1]) * (y - gv->distortedObstacles[i - 12].offsets[1])) + gv->distortedObstacles[i - 12].offsets[2];
                        if (tempZVal > maxZVal) {
                            maxZVal = tempZVal;
                        }
                    }
                    else if (tempRadius <= 1.5 * gv->distortedObstacles[i - 12].maxSize) {
                        tempZVal = gv->distortedObstacles[i - 12].offsets[2] * (1.0f - (tempRadius - gv->distortedObstacles[i - 12].maxSize) / (0.5f * gv->distortedObstacles[i - 12].maxSize));
                        if (tempZVal > maxZVal) {
                            maxZVal = tempZVal;
                        }
                    }
                }
            }
            else if (i != gv->objectToTransport) {
                tempRadius = sqrt((x - gv->colliders[i].offsets[0]) * (x - gv->colliders[i].offsets[0]) + (y - gv->colliders[i].offsets[1]) * (y - gv->colliders[i].offsets[1]));
                if (tempRadius <= gv->colliders[i].maxSize) {
                    tempZVal = sqrt(gv->colliders[i].maxSize * gv->colliders[i].maxSize - (x - gv->colliders[i].offsets[0]) * (x - gv->colliders[i].offsets[0]) - (y - gv->colliders[i].offsets[1]) * (y - gv->colliders[i].offsets[1])) + gv->colliders[i].offsets[2];
                    if (tempZVal > maxZVal) {
                        maxZVal = tempZVal;
                    }
                }
                else if (tempRadius <= 1.5 * gv->colliders[i].maxSize) {
                    tempZVal = gv->colliders[i].offsets[2] * (1.0f - (tempRadius - gv->colliders[i].maxSize) / (0.5f * gv->colliders[i].maxSize));
                    if (tempZVal > maxZVal) {
                        maxZVal = tempZVal;
                    }
                }
            }
        }
    }
    else {
        for (int i = 8; i < gv->collidersCount; i++) {
            if (i >= 12 && i <= 14) {
                if (abs(gv->currentDetectedObsPosX[i - 12]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[i - 12]) > cd->epsilon3) {
                    tempRadius = sqrt((x - gv->distortedObstacles[i - 12].offsets[0]) * (x - gv->distortedObstacles[i - 12].offsets[0]) + (y - gv->distortedObstacles[i - 12].offsets[1]) * (y - gv->distortedObstacles[i - 12].offsets[1]));
                    if (tempRadius <= gv->distortedObstacles[i - 12].maxSize) {
                        tempZVal = sqrt(gv->distortedObstacles[i - 12].maxSize * gv->distortedObstacles[i - 12].maxSize - (x - gv->distortedObstacles[i - 12].offsets[0]) * (x - gv->distortedObstacles[i - 12].offsets[0]) - (y - gv->distortedObstacles[i - 12].offsets[1]) * (y - gv->distortedObstacles[i - 12].offsets[1])) + gv->distortedObstacles[i - 12].offsets[2];
                        if (tempZVal > maxZVal) {
                            maxZVal = tempZVal;
                        }
                    }
                    else if (tempRadius <= 1.5 * gv->distortedObstacles[i - 12].maxSize) {
                        tempZVal = gv->distortedObstacles[i - 12].offsets[2] * (1.0f - (tempRadius - gv->distortedObstacles[i - 12].maxSize) / (0.5f * gv->distortedObstacles[i - 12].maxSize));
                        if (tempZVal > maxZVal) {
                            maxZVal = tempZVal;
                        }
                    }
                }
            }
            else {
                tempRadius = sqrt((x - gv->colliders[i].offsets[0]) * (x - gv->colliders[i].offsets[0]) + (y - gv->colliders[i].offsets[1]) * (y - gv->colliders[i].offsets[1]));
                if (tempRadius <= gv->colliders[i].maxSize) {
                    tempZVal = sqrt(gv->colliders[i].maxSize * gv->colliders[i].maxSize - (x - gv->colliders[i].offsets[0]) * (x - gv->colliders[i].offsets[0]) - (y - gv->colliders[i].offsets[1]) * (y - gv->colliders[i].offsets[1])) + gv->colliders[i].offsets[2];
                    if (tempZVal > maxZVal) {
                        maxZVal = tempZVal;
                    }
                }
                else if (tempRadius <= 1.5 * gv->colliders[i].maxSize) {
                    tempZVal = gv->colliders[i].offsets[2] * (1.0f - (tempRadius - gv->colliders[i].maxSize) / (0.5f * gv->colliders[i].maxSize));
                    if (tempZVal > maxZVal) {
                        maxZVal = tempZVal;
                    }
                }
            }
        }
    }
    return maxZVal;
}

// initialize the Dijkstra algorithm. Has to be called once at the beginning:
void PathPlanner::initNodeList() {
    for (int i = 0; i < targetNodesCount; i++) {
        nodeList[i].x = 0;
        nodeList[i].y = 0;
        nodeList[i].z = 0;
        nodeList[i].neighbors = new int[neighborsCount];
        nodeList[i].fromOthers = new int[othersCount];
        nodeList[i].verifiedNeighs = new int[neighborsCount];
        nodeList[i].verifiedOthers = new int[othersCount];
        nodeList[i].otherCount = 0;
        nodeList[i].dist = 0;
        nodeList[i].previous = -1;
        nodeList[i].processed = false;
        nodeList[i].nodeQVal = new float[5];
    }
}

// resets the node list. Has to be called before every path calculation:
void PathPlanner::resetNodeList() {
    for (int i = 0; i < targetNodesCount; i++) {
        nodeList[i].dist = 10.0e10f;
        nodeList[i].previous = -1;
        nodeList[i].processed = false;
    }
}

// verifies the collision-freeness of a path:
bool PathPlanner::verifyPath() {
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();

    currentNode = 1;
    isPathValid = true;
    while (currentNode != 0) {
        nextNode = nodeList[currentNode].previous;

        inNeigh = false;
        inOthers = false;
        nodeIndex = -1;

        for (int i = 0; i < neighborsCount; i++) {
            if (nodeList[currentNode].neighbors[i] == nextNode) {
                inNeigh = true;
                nodeIndex = i;
                break;
            }
        }

        if (!inNeigh) {
            for (int i = 0; i < nodeList[currentNode].otherCount; i++) {
                if (nodeList[currentNode].fromOthers[i] == nextNode) {
                    inOthers = true;
                    nodeIndex = i;
                    break;
                }
            }
        }

        if (!inNeigh && !inOthers) {
            std::cout << "Error: not contained in next elements!" << std::endl;
        }

        if (!((inNeigh && nodeList[currentNode].verifiedNeighs[nodeIndex] >= 0) || (inOthers && nodeList[currentNode].verifiedOthers[nodeIndex] >= 0)))
        {
            qValStart = nodeList[currentNode].nodeQVal;
            qValEnd = nodeList[nextNode].nodeQVal;

            maxJointDiff = 0;
            for (int i = 0; i < 5; i++) {
                if (abs(qValEnd[i] - qValStart[i]) > maxJointDiff) {
                    maxJointDiff = abs(qValEnd[i] - qValStart[i]);
                }
            }
            sampleCount = (int)(ceil(maxJointDiff / samplingStepSize));
            // nodeDistance = sqrt((nodeList[currentNode].x - nodeList[nextNode].x) * (nodeList[currentNode].x - nodeList[nextNode].x) + (nodeList[currentNode].y - nodeList[nextNode].y) * (nodeList[currentNode].y - nodeList[nextNode].y) + (nodeList[currentNode].z - nodeList[nextNode].z) * (nodeList[currentNode].z - nodeList[nextNode].z));
            // sampleCount = (int) (ceil(nodeDistance / samplingStepSize));


            hitCount = 0;
            if (sampleCount > 0) {
                for (int i = 1; i <= sampleCount; i++) {
                    for (int j = 0; j < 5; j++) {
                        qValInter[j] = qValStart[j] + ((float)i) * (qValEnd[j] - qValStart[j]) / ((float)(sampleCount + 1));
                    }

                    gv->q0Sin_pp = sin(qValInter[0] / 180.0f * M_PI);
                    gv->q0Cos_pp = cos(qValInter[0] / 180.0f * M_PI);
                    gv->q1Sin_pp = sin(qValInter[1] / 180.0f * M_PI);
                    gv->q1Cos_pp = cos(qValInter[1] / 180.0f * M_PI);
                    gv->q12Sin_pp = sin((qValInter[1] + qValInter[2]) / 180.0f * M_PI);
                    gv->q12Cos_pp = cos((qValInter[1] + qValInter[2]) / 180.0f * M_PI);
                    gv->q123Sin_pp = sin((qValInter[1] + qValInter[2] + qValInter[3]) / 180.0f * M_PI);
                    gv->q123Cos_pp = cos((qValInter[1] + qValInter[2] + qValInter[3]) / 180.0f * M_PI);

                    cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-qValInter[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
                    cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin_pp) * gv->q0Sin_pp + 4.5f * gv->q0Cos_pp, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin_pp) * gv->q0Cos_pp - 4.5f * gv->q0Sin_pp, ik->len0 + ik->len2 / 2.0f * gv->q1Cos_pp, (-qValInter[0] + 90) / 180.0f * M_PI, (90 - qValInter[1]) / 180.0f * M_PI, 0);
                    cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 / 2.0f * gv->q12Sin_pp) * gv->q0Sin_pp - 4.155f * gv->q0Cos_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 / 2.0f * gv->q12Sin_pp) * gv->q0Cos_pp + 4.155f * gv->q0Sin_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 / 2.0f * gv->q12Cos_pp, (-qValInter[0] + 90) / 180.0f * M_PI, (90 - qValInter[1] - qValInter[2]) / 180 * M_PI, 0);
                    cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + ik->lastSegmentMid * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + ik->lastSegmentMid * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + ik->lastSegmentMid * gv->q123Cos_pp, (-qValInter[0] + 90) / 180.0f * M_PI, (90 - qValInter[1] - qValInter[2] - qValInter[3]) / 180 * M_PI, 0);
                    cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-qValInter[0] + 90) / 180.0f * M_PI, (90 - qValInter[1] - qValInter[2] - qValInter[3]) / 180 * M_PI, qValInter[4] / 180.0f * M_PI);

                    cd->updateMatricesTransport((-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, qValInter[4] / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                    cd->rotationMatrix(gv->q0Cos_pp * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
                    gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

                    cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, qValInter[4] / 180.0f * M_PI);
                    cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, qValInter[4] / 180.0f * M_PI);

                    if (gv->transportPhase == 1) {
                        cd->updateMatricesTransport((-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, (qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                        cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, (qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
                        cd->recalculateColliderTransport(gv->safetyCollider, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, (qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);

                        if ((currentNode == 1 && nextNode != 0) || (currentNode == 0 && nextNode != 1) || (nextNode == 0 && currentNode != 1) || (nextNode == 1 && currentNode != 0)) {
                            collWithEnv = collisionWithEnvTransport(true, false);
                        }
                        else {
                            collWithEnv = collisionWithEnvTransport(false, false);
                        }
                    }
                    else {
                        collWithEnv = collisionWithEnv(false);
                    }

                    if (collWithEnv) {
                        hitCount++;
                    }
                }
            }

            if (inNeigh) {
                nodeList[currentNode].verifiedNeighs[nodeIndex] = hitCount;
                /*
                if (collWithEnv) {
                    nodeList[currentNode].neighbors[nodeIndex] = -1;
                }
                */

                for (int i = 0; i < nodeList[nextNode].otherCount; i++) {
                    if (nodeList[nextNode].fromOthers[i] == currentNode) {
                        nodeList[nextNode].verifiedOthers[i] = hitCount;
                        /*
                        if (collWithEnv) {
                            nodeList[nextNode].fromOthers[i] = -1;
                        }
                        */
                    }
                }
            }
            else if (inOthers) {
                nodeList[currentNode].verifiedOthers[nodeIndex] = hitCount;
                /*
                if (collWithEnv) {
                    nodeList[currentNode].fromOthers[nodeIndex] = -1;
                }
                */

                for (int i = 0; i < neighborsCount; i++) {
                    if (nodeList[nextNode].neighbors[i] == currentNode) {
                        nodeList[nextNode].verifiedNeighs[i] = hitCount;
                        /*
                        if (collWithEnv) {
                            nodeList[nextNode].neighbors[i] = -1;
                        }
                        */
                    }
                }
            }

            if (hitCount > 0) {
                // std::cout << "Collision: " << currentNode << " " << nextNode << std::endl;
                isPathValid = false;

            }
        }
        else {
            // std::cout << iter << " Not neces." << std::endl;
        }
        currentNode = nextNode;
    }
    return isPathValid;
}

// optimizePath() determines which nodes can be removed from the determined node sequence that forms the path, without making the path unsafe as a result:
void PathPlanner::optimizePath() {
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();

    nodeIndexStart = 0;
    nodeIndexEnd = 1;

    while (true) {
        while (nodeIndexStart != nodeList[nodeIndexEnd].previous)
        {
            qValStart = nodeList[nodeIndexStart].nodeQVal;
            qValEnd = nodeList[nodeIndexEnd].nodeQVal;
            nodeDistance = sqrt((nodeList[nodeIndexStart].x - nodeList[nodeIndexEnd].x) * (nodeList[nodeIndexStart].x - nodeList[nodeIndexEnd].x) + (nodeList[nodeIndexStart].y - nodeList[nodeIndexEnd].y) * (nodeList[nodeIndexStart].y - nodeList[nodeIndexEnd].y) + (nodeList[nodeIndexStart].z - nodeList[nodeIndexEnd].z) * (nodeList[nodeIndexStart].z - nodeList[nodeIndexEnd].z));

            maxJointDiff = 0;
            for (int i = 0; i < 5; i++) {
                if (abs(qValEnd[i] - qValStart[i]) > maxJointDiff) {
                    maxJointDiff = abs(qValEnd[i] - qValStart[i]);
                }
            }
            sampleCount = (int)(ceil(maxJointDiff / samplingStepSize));
            // sampleCount = (int)(ceil(nodeDistance / samplingStepSize));

            hitCount = 0;
            if (sampleCount > 0) {
                for (int i = 1; i <= sampleCount; i++) {
                    collWithEnv = false;
                    for (int j = 0; j < 5; j++) {
                        qValInter[j] = qValStart[j] + ((float)i) * (qValEnd[j] - qValStart[j]) / ((float)(sampleCount + 1));
                    }

                    gv->q0Sin_pp = sinf(qValInter[0] / 180.0f * M_PI);
                    gv->q0Cos_pp = cosf(qValInter[0] / 180.0f * M_PI);
                    gv->q1Sin_pp = sinf(qValInter[1] / 180.0f * M_PI);
                    gv->q1Cos_pp = cosf(qValInter[1] / 180.0f * M_PI);
                    gv->q12Sin_pp = sinf((qValInter[1] + qValInter[2]) / 180.0f * M_PI);
                    gv->q12Cos_pp = cosf((qValInter[1] + qValInter[2]) / 180.0f * M_PI);
                    gv->q123Sin_pp = sinf((qValInter[1] + qValInter[2] + qValInter[3]) / 180.0f * M_PI);
                    gv->q123Cos_pp = cosf((qValInter[1] + qValInter[2] + qValInter[3]) / 180.0f * M_PI);

                    cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-qValInter[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
                    cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin_pp) * gv->q0Sin_pp + 4.5f * gv->q0Cos_pp, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin_pp) * gv->q0Cos_pp - 4.5f * gv->q0Sin_pp, ik->len0 + ik->len2 / 2.0f * gv->q1Cos_pp, (-qValInter[0] + 90) / 180.0f * M_PI, (90 - qValInter[1]) / 180.0f * M_PI, 0);
                    cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 / 2.0f * gv->q12Sin_pp) * gv->q0Sin_pp - 4.155f * gv->q0Cos_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 / 2.0f * gv->q12Sin_pp) * gv->q0Cos_pp + 4.155f * gv->q0Sin_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 / 2.0f * gv->q12Cos_pp, (-qValInter[0] + 90) / 180.0f * M_PI, (90 - qValInter[1] - qValInter[2]) / 180 * M_PI, 0);
                    cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + ik->lastSegmentMid * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + ik->lastSegmentMid * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + ik->lastSegmentMid * gv->q123Cos_pp, (-qValInter[0] + 90) / 180.0f * M_PI, (90 - qValInter[1] - qValInter[2] - qValInter[3]) / 180 * M_PI, 0);
                    cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-qValInter[0] + 90) / 180.0f * M_PI, (90 - qValInter[1] - qValInter[2] - qValInter[3]) / 180 * M_PI, qValInter[4] / 180.0f * M_PI);

                    cd->updateMatricesTransport((-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, qValInter[4] / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                    cd->rotationMatrix(gv->q0Cos_pp * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
                    gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

                    cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, qValInter[4] / 180.0f * M_PI);
                    cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, qValInter[4] / 180.0f * M_PI);

                    if (gv->transportPhase == 1) {
                        cd->updateMatricesTransport((-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, (qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                        cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, (qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
                        cd->recalculateColliderTransport(gv->safetyCollider, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValInter[1] - qValInter[2] - qValInter[3]) / 180.0f * M_PI, (qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);

                        if (	   (nodeIndexStart == 1 && nodeIndexEnd != 0)
                        		|| ((nodeIndexStart == 0 && nodeIndexEnd != 1) && (nodeIndexStart != 1 && nodeIndexEnd == 0))
								|| (nodeIndexStart != 0 && nodeIndexEnd == 1)) {
                        	collWithEnv = collisionWithEnvTransport(true, false);
                        }
                        else {
                        	collWithEnv = collisionWithEnvTransport(false, false);
                        }
                    }
                    else {
                        collWithEnv = collisionWithEnv(false);
                    }

                    if (collWithEnv) {
                        hitCount++;
                    }
                }
            }

            if (nodeDistance + 10000.0f * ((float)hitCount) <= nodeList[nodeIndexEnd].dist - nodeList[nodeIndexStart].dist) {
                nodeList[nodeIndexEnd].previous = nodeIndexStart;
                nodeList[nodeIndexEnd].dist = nodeDistance + 10000.0f * ((float)hitCount) + nodeList[nodeIndexStart].dist;
                break;
            }

            nodeIndexEnd = nodeList[nodeIndexEnd].previous;
        }

        if (nodeIndexEnd == 1) {
            break;
        }
        nodeIndexStart = nodeIndexEnd;
        nodeIndexEnd = 1;
    }
}

// calcPath() finds save nodes, builds up a save network with these nodes and performs the Dijkstra algorithm:
bool PathPlanner::calcPath(float* q_start, float* q_end, collider::Edge* startPoint,
		collider::Edge* endPoint, bool extended,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3) {
	InverseKinematic *ik = InverseKinematic::get_instance();
	CollisionDetection *cd = CollisionDetection::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();

	if (sqrt((endPoint->x - startPoint->x) * (endPoint->x - startPoint->x) + (endPoint->y - startPoint->y) * (endPoint->y - startPoint->y) + (endPoint->z - startPoint->z) * (endPoint->z - startPoint->z)) > cd->epsilon2)
    {
        nodesCount = 0;
        realCount = 0;

        startPointRadius = sqrt(startPoint->x * startPoint->x + startPoint->y * startPoint->y);
        startPointAngle = acos(startPoint->y / startPointRadius) / M_PI * 180.0f;
        if (startPoint->x < 0) {
            startPointAngle = -startPointAngle;
        }

        endPointRadius = sqrt(endPoint->x * endPoint->x + endPoint->y * endPoint->y);
        endPointAngle = acos(endPoint->y / endPointRadius) / M_PI * 180.0f;
        if (endPoint->x < 0) {
            endPointAngle = -endPointAngle;
        }

        angleLayer_nx = endPoint->x - startPoint->x;
        angleLayer_ny = endPoint->y - startPoint->y;
        angleLayer_nlen = sqrt(angleLayer_nx * angleLayer_nx + angleLayer_ny * angleLayer_ny);

        if (angleLayer_nlen < cd->epsilon3) {
            angleLayer_nz = endPoint->z - startPoint->z;
            angleLayer_nlen2 = sqrt(angleLayer_nx * angleLayer_nx + angleLayer_ny * angleLayer_ny + angleLayer_nz * angleLayer_nz);
            angleLayer_nx /= angleLayer_nlen2;
            angleLayer_ny /= angleLayer_nlen2;
            angleLayer_nz /= angleLayer_nlen2;
            angleLayer_d = angleLayer_nx * (startPoint->x + endPoint->x) / 2.0f + angleLayer_ny * (startPoint->y + endPoint->y) / 2.0f + angleLayer_nz * (startPoint->z + endPoint->z) / 2.0f;
        }
        else {
            angleLayer_nx /= angleLayer_nlen;
            angleLayer_ny /= angleLayer_nlen;
            angleLayer_d = angleLayer_nx * (startPoint->x + endPoint->x) / 2.0f + angleLayer_ny * (startPoint->y + endPoint->y) / 2.0f;
        }

        while (nodesCount < targetNodesCount) {
            if (realCount == 0) {
                xRand_pp = startPoint->x;
                yRand_pp = startPoint->y;
                zRand_pp = startPoint->z;
            }
            else if (realCount == 1) {
                xRand_pp = endPoint->x;
                yRand_pp = endPoint->y;
                zRand_pp = endPoint->z;
            }
            else if (nodesCount < targetNodesCount / 2 + 25) {
                if (nodesCount < 25) {
                    xRand_pp = startPoint->x + distribution3(generator3);
                    yRand_pp = startPoint->y + distribution3(generator3);
                    zRand_pp = ((float)rand() / RAND_MAX) * 25.0f;
                }
                else if (nodesCount < 50) {
                    xRand_pp = endPoint->x + distribution3(generator3);
                    yRand_pp = endPoint->y + distribution3(generator3);
                    zRand_pp = ((float)rand() / RAND_MAX) * 25.0f;
                }
                else {
                    currTVal = ((float)rand() / RAND_MAX);
                    currAngle = (startPointAngle + currTVal * (endPointAngle - startPointAngle)) / 180.0f * M_PI;
                    currRadius = startPointRadius + currTVal * (endPointRadius - startPointRadius);
                    currHeight = startPoint->z + currTVal * (endPoint->z - startPoint->z);
                    currFactor = 0.15f + (360 - abs(endPointAngle - startPointAngle)) / 360 * 0.3;

                    xRand_pp = (1 - currFactor) * currRadius * sin(currAngle) + currFactor * (startPoint->x + currTVal * (endPoint->x + startPoint->x));
                    yRand_pp = (1 - currFactor) * currRadius * cos(currAngle) + currFactor * (startPoint->y + currTVal * (endPoint->y + startPoint->y));
                    zRand_pp = getMaxZVal(xRand_pp, yRand_pp, currHeight);

                    xRand_pp += distribution2(generator2);
                    yRand_pp += distribution2(generator2);
                    zRand_pp += distribution2(generator2);
                }
            }
            else {
                if (!extended) {
                    xRand_pp = ((float)rand() / RAND_MAX) * 90.0f - 45.0f;
                    yRand_pp = ((float)rand() / RAND_MAX) * 57.5f - 12.5f;
                    zRand_pp = ((float)rand() / RAND_MAX) * 25.0f;
                }
                else {
                    if (((float)rand() / RAND_MAX) < 0.0558f) {
                        xRand_pp = ((float)rand() / RAND_MAX) * 17.0f - 8.5f + 21.0f;
                        yRand_pp = ((float)rand() / RAND_MAX) * 17.0f - 8.5f - 21.0f;
                        zRand_pp = ((float)rand() / RAND_MAX) * 25.0f;
                    }
                    else {
                        xRand_pp = ((float)rand() / RAND_MAX) * 90.0f - 45.0f;
                        yRand_pp = ((float)rand() / RAND_MAX) * 57.5f - 12.5f;
                        zRand_pp = ((float)rand() / RAND_MAX) * 25.0f;
                    }
                }
            }


            if ((xRand_pp >= -45.0f && xRand_pp <= 45.0f && yRand_pp >= -12.5f && yRand_pp <= 45.0f && zRand_pp >= 0.0f && zRand_pp <= 25.0f) || (extended && xRand_pp >= 12.5f && xRand_pp <= 29.5f && yRand_pp >= -29.5f && yRand_pp <= -12.5f && zRand_pp >= 0.0f && zRand_pp <= 25.0f)) {
                if (ik->doesThetaExist(xRand_pp, yRand_pp, zRand_pp)) {
                    if (realCount == 0) {
                        for (int i = 0; i < 5; i++) qValRand[i] = q_start[i];
                    }
                    else if (realCount == 1) {
                        for (int i = 0; i < 5; i++) qValRand[i] = q_end[i];
                    }
                    else {
                        ik->writeQValues(qValRand);

                        if (angleLayer_nlen < cd->epsilon3) {
                            angleLayer_factor = ((angleLayer_nx * xRand_pp + angleLayer_ny * yRand_pp + angleLayer_nz * zRand_pp) - angleLayer_d + angleLayer_nlen2 / 2.0f) / angleLayer_nlen2;
                            if (angleLayer_factor > 1.0f) {
                                angleLayer_factor = 1.0f;
                            }
                            if (angleLayer_factor < 0.0f) {
                                angleLayer_factor = 0.0f;
                            }
                        }
                        else {
                            angleLayer_factor = ((angleLayer_nx * xRand_pp + angleLayer_ny * yRand_pp) - angleLayer_d + angleLayer_nlen / 2.0f) / angleLayer_nlen;
                            if (angleLayer_factor > 1.0f) {
                                angleLayer_factor = 1.0f;
                            }
                            if (angleLayer_factor < 0.0f) {
                                angleLayer_factor = 0.0f;
                            }
                        }

                        qValRand[4] = (q_end[4] - q_start[4]) * angleLayer_factor + q_start[4];
                    }

                    gv->q0Sin_pp = sinf(qValRand[0] / 180.0f * M_PI);
                    gv->q0Cos_pp = cosf(qValRand[0] / 180.0f * M_PI);
                    gv->q1Sin_pp = sinf(qValRand[1] / 180.0f * M_PI);
                    gv->q1Cos_pp = cosf(qValRand[1] / 180.0f * M_PI);
                    gv->q12Sin_pp = sinf((qValRand[1] + qValRand[2]) / 180.0f * M_PI);
                    gv->q12Cos_pp = cosf((qValRand[1] + qValRand[2]) / 180.0f * M_PI);
                    gv->q123Sin_pp = sinf((qValRand[1] + qValRand[2] + qValRand[3]) / 180.0f * M_PI);
                    gv->q123Cos_pp = cosf((qValRand[1] + qValRand[2] + qValRand[3]) / 180.0f * M_PI);

                    collWithEnv = false;

                    cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-qValRand[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
                    cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin_pp) * gv->q0Sin_pp + 4.5f * gv->q0Cos_pp, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin_pp) * gv->q0Cos_pp - 4.5f * gv->q0Sin_pp, ik->len0 + ik->len2 / 2.0f * gv->q1Cos_pp, (-qValRand[0] + 90) / 180.0f * M_PI, (90 - qValRand[1]) / 180.0f * M_PI, 0);
                    cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 / 2.0f * gv->q12Sin_pp) * gv->q0Sin_pp - 4.155f * gv->q0Cos_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 / 2.0f * gv->q12Sin_pp) * gv->q0Cos_pp + 4.155f * gv->q0Sin_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 / 2.0f * gv->q12Cos_pp, (-qValRand[0] + 90) / 180.0f * M_PI, (90 - qValRand[1] - qValRand[2]) / 180 * M_PI, 0);
                    cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + ik->lastSegmentMid * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + ik->lastSegmentMid * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + ik->lastSegmentMid * gv->q123Cos_pp, (-qValRand[0] + 90) / 180.0f * M_PI, (90 - qValRand[1] - qValRand[2] - qValRand[3]) / 180 * M_PI, 0);
                    cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-qValRand[0] + 90) / 180.0f * M_PI, (90 - qValRand[1] - qValRand[2] - qValRand[3]) / 180 * M_PI, qValRand[4] / 180.0f * M_PI);

                    cd->updateMatricesTransport((-qValRand[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValRand[1] - qValRand[2] - qValRand[3]) / 180.0f * M_PI, qValRand[4] / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                    cd->rotationMatrix(gv->q0Cos_pp * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
                    gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

                    cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-qValRand[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValRand[1] - qValRand[2] - qValRand[3]) / 180.0f * M_PI, qValRand[4] / 180.0f * M_PI);
                    cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-qValRand[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValRand[1] - qValRand[2] - qValRand[3]) / 180.0f * M_PI, qValRand[4] / 180.0f * M_PI);

                    if (gv->transportPhase == 1) {
                        cd->updateMatricesTransport((-qValRand[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValRand[1] - qValRand[2] - qValRand[3]) / 180.0f * M_PI, (qValRand[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                        cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-qValRand[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValRand[1] - qValRand[2] - qValRand[3]) / 180.0f * M_PI, (qValRand[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
                        cd->recalculateColliderTransport(gv->safetyCollider, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-qValRand[0] + 90.0f) / 180.0f * M_PI, (180.0f - qValRand[1] - qValRand[2] - qValRand[3]) / 180.0f * M_PI, (qValRand[4] + gv->grippingAngleDiff) / 180.0f * M_PI);

                        if (realCount == 0 || realCount == 1) {
                            collWithEnv = collisionWithEnvTransport(true, false);
                        }
                        else {
                            collWithEnv = collisionWithEnvTransport(false, false);
                        }
                    }
                    else {
                        collWithEnv = collisionWithEnv(false);
                    }

                    if (!collWithEnv) {
                        nodeList[nodesCount].x = xRand_pp;
                        nodeList[nodesCount].y = yRand_pp;
                        nodeList[nodesCount].z = zRand_pp;
                        for (int k = 0; k < neighborsCount; k++) {
                            nodeList[nodesCount].neighbors[k] = -1;
                            nodeList[nodesCount].verifiedNeighs[k] = -1;
                        }
                        for (int k = 0; k < othersCount; k++) {
                            nodeList[nodesCount].fromOthers[k] = -1;
                            nodeList[nodesCount].verifiedOthers[k] = -1;
                        }
                        nodeList[nodesCount].otherCount = 0;
                        nodeList[nodesCount].dist = 10.0e10f;
                        nodeList[nodesCount].previous = -1;
                        nodeList[nodesCount].processed = false;
                        for (int k = 0; k < 5; k++) {
                            nodeList[nodesCount].nodeQVal[k] = qValRand[k];
                        }
                        nodesCount++;
                    }
                    else if (realCount == 0 || realCount == 1) {
                        // std::cout << "Error: invalid start or end" << std::endl;
                        return false;
                    }
                }
                else if (realCount == 0 || realCount == 1) {
                    // std::cout << "Error: invalid start or end" << std::endl;
                    return false;
                }
            }
            else if (realCount == 0 || realCount == 1) {
                // std::cout << "Error: invalid start or end" << std::endl;
                return false;
            }
            realCount++;
        }

        for (int i = 0; i < targetNodesCount; i++) {
            for (int k = 0; k < neighborsCount; k++) {
                distanceList[k] = 10000.0f;
            }

            for (int j = 0; j < targetNodesCount; j++) {
                if (i != j) {

                    nodeDistance = sqrt((nodeList[i].x - nodeList[j].x) * (nodeList[i].x - nodeList[j].x) + (nodeList[i].y - nodeList[j].y) * (nodeList[i].y - nodeList[j].y) + (nodeList[i].z - nodeList[j].z) * (nodeList[i].z - nodeList[j].z));
                    nodeIndex = j;

                    if (nodeDistance < distanceList[neighborsCount - 1]) {
                        inOthers = false;
                        for (int k = 0; k < nodeList[i].otherCount; k++) {
                            if (nodeList[i].fromOthers[k] == j) {
                                inOthers = true;
                                break;
                            }
                        }

                        if (!inOthers) {
                            for (int k = 0; k < neighborsCount; k++) {
                                if (nodeDistance < distanceList[k]) {
                                    nodeDistanceTemp = distanceList[k];
                                    distanceList[k] = nodeDistance;
                                    nodeDistance = nodeDistanceTemp;
                                    nodeIndexTemp = nodeList[i].neighbors[k];
                                    nodeList[i].neighbors[k] = nodeIndex;
                                    nodeIndex = nodeIndexTemp;
                                }
                            }
                        }
                    }
                }
            }

            for (int k = 0; k < neighborsCount; k++) {
                neighborNode = nodeList[i].neighbors[k];
                if (neighborNode > i) {
                    if (nodeList[neighborNode].otherCount < othersCount) {
                        nodeList[neighborNode].fromOthers[nodeList[neighborNode].otherCount] = i;
                        nodeList[neighborNode].otherCount++;
                    }
                }
                else {
                    inNeigh = false;
                    for (int j = 0; j < neighborsCount; j++) {
                        if (nodeList[neighborNode].neighbors[k] == i) {
                            inNeigh = true;
                            break;
                        }
                    }

                    if (!inNeigh) {
                        if (nodeList[neighborNode].otherCount < othersCount) {
                            nodeList[neighborNode].fromOthers[nodeList[neighborNode].otherCount] = i;
                            nodeList[neighborNode].otherCount++;
                        }
                        else {
                            std::cout << "Warning: others array overflow!" << std::endl;
                        }
                    }
                }
            }
        }

        redoDijkstra = true;
        isFirstIteration = true;
        while (redoDijkstra) {
            nodeList[0].dist = 0.0f;
            qSize = targetNodesCount;
            while (qSize > 0) {
                minValue_pp = 10.0e9;
                minValueIndex_pp = -1;
                for (int i = 0; i < targetNodesCount; i++) {
                    if (nodeList[i].processed == false) {
                        if (nodeList[i].dist <= minValue_pp) {
                            minValue_pp = nodeList[i].dist;
                            minValueIndex_pp = i;
                        }
                    }
                }

                if (minValueIndex_pp == -1) {
                    // std::cout << "Warning: loose graph!" << std::endl;
                    break;
                }

                nodeList[minValueIndex_pp].processed = true;
                qSize--;

                for (int k = 0; k < neighborsCount; k++) {
                    if (nodeList[minValueIndex_pp].neighbors[k] != -1) {
                        if (nodeList[nodeList[minValueIndex_pp].neighbors[k]].processed == false) {
                            alt_pp = minValue_pp + sqrt((nodeList[minValueIndex_pp].x - nodeList[nodeList[minValueIndex_pp].neighbors[k]].x) * (nodeList[minValueIndex_pp].x - nodeList[nodeList[minValueIndex_pp].neighbors[k]].x) + (nodeList[minValueIndex_pp].y - nodeList[nodeList[minValueIndex_pp].neighbors[k]].y) * (nodeList[minValueIndex_pp].y - nodeList[nodeList[minValueIndex_pp].neighbors[k]].y) + (nodeList[minValueIndex_pp].z - nodeList[nodeList[minValueIndex_pp].neighbors[k]].z) * (nodeList[minValueIndex_pp].z - nodeList[nodeList[minValueIndex_pp].neighbors[k]].z));

                            alt_pp += 0.3f * max(abs(nodeList[minValueIndex_pp].nodeQVal[0] - nodeList[nodeList[minValueIndex_pp].neighbors[k]].nodeQVal[0]),
                                max(abs(nodeList[minValueIndex_pp].nodeQVal[1] - nodeList[nodeList[minValueIndex_pp].neighbors[k]].nodeQVal[1]),
                                    max(abs(nodeList[minValueIndex_pp].nodeQVal[2] - nodeList[nodeList[minValueIndex_pp].neighbors[k]].nodeQVal[2]),
                                        max(abs(nodeList[minValueIndex_pp].nodeQVal[3] - nodeList[nodeList[minValueIndex_pp].neighbors[k]].nodeQVal[3]),
                                            abs(nodeList[minValueIndex_pp].nodeQVal[4] - nodeList[nodeList[minValueIndex_pp].neighbors[k]].nodeQVal[4])))));

                            if (nodeList[minValueIndex_pp].verifiedNeighs[k] >= 0) {
                                alt_pp += ((float)nodeList[minValueIndex_pp].verifiedNeighs[k]) * 10000.0f;
                            }

                            if (alt_pp < nodeList[nodeList[minValueIndex_pp].neighbors[k]].dist) {
                                nodeList[nodeList[minValueIndex_pp].neighbors[k]].dist = alt_pp;
                                nodeList[nodeList[minValueIndex_pp].neighbors[k]].previous = minValueIndex_pp;
                            }
                        }
                    }
                }

                for (int k = 0; k < nodeList[minValueIndex_pp].otherCount; k++) {
                    if (nodeList[minValueIndex_pp].fromOthers[k] != -1) {
                        if (nodeList[nodeList[minValueIndex_pp].fromOthers[k]].processed == false) {
                            alt_pp = minValue_pp + sqrt((nodeList[minValueIndex_pp].x - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].x) * (nodeList[minValueIndex_pp].x - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].x) + (nodeList[minValueIndex_pp].y - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].y) * (nodeList[minValueIndex_pp].y - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].y) + (nodeList[minValueIndex_pp].z - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].z) * (nodeList[minValueIndex_pp].z - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].z));

                            alt_pp += 0.3f * max(abs(nodeList[minValueIndex_pp].nodeQVal[0] - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].nodeQVal[0]),
                                max(abs(nodeList[minValueIndex_pp].nodeQVal[1] - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].nodeQVal[1]),
                                    max(abs(nodeList[minValueIndex_pp].nodeQVal[2] - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].nodeQVal[2]),
                                        max(abs(nodeList[minValueIndex_pp].nodeQVal[3] - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].nodeQVal[3]),
                                            abs(nodeList[minValueIndex_pp].nodeQVal[4] - nodeList[nodeList[minValueIndex_pp].fromOthers[k]].nodeQVal[4])))));

                            if (nodeList[minValueIndex_pp].verifiedOthers[k] >= 0) {
                                alt_pp += ((float)nodeList[minValueIndex_pp].verifiedOthers[k]) * 10000.0f;
                            }

                            if (alt_pp < nodeList[nodeList[minValueIndex_pp].fromOthers[k]].dist) {
                                nodeList[nodeList[minValueIndex_pp].fromOthers[k]].dist = alt_pp;
                                nodeList[nodeList[minValueIndex_pp].fromOthers[k]].previous = minValueIndex_pp;
                            }
                        }
                    }
                }
            }

            if (isFirstIteration) {
                nodeIndex = 1;
                while (nodeIndex != 0 && nodeIndex != -1) {
                    nodeIndex = nodeList[nodeIndex].previous;
                }

                if (nodeIndex == -1) {
                    std::cout << "Info: no path found" << std::endl;
                    return false;
                }

                isFirstIteration = false;
            }

            if (verifyPath()) {
                redoDijkstra = false;
            }
            else {
                resetNodeList();
            }
        }

        if (nodeList[1].dist >= 10000.0f) {
            cout << "Info: no collision-free planning possible" << endl;
            return false;
        }

        return true;
    }
    else {
        return false;
    }
}
