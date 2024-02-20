/*
 * FrequentOps.cpp
 *
 *  Created on: 05.12.2022
 *      Author: manuel
 */

#include "FrequentOps.h"

FrequentOps::FrequentOps() {
//	HelperFunctions *hf = HelperFunctions::get_instance();
//	GlobalVariables *gv = GlobalVariables::get_instance();
}

FrequentOps::~FrequentOps() {

}

/*
 * singelton pattern
 */
FrequentOps* FrequentOps::fo = 0;
FrequentOps* FrequentOps::get_instance(){
	static bool isInit=false;
	if(!isInit){
		fo  = new FrequentOps();
		isInit=true;
	}
	return fo;
}


// draw the planned path in the 3d environment:
void FrequentOps::drawPathAndCalculation() {
	InverseKinematic *ik = InverseKinematic::get_instance();
    PathPlanner *pp = PathPlanner::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();

	gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
    gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
    gv->numPointsPlannedPath = 0;

    gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
    gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
    gv->numPointsRealPath = 0;

    pp->nodeIndex = 1;
    while (pp->nodeIndex != 0) {
        pp->nodeIndexTemp = pp->nodeList[pp->nodeIndex].previous;
        gv->planned_path_edges = ik->increaseSize(gv->planned_path_edges, gv->numPointsPlannedPath, 2);
        gv->planned_path_vertices = ik->increaseSize(gv->planned_path_vertices, gv->numPointsPlannedPath, 2);

        gv->planned_path_edges[gv->numPointsPlannedPath].x = pp->nodeList[pp->nodeIndex].x;
        gv->planned_path_edges[gv->numPointsPlannedPath].y = pp->nodeList[pp->nodeIndex].y;
        gv->planned_path_edges[gv->numPointsPlannedPath].z = pp->nodeList[pp->nodeIndex].z;
        gv->planned_path_vertices[gv->numPointsPlannedPath] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->pathColors[2].x, gv->pathColors[2].y, gv->pathColors[2].z, 1.0f };

        gv->numPointsPlannedPath++;

        gv->planned_path_edges[gv->numPointsPlannedPath].x = pp->nodeList[pp->nodeIndexTemp].x;
        gv->planned_path_edges[gv->numPointsPlannedPath].y = pp->nodeList[pp->nodeIndexTemp].y;
        gv->planned_path_edges[gv->numPointsPlannedPath].z = pp->nodeList[pp->nodeIndexTemp].z;
        gv->planned_path_vertices[gv->numPointsPlannedPath] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->pathColors[2].x, gv->pathColors[2].y, gv->pathColors[2].z, 1.0f };

        gv->numPointsPlannedPath++;

        pp->nodeIndex = pp->nodeIndexTemp;
    }

    for (int i = 0; i < 5; i++) {
        gv->qValuesCurrent[i] = pp->nodeList[1].nodeQVal[i];
    }

    pp->nodeIndex = pp->nodeList[1].previous;

    for (int i = 0; i < 5; i++) {
        gv->qValuesTarget[i] = pp->nodeList[pp->nodeIndex].nodeQVal[i];
        gv->qValuesDiff[i] = gv->qValuesTarget[i] - gv->qValuesCurrent[i];
    }

    gv->q0Sin = sin(gv->qValuesCurrent[0] / 180.0f * M_PI);
    gv->q0Cos = cos(gv->qValuesCurrent[0] / 180.0f * M_PI);
    gv->q1Sin = sin(gv->qValuesCurrent[1] / 180.0f * M_PI);
    gv->q1Cos = cos(gv->qValuesCurrent[1] / 180.0f * M_PI);
    gv->q12Sin = sin((gv->qValuesCurrent[1] + gv->qValuesCurrent[2]) / 180.0f * M_PI);
    gv->q12Cos = cos((gv->qValuesCurrent[1] + gv->qValuesCurrent[2]) / 180.0f * M_PI);
    gv->q123Sin = sin((gv->qValuesCurrent[1] + gv->qValuesCurrent[2] + gv->qValuesCurrent[3]) / 180.0f * M_PI);
    gv->q123Cos = cos((gv->qValuesCurrent[1] + gv->qValuesCurrent[2] + gv->qValuesCurrent[3]) / 180.0f * M_PI);

    gv->maxValue = -1.0f;
    for (int i = 0; i < 5; i++) {
        if (abs(gv->qValuesDiff[i]) > gv->maxValue) {
            gv->maxValue = abs(gv->qValuesDiff[i]);
            gv->maxValueIndex = i;
        }
    }

    if (gv->maxValue >= pp->simulationStepSize) {
        for (int i = 0; i < 5; i++) {
            gv->qValuesDiff[i] /= ceil(gv->maxValue /  pp->simulationStepSize);
        }
    }
}

// calculate a usable gripping value (means the last arm joint angle value):
void FrequentOps::gripAngleCalculation(int gripDirections)
{
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();


    if (gv->transportableObjectsGrip[gripDirections] == 0)
    {
        lastJointAngleStartTemps[0] = gv->qValuesObjectStart[4];
        lastJointAngleStartTemps[1] = gv->qValuesObjectStart[4] - 90.0f;
        lastJointAngleStartTemps[2] = gv->qValuesObjectStart[4] - 180.0f;
        lastJointAngleStartTemps[3] = gv->qValuesObjectStart[4] - 270.0f;

        lastJointAngleEndTemps[0] = gv->qValuesObjectEnd[4];
        lastJointAngleEndTemps[1] = gv->qValuesObjectEnd[4] - 90.0f;
        lastJointAngleEndTemps[2] = gv->qValuesObjectEnd[4] - 180.0f;
        lastJointAngleEndTemps[3] = gv->qValuesObjectEnd[4] - 270.0f;

        lastJointAngleMax = 100000.0f;
        for (int j = 0; j < 4; j++) {
            while (lastJointAngleStartTemps[j] > 180.0f) {
                lastJointAngleStartTemps[j] -= 360.0f;
            }
            while (lastJointAngleStartTemps[j] <= -180.0f) {
                lastJointAngleStartTemps[j] += 360.0f;
            }

            while (lastJointAngleEndTemps[j] > 180.0f) {
                lastJointAngleEndTemps[j] -= 360.0f;
            }
            while (lastJointAngleEndTemps[j] <= -180.0f) {
                lastJointAngleEndTemps[j] += 360.0f;
            }

            if (abs(lastJointAngleStartTemps[j]) + abs(lastJointAngleEndTemps[j]) < lastJointAngleMax) {
                lastJointAngleMax = abs(lastJointAngleStartTemps[j]) + abs(lastJointAngleEndTemps[j]);
                gv->qValuesObjectStart[4] = lastJointAngleStartTemps[j];
                gv->qValuesObjectEnd[4] = lastJointAngleEndTemps[j];

                gv->grippingAngleDiff = 90.0f * j;
                if (j % 2 == 0) {
                    gv->grippingWidthOpen = max(min(gv->objectDimensions[gripDirections].y * 2.0f + gv->grippingWidthAdditional, gv->grippingWidthMax), 0.0f);
                    gv->grippingWidthFixed = max(min(gv->objectDimensions[gripDirections].y * 2.0f, gv->grippingWidthMax), 0.0f);
                }
                else {
                    gv->grippingWidthOpen = max(min(gv->objectDimensions[gripDirections].x * 2.0f + gv->grippingWidthAdditional, gv->grippingWidthMax), 0.0f);
                    gv->grippingWidthFixed = max(min(gv->objectDimensions[gripDirections].x * 2.0f, gv->grippingWidthMax), 0.0f);
                }
            }
        }
    }
    else if (gv->transportableObjectsGrip[gripDirections] == 1)
    {
        lastJointAngleStartTemps[0] = gv->qValuesObjectStart[4];
        lastJointAngleStartTemps[1] = gv->qValuesObjectStart[4] - 180.0f;

        lastJointAngleEndTemps[0] = gv->qValuesObjectEnd[4];
        lastJointAngleEndTemps[1] = gv->qValuesObjectEnd[4] - 180.0f;

        gv->grippingWidthOpen = max(min(gv->objectDimensions[gripDirections].y * 2.0f + gv->grippingWidthAdditional, gv->grippingWidthMax), 0.0f);
        gv->grippingWidthFixed = max(min(gv->objectDimensions[gripDirections].y * 2.0f, gv->grippingWidthMax), 0.0f);
        lastJointAngleMax = 100000.0f;
        for (int j = 0; j < 2; j++) {
            while (lastJointAngleStartTemps[j] > 180.0f) {
                lastJointAngleStartTemps[j] -= 360.0f;
            }
            while (lastJointAngleStartTemps[j] <= -180.0f) {
                lastJointAngleStartTemps[j] += 360.0f;
            }

            while (lastJointAngleEndTemps[j] > 180.0f) {
                lastJointAngleEndTemps[j] -= 360.0f;
            }
            while (lastJointAngleEndTemps[j] <= -180.0f) {
                lastJointAngleEndTemps[j] += 360.0f;
            }

            if (abs(lastJointAngleStartTemps[j]) + abs(lastJointAngleEndTemps[j]) < lastJointAngleMax) {
                lastJointAngleMax = abs(lastJointAngleStartTemps[j]) + abs(lastJointAngleEndTemps[j]);
                gv->qValuesObjectStart[4] = lastJointAngleStartTemps[j];
                gv->qValuesObjectEnd[4] = lastJointAngleEndTemps[j];
                gv->grippingAngleDiff = 180.0f * j;
            }
        }
    }
    else {


        lastJointAngleStartTemps[0] = gv->qValuesObjectStart[4] - 90.0f;
        lastJointAngleStartTemps[1] = gv->qValuesObjectStart[4] - 270.0f;

        lastJointAngleEndTemps[0] = gv->qValuesObjectEnd[4] - 90.0f;
        lastJointAngleEndTemps[1] = gv->qValuesObjectEnd[4] - 270.0f;

        gv->grippingWidthOpen = max(min(gv->objectDimensions[gripDirections].x * 2.0f + gv->grippingWidthAdditional, gv->grippingWidthMax), 0.0f);
        gv->grippingWidthFixed = max(min(gv->objectDimensions[gripDirections].x * 2.0f, gv->grippingWidthMax), 0.0f);
        lastJointAngleMax = 100000.0f;
        for (int j = 0; j < 2; j++) {
            while (lastJointAngleStartTemps[j] > 180.0f) {
                lastJointAngleStartTemps[j] -= 360.0f;
            }
            while (lastJointAngleStartTemps[j] <= -180.0f) {
                lastJointAngleStartTemps[j] += 360.0f;
            }

            while (lastJointAngleEndTemps[j] > 180.0f) {
                lastJointAngleEndTemps[j] -= 360.0f;
            }
            while (lastJointAngleEndTemps[j] <= -180.0f) {
                lastJointAngleEndTemps[j] += 360.0f;
            }

            if (abs(lastJointAngleStartTemps[j]) + abs(lastJointAngleEndTemps[j]) < lastJointAngleMax) {
                lastJointAngleMax = abs(lastJointAngleStartTemps[j]) + abs(lastJointAngleEndTemps[j]);
                gv->qValuesObjectStart[4] = lastJointAngleStartTemps[j];
                gv->qValuesObjectEnd[4] = lastJointAngleEndTemps[j];
                gv->grippingAngleDiff = 180.0f * j + 90.0f;
            }
        }
    }
}

void FrequentOps::calcMetricValue() {
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();

    for (int i = 0; i < gv->totalObjectsCount + 2; i++) {
        minDistancesForMetric[i] = 10000.0f;
    }

    isPartialSegment = true;
    pp->nodeIndexStart = 1;
    while (pp->nodeList[pp->nodeIndexStart].previous != pp->nodeIndex) pp->nodeIndexStart = pp->nodeList[pp->nodeIndexStart].previous;

    while (pp->nodeIndexStart != 0) {
        pp->nodeIndexEnd = pp->nodeList[pp->nodeIndexStart].previous;

        pp->maxJointDiff = -1.0f;
        pp->maxJointIndex = -1;
        for (int i = 0; i < 5; i++) {
            if (abs(pp->nodeList[pp->nodeIndexEnd].nodeQVal[i] - pp->nodeList[pp->nodeIndexStart].nodeQVal[i]) > pp->maxJointDiff) {
                pp->maxJointDiff = abs(pp->nodeList[pp->nodeIndexEnd].nodeQVal[i] - pp->nodeList[pp->nodeIndexStart].nodeQVal[i]);
                pp->maxJointIndex = i;
            }
        }
        pp->sampleCount = (int)(ceil(pp->maxJointDiff /  pp->samplingStepSize));

        if (pp->sampleCount > 0) {
            if (isPartialSegment) {
                isPartialSegment = false;
                gv->pathInterpolationFactor = (gv->qValuesCurrent[pp->maxJointIndex] - pp->nodeList[pp->nodeIndexStart].nodeQVal[pp->maxJointIndex]) / (pp->nodeList[pp->nodeIndexEnd].nodeQVal[pp->maxJointIndex] - pp->nodeList[pp->nodeIndexStart].nodeQVal[pp->maxJointIndex]);
                if (gv->pathInterpolationFactor > 1.0f) gv->pathInterpolationFactor = 1.0f;
                if (gv->pathInterpolationFactor < 0.0f) gv->pathInterpolationFactor = 0.0f;
                interpolationStartPoint = int(floor((pp->sampleCount + 1) * gv->pathInterpolationFactor));
            }
            else {
                interpolationStartPoint = 1;
            }

            for (int i = interpolationStartPoint; i <= pp->sampleCount + 1; i++) {
                for (int j = 0; j < 5; j++) {
                    pp->qValInter[j] = pp->nodeList[pp->nodeIndexStart].nodeQVal[j] + (float(i)) * (pp->nodeList[pp->nodeIndexEnd].nodeQVal[j] - pp->nodeList[pp->nodeIndexStart].nodeQVal[j]) / (float(pp->sampleCount + 1));
                }

                gv->q0Sin_pp = sin(pp->qValInter[0] / 180.0f * M_PI);
                gv->q0Cos_pp = cos(pp->qValInter[0] / 180.0f * M_PI);
                gv->q1Sin_pp = sin(pp->qValInter[1] / 180.0f * M_PI);
                gv->q1Cos_pp = cos(pp->qValInter[1] / 180.0f * M_PI);

                gv->q12Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
                gv->q12Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
                gv->q123Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);
                gv->q123Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);

                cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-pp->qValInter[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
                cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin_pp) * gv->q0Sin_pp + 4.5f * gv->q0Cos_pp, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin_pp) * gv->q0Cos_pp - 4.5f * gv->q0Sin_pp, ik->len0 + ik->len2 / 2.0f * gv->q1Cos_pp, (-pp->qValInter[0] + 90) / 180.0f * M_PI, (90 - pp->qValInter[1]) / 180.0f * M_PI, 0);
                cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 / 2.0f * gv->q12Sin_pp) * gv->q0Sin_pp - 4.155f * gv->q0Cos_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 / 2.0f * gv->q12Sin_pp) * gv->q0Cos_pp + 4.155f * gv->q0Sin_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 / 2.0f * gv->q12Cos_pp, (-pp->qValInter[0] + 90) / 180.0f * M_PI, (90 - pp->qValInter[1] - pp->qValInter[2]) / 180 * M_PI, 0);
                cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + ik->lastSegmentMid * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + ik->lastSegmentMid * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + ik->lastSegmentMid * gv->q123Cos_pp, (-pp->qValInter[0] + 90) / 180.0f * M_PI, (90 - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180 * M_PI, 0);
                cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-pp->qValInter[0] + 90) / 180.0f * M_PI, (90 - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180 * M_PI, pp->qValInter[4] / 180.0f * M_PI);

                cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
//TODO: resolve problem: cannot convert from float to collider
                cd->rotationMatrix(gv->q0Cos_pp * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
                gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

                cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);
                cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

                if (gv->transportPhase == 1) {
                    cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                    cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
                    cd->recalculateColliderTransport(gv->safetyCollider, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);

                    for (int j = 8; j <= 16; j++) {
                        if (j >= 12 && j <= 14) {
                            if (abs(gv->currentDetectedObsPosX[j - 12]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[j - 12]) > cd->epsilon3) {
                                minDistancesForMetric[j - 8] = pp->getMinDistanceTo(&gv->distortedObstacles[j - 12], false, minDistancesForMetric[j - 8]);
                            }
                        }
                        else if (j != gv->objectToTransport) {
                            minDistancesForMetric[j - 8] = pp->getMinDistanceTo(&gv->colliders[j], false, minDistancesForMetric[j - 8]);
                        }
                    }

                    if (minDistancesForMetric[gv->objectToTransport - 8] > 0.0f) {
                        minDistancesForMetric[gv->objectToTransport - 8] = 0.0f;
                    }

                    minDistancesForMetric[9] = pp->getMinDistanceTo(&gv->colliders[1], false, minDistancesForMetric[9]);

                    pp->tempMinDistance = minDistancesForMetric[10];
                    if (max(0.0f, gv->safetyCollider->limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->safetyCollider->limits[5]);
                    }

                    if (max(0.0f, gv->grippers[0].limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->grippers[0].limits[5]);
                    }

                    if (max(0.0f, gv->grippers[1].limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->grippers[1].limits[5]);
                    }

                    if (max(0.0f, gv->colliders[7].limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->colliders[7].limits[5]);
                    }

                    if (max(0.0f, gv->colliders[6].limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->colliders[6].limits[5]);
                    }

                    if (max(0.0f, gv->colliders[5].limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->colliders[5].limits[5]);
                    }
                    minDistancesForMetric[10] = pp->tempMinDistance;
                }
                else {
                    for (int j = 8; j <= 16; j++) {
                        if (j >= 12 && j <= 14) {
                            if (abs(gv->currentDetectedObsPosX[j - 12]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[j - 12]) > cd->epsilon3) {
                                minDistancesForMetric[j - 8] = pp->getMinDistanceTo(&gv->distortedObstacles[j - 12], false, minDistancesForMetric[j - 8]);
                            }
                        }
                        else {
                            minDistancesForMetric[j - 8] = pp->getMinDistanceTo(&gv->colliders[j], false, minDistancesForMetric[j - 8]);
                        }
                    }

                    minDistancesForMetric[9] = pp->getMinDistanceTo(&gv->colliders[1], false, minDistancesForMetric[9]);

                    pp->tempMinDistance = minDistancesForMetric[10];
                    if (max(0.0f, gv->grippers[0].limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->grippers[0].limits[5]);
                    }

                    if (max(0.0f, gv->grippers[1].limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->grippers[1].limits[5]);
                    }

                    if (max(0.0f, gv->colliders[7].limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->colliders[7].limits[5]);
                    }

                    if (max(0.0f, gv->colliders[6].limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->colliders[6].limits[5]);
                    }

                    if (max(0.0f, gv->colliders[5].limits[5]) < pp->tempMinDistance) {
                        pp->tempMinDistance = max(0.0f, gv->colliders[5].limits[5]);
                    }
                    minDistancesForMetric[10] = pp->tempMinDistance;
                }
            }
        }
        pp->nodeIndexStart = pp->nodeIndexEnd;
    }
}

// verify the current planned path in a dynamic environment:
bool FrequentOps::checkIfPathIsCollisionFree() {
	CollisionDetection *cd=CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();

    isPartialSegment = true;
    pp->nodeIndexStart = 1;
    while (pp->nodeList[pp->nodeIndexStart].previous != pp->nodeIndex) pp->nodeIndexStart = pp->nodeList[pp->nodeIndexStart].previous;

    while (pp->nodeIndexStart != 0) {
        pp->nodeIndexEnd = pp->nodeList[pp->nodeIndexStart].previous;

        pp->maxJointDiff = -1.0f;
        pp->maxJointIndex = -1;
        for (int i = 0; i < 5; i++) {
            if (abs(pp->nodeList[pp->nodeIndexEnd].nodeQVal[i] - pp->nodeList[pp->nodeIndexStart].nodeQVal[i]) > pp->maxJointDiff) {
                pp->maxJointDiff = abs(pp->nodeList[pp->nodeIndexEnd].nodeQVal[i] - pp->nodeList[pp->nodeIndexStart].nodeQVal[i]);
                pp->maxJointIndex = i;
            }
        }
        pp->sampleCount = (int)(ceil(pp->maxJointDiff /  pp->samplingStepSize));

        if (pp->sampleCount > 0) {
            if (isPartialSegment) {
                isPartialSegment = false;
                gv->pathInterpolationFactor = (gv->qValuesCurrent[pp->maxJointIndex] - pp->nodeList[pp->nodeIndexStart].nodeQVal[pp->maxJointIndex]) / (pp->nodeList[pp->nodeIndexEnd].nodeQVal[pp->maxJointIndex] - pp->nodeList[pp->nodeIndexStart].nodeQVal[pp->maxJointIndex]);
                if (gv->pathInterpolationFactor > 1.0f) gv->pathInterpolationFactor = 1.0f;
                if (gv->pathInterpolationFactor < 0.0f) gv->pathInterpolationFactor = 0.0f;
                interpolationStartPoint = int(floor((pp->sampleCount + 1) * gv->pathInterpolationFactor));
            }
            else {
                interpolationStartPoint = 1;
            }

            for (int i = interpolationStartPoint; i <= pp->sampleCount + 1; i++) {
                for (int j = 0; j < 5; j++) {
                    pp->qValInter[j] = pp->nodeList[pp->nodeIndexStart].nodeQVal[j] + (float(i)) * (pp->nodeList[pp->nodeIndexEnd].nodeQVal[j] - pp->nodeList[pp->nodeIndexStart].nodeQVal[j]) / (float(pp->sampleCount + 1));
                }

                gv->q0Sin_pp = sin(pp->qValInter[0] / 180.0f * M_PI);
                gv->q0Cos_pp = cos(pp->qValInter[0] / 180.0f * M_PI);
                gv->q1Sin_pp = sin(pp->qValInter[1] / 180.0f * M_PI);
                gv->q1Cos_pp = cos(pp->qValInter[1] / 180.0f * M_PI);
                gv->q12Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
                gv->q12Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
                gv->q123Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);
                gv->q123Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);

                cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-pp->qValInter[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
                cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin_pp) * gv->q0Sin_pp + 4.5f * gv->q0Cos_pp, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin_pp) * gv->q0Cos_pp - 4.5f * gv->q0Sin_pp, ik->len0 + ik->len2 / 2.0f * gv->q1Cos_pp, (-pp->qValInter[0] + 90) / 180.0f * M_PI, (90 - pp->qValInter[1]) / 180.0f * M_PI, 0);
                cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 / 2.0f * gv->q12Sin_pp) * gv->q0Sin_pp - 4.155f * gv->q0Cos_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 / 2.0f * gv->q12Sin_pp) * gv->q0Cos_pp + 4.155f * gv->q0Sin_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 / 2.0f * gv->q12Cos_pp, (-pp->qValInter[0] + 90) / 180.0f * M_PI, (90 - pp->qValInter[1] - pp->qValInter[2]) / 180 * M_PI, 0);
                cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + ik->lastSegmentMid * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + ik->lastSegmentMid * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + ik->lastSegmentMid * gv->q123Cos_pp, (-pp->qValInter[0] + 90) / 180.0f * M_PI, (90 - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180 * M_PI, 0);
                cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-pp->qValInter[0] + 90) / 180.0f * M_PI, (90 - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180 * M_PI, pp->qValInter[4] / 180.0f * M_PI);

                cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI, gv->q123Sin_pp* gv->q0Sin_pp, gv->q123Sin_pp* gv->q0Cos_pp, gv->q123Cos_pp);
                //TODO: resolve problem: cannot convert from float to collider
                cd->rotationMatrix(gv->q0Cos_pp* (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
                gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

                cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);
                cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

                if (gv->transportPhase == 1) {
                    cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                    cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
                    cd->recalculateColliderTransport(gv->safetyCollider, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);

                    if ((pp->nodeIndexStart == 1 && pp->nodeIndexEnd != 0) || (pp->nodeIndexStart == 0 && pp->nodeIndexEnd != 1) || (pp->nodeIndexStart != 1 && pp->nodeIndexEnd == 0) || (pp->nodeIndexStart != 0 && pp->nodeIndexEnd == 1)) {
                        if (pp->collisionWithEnvTransport(true, false)) return false;
                    }
                    else {
                        if (pp->collisionWithEnvTransport(false, false)) return false;
                    }
                }
                else {
                    if (pp->collisionWithEnv(false)) return false;
                }
            }
        }
        pp->nodeIndexStart = pp->nodeIndexEnd;
    }

    return true;
}

void FrequentOps::getRandomObstaclePosition(
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3
) {
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	PathPlanner *pp=PathPlanner::get_instance();
    pp->currFactor = ((float)rand() / RAND_MAX) * (gv->secondSegLengthTotal - cd->epsilon4);
    int secondSegIndex = 0;
    while (secondSegIndex < gv->secondSegSize - 1) {
        if (gv->secondSegLengths[secondSegIndex] >= pp->currFactor) {
            break;
        }
        secondSegIndex++;
    }
    pp->currFactor = ((float)rand() / RAND_MAX);
    pp->qValRand[0] = (gv->q0ValsSecondSeg[secondSegIndex + 1] - gv->q0ValsSecondSeg[secondSegIndex]) * pp->currFactor + gv->q0ValsSecondSeg[secondSegIndex];
    pp->qValRand[1] = (gv->q1ValsSecondSeg[secondSegIndex + 1] - gv->q1ValsSecondSeg[secondSegIndex]) * pp->currFactor + gv->q1ValsSecondSeg[secondSegIndex];
    pp->qValRand[2] = (gv->q2ValsSecondSeg[secondSegIndex + 1] - gv->q2ValsSecondSeg[secondSegIndex]) * pp->currFactor + gv->q2ValsSecondSeg[secondSegIndex];
    pp->qValRand[3] = (gv->q3ValsSecondSeg[secondSegIndex + 1] - gv->q3ValsSecondSeg[secondSegIndex]) * pp->currFactor + gv->q3ValsSecondSeg[secondSegIndex];

    gv->q0Sin_pp = sin(pp->qValRand[0] / 180.0f * M_PI);
    gv->q0Cos_pp = cos(pp->qValRand[0] / 180.0f * M_PI);
    gv->q1Sin_pp = sin(pp->qValRand[1] / 180.0f * M_PI);
    gv->q12Sin_pp = sin((pp->qValRand[1] + pp->qValRand[2]) / 180.0f * M_PI);
    gv->q123Sin_pp = sin((pp->qValRand[1] + pp->qValRand[2] + pp->qValRand[3]) / 180.0f * M_PI);

    randomObstalcePosition[0] = (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp + distribution1(generator1);
    randomObstalcePosition[1] = (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp + distribution1(generator1);
}

void FrequentOps::drawPathOnly(int pathColorIndex) {
    InverseKinematic *ik = InverseKinematic::get_instance();
    PathPlanner *pp = PathPlanner::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();
    pp->nodeIndex = 1;

    while (pp->nodeIndex != 0) {
        pp->nodeIndexTemp = pp->nodeList[pp->nodeIndex].previous;
        gv->planned_path_edges = ik->increaseSize(gv->planned_path_edges, gv->numPointsPlannedPath, 2);
        gv->planned_path_vertices = ik->increaseSize(gv->planned_path_vertices, gv->numPointsPlannedPath, 2);

        gv->planned_path_edges[gv->numPointsPlannedPath].x = pp->nodeList[pp->nodeIndex].x;
        gv->planned_path_edges[gv->numPointsPlannedPath].y = pp->nodeList[pp->nodeIndex].y;
        gv->planned_path_edges[gv->numPointsPlannedPath].z = pp->nodeList[pp->nodeIndex].z;
        gv->planned_path_vertices[gv->numPointsPlannedPath] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->pathColors[pathColorIndex].x, gv->pathColors[pathColorIndex].y, gv->pathColors[pathColorIndex].z, 1.0f };

        gv->numPointsPlannedPath++;

        gv->planned_path_edges[gv->numPointsPlannedPath].x = pp->nodeList[pp->nodeIndexTemp].x;
        gv->planned_path_edges[gv->numPointsPlannedPath].y = pp->nodeList[pp->nodeIndexTemp].y;
        gv->planned_path_edges[gv->numPointsPlannedPath].z = pp->nodeList[pp->nodeIndexTemp].z;
        gv->planned_path_vertices[gv->numPointsPlannedPath] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->pathColors[pathColorIndex].x, gv->pathColors[pathColorIndex].y, gv->pathColors[pathColorIndex].z, 1.0f };

        gv->numPointsPlannedPath++;

        pp->nodeIndex = pp->nodeIndexTemp;
    }
}

// transform the data coming from Monitoring Tools into absolute object positions:
void FrequentOps::calcPosFromMTData(collider* col, float patternPosX, float patternPosY, float patternPosZ, float patternAngle, int patternIndex)
{
    CollisionDetection *cd = CollisionDetection::get_instance();
	relativeObjPositions[0] = cos(-patternAngle) * objectPatternOffsetsX[patternIndex] - sin(-patternAngle) * objectPatternOffsetsY[patternIndex] + patternPosX;
    relativeObjPositions[1] = sin(-patternAngle) * objectPatternOffsetsX[patternIndex] + cos(-patternAngle) * objectPatternOffsetsY[patternIndex] + patternPosY;

    distortedAbsolutePosition[0] = col->offsets[0] + (cd->camVectorsStatic[0] + relativeObjPositions[1]) * cos(col->angles[0]) + relativeObjPositions[0] * sin(col->angles[0]);
    distortedAbsolutePosition[1] = col->offsets[1] + (cd->camVectorsStatic[0] + relativeObjPositions[1]) * sin(col->angles[0]) - relativeObjPositions[0] * cos(col->angles[0]);

    distortedPatternCenter[0] = col->offsets[0] + (cd->camVectorsStatic[0] + patternPosY) * cos(col->angles[0]) + patternPosX * sin(col->angles[0]);
    distortedPatternCenter[1] = col->offsets[1] + (cd->camVectorsStatic[0] + patternPosY) * sin(col->angles[0]) - patternPosX * cos(col->angles[0]);
    distortedPatternCenter[2] = patternPosZ;

    relativeObjectDir[0] = cos(-patternAngle) * patternDirRotationX[patternIndex] - sin(-patternAngle) * patternDirRotationY[patternIndex];
    relativeObjectDir[1] = sin(-patternAngle) * patternDirRotationX[patternIndex] + cos(-patternAngle) * patternDirRotationY[patternIndex];

    absoluteObjectDir[0] = cos(col->angles[0] - M_PI / 2.0f) * relativeObjectDir[0] - sin(col->angles[0] - M_PI / 2.0f) * relativeObjectDir[1];
    absoluteObjectDir[1] = sin(col->angles[0] - M_PI / 2.0f) * relativeObjectDir[0] + cos(col->angles[0] - M_PI / 2.0f) * relativeObjectDir[1];
    distortedAbsolutePosition[2] = acos(absoluteObjectDir[0]);

    if (absoluteObjectDir[1] < 0.0f) {
        distortedAbsolutePosition[2] = 2 * M_PI - distortedAbsolutePosition[2];
    }
}

void FrequentOps::resetDetectedPatterns() {
	InverseKinematic *ik = InverseKinematic::get_instance();
    for (int i = 0; i < objectsWithPatternsCount; i++) {
        if (detectionCounts[i] > 0) {
            switch (i) {
            case 0:  firstWorkpieceDetections = ik->cutOff(firstWorkpieceDetections, detectionCounts[i], 0);
                break;
            case 1:  secondWorkpieceDetections = ik->cutOff(secondWorkpieceDetections, detectionCounts[i], 0);
                break;
            case 2:  thirdWorkpieceDetections = ik->cutOff(thirdWorkpieceDetections, detectionCounts[i], 0);
                break;
            case 3:  fourthWorkpieceDetections = ik->cutOff(fourthWorkpieceDetections, detectionCounts[i], 0);
                break;
            case 4:  firstObsDetections = ik->cutOff(firstObsDetections, detectionCounts[i], 0);
                break;
            case 5:  secondObsDetections = ik->cutOff(secondObsDetections, detectionCounts[i], 0);
                break;
            case 6:  thirdObsDetections = ik->cutOff(thirdObsDetections, detectionCounts[i], 0);
                break;
            }
            detectionCounts[i] = 0;
        }
    }
}

void FrequentOps::resetDetectedPatternsAlt() {
	InverseKinematic *ik = InverseKinematic::get_instance();
    for (int i = 0; i < objectsWithPatternsCount; i++) {
        if (detectionCounts2[i] > 0) {
            switch (i) {
            case 0:  firstWorkpieceDetections2 = ik->cutOff(firstWorkpieceDetections2, detectionCounts2[i], 0);
                break;
            case 1:  secondWorkpieceDetections2 = ik->cutOff(secondWorkpieceDetections2, detectionCounts2[i], 0);
                break;
            case 2:  thirdWorkpieceDetections2 = ik->cutOff(thirdWorkpieceDetections2, detectionCounts2[i], 0);
                break;
            case 3:  fourthWorkpieceDetections2 = ik->cutOff(fourthWorkpieceDetections2, detectionCounts2[i], 0);
                break;
            case 4:  firstObsDetections2 = ik->cutOff(firstObsDetections2, detectionCounts2[i], 0);
                break;
            case 5:  secondObsDetections2 = ik->cutOff(secondObsDetections2, detectionCounts2[i], 0);
                break;
            case 6:  thirdObsDetections2 = ik->cutOff(thirdObsDetections2, detectionCounts2[i], 0);
                break;
            }
            detectionCounts2[i] = 0;
        }
    }
}

void FrequentOps::insertDetectedPattern(int objectIndex) {
	InverseKinematic *ik = InverseKinematic::get_instance();
    switch (objectIndex) {
    case 0:  firstWorkpieceDetections = ik->increaseSize(firstWorkpieceDetections, detectionCounts[0], 1);
        firstWorkpieceDetections[detectionCounts[0]].x = distortedAbsolutePosition[0];
        firstWorkpieceDetections[detectionCounts[0]].y = distortedAbsolutePosition[1];
        firstWorkpieceDetections[detectionCounts[0]].z = distortedAbsolutePosition[2];
        detectionCounts[0]++;
        break;
    case 1:  secondWorkpieceDetections = ik->increaseSize(secondWorkpieceDetections, detectionCounts[1], 1);
        secondWorkpieceDetections[detectionCounts[1]].x = distortedAbsolutePosition[0];
        secondWorkpieceDetections[detectionCounts[1]].y = distortedAbsolutePosition[1];
        secondWorkpieceDetections[detectionCounts[1]].z = distortedAbsolutePosition[2];
        detectionCounts[1]++;
        break;
    case 2:  thirdWorkpieceDetections = ik->increaseSize(thirdWorkpieceDetections, detectionCounts[2], 1);
        thirdWorkpieceDetections[detectionCounts[2]].x = distortedAbsolutePosition[0];
        thirdWorkpieceDetections[detectionCounts[2]].y = distortedAbsolutePosition[1];
        thirdWorkpieceDetections[detectionCounts[2]].z = distortedAbsolutePosition[2];
        detectionCounts[2]++;
        break;
    case 3:  fourthWorkpieceDetections = ik->increaseSize(fourthWorkpieceDetections, detectionCounts[3], 1);
        fourthWorkpieceDetections[detectionCounts[3]].x = distortedAbsolutePosition[0];
        fourthWorkpieceDetections[detectionCounts[3]].y = distortedAbsolutePosition[1];
        fourthWorkpieceDetections[detectionCounts[3]].z = distortedAbsolutePosition[2];
        detectionCounts[3]++;
        break;
    case 4:  firstObsDetections = ik->increaseSize(firstObsDetections, detectionCounts[4], 1);
        firstObsDetections[detectionCounts[4]].x = distortedAbsolutePosition[0];
        firstObsDetections[detectionCounts[4]].y = distortedAbsolutePosition[1];
        firstObsDetections[detectionCounts[4]].z = distortedAbsolutePosition[2];
        detectionCounts[4]++;
        break;
    case 5:  secondObsDetections = ik->increaseSize(secondObsDetections, detectionCounts[5], 1);
        secondObsDetections[detectionCounts[5]].x = distortedAbsolutePosition[0];
        secondObsDetections[detectionCounts[5]].y = distortedAbsolutePosition[1];
        secondObsDetections[detectionCounts[5]].z = distortedAbsolutePosition[2];
        detectionCounts[5]++;
        break;
    case 6:  thirdObsDetections = ik->increaseSize(thirdObsDetections, detectionCounts[6], 1);
        thirdObsDetections[detectionCounts[6]].x = distortedAbsolutePosition[0];
        thirdObsDetections[detectionCounts[6]].y = distortedAbsolutePosition[1];
        thirdObsDetections[detectionCounts[6]].z = distortedAbsolutePosition[2];
        detectionCounts[6]++;
        break;
    }
}

void FrequentOps::insertDetectedPatternAlt(int objectIndex)
{
	InverseKinematic *ik = InverseKinematic::get_instance();
    switch (objectIndex) {
    case 0:  firstWorkpieceDetections2 = ik->increaseSize(firstWorkpieceDetections2, detectionCounts2[0], 1);
        firstWorkpieceDetections2[detectionCounts2[0]].x = distortedAbsolutePosition[0];
        firstWorkpieceDetections2[detectionCounts2[0]].y = distortedAbsolutePosition[1];
        firstWorkpieceDetections2[detectionCounts2[0]].z = distortedAbsolutePosition[2];
        detectionCounts2[0]++;
        break;
    case 1:  secondWorkpieceDetections2 = ik->increaseSize(secondWorkpieceDetections2, detectionCounts2[1], 1);
        secondWorkpieceDetections2[detectionCounts2[1]].x = distortedAbsolutePosition[0];
        secondWorkpieceDetections2[detectionCounts2[1]].y = distortedAbsolutePosition[1];
        secondWorkpieceDetections2[detectionCounts2[1]].z = distortedAbsolutePosition[2];
        detectionCounts2[1]++;
        break;
    case 2:  thirdWorkpieceDetections2 = ik->increaseSize(thirdWorkpieceDetections2, detectionCounts2[2], 1);
        thirdWorkpieceDetections2[detectionCounts2[2]].x = distortedAbsolutePosition[0];
        thirdWorkpieceDetections2[detectionCounts2[2]].y = distortedAbsolutePosition[1];
        thirdWorkpieceDetections2[detectionCounts2[2]].z = distortedAbsolutePosition[2];
        detectionCounts2[2]++;
        break;
    case 3:  fourthWorkpieceDetections2 = ik->increaseSize(fourthWorkpieceDetections2, detectionCounts2[3], 1);
        fourthWorkpieceDetections2[detectionCounts2[3]].x = distortedAbsolutePosition[0];
        fourthWorkpieceDetections2[detectionCounts2[3]].y = distortedAbsolutePosition[1];
        fourthWorkpieceDetections2[detectionCounts2[3]].z = distortedAbsolutePosition[2];
        detectionCounts2[3]++;
        break;
    case 4:  firstObsDetections2 = ik->increaseSize(firstObsDetections2, detectionCounts2[4], 1);
        firstObsDetections2[detectionCounts2[4]].x = distortedAbsolutePosition[0];
        firstObsDetections2[detectionCounts2[4]].y = distortedAbsolutePosition[1];
        firstObsDetections2[detectionCounts2[4]].z = distortedAbsolutePosition[2];
        detectionCounts2[4]++;
        break;
    case 5:  secondObsDetections2 = ik->increaseSize(secondObsDetections2, detectionCounts2[5], 1);
        secondObsDetections2[detectionCounts2[5]].x = distortedAbsolutePosition[0];
        secondObsDetections2[detectionCounts2[5]].y = distortedAbsolutePosition[1];
        secondObsDetections2[detectionCounts2[5]].z = distortedAbsolutePosition[2];
        detectionCounts2[5]++;
        break;
    case 6:  thirdObsDetections2 = ik->increaseSize(thirdObsDetections2, detectionCounts2[6], 1);
        thirdObsDetections2[detectionCounts2[6]].x = distortedAbsolutePosition[0];
        thirdObsDetections2[detectionCounts2[6]].y = distortedAbsolutePosition[1];
        thirdObsDetections2[detectionCounts2[6]].z = distortedAbsolutePosition[2];
        detectionCounts2[6]++;
        break;
    }
}

void FrequentOps::evaluateDetections(bool calcActionSpace) {
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();

	cout << "debug: FrequentOps. " << __LINE__ <<endl;

    if (calcActionSpace) {
    	cout << "debug: FrequentOps. This function is deprecated!!! " << __LINE__ <<endl;

        actionSpaceSamples = 0;
        actionSpaceDist = 0.0f;
        actionSpaceAng = 0.0f;

        if (detectionCounts[0] > 0) {

        	cout << "debug: FrequentOps " << __LINE__ <<endl;

            /*
            evaluatedDetections[0].x = 0.0f;
            evaluatedDetections[0].y = 0.0f;
            evaluatedDetections[0].z = 0.0f;
            targetDetectionAngle = gv->colliders[8].angles[0];

            for (int i = 0; i < detectionLengths[0]; i++) {
                evaluatedDetections[0].x += firstWorkpieceDetections[i].x;
                evaluatedDetections[0].y += firstWorkpieceDetections[i].y;

                if (abs(firstWorkpieceDetections[i].z - targetDetectionAngle) < abs(firstWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) && abs(firstWorkpieceDetections[i].z - targetDetectionAngle) < abs(firstWorkpieceDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[0].z += firstWorkpieceDetections[i].z;
                }
                else if (abs(firstWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(firstWorkpieceDetections[i].z - targetDetectionAngle) && abs(firstWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(firstWorkpieceDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[0].z += firstWorkpieceDetections[i].z + M_PI * 2.0f;
                }
                else {
                    evaluatedDetections[0].z += firstWorkpieceDetections[i].z - M_PI * 2.0f;
                }
            }
            evaluatedDetections[0].x /= float(detectionLengths[0]);
            evaluatedDetections[0].y /= float(detectionLengths[0]);
            evaluatedDetections[0].z /= float(detectionLengths[0]);
            */

            for (int i = 0; i < detectionCounts[0]; i++) {
                // cout << "A: " << sqrt((firstWorkpieceDetections[i].x - gv->colliders[8].offsets[0]) * (firstWorkpieceDetections[i].x - gv->colliders[8].offsets[0]) + (firstWorkpieceDetections[i].y - gv->colliders[8].offsets[1]) * (firstWorkpieceDetections[i].y - gv->colliders[8].offsets[1])) << endl;
                actionSpaceDist = (actionSpaceDist * float(actionSpaceSamples) + sqrt((firstWorkpieceDetections[i].x - gv->colliders[8].offsets[0]) * (firstWorkpieceDetections[i].x - gv->colliders[8].offsets[0]) + (firstWorkpieceDetections[i].y - gv->colliders[8].offsets[1]) * (firstWorkpieceDetections[i].y - gv->colliders[8].offsets[1]))) / float(actionSpaceSamples + 1);
                actionSpaceAng = (actionSpaceAng * float(actionSpaceSamples) + min(abs(firstWorkpieceDetections[i].z - gv->colliders[8].angles[0]), float(min(abs(firstWorkpieceDetections[i].z + 2 * M_PI - gv->colliders[8].angles[0]), abs(firstWorkpieceDetections[i].z - 2 * M_PI - gv->colliders[8].angles[0]))))) / float(actionSpaceSamples + 1);
                actionSpaceSamples++;
            }
        }

        if (detectionCounts[1] > 0) {
            /*
            evaluatedDetections[1].x = 0.0f;
            evaluatedDetections[1].y = 0.0f;
            evaluatedDetections[1].z = 0.0f;
            targetDetectionAngle = gv->colliders[9].angles[0];

            for (int i = 0; i < detectionLengths[1]; i++) {
                evaluatedDetections[1].x += secondWorkpieceDetections[i].x;
                evaluatedDetections[1].y += secondWorkpieceDetections[i].y;

                if (abs(secondWorkpieceDetections[i].z - targetDetectionAngle) < abs(secondWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) && abs(secondWorkpieceDetections[i].z - targetDetectionAngle) < abs(secondWorkpieceDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[1].z += secondWorkpieceDetections[i].z;
                }
                else if (abs(secondWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(secondWorkpieceDetections[i].z - targetDetectionAngle) && abs(secondWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(secondWorkpieceDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[1].z += secondWorkpieceDetections[i].z + M_PI * 2.0f;
                }
                else {
                    evaluatedDetections[1].z += secondWorkpieceDetections[i].z - M_PI * 2.0f;
                }
            }
            evaluatedDetections[1].x /= float(detectionLengths[1]);
            evaluatedDetections[1].y /= float(detectionLengths[1]);
            evaluatedDetections[1].z /= float(detectionLengths[1]);
            */

            for (int i = 0; i < detectionCounts[1]; i++) {
                // cout << "B: " << sqrt((secondWorkpieceDetections[i].x - gv->colliders[9].offsets[0]) * (secondWorkpieceDetections[i].x - gv->colliders[9].offsets[0]) + (secondWorkpieceDetections[i].y - gv->colliders[9].offsets[1]) * (secondWorkpieceDetections[i].y - gv->colliders[9].offsets[1])) << endl;
                actionSpaceDist = (actionSpaceDist * float(actionSpaceSamples) + sqrt((secondWorkpieceDetections[i].x - gv->colliders[9].offsets[0]) * (secondWorkpieceDetections[i].x - gv->colliders[9].offsets[0]) + (secondWorkpieceDetections[i].y - gv->colliders[9].offsets[1]) * (secondWorkpieceDetections[i].y - gv->colliders[9].offsets[1]))) / float(actionSpaceSamples + 1);
                actionSpaceAng = (actionSpaceAng * float(actionSpaceSamples) + min(abs(secondWorkpieceDetections[i].z - gv->colliders[9].angles[0]), float(min(abs(secondWorkpieceDetections[i].z + 2 * M_PI - gv->colliders[9].angles[0]), abs(secondWorkpieceDetections[i].z - 2 * M_PI - gv->colliders[9].angles[0]))))) / float(actionSpaceSamples + 1);
                actionSpaceSamples++;
            }
        }

        if (detectionCounts[2] > 0) {
            /*
            evaluatedDetections[2].x = 0.0f;
            evaluatedDetections[2].y = 0.0f;
            evaluatedDetections[2].z = 0.0f;
            targetDetectionAngle = gv->colliders[10].angles[0];

            for (int i = 0; i < detectionLengths[2]; i++) {
                evaluatedDetections[2].x += thirdWorkpieceDetections[i].x;
                evaluatedDetections[2].y += thirdWorkpieceDetections[i].y;

                if (abs(thirdWorkpieceDetections[i].z - targetDetectionAngle) < abs(thirdWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) && abs(thirdWorkpieceDetections[i].z - targetDetectionAngle) < abs(thirdWorkpieceDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[2].z += thirdWorkpieceDetections[i].z;
                }
                else if (abs(thirdWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(thirdWorkpieceDetections[i].z - targetDetectionAngle) && abs(thirdWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(thirdWorkpieceDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[2].z += thirdWorkpieceDetections[i].z + M_PI * 2.0f;
                }
                else {
                    evaluatedDetections[2].z += thirdWorkpieceDetections[i].z - M_PI * 2.0f;
                }
            }
            evaluatedDetections[2].x /= float(detectionLengths[2]);
            evaluatedDetections[2].y /= float(detectionLengths[2]);
            evaluatedDetections[2].z /= float(detectionLengths[2]);
            */

            for (int i = 0; i < detectionCounts[2]; i++) {
                // cout << "C: " << sqrt((thirdWorkpieceDetections[i].x - gv->colliders[10].offsets[0]) * (thirdWorkpieceDetections[i].x - gv->colliders[10].offsets[0]) + (thirdWorkpieceDetections[i].y - gv->colliders[10].offsets[1]) * (thirdWorkpieceDetections[i].y - gv->colliders[10].offsets[1])) << endl;
                actionSpaceDist = (actionSpaceDist * float(actionSpaceSamples) + sqrt((thirdWorkpieceDetections[i].x - gv->colliders[10].offsets[0]) * (thirdWorkpieceDetections[i].x - gv->colliders[10].offsets[0]) + (thirdWorkpieceDetections[i].y - gv->colliders[10].offsets[1]) * (thirdWorkpieceDetections[i].y - gv->colliders[10].offsets[1]))) / float(actionSpaceSamples + 1);
                actionSpaceAng = (actionSpaceAng * float(actionSpaceSamples) + min(abs(thirdWorkpieceDetections[i].z - gv->colliders[10].angles[0]), float(min(abs(thirdWorkpieceDetections[i].z + 2 * M_PI - gv->colliders[10].angles[0]), abs(thirdWorkpieceDetections[i].z - 2 * M_PI - gv->colliders[10].angles[0]))))) / float(actionSpaceSamples + 1);
                actionSpaceSamples++;
            }
        }

        if (detectionCounts[3] > 0) {
            /*
            evaluatedDetections[3].x = 0.0f;
            evaluatedDetections[3].y = 0.0f;
            evaluatedDetections[3].z = 0.0f;
            targetDetectionAngle = gv->colliders[11].angles[0];

            for (int i = 0; i < detectionLengths[3]; i++) {
                evaluatedDetections[3].x += fourthWorkpieceDetections[i].x;
                evaluatedDetections[3].y += fourthWorkpieceDetections[i].y;

                if (abs(fourthWorkpieceDetections[i].z - targetDetectionAngle) < abs(fourthWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) && abs(fourthWorkpieceDetections[i].z - targetDetectionAngle) < abs(fourthWorkpieceDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[3].z += fourthWorkpieceDetections[i].z;
                }
                else if (abs(fourthWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(fourthWorkpieceDetections[i].z - targetDetectionAngle) && abs(fourthWorkpieceDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(fourthWorkpieceDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[3].z += fourthWorkpieceDetections[i].z + M_PI * 2.0f;
                }
                else {
                    evaluatedDetections[3].z += fourthWorkpieceDetections[i].z - M_PI * 2.0f;
                }
            }
            evaluatedDetections[3].x /= float(detectionLengths[3]);
            evaluatedDetections[3].y /= float(detectionLengths[3]);
            evaluatedDetections[3].z /= float(detectionLengths[3]);
            */

            for (int i = 0; i < detectionCounts[3]; i++) {
                // cout << "D: " << sqrt((fourthWorkpieceDetections[i].x - gv->colliders[11].offsets[0]) * (fourthWorkpieceDetections[i].x - gv->colliders[11].offsets[0]) + (fourthWorkpieceDetections[i].y - gv->colliders[11].offsets[1]) * (fourthWorkpieceDetections[i].y - gv->colliders[11].offsets[1])) << endl;
                actionSpaceDist = (actionSpaceDist * float(actionSpaceSamples) + sqrt((fourthWorkpieceDetections[i].x - gv->colliders[11].offsets[0]) * (fourthWorkpieceDetections[i].x - gv->colliders[11].offsets[0]) + (fourthWorkpieceDetections[i].y - gv->colliders[11].offsets[1]) * (fourthWorkpieceDetections[i].y - gv->colliders[11].offsets[1]))) / float(actionSpaceSamples + 1);
                actionSpaceAng = (actionSpaceAng * float(actionSpaceSamples) + min(abs(fourthWorkpieceDetections[i].z - gv->colliders[11].angles[0]), float(min(abs(fourthWorkpieceDetections[i].z + 2 * M_PI - gv->colliders[11].angles[0]), abs(fourthWorkpieceDetections[i].z - 2 * M_PI - gv->colliders[11].angles[0]))))) / float(actionSpaceSamples + 1);
                actionSpaceSamples++;
            }
        }
    }

    if (detectionCounts[4] > 0) {
    	cout << "debug: FrequentOps. " << __LINE__ <<endl;

        if (detectionCounts[4] == 1) {
            evaluatedDetections[4].x = firstObsDetections[0].x;
            evaluatedDetections[4].y = firstObsDetections[0].y;
            evaluatedDetections[4].z = firstObsDetections[0].z;
        }
        else {
            evaluatedDetections[4].x = 0.0f;
            evaluatedDetections[4].y = 0.0f;
            evaluatedDetections[4].z = 0.0f;
            targetDetectionAngle = firstObsDetections[0].z;

            for (int i = 0; i < detectionCounts[4]; i++) {
                evaluatedDetections[4].x += firstObsDetections[i].x;
                evaluatedDetections[4].y += firstObsDetections[i].y;

                if (abs(firstObsDetections[i].z - targetDetectionAngle) < abs(firstObsDetections[i].z + M_PI * 2.0f - targetDetectionAngle) && abs(firstObsDetections[i].z - targetDetectionAngle) < abs(firstObsDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[4].z += firstObsDetections[i].z;
                }
                else if (abs(firstObsDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(firstObsDetections[i].z - targetDetectionAngle) && abs(firstObsDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(firstObsDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[4].z += firstObsDetections[i].z + M_PI * 2.0f;
                }
                else {
                    evaluatedDetections[4].z += firstObsDetections[i].z - M_PI * 2.0f;
                }
            }
            evaluatedDetections[4].x /= float(detectionCounts[4]);
            evaluatedDetections[4].y /= float(detectionCounts[4]);
            evaluatedDetections[4].z /= float(detectionCounts[4]);

            if (calcActionSpace) {
                for (int i = 0; i < detectionCounts[4]; i++) {
                    // cout << "E: " << sqrt((firstObsDetections[i].x - evaluatedDetections[4].x) * (firstObsDetections[i].x - evaluatedDetections[4].x) + (firstObsDetections[i].y - evaluatedDetections[4].y) * (firstObsDetections[i].y - evaluatedDetections[4].y)) << endl;
                    actionSpaceDist = (actionSpaceDist * float(actionSpaceSamples) + sqrt((firstObsDetections[i].x - evaluatedDetections[4].x) * (firstObsDetections[i].x - evaluatedDetections[4].x) + (firstObsDetections[i].y - evaluatedDetections[4].y) * (firstObsDetections[i].y - evaluatedDetections[4].y))) / float(actionSpaceSamples + 1);
                    actionSpaceAng = (actionSpaceAng * float(actionSpaceSamples) + min(abs(firstObsDetections[i].z - evaluatedDetections[4].z), float(min(abs(firstObsDetections[i].z + 2 * M_PI - evaluatedDetections[4].z), abs(firstObsDetections[i].z - 2 * M_PI - evaluatedDetections[4].z))))) / float(actionSpaceSamples + 1);
                    actionSpaceSamples++;
                }
            }
        }
    }

    if (detectionCounts[5] > 0) {
    	cout << "debug: FrequentOps. " << __LINE__ <<endl;

        if (detectionCounts[5] == 1) {
            evaluatedDetections[5].x = secondObsDetections[0].x;
            evaluatedDetections[5].y = secondObsDetections[0].y;
            evaluatedDetections[5].z = secondObsDetections[0].z;
        }
        else {
            evaluatedDetections[5].x = 0.0f;
            evaluatedDetections[5].y = 0.0f;
            evaluatedDetections[5].z = 0.0f;
            targetDetectionAngle = secondObsDetections[0].z;

            for (int i = 0; i < detectionCounts[5]; i++) {
                evaluatedDetections[5].x += secondObsDetections[i].x;
                evaluatedDetections[5].y += secondObsDetections[i].y;

                if (abs(secondObsDetections[i].z - targetDetectionAngle) < abs(secondObsDetections[i].z + M_PI * 2.0f - targetDetectionAngle) && abs(secondObsDetections[i].z - targetDetectionAngle) < abs(secondObsDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[5].z += secondObsDetections[i].z;
                }
                else if (abs(secondObsDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(secondObsDetections[i].z - targetDetectionAngle) && abs(secondObsDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(secondObsDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[5].z += secondObsDetections[i].z + M_PI * 2.0f;
                }
                else {
                    evaluatedDetections[5].z += secondObsDetections[i].z - M_PI * 2.0f;
                }
            }
            evaluatedDetections[5].x /= float(detectionCounts[5]);
            evaluatedDetections[5].y /= float(detectionCounts[5]);
            evaluatedDetections[5].z /= float(detectionCounts[5]);

            if (calcActionSpace) {
                for (int i = 0; i < detectionCounts[5]; i++) {
                    // cout << "F: " << sqrt((secondObsDetections[i].x - evaluatedDetections[5].x) * (secondObsDetections[i].x - evaluatedDetections[5].x) + (secondObsDetections[i].y - evaluatedDetections[5].y) * (secondObsDetections[i].y - evaluatedDetections[5].y)) << endl;
                    actionSpaceDist = (actionSpaceDist * float(actionSpaceSamples) + sqrt((secondObsDetections[i].x - evaluatedDetections[5].x) * (secondObsDetections[i].x - evaluatedDetections[5].x) + (secondObsDetections[i].y - evaluatedDetections[5].y) * (secondObsDetections[i].y - evaluatedDetections[5].y))) / float(actionSpaceSamples + 1);
                    actionSpaceAng = (actionSpaceAng * float(actionSpaceSamples) + min(abs(secondObsDetections[i].z - evaluatedDetections[5].z), float(min(abs(secondObsDetections[i].z + 2 * M_PI - evaluatedDetections[5].z), abs(secondObsDetections[i].z - 2 * M_PI - evaluatedDetections[5].z))))) / float(actionSpaceSamples + 1);
                    actionSpaceSamples++;
                }
            }
        }
    }

    if (detectionCounts[6] > 0) {
    	cout << "debug: FrequentOps. " << __LINE__ <<endl;

        if (detectionCounts[6] == 1) {
            evaluatedDetections[6].x = thirdObsDetections[0].x;
            evaluatedDetections[6].y = thirdObsDetections[0].y;
            evaluatedDetections[6].z = thirdObsDetections[0].z;
        }
        else {
            evaluatedDetections[6].x = 0.0f;
            evaluatedDetections[6].y = 0.0f;
            evaluatedDetections[6].z = 0.0f;
            targetDetectionAngle = thirdObsDetections[0].z;

            for (int i = 0; i < detectionCounts[6]; i++) {
                evaluatedDetections[6].x += thirdObsDetections[i].x;
                evaluatedDetections[6].y += thirdObsDetections[i].y;

                if (abs(thirdObsDetections[i].z - targetDetectionAngle) < abs(thirdObsDetections[i].z + M_PI * 2.0f - targetDetectionAngle) && abs(thirdObsDetections[i].z - targetDetectionAngle) < abs(thirdObsDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[6].z += thirdObsDetections[i].z;
                }
                else if (abs(thirdObsDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(thirdObsDetections[i].z - targetDetectionAngle) && abs(thirdObsDetections[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(thirdObsDetections[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[6].z += thirdObsDetections[i].z + M_PI * 2.0f;
                }
                else {
                    evaluatedDetections[6].z += thirdObsDetections[i].z - M_PI * 2.0f;
                }
            }
            evaluatedDetections[6].x /= float(detectionCounts[6]);
            evaluatedDetections[6].y /= float(detectionCounts[6]);
            evaluatedDetections[6].z /= float(detectionCounts[6]);

            if (calcActionSpace) {
                for (int i = 0; i < detectionCounts[6]; i++) {
                    // cout << "G: " << sqrt((thirdObsDetections[i].x - evaluatedDetections[6].x) * (thirdObsDetections[i].x - evaluatedDetections[6].x) + (thirdObsDetections[i].y - evaluatedDetections[6].y) * (thirdObsDetections[i].y - evaluatedDetections[6].y)) << endl;
                    actionSpaceDist = (actionSpaceDist * float(actionSpaceSamples) + sqrt((thirdObsDetections[i].x - evaluatedDetections[6].x) * (thirdObsDetections[i].x - evaluatedDetections[6].x) + (thirdObsDetections[i].y - evaluatedDetections[6].y) * (thirdObsDetections[i].y - evaluatedDetections[6].y))) / float(actionSpaceSamples + 1);
                    actionSpaceAng = (actionSpaceAng * float(actionSpaceSamples) + min(abs(thirdObsDetections[i].z - evaluatedDetections[6].z), float(min(abs(thirdObsDetections[i].z + 2 * M_PI - evaluatedDetections[6].z), abs(thirdObsDetections[i].z - 2 * M_PI - evaluatedDetections[6].z))))) / float(actionSpaceSamples + 1);
                    actionSpaceSamples++;
                }
            }
        }
    }
}

void FrequentOps::evaluateDetectionsAlt() {
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();

	cout << "debug: FrequentOps. This function is deprecated!!! " << __LINE__ <<endl;

    if (detectionCounts2[4] > 0) {
        if (detectionCounts2[4] == 1) {
            evaluatedDetections[4].x = firstObsDetections2[0].x;
            evaluatedDetections[4].y = firstObsDetections2[0].y;
            evaluatedDetections[4].z = firstObsDetections2[0].z;
        }
        else {
            evaluatedDetections[4].x = 0.0f;
            evaluatedDetections[4].y = 0.0f;
            evaluatedDetections[4].z = 0.0f;
            targetDetectionAngle = firstObsDetections2[0].z;

            for (int i = 0; i < detectionCounts2[4]; i++) {
                evaluatedDetections[4].x += firstObsDetections2[i].x;
                evaluatedDetections[4].y += firstObsDetections2[i].y;

                if (abs(firstObsDetections2[i].z - targetDetectionAngle) < abs(firstObsDetections2[i].z + M_PI * 2.0f - targetDetectionAngle) && abs(firstObsDetections2[i].z - targetDetectionAngle) < abs(firstObsDetections2[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[4].z += firstObsDetections2[i].z;
                }
                else if (abs(firstObsDetections2[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(firstObsDetections2[i].z - targetDetectionAngle) && abs(firstObsDetections2[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(firstObsDetections2[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[4].z += firstObsDetections2[i].z + M_PI * 2.0f;
                }
                else {
                    evaluatedDetections[4].z += firstObsDetections2[i].z - M_PI * 2.0f;
                }
            }
            evaluatedDetections[4].x /= float(detectionCounts2[4]);
            evaluatedDetections[4].y /= float(detectionCounts2[4]);
            evaluatedDetections[4].z /= float(detectionCounts2[4]);
        }
    }

    if (detectionCounts2[5] > 0) {
        if (detectionCounts2[5] == 1) {
            evaluatedDetections[5].x = secondObsDetections2[0].x;
            evaluatedDetections[5].y = secondObsDetections2[0].y;
            evaluatedDetections[5].z = secondObsDetections2[0].z;
        }
        else {
            evaluatedDetections[5].x = 0.0f;
            evaluatedDetections[5].y = 0.0f;
            evaluatedDetections[5].z = 0.0f;
            targetDetectionAngle = secondObsDetections2[0].z;

            for (int i = 0; i < detectionCounts2[5]; i++) {
                evaluatedDetections[5].x += secondObsDetections2[i].x;
                evaluatedDetections[5].y += secondObsDetections2[i].y;

                if (abs(secondObsDetections2[i].z - targetDetectionAngle) < abs(secondObsDetections2[i].z + M_PI * 2.0f - targetDetectionAngle) && abs(secondObsDetections2[i].z - targetDetectionAngle) < abs(secondObsDetections2[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[5].z += secondObsDetections2[i].z;
                }
                else if (abs(secondObsDetections2[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(secondObsDetections2[i].z - targetDetectionAngle) && abs(secondObsDetections2[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(secondObsDetections2[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[5].z += secondObsDetections2[i].z + M_PI * 2.0f;
                }
                else {
                    evaluatedDetections[5].z += secondObsDetections2[i].z - M_PI * 2.0f;
                }
            }
            evaluatedDetections[5].x /= float(detectionCounts2[5]);
            evaluatedDetections[5].y /= float(detectionCounts2[5]);
            evaluatedDetections[5].z /= float(detectionCounts2[5]);
        }
    }

    if (detectionCounts2[6] > 0) {
        if (detectionCounts2[6] == 1) {
            evaluatedDetections[6].x = thirdObsDetections2[0].x;
            evaluatedDetections[6].y = thirdObsDetections2[0].y;
            evaluatedDetections[6].z = thirdObsDetections2[0].z;
        }
        else {
            evaluatedDetections[6].x = 0.0f;
            evaluatedDetections[6].y = 0.0f;
            evaluatedDetections[6].z = 0.0f;
            targetDetectionAngle = thirdObsDetections2[0].z;

            for (int i = 0; i < detectionCounts[6]; i++) {
                evaluatedDetections[6].x += thirdObsDetections2[i].x;
                evaluatedDetections[6].y += thirdObsDetections2[i].y;

                if (abs(thirdObsDetections2[i].z - targetDetectionAngle) < abs(thirdObsDetections2[i].z + M_PI * 2.0f - targetDetectionAngle) && abs(thirdObsDetections2[i].z - targetDetectionAngle) < abs(thirdObsDetections2[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[6].z += thirdObsDetections2[i].z;
                }
                else if (abs(thirdObsDetections2[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(thirdObsDetections2[i].z - targetDetectionAngle) && abs(thirdObsDetections2[i].z + M_PI * 2.0f - targetDetectionAngle) < abs(thirdObsDetections2[i].z - M_PI * 2.0f - targetDetectionAngle)) {
                    evaluatedDetections[6].z += thirdObsDetections2[i].z + M_PI * 2.0f;
                }
                else {
                    evaluatedDetections[6].z += thirdObsDetections2[i].z - M_PI * 2.0f;
                }
            }
            evaluatedDetections[6].x /= float(detectionCounts2[6]);
            evaluatedDetections[6].y /= float(detectionCounts2[6]);
            evaluatedDetections[6].z /= float(detectionCounts2[6]);
        }
    }
}

void FrequentOps::writeIntoTempSegments() {
	InverseKinematic *ik = InverseKinematic::get_instance();
	PathPlanner *pp=PathPlanner::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();

    if (gv->tempSegSize != 0) {
        gv->q0ValsTempSeg = ik->cutOff(gv->q0ValsTempSeg, gv->tempSegSize, 0);
        gv->q1ValsTempSeg = ik->cutOff(gv->q1ValsTempSeg, gv->tempSegSize, 0);
        gv->q2ValsTempSeg = ik->cutOff(gv->q2ValsTempSeg, gv->tempSegSize, 0);
        gv->q3ValsTempSeg = ik->cutOff(gv->q3ValsTempSeg, gv->tempSegSize, 0);
        gv->q4ValsTempSeg = ik->cutOff(gv->q4ValsTempSeg, gv->tempSegSize, 0);
    }

    gv->q0ValsTempSeg = ik->increaseSize(gv->q0ValsTempSeg, 0, 1);
    gv->q1ValsTempSeg = ik->increaseSize(gv->q1ValsTempSeg, 0, 1);
    gv->q2ValsTempSeg = ik->increaseSize(gv->q2ValsTempSeg, 0, 1);
    gv->q3ValsTempSeg = ik->increaseSize(gv->q3ValsTempSeg, 0, 1);
    gv->q4ValsTempSeg = ik->increaseSize(gv->q4ValsTempSeg, 0, 1);

    gv->q0ValsTempSeg[0] = pp->nodeList[1].nodeQVal[0];
    gv->q1ValsTempSeg[0] = pp->nodeList[1].nodeQVal[1];
    gv->q2ValsTempSeg[0] = pp->nodeList[1].nodeQVal[2];
    gv->q3ValsTempSeg[0] = pp->nodeList[1].nodeQVal[3];
    gv->q4ValsTempSeg[0] = pp->nodeList[1].nodeQVal[4];

    gv->tempSegSize = 1;
    pp->nodeIndex = 1;
    while (pp->nodeIndex != 0) {
        pp->nodeIndex = pp->nodeList[pp->nodeIndex].previous;
        gv->q0ValsTempSeg = ik->increaseSize(gv->q0ValsTempSeg, gv->tempSegSize, 1);
        gv->q1ValsTempSeg = ik->increaseSize(gv->q1ValsTempSeg, gv->tempSegSize, 1);
        gv->q2ValsTempSeg = ik->increaseSize(gv->q2ValsTempSeg, gv->tempSegSize, 1);
        gv->q3ValsTempSeg = ik->increaseSize(gv->q3ValsTempSeg, gv->tempSegSize, 1);
        gv->q4ValsTempSeg = ik->increaseSize(gv->q4ValsTempSeg, gv->tempSegSize, 1);

        gv->q0ValsTempSeg[gv->tempSegSize] = pp->nodeList[pp->nodeIndex].nodeQVal[0];
        gv->q1ValsTempSeg[gv->tempSegSize] = pp->nodeList[pp->nodeIndex].nodeQVal[1];
        gv->q2ValsTempSeg[gv->tempSegSize] = pp->nodeList[pp->nodeIndex].nodeQVal[2];
        gv->q3ValsTempSeg[gv->tempSegSize] = pp->nodeList[pp->nodeIndex].nodeQVal[3];
        gv->q4ValsTempSeg[gv->tempSegSize] = pp->nodeList[pp->nodeIndex].nodeQVal[4];

        gv->tempSegSize++;
    }
}

// check the safety of transportation processes:
bool FrequentOps::checkMTTransport(int mtIndex,
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3
) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	PathPlanner *pp=PathPlanner::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();
    isTransportPossible = false;

    gv->objectToTransport = gv->transportableObjects[mtIndex + 3];
    gv->objectHeight = gv->objectHeights[mtIndex + 3];
    gv->objectOffset = gv->objectOffsets[mtIndex + 3];
    gv->objectGripDepth = gv->objectGripDepths[mtIndex + 3];

    gv->objectDimension.x = gv->objectDimensions[mtIndex + 3].x + 0.25f;
    gv->objectDimension.y = gv->objectDimensions[mtIndex + 3].y + 0.25f;
    gv->objectDimension.z = gv->objectDimensions[mtIndex + 3].z + 0.25f;

    cd->updateColliderSize(gv->safetyCollider, gv->objectDimension.x, gv->objectDimension.y, gv->objectDimension.z);

    gv->pathStartPos.x = gv->colliders[gv->objectToTransport].offsets[0];
    gv->pathStartPos.y = gv->colliders[gv->objectToTransport].offsets[1];
    gv->pathStartPos.z = gv->objectOffset + gv->objectHeight + gv->objectGripDepth;
    gv->startObjectAngle = gv->colliders[gv->objectToTransport].angles[0] * 180.0f / M_PI;

    if (mtIndex == 1) {
        gv->pathEndPos.x = gv->mtPositionsX[0];
        gv->pathEndPos.y = gv->mtPositionsY[0];
        gv->endObjectAngle = gv->mtPositionsAngles[0] * 180.0f / M_PI;
    }
    else if (mtIndex == 2) {
        gv->pathEndPos.x = gv->mtPositionsX[1];
        gv->pathEndPos.y = gv->mtPositionsY[1];
        gv->endObjectAngle = gv->mtPositionsAngles[1] * 180.0f / M_PI;
    }
    else {
        cout << "Error: invalid MT index!" << endl;
    }
    gv->pathEndPos.z = gv->objectOffset + gv->objectHeight + gv->objectGripDepth;

    if (ik->doesThetaExist(gv->pathStartPos.x, gv->pathStartPos.y, gv->pathStartPos.z)) {
        ik->writeQValues(gv->qValuesObjectStart);
        if (abs(gv->qValuesObjectStart[1] + gv->qValuesObjectStart[2] + gv->qValuesObjectStart[3] - 180.0f) < cd->epsilon3) {
            if (ik->doesThetaExist(gv->pathEndPos.x, gv->pathEndPos.y, gv->pathEndPos.z)) {
                ik->writeQValues(gv->qValuesObjectEnd);
                if (abs(gv->qValuesObjectEnd[1] + gv->qValuesObjectEnd[2] + gv->qValuesObjectEnd[3] - 180.0f) < cd->epsilon3) {
                    gv->qValuesObjectStart[4] = gv->startObjectAngle - (90.0f - gv->qValuesObjectStart[0]);
                    gv->qValuesObjectEnd[4] = gv->endObjectAngle - (90.0f - gv->qValuesObjectEnd[0]);

                    gripAngleCalculation(mtIndex + 3);

                    if (gv->qValuesObjectStart[4] > ik->qUpperLimits[4] || gv->qValuesObjectStart[4] < ik->qLowerLimits[4] || gv->qValuesObjectEnd[4] > ik->qUpperLimits[4] || gv->qValuesObjectEnd[4] < ik->qLowerLimits[4]) {
                        isTransportPossible = false;
                    } else {
                        gv->transportPhase = 0;
                        gv->grippingWidth = gv->grippingWidthOpen;

                        gv->objectPosReachable = pp->calcPath(gv->qValuesObjectStart, gv->qValuesStandby, &gv->pathStartPos, &gv->standbyPos, false,
                        		generator1,distribution1,generator2,distribution2,generator3,distribution3);
                        if (gv->objectPosReachable) {
                            gv->transportPhase = 1;
                            gv->grippingWidth = gv->grippingWidthFixed;

                            gv->objectPosReachable = pp->calcPath(gv->qValuesObjectEnd, gv->qValuesObjectStart, &gv->pathEndPos, &gv->pathStartPos, false,generator1,distribution1,generator2,distribution2,generator3,distribution3);
                            if (gv->objectPosReachable) {
                                cd->recalculateCollider(&gv->colliders[gv->objectToTransport], gv->pathEndPos.x, gv->pathEndPos.y, gv->objectOffset, gv->endObjectAngle / 180.0f * M_PI, 0, 0);

                                gv->transportPhase = 2;
                                gv->grippingWidth = gv->grippingWidthOpen;

                                gv->objectPosReachable = pp->calcPath(gv->qValuesStandby, gv->qValuesObjectEnd, &gv->standbyPos, &gv->pathEndPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3);
                                if (gv->objectPosReachable) {
                                    isTransportPossible = true;
                                }

                                cd->recalculateCollider(&gv->colliders[gv->objectToTransport], gv->pathStartPos.x, gv->pathStartPos.y, gv->objectOffset, gv->startObjectAngle / 180.0f * M_PI, 0, 0);
                            }
                            gv->transportPhase = 0;
                        }
                    }
                }
            }
        }
    }

    return isTransportPossible;
}

bool FrequentOps::checkUnfinishedTransport(
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3
) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	PathPlanner *pp=PathPlanner::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();

    isTransportPossible = false;

    gv->pathEndPos.x = gv->selectedTargetPosition[0];
    gv->pathEndPos.y = gv->selectedTargetPosition[1];
    gv->pathEndPos.z = gv->objectOffset + gv->objectHeight + gv->objectGripDepth;
    gv->endObjectAngle = gv->selectedTargetPosition[3];

    if (ik->doesThetaExist(gv->pathEndPos.x, gv->pathEndPos.y, gv->pathEndPos.z)) {
        ik->writeQValues(gv->qValuesObjectEnd);
        if (abs(gv->qValuesObjectEnd[1] + gv->qValuesObjectEnd[2] + gv->qValuesObjectEnd[3] - 180.0f) < cd->epsilon3) {
            gv->qValuesObjectEnd[4] = gv->endObjectAngle - (90.0f - gv->qValuesObjectEnd[0]);
            gv->qValuesObjectEnd[4] -= gv->grippingAngleDiff;
            while (gv->qValuesObjectEnd[4] > 180.0f) {
                gv->qValuesObjectEnd[4] -= 360.0f;
            }
            while (gv->qValuesObjectEnd[4] <= -180.0f) {
                gv->qValuesObjectEnd[4] += 360.0f;
            }

            if (gv->qValuesObjectEnd[4] > ik->qUpperLimits[4] || gv->qValuesObjectEnd[4] < ik->qLowerLimits[4]) {
                isTransportPossible = false;
            }
            else {
                gv->transportPhase = 1;
                gv->grippingWidth = gv->grippingWidthFixed;

                gv->objectPosReachable = pp->calcPath(gv->qValuesObjectEnd, gv->qValuesCurrent, &gv->pathEndPos, &gv->pathAltStartPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3);
                if (gv->objectPosReachable) {
                    cd->recalculateCollider(&gv->colliders[gv->objectToTransport], gv->pathEndPos.x, gv->pathEndPos.y, gv->objectOffset, gv->endObjectAngle / 180.0f * M_PI, 0, 0);

                    gv->transportPhase = 2;
                    gv->grippingWidth = gv->grippingWidthOpen;

                    gv->objectPosReachable = pp->calcPath(gv->qValuesStandby, gv->qValuesObjectEnd, &gv->standbyPos, &gv->pathEndPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3);
                    if (gv->objectPosReachable) {
                        isTransportPossible = true;
                    }
                    cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, (gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI, false);
                }
            }
        }
    }
    return isTransportPossible;
}

bool FrequentOps::checkNewTransport(
		default_random_engine generator1,
		normal_distribution<float> distribution1,
		default_random_engine generator2,
		normal_distribution<float> distribution2,
		default_random_engine generator3,
		normal_distribution<float> distribution3
) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	PathPlanner *pp=PathPlanner::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();
    isTransportPossible = false;

    gv->objectToTransport = gv->transportableObjects[gv->selectedWorkpieceIndex];
    gv->objectHeight = gv->objectHeights[gv->selectedWorkpieceIndex];
    gv->objectOffset = gv->objectOffsets[gv->selectedWorkpieceIndex];
    gv->objectGripDepth = gv->objectGripDepths[gv->selectedWorkpieceIndex];

    gv->objectDimension.x = gv->objectDimensions[gv->selectedWorkpieceIndex].x + 0.25f;
    gv->objectDimension.y = gv->objectDimensions[gv->selectedWorkpieceIndex].y + 0.25f;
    gv->objectDimension.z = gv->objectDimensions[gv->selectedWorkpieceIndex].z + 0.25f;

    cd->updateColliderSize(gv->safetyCollider, gv->objectDimension.x, gv->objectDimension.y, gv->objectDimension.z);

    gv->pathStartPos.x = gv->colliders[gv->objectToTransport].offsets[0];
    gv->pathStartPos.y = gv->colliders[gv->objectToTransport].offsets[1];
    gv->pathStartPos.z = gv->objectOffset + gv->objectHeight + gv->objectGripDepth;
    gv->startObjectAngle = gv->colliders[gv->objectToTransport].angles[0] * 180.0f / M_PI;

    gv->pathEndPos.x = gv->selectedTargetPosition[0];
    gv->pathEndPos.y = gv->selectedTargetPosition[1];
    gv->pathEndPos.z = gv->objectOffset + gv->objectHeight + gv->objectGripDepth;
    gv->endObjectAngle = gv->selectedTargetPosition[3];

    if (ik->doesThetaExist(gv->pathStartPos.x, gv->pathStartPos.y, gv->pathStartPos.z)) {
        ik->writeQValues(gv->qValuesObjectStart);;
        if (abs(gv->qValuesObjectStart[1] + gv->qValuesObjectStart[2] + gv->qValuesObjectStart[3] - 180.0f) < cd->epsilon3) {
            if (ik->doesThetaExist(gv->pathEndPos.x, gv->pathEndPos.y, gv->pathEndPos.z)) {
                ik->writeQValues(gv->qValuesObjectEnd);
                if (abs(gv->qValuesObjectEnd[1] + gv->qValuesObjectEnd[2] + gv->qValuesObjectEnd[3] - 180.0f) < cd->epsilon3) {
                    gv->qValuesObjectStart[4] = gv->startObjectAngle - (90.0f - gv->qValuesObjectStart[0]);
                    gv->qValuesObjectEnd[4] = gv->endObjectAngle - (90.0f - gv->qValuesObjectEnd[0]);

                    gripAngleCalculation(gv->selectedWorkpieceIndex);

                    if (gv->qValuesObjectStart[4] > ik->qUpperLimits[4] || gv->qValuesObjectStart[4] < ik->qLowerLimits[4] || gv->qValuesObjectEnd[4] > ik->qUpperLimits[4] || gv->qValuesObjectEnd[4] < ik->qLowerLimits[4]) {
                        isTransportPossible = false;
                    }
                    else {
                        gv->transportPhase = 0;
                        gv->grippingWidth = gv->grippingWidthOpen;

                        if (gv->atStandby) {
                            gv->objectPosReachable = pp->calcPath(gv->qValuesObjectStart, gv->qValuesStandby, &gv->pathStartPos, &gv->standbyPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3);
                        }
                        else {
                            gv->objectPosReachable = pp->calcPath(gv->qValuesObjectStart, gv->qValuesCurrent, &gv->pathStartPos, &gv->pathAltStartPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3);
                        }

                        if (gv->objectPosReachable) {
                            gv->transportPhase = 1;
                            gv->grippingWidth = gv->grippingWidthFixed;

                            gv->objectPosReachable = pp->calcPath(gv->qValuesObjectEnd, gv->qValuesObjectStart, &gv->pathEndPos, &gv->pathStartPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3);
                            if (gv->objectPosReachable) {
                                cd->recalculateCollider(&gv->colliders[gv->objectToTransport], gv->pathEndPos.x, gv->pathEndPos.y, gv->objectOffset, gv->endObjectAngle / 180.0f * M_PI, 0, 0);

                                gv->transportPhase = 2;
                                gv->grippingWidth = gv->grippingWidthOpen;

                                gv->objectPosReachable = pp->calcPath(gv->qValuesStandby, gv->qValuesObjectEnd, &gv->standbyPos, &gv->pathEndPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3);
                                if (gv->objectPosReachable) {
                                    isTransportPossible = true;
                                }

                                cd->recalculateCollider(&gv->colliders[gv->objectToTransport], gv->pathStartPos.x, gv->pathStartPos.y, gv->objectOffset, gv->startObjectAngle / 180.0f * M_PI, 0, 0);
                            }
                        }
                    }
                }
            }
        }
    }

    return isTransportPossible;
}
