/*
 * MonitoringToolModels.cpp
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#include "MonitoringToolModels.h"
//#include "../inverse_kinematics.h"

MonitoringToolModels::MonitoringToolModels() {

}

MonitoringToolModels::~MonitoringToolModels() {
}

/*
 * singelton pattern
 */
MonitoringToolModels* MonitoringToolModels::mt = 0;
MonitoringToolModels* MonitoringToolModels::get_instance(){
	static bool isInit=false;
	if(!isInit){
		mt  = new MonitoringToolModels();
		isInit=true;
	}
	return mt;
}

/*
 *
 */
void MonitoringToolModels::init_monitoringTool1(collider* colliders) {
	colliders[15].points_stat = 9;
	colliders[15].x_stat = new float[colliders[15].points_stat] { 0, 2.5f,
			-2.5f, 2.5f, -2.5f, 2.5f, -2.5f, 2.5f, -2.5f };
	colliders[15].y_stat = new float[colliders[15].points_stat] { 0, 1.5f, 1.5f,
			-1.5f, -1.5f, 1.5f, 1.5f, -1.5f, -1.5f };
	colliders[15].z_stat = new float[colliders[15].points_stat] { 0, -3.5f,
			-3.5f, -3.5f, -3.5f, 3.5f, 3.5f, 3.5f, 3.5f };
	colliders[15].color = collider::Edge { 0.0f, 153.0f / 255.0f, 204.0f / 255.0f };
}

/*
 *
 */
void MonitoringToolModels::init_monitoringTool2(collider* colliders) {
	colliders[16].points_stat = 9;
	colliders[16].x_stat = new float[colliders[16].points_stat] { 0, 2.5f,
			-2.5f, 2.5f, -2.5f, 2.5f, -2.5f, 2.5f, -2.5f };
	colliders[16].y_stat = new float[colliders[16].points_stat] { 0, 1.5f, 1.5f,
			-1.5f, -1.5f, 1.5f, 1.5f, -1.5f, -1.5f };
	colliders[16].z_stat = new float[colliders[16].points_stat] { 0, -3.5f,
			-3.5f, -3.5f, -3.5f, 3.5f, 3.5f, 3.5f, 3.5f };
	colliders[16].color = collider::Edge { 0.0f, 153.0f / 255.0f * 0.75f, 204.0f / 255.0f
			* 0.75f };
}


/*
 *
 */
void MonitoringToolModels::init_safetyCollider4MT2(collider* safetyCollidersForMTs) {
	safetyCollidersForMTs[1].points_stat = 9;
	safetyCollidersForMTs[1].x_stat =
			new float[safetyCollidersForMTs[1].points_stat] { 0, 1.0f, 1.0f,
					-1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f };
	safetyCollidersForMTs[1].y_stat =
			new float[safetyCollidersForMTs[1].points_stat] { 0, 1.0f, -1.0f,
					1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f };
	safetyCollidersForMTs[1].z_stat =
			new float[safetyCollidersForMTs[1].points_stat] { 0, -1.0f, -1.0f,
					-1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
}

/*
 *
 */
void MonitoringToolModels::init_safetyCollider4MT1(collider* safetyCollidersForMTs) {
	safetyCollidersForMTs[0].points_stat = 9;
	safetyCollidersForMTs[0].x_stat =
			new float[safetyCollidersForMTs[0].points_stat] { 0, 1.0f, 1.0f,
					-1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f };
	safetyCollidersForMTs[0].y_stat =
			new float[safetyCollidersForMTs[0].points_stat] { 0, 1.0f, -1.0f,
					1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f };
	safetyCollidersForMTs[0].z_stat =
			new float[safetyCollidersForMTs[0].points_stat] { 0, -1.0f, -1.0f,
					-1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
}

// simulate the MT behaviour in the virtual representation, check which objects are localized and which not:
void MonitoringToolModels::updateMonitoredSpace(bool init, bool plot) {
	InverseKinematic *ik = InverseKinematic::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();
//	HelperFunctions *hf = HelperFunctions::get_instance();
    if (!init) {
        gv->lastDetectedObsPosX[0] = gv->currentDetectedObsPosX[0]; gv->lastDetectedObsPosX[1] = gv->currentDetectedObsPosX[1]; gv->lastDetectedObsPosX[2] = gv->currentDetectedObsPosX[2];
        gv->lastDetectedObsPosY[0] = gv->currentDetectedObsPosY[0]; gv->lastDetectedObsPosY[1] = gv->currentDetectedObsPosY[1]; gv->lastDetectedObsPosY[2] = gv->currentDetectedObsPosY[2];
        gv->lastDetectedObsPosA[0] = gv->currentDetectedObsPosA[0]; gv->lastDetectedObsPosA[1] = gv->currentDetectedObsPosA[1]; gv->lastDetectedObsPosA[2] = gv->currentDetectedObsPosA[2];
    }

    gv->currentDetectedObsPosX[0] = 0.0f; gv->currentDetectedObsPosX[1] = 0.0f; gv->currentDetectedObsPosX[2] = 0.0f;
    gv->currentDetectedObsPosY[0] = 0.0f; gv->currentDetectedObsPosY[1] = 0.0f; gv->currentDetectedObsPosY[2] = 0.0f;
    gv->currentDetectedObsPosA[0] = 0.0f; gv->currentDetectedObsPosA[1] = 0.0f; gv->currentDetectedObsPosA[2] = 0.0f;

    if (plot) {
        gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges, gv->numPointsMTrays, 0);
        gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices, gv->numPointsMTrays, 0);
        gv->numPointsMTrays = 0;
    }

    for (int i = 8; i < objectsWithPatternsCount + 8; i++) {
        if (gv->MT1currentlyActive) {
            visiblePatternsMT1[i - 8] = isObjectVisible(i, 1);

            if (visiblePatternsMT1[i - 8] != -1) {
                if (i >= 12) {
                    gv->currentDetectedObsPosX[i - 12] = gv->distortedObstacles[i - 12].offsets[0];
                    gv->currentDetectedObsPosY[i - 12] = gv->distortedObstacles[i - 12].offsets[1];
                    gv->currentDetectedObsPosA[i - 12] = gv->distortedObstacles[i - 12].angles[0];
                }
                if (plot) {
                    gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2);
                    gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

                    gv->mt_rays_edges[gv->numPointsMTrays].x = 0.25f * (gv->colliders[i].x_pattern_tf[visiblePatternsMT1[i - 8]] + gv->colliders[i].x_pattern_tf[visiblePatternsMT1[i - 8] + 1] + gv->colliders[i].x_pattern_tf[visiblePatternsMT1[i - 8] + 2] + gv->colliders[i].x_pattern_tf[visiblePatternsMT1[i - 8] + 3]);
                    gv->mt_rays_edges[gv->numPointsMTrays].y = 0.25f * (gv->colliders[i].y_pattern_tf[visiblePatternsMT1[i - 8]] + gv->colliders[i].y_pattern_tf[visiblePatternsMT1[i - 8] + 1] + gv->colliders[i].y_pattern_tf[visiblePatternsMT1[i - 8] + 2] + gv->colliders[i].y_pattern_tf[visiblePatternsMT1[i - 8] + 3]);
                    gv->mt_rays_edges[gv->numPointsMTrays].z = 0.25f * (gv->colliders[i].z_pattern_tf[visiblePatternsMT1[i - 8]] + gv->colliders[i].z_pattern_tf[visiblePatternsMT1[i - 8] + 1] + gv->colliders[i].z_pattern_tf[visiblePatternsMT1[i - 8] + 2] + gv->colliders[i].z_pattern_tf[visiblePatternsMT1[i - 8] + 3]);
                    gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
                    gv->numPointsMTrays++;

                    gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0];
                    gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1];
                    gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
                    gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
                    gv->numPointsMTrays++;
                }
            }
        }
        else {
            visiblePatternsMT1[i - 8] = -1;
        }

        if (gv->MT2currentlyActive) {
            visiblePatternsMT2[i - 8] = isObjectVisible(i, 2);
            if (visiblePatternsMT2[i - 8] != -1) {
                if (i >= 12) {
                    gv->currentDetectedObsPosX[i - 12] = gv->distortedObstacles[i - 12].offsets[0];
                    gv->currentDetectedObsPosY[i - 12] = gv->distortedObstacles[i - 12].offsets[1];
                    gv->currentDetectedObsPosA[i - 12] = gv->distortedObstacles[i - 12].angles[0];
                }

                if (plot) {
                    gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2);
                    gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

                    gv->mt_rays_edges[gv->numPointsMTrays].x = 0.25f * (gv->colliders[i].x_pattern_tf[visiblePatternsMT2[i - 8]] + gv->colliders[i].x_pattern_tf[visiblePatternsMT2[i - 8] + 1] + gv->colliders[i].x_pattern_tf[visiblePatternsMT2[i - 8] + 2] + gv->colliders[i].x_pattern_tf[visiblePatternsMT2[i - 8] + 3]);
                    gv->mt_rays_edges[gv->numPointsMTrays].y = 0.25f * (gv->colliders[i].y_pattern_tf[visiblePatternsMT2[i - 8]] + gv->colliders[i].y_pattern_tf[visiblePatternsMT2[i - 8] + 1] + gv->colliders[i].y_pattern_tf[visiblePatternsMT2[i - 8] + 2] + gv->colliders[i].y_pattern_tf[visiblePatternsMT2[i - 8] + 3]);
                    gv->mt_rays_edges[gv->numPointsMTrays].z = 0.25f * (gv->colliders[i].z_pattern_tf[visiblePatternsMT2[i - 8]] + gv->colliders[i].z_pattern_tf[visiblePatternsMT2[i - 8] + 1] + gv->colliders[i].z_pattern_tf[visiblePatternsMT2[i - 8] + 2] + gv->colliders[i].z_pattern_tf[visiblePatternsMT2[i - 8] + 3]);
                    gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
                    gv->numPointsMTrays++;

                    gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0];
                    gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1];
                    gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
                    gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
                    gv->numPointsMTrays++;
                }
            }
        }
        else {
            visiblePatternsMT2[i - 8] = -1;
        }
    }
}

void MonitoringToolModels::updateMonitoredSpaceJustPlot() {
	InverseKinematic *ik = InverseKinematic::get_instance(); GlobalVariables *gv = GlobalVariables::get_instance();
    gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges, gv->numPointsMTrays, 0);
    gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices, gv->numPointsMTrays, 0);
    gv->numPointsMTrays = 0;

    for (int i = 8; i < objectsWithPatternsCount + 8; i++) {
        if (gv->MT1currentlyActive) {
            if (visiblePatternsMT1[i - 8] != -1) {

                gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2);
                gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

                gv->mt_rays_edges[gv->numPointsMTrays].x = 0.25f * (gv->colliders[i].x_pattern_tf[visiblePatternsMT1[i - 8]] + gv->colliders[i].x_pattern_tf[visiblePatternsMT1[i - 8] + 1] + gv->colliders[i].x_pattern_tf[visiblePatternsMT1[i - 8] + 2] + gv->colliders[i].x_pattern_tf[visiblePatternsMT1[i - 8] + 3]);
                gv->mt_rays_edges[gv->numPointsMTrays].y = 0.25f * (gv->colliders[i].y_pattern_tf[visiblePatternsMT1[i - 8]] + gv->colliders[i].y_pattern_tf[visiblePatternsMT1[i - 8] + 1] + gv->colliders[i].y_pattern_tf[visiblePatternsMT1[i - 8] + 2] + gv->colliders[i].y_pattern_tf[visiblePatternsMT1[i - 8] + 3]);
                gv->mt_rays_edges[gv->numPointsMTrays].z = 0.25f * (gv->colliders[i].z_pattern_tf[visiblePatternsMT1[i - 8]] + gv->colliders[i].z_pattern_tf[visiblePatternsMT1[i - 8] + 1] + gv->colliders[i].z_pattern_tf[visiblePatternsMT1[i - 8] + 2] + gv->colliders[i].z_pattern_tf[visiblePatternsMT1[i - 8] + 3]);
                gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
                gv->numPointsMTrays++;

                gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0];
                gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1];
                gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
                gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
                gv->numPointsMTrays++;
            }
        }
        if (gv->MT2currentlyActive) {
            if (visiblePatternsMT2[i - 8] != -1) {

                gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2);
                gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

                gv->mt_rays_edges[gv->numPointsMTrays].x = 0.25f * (gv->colliders[i].x_pattern_tf[visiblePatternsMT2[i - 8]] + gv->colliders[i].x_pattern_tf[visiblePatternsMT2[i - 8] + 1] + gv->colliders[i].x_pattern_tf[visiblePatternsMT2[i - 8] + 2] + gv->colliders[i].x_pattern_tf[visiblePatternsMT2[i - 8] + 3]);
                gv->mt_rays_edges[gv->numPointsMTrays].y = 0.25f * (gv->colliders[i].y_pattern_tf[visiblePatternsMT2[i - 8]] + gv->colliders[i].y_pattern_tf[visiblePatternsMT2[i - 8] + 1] + gv->colliders[i].y_pattern_tf[visiblePatternsMT2[i - 8] + 2] + gv->colliders[i].y_pattern_tf[visiblePatternsMT2[i - 8] + 3]);
                gv->mt_rays_edges[gv->numPointsMTrays].z = 0.25f * (gv->colliders[i].z_pattern_tf[visiblePatternsMT2[i - 8]] + gv->colliders[i].z_pattern_tf[visiblePatternsMT2[i - 8] + 1] + gv->colliders[i].z_pattern_tf[visiblePatternsMT2[i - 8] + 2] + gv->colliders[i].z_pattern_tf[visiblePatternsMT2[i - 8] + 3]);
                gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
                gv->numPointsMTrays++;

                gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0];
                gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1];
                gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
                gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
                gv->numPointsMTrays++;
            }
        }
    }
}

bool MonitoringToolModels::checkForFreeSight(float startX, float startY, float startZ, float endX, float endY, float endZ, int obstacleIndex, int mtIndex, float* camVectors) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();
	gv->distInDir = endX * camVectors[3] + endY * camVectors[4] + endZ * camVectors[5] - (startX * camVectors[3] + startY * camVectors[4] + startZ * camVectors[5]);
	if (gv->distInDir >= cd->minObjectDistance && gv->distInDir <= cd->maxObjectDistance) {
		gv->distInVert = endX * camVectors[6] + endY * camVectors[7] + endZ * camVectors[8] - (startX * camVectors[6] + startY * camVectors[7] + startZ * camVectors[8]);
		if (abs(atan(gv->distInVert / gv->distInDir) * 180.0f / M_PI) <= cd->vertViewAngle / 2.0f) {
			gv->distInHor = endX * camVectors[9] + endY * camVectors[10] + endZ * camVectors[11] - (startX * camVectors[9] + startY * camVectors[10] + startZ * camVectors[11]);
			if (abs(atan(gv->distInHor / gv->distInDir) * 180.0f / M_PI) <= cd->horViewAngle / 2.0f) {
				gv->freeSight = true;
				for (int j = 1; j < gv->collidersCount; j++) {
					if (j != obstacleIndex && j != mtIndex) {
						gv->minTValueLine = -((startX - gv->colliders[j].offsets[0]) * (endX - startX) + (startY - gv->colliders[j].offsets[1]) * (endY - startY) + (startZ - gv->colliders[j].offsets[2]) * (endZ - startZ)) / ((endX - startX) * (endX - startX) + (endY - startY) * (endY - startY) + (endZ - startZ) * (endZ - startZ));
						if (gv->minTValueLine >= 0.0f && gv->minTValueLine <= 1.0f) {
							gv->minLinePointVector[0] = (endX - startX) * gv->minTValueLine + startX - gv->colliders[j].offsets[0];
							gv->minLinePointVector[1] = (endY - startY) * gv->minTValueLine + startY - gv->colliders[j].offsets[1];
							gv->minLinePointVector[2] = (endZ - startZ) * gv->minTValueLine + startZ - gv->colliders[j].offsets[2];
							gv->minDistValueLine = sqrt(gv->minLinePointVector[0] * gv->minLinePointVector[0] + gv->minLinePointVector[1] * gv->minLinePointVector[1] + gv->minLinePointVector[2] * gv->minLinePointVector[2]);
						}
						else if (gv->minTValueLine > 1.0f) {
							gv->minLinePointVector[0] = endX - gv->colliders[j].offsets[0];
							gv->minLinePointVector[1] = endY - gv->colliders[j].offsets[1];
							gv->minLinePointVector[2] = endZ - gv->colliders[j].offsets[2];
							gv->minDistValueLine = sqrt(gv->minLinePointVector[0] * gv->minLinePointVector[0] + gv->minLinePointVector[1] * gv->minLinePointVector[1] + gv->minLinePointVector[2] * gv->minLinePointVector[2]);
						}
						else {
							gv->minLinePointVector[0] = startX - gv->colliders[j].offsets[0];
							gv->minLinePointVector[1] = startY - gv->colliders[j].offsets[1];
							gv->minLinePointVector[2] = startZ - gv->colliders[j].offsets[2];
							gv->minDistValueLine = sqrt(gv->minLinePointVector[0] * gv->minLinePointVector[0] + gv->minLinePointVector[1] * gv->minLinePointVector[1] + gv->minLinePointVector[2] * gv->minLinePointVector[2]);
						}

						if (gv->minDistValueLine <= gv->colliders[j].maxSize) {
							gv->intersectingBoundingBox = false;

							gv->surfaceTval = (gv->colliders[j].limits[0] - startX) / (endX - startX);
							if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
								if ((endY - startY) * gv->surfaceTval + startY <= gv->colliders[j].limits[2] && (endY - startY) * gv->surfaceTval + startY >= gv->colliders[j].limits[3] && (endZ - startZ) * gv->surfaceTval + startZ <= gv->colliders[j].limits[4] && (endZ - startZ) * gv->surfaceTval + startZ >= gv->colliders[j].limits[5]) {
									gv->intersectingBoundingBox = true;
								}
							}
							if (!gv->intersectingBoundingBox) {
								gv->surfaceTval = (gv->colliders[j].limits[1] - startX) / (endX - startX);
								if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
									if ((endY - startY) * gv->surfaceTval + startY <= gv->colliders[j].limits[2] && (endY - startY) * gv->surfaceTval + startY >= gv->colliders[j].limits[3] && (endZ - startZ) * gv->surfaceTval + startZ <= gv->colliders[j].limits[4] && (endZ - startZ) * gv->surfaceTval + startZ >= gv->colliders[j].limits[5]) {
										gv->intersectingBoundingBox = true;
									}
								}
							}
							if (!gv->intersectingBoundingBox) {
								gv->surfaceTval = (gv->colliders[j].limits[2] - startY) / (endY - startY);
								if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
									if ((endX - startX) * gv->surfaceTval + startX <= gv->colliders[j].limits[0] && (endX - startX) * gv->surfaceTval + startX >= gv->colliders[j].limits[1] && (endZ - startZ) * gv->surfaceTval + startZ <= gv->colliders[j].limits[4] && (endZ - startZ) * gv->surfaceTval + startZ >= gv->colliders[j].limits[5]) {
										gv->intersectingBoundingBox = true;
									}
								}
							}
							if (!gv->intersectingBoundingBox) {
								gv->surfaceTval = (gv->colliders[j].limits[3] - startY) / (endY - startY);
								if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
									if ((endX - startX) * gv->surfaceTval + startX <= gv->colliders[j].limits[0] && (endX - startX) * gv->surfaceTval + startX >= gv->colliders[j].limits[1] && (endZ - startZ) * gv->surfaceTval + startZ <= gv->colliders[j].limits[4] && (endZ - startZ) * gv->surfaceTval + startZ >= gv->colliders[j].limits[5]) {
										gv->intersectingBoundingBox = true;
									}
								}
							}
							if (!gv->intersectingBoundingBox) {
								gv->surfaceTval = (gv->colliders[j].limits[4] - startZ) / (endZ - startZ);
								if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
									if ((endX - startX) * gv->surfaceTval + startX <= gv->colliders[j].limits[0] && (endX - startX) * gv->surfaceTval + startX >= gv->colliders[j].limits[1] && (endY - startY) * gv->surfaceTval + startY <= gv->colliders[j].limits[2] && (endY - startY) * gv->surfaceTval + startY >= gv->colliders[j].limits[3]) {
										gv->intersectingBoundingBox = true;
									}
								}
							}
							if (!gv->intersectingBoundingBox) {
								gv->surfaceTval = (gv->colliders[j].limits[5] - startZ) / (endZ - startZ);
								if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
									if ((endX - startX) * gv->surfaceTval + startX <= gv->colliders[j].limits[0] && (endX - startX) * gv->surfaceTval + startX >= gv->colliders[j].limits[1] && (endY - startY) * gv->surfaceTval + startY <= gv->colliders[j].limits[2] && (endY - startY) * gv->surfaceTval + startY >= gv->colliders[j].limits[3]) {
										gv->intersectingBoundingBox = true;
									}
								}
							}

							if (gv->intersectingBoundingBox) {
								for (int i = 0; i < gv->colliders[j].facesTimes3; i++) {
									if (i % 3 == 0) {
										gv->surfaceO[0] = gv->colliders[j].x_coll[gv->colliders[j].connections[i]];
										gv->surfaceO[1] = gv->colliders[j].y_coll[gv->colliders[j].connections[i]];
										gv->surfaceO[2] = gv->colliders[j].z_coll[gv->colliders[j].connections[i]];

										gv->surfaceV1[0] = gv->colliders[j].x_coll[gv->colliders[j].connections[i + 1]] - gv->surfaceO[0];
										gv->surfaceV1[1] = gv->colliders[j].y_coll[gv->colliders[j].connections[i + 1]] - gv->surfaceO[1];
										gv->surfaceV1[2] = gv->colliders[j].z_coll[gv->colliders[j].connections[i + 1]] - gv->surfaceO[2];

										gv->surfaceV2[0] = gv->colliders[j].x_coll[gv->colliders[j].connections[i + 2]] - gv->surfaceO[0];
										gv->surfaceV2[1] = gv->colliders[j].y_coll[gv->colliders[j].connections[i + 2]] - gv->surfaceO[1];
										gv->surfaceV2[2] = gv->colliders[j].z_coll[gv->colliders[j].connections[i + 2]] - gv->surfaceO[2];

										gv->surfaceNormal[0] = gv->surfaceV1[1] * gv->surfaceV2[2] - gv->surfaceV1[2] * gv->surfaceV2[1];
										gv->surfaceNormal[1] = gv->surfaceV1[2] * gv->surfaceV2[0] - gv->surfaceV1[0] * gv->surfaceV2[2];
										gv->surfaceNormal[2] = gv->surfaceV1[0] * gv->surfaceV2[1] - gv->surfaceV1[1] * gv->surfaceV2[0];
										gv->surfaceDval = gv->surfaceNormal[0] * gv->surfaceO[0] + gv->surfaceNormal[1] * gv->surfaceO[1] + gv->surfaceNormal[2] * gv->surfaceO[2];

										if (abs(gv->surfaceNormal[0] * (endX - startX) + gv->surfaceNormal[1] * (endY - startY) + gv->surfaceNormal[2] * (endZ - startZ)) > cd->epsilon3) {
											gv->surfaceTval = (gv->surfaceDval - gv->surfaceNormal[0] * startX - gv->surfaceNormal[1] * startY - gv->surfaceNormal[2] * startZ) / (gv->surfaceNormal[0] * (endX - startX) + gv->surfaceNormal[1] * (endY - startY) + gv->surfaceNormal[2] * (endZ - startZ));
											if (gv->surfaceTval <= 1.0f && gv->surfaceTval >= 0.0f) {
												gv->surfaceV[0] = startX + (endX - startX) * gv->surfaceTval;
												gv->surfaceV[1] = startY + (endY - startY) * gv->surfaceTval;
												gv->surfaceV[2] = startZ + (endZ - startZ) * gv->surfaceTval;

												gv->surfaceN0 = gv->surfaceV2[0] * gv->surfaceV1[1] - gv->surfaceV1[0] * gv->surfaceV2[1];
												gv->surfaceN1 = gv->surfaceV2[0] * gv->surfaceV1[2] - gv->surfaceV1[0] * gv->surfaceV2[2];
												gv->surfaceN2 = gv->surfaceV2[1] * gv->surfaceV1[2] - gv->surfaceV1[1] * gv->surfaceV2[2];

												if (abs(gv->surfaceN0) > cd->epsilon4) {
													gv->surfaceA = (gv->surfaceV2[0] * (gv->surfaceV[1] - gv->surfaceO[1]) - gv->surfaceV2[1] * (gv->surfaceV[0] - gv->surfaceO[0])) / gv->surfaceN0;
													gv->surfaceB = (gv->surfaceV1[1] * (gv->surfaceV[0] - gv->surfaceO[0]) - gv->surfaceV1[0] * (gv->surfaceV[1] - gv->surfaceO[1])) / gv->surfaceN0;
													if (gv->surfaceA >= 0.0f && gv->surfaceB >= 0.0f && gv->surfaceA + gv->surfaceB <= 1.0f) {
														gv->freeSight = false;
														break;
													}
												}
												else if (abs(gv->surfaceN1) > cd->epsilon4) {
													gv->surfaceA = (gv->surfaceV2[0] * (gv->surfaceV[2] - gv->surfaceO[2]) - gv->surfaceV2[2] * (gv->surfaceV[0] - gv->surfaceO[0])) / gv->surfaceN1;
													gv->surfaceB = (gv->surfaceV1[2] * (gv->surfaceV[0] - gv->surfaceO[0]) - gv->surfaceV1[0] * (gv->surfaceV[2] - gv->surfaceO[2])) / gv->surfaceN1;
													if (gv->surfaceA >= 0.0f && gv->surfaceB >= 0.0f && gv->surfaceA + gv->surfaceB <= 1.0f) {
														gv->freeSight = false;
														break;
													}
												}
												else if (abs(gv->surfaceN2) > cd->epsilon4) {
													gv->surfaceA = (gv->surfaceV2[1] * (gv->surfaceV[2] - gv->surfaceO[2]) - gv->surfaceV2[2] * (gv->surfaceV[1] - gv->surfaceO[1])) / gv->surfaceN2;
													gv->surfaceB = (gv->surfaceV1[2] * (gv->surfaceV[1] - gv->surfaceO[1]) - gv->surfaceV1[1] * (gv->surfaceV[2] - gv->surfaceO[2])) / gv->surfaceN2;
													if (gv->surfaceA >= 0.0f && gv->surfaceB >= 0.0f && gv->surfaceA + gv->surfaceB <= 1.0f) {
														gv->freeSight = false;
														break;
													}
												}
											}
										}
									}
								}
								if (!gv->freeSight) break;
							}
						}
					}
				}

				if (gv->freeSight) {
					for (int k = 0; k <= 1; k++) {
						gv->minTValueLine = -((startX - gv->grippers[k].offsets[0]) * (endX - startX) + (startY - gv->grippers[k].offsets[1]) * (endY - startY) + (startZ - gv->grippers[k].offsets[2]) * (endZ - startZ)) / ((endX - startX) * (endX - startX) + (endY - startY) * (endY - startY) + (endZ - startZ) * (endZ - startZ));
						if (gv->minTValueLine >= 0.0f && gv->minTValueLine <= 1.0f) {
							gv->minLinePointVector[0] = (endX - startX) * gv->minTValueLine + startX - gv->grippers[k].offsets[0];
							gv->minLinePointVector[1] = (endY - startY) * gv->minTValueLine + startY - gv->grippers[k].offsets[1];
							gv->minLinePointVector[2] = (endZ - startZ) * gv->minTValueLine + startZ - gv->grippers[k].offsets[2];
							gv->minDistValueLine = sqrt(gv->minLinePointVector[0] * gv->minLinePointVector[0] + gv->minLinePointVector[1] * gv->minLinePointVector[1] + gv->minLinePointVector[2] * gv->minLinePointVector[2]);
						}
						else if (gv->minTValueLine > 1.0f) {
							gv->minLinePointVector[0] = endX - gv->grippers[k].offsets[0];
							gv->minLinePointVector[1] = endY - gv->grippers[k].offsets[1];
							gv->minLinePointVector[2] = endZ - gv->grippers[k].offsets[2];
							gv->minDistValueLine = sqrt(gv->minLinePointVector[0] * gv->minLinePointVector[0] + gv->minLinePointVector[1] * gv->minLinePointVector[1] + gv->minLinePointVector[2] * gv->minLinePointVector[2]);
						}
						else {
							gv->minLinePointVector[0] = startX - gv->grippers[k].offsets[0];
							gv->minLinePointVector[1] = startY - gv->grippers[k].offsets[1];
							gv->minLinePointVector[2] = startZ - gv->grippers[k].offsets[2];
							gv->minDistValueLine = sqrt(gv->minLinePointVector[0] * gv->minLinePointVector[0] + gv->minLinePointVector[1] * gv->minLinePointVector[1] + gv->minLinePointVector[2] * gv->minLinePointVector[2]);
						}

						if (gv->minDistValueLine <= gv->grippers[k].maxSize) {
							gv->intersectingBoundingBox = false;

							gv->surfaceTval = (gv->grippers[k].limits[0] - startX) / (endX - startX);
							if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
								if ((endY - startY) * gv->surfaceTval + startY <= gv->grippers[k].limits[2] && (endY - startY) * gv->surfaceTval + startY >= gv->grippers[k].limits[3] && (endZ - startZ) * gv->surfaceTval + startZ <= gv->grippers[k].limits[4] && (endZ - startZ) * gv->surfaceTval + startZ >= gv->grippers[k].limits[5]) {
									gv->intersectingBoundingBox = true;
								}
							}
							if (!gv->intersectingBoundingBox) {
								gv->surfaceTval = (gv->grippers[k].limits[1] - startX) / (endX - startX);
								if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
									if ((endY - startY) * gv->surfaceTval + startY <= gv->grippers[k].limits[2] && (endY - startY) * gv->surfaceTval + startY >= gv->grippers[k].limits[3] && (endZ - startZ) * gv->surfaceTval + startZ <= gv->grippers[k].limits[4] && (endZ - startZ) * gv->surfaceTval + startZ >= gv->grippers[k].limits[5]) {
										gv->intersectingBoundingBox = true;
									}
								}
							}
							if (!gv->intersectingBoundingBox) {
								gv->surfaceTval = (gv->grippers[k].limits[2] - startY) / (endY - startY);
								if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
									if ((endX - startX) * gv->surfaceTval + startX <= gv->grippers[k].limits[0] && (endX - startX) * gv->surfaceTval + startX >= gv->grippers[k].limits[1] && (endZ - startZ) * gv->surfaceTval + startZ <= gv->grippers[k].limits[4] && (endZ - startZ) * gv->surfaceTval + startZ >= gv->grippers[k].limits[5]) {
										gv->intersectingBoundingBox = true;
									}
								}
							}
							if (!gv->intersectingBoundingBox) {
								gv->surfaceTval = (gv->grippers[k].limits[3] - startY) / (endY - startY);
								if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
									if ((endX - startX) * gv->surfaceTval + startX <= gv->grippers[k].limits[0] && (endX - startX) * gv->surfaceTval + startX >= gv->grippers[k].limits[1] && (endZ - startZ) * gv->surfaceTval + startZ <= gv->grippers[k].limits[4] && (endZ - startZ) * gv->surfaceTval + startZ >= gv->grippers[k].limits[5]) {
										gv->intersectingBoundingBox = true;
									}
								}
							}
							if (!gv->intersectingBoundingBox) {
								gv->surfaceTval = (gv->grippers[k].limits[4] - startZ) / (endZ - startZ);
								if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
									if ((endX - startX) * gv->surfaceTval + startX <= gv->grippers[k].limits[0] && (endX - startX) * gv->surfaceTval + startX >= gv->grippers[k].limits[1] && (endY - startY) * gv->surfaceTval + startY <= gv->grippers[k].limits[2] && (endY - startY) * gv->surfaceTval + startY >= gv->grippers[k].limits[3]) {
										gv->intersectingBoundingBox = true;
									}
								}
							}
							if (!gv->intersectingBoundingBox) {
								gv->surfaceTval = (gv->grippers[k].limits[5] - startZ) / (endZ - startZ);
								if (gv->surfaceTval >= 0.0f && gv->surfaceTval <= 1.0f) {
									if ((endX - startX) * gv->surfaceTval + startX <= gv->grippers[k].limits[0] && (endX - startX) * gv->surfaceTval + startX >= gv->grippers[k].limits[1] && (endY - startY) * gv->surfaceTval + startY <= gv->grippers[k].limits[2] && (endY - startY) * gv->surfaceTval + startY >= gv->grippers[k].limits[3]) {
										gv->intersectingBoundingBox = true;
									}
								}
							}

							if (gv->intersectingBoundingBox) {
								for (int i = 0; i < gv->grippers[k].facesTimes3; i++) {
									if (i % 3 == 0) {
										gv->surfaceO[0] = gv->grippers[k].x_coll[gv->grippers[k].connections[i]];
										gv->surfaceO[1] = gv->grippers[k].y_coll[gv->grippers[k].connections[i]];
										gv->surfaceO[2] = gv->grippers[k].z_coll[gv->grippers[k].connections[i]];

										gv->surfaceV1[0] = gv->grippers[k].x_coll[gv->grippers[k].connections[i + 1]] - gv->surfaceO[0];
										gv->surfaceV1[1] = gv->grippers[k].y_coll[gv->grippers[k].connections[i + 1]] - gv->surfaceO[1];
										gv->surfaceV1[2] = gv->grippers[k].z_coll[gv->grippers[k].connections[i + 1]] - gv->surfaceO[2];

										gv->surfaceV2[0] = gv->grippers[k].x_coll[gv->grippers[k].connections[i + 2]] - gv->surfaceO[0];
										gv->surfaceV2[1] = gv->grippers[k].y_coll[gv->grippers[k].connections[i + 2]] - gv->surfaceO[1];
										gv->surfaceV2[2] = gv->grippers[k].z_coll[gv->grippers[k].connections[i + 2]] - gv->surfaceO[2];

										gv->surfaceNormal[0] = gv->surfaceV1[1] * gv->surfaceV2[2] - gv->surfaceV1[2] * gv->surfaceV2[1];
										gv->surfaceNormal[1] = gv->surfaceV1[2] * gv->surfaceV2[0] - gv->surfaceV1[0] * gv->surfaceV2[2];
										gv->surfaceNormal[2] = gv->surfaceV1[0] * gv->surfaceV2[1] - gv->surfaceV1[1] * gv->surfaceV2[0];
										gv->surfaceDval = gv->surfaceNormal[0] * gv->surfaceO[0] + gv->surfaceNormal[1] * gv->surfaceO[1] + gv->surfaceNormal[2] * gv->surfaceO[2];

										if (abs(gv->surfaceNormal[0] * (endX - startX) + gv->surfaceNormal[1] * (endY - startY) + gv->surfaceNormal[2] * (endZ - startZ)) > cd->epsilon3) {
											gv->surfaceTval = (gv->surfaceDval - gv->surfaceNormal[0] * startX - gv->surfaceNormal[1] * startY - gv->surfaceNormal[2] * startZ) / (gv->surfaceNormal[0] * (endX - startX) + gv->surfaceNormal[1] * (endY - startY) + gv->surfaceNormal[2] * (endZ - startZ));
											if (gv->surfaceTval <= 1.0f && gv->surfaceTval >= 0.0f) {
												gv->surfaceV[0] = startX + (endX - startX) * gv->surfaceTval;
												gv->surfaceV[1] = startY + (endY - startY) * gv->surfaceTval;
												gv->surfaceV[2] = startZ + (endZ - startZ) * gv->surfaceTval;

												gv->surfaceN0 = gv->surfaceV2[0] * gv->surfaceV1[1] - gv->surfaceV1[0] * gv->surfaceV2[1];
												gv->surfaceN1 = gv->surfaceV2[0] * gv->surfaceV1[2] - gv->surfaceV1[0] * gv->surfaceV2[2];
												gv->surfaceN2 = gv->surfaceV2[1] * gv->surfaceV1[2] - gv->surfaceV1[1] * gv->surfaceV2[2];

												if (abs(gv->surfaceN0) > cd->epsilon4) {
													gv->surfaceA = (gv->surfaceV2[0] * (gv->surfaceV[1] - gv->surfaceO[1]) - gv->surfaceV2[1] * (gv->surfaceV[0] - gv->surfaceO[0])) / gv->surfaceN0;
													gv->surfaceB = (gv->surfaceV1[1] * (gv->surfaceV[0] - gv->surfaceO[0]) - gv->surfaceV1[0] * (gv->surfaceV[1] - gv->surfaceO[1])) / gv->surfaceN0;
													if (gv->surfaceA >= 0.0f && gv->surfaceB >= 0.0f && gv->surfaceA + gv->surfaceB <= 1.0f) {
														gv->freeSight = false;
														break;
													}
												}
												else if (abs(gv->surfaceN1) > cd->epsilon4) {
													gv->surfaceA = (gv->surfaceV2[0] * (gv->surfaceV[2] - gv->surfaceO[2]) - gv->surfaceV2[2] * (gv->surfaceV[0] - gv->surfaceO[0])) / gv->surfaceN1;
													gv->surfaceB = (gv->surfaceV1[2] * (gv->surfaceV[0] - gv->surfaceO[0]) - gv->surfaceV1[0] * (gv->surfaceV[2] - gv->surfaceO[2])) / gv->surfaceN1;
													if (gv->surfaceA >= 0.0f && gv->surfaceB >= 0.0f && gv->surfaceA + gv->surfaceB <= 1.0f) {
														gv->freeSight = false;
														break;
													}
												}
												else if (abs(gv->surfaceN2) > cd->epsilon4) {
													gv->surfaceA = (gv->surfaceV2[1] * (gv->surfaceV[2] - gv->surfaceO[2]) - gv->surfaceV2[2] * (gv->surfaceV[1] - gv->surfaceO[1])) / gv->surfaceN2;
													gv->surfaceB = (gv->surfaceV1[2] * (gv->surfaceV[1] - gv->surfaceO[1]) - gv->surfaceV1[1] * (gv->surfaceV[2] - gv->surfaceO[2])) / gv->surfaceN2;
													if (gv->surfaceA >= 0.0f && gv->surfaceB >= 0.0f && gv->surfaceA + gv->surfaceB <= 1.0f) {
														gv->freeSight = false;
														break;
													}
												}
											}
										}
									}
								}
								if (!gv->freeSight) break;
							}
						}
					}
				}
				return gv->freeSight;
			}
			else {
				return false;
			}
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

int MonitoringToolModels::isObjectVisible(int colIndex, int mtIndex) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();

	if ((mtIndex == 1 && !gv->MT1currentlyActive) || (mtIndex == 2 && !gv->MT2currentlyActive)){
		return -1;
	}
	else if (gv->colliders[colIndex].offsets[0] > 45.0f || gv->colliders[colIndex].offsets[0] < -45.0f || gv->colliders[colIndex].offsets[1] > 45.0f) {
		return -1;
	}
	else {
		if (mtIndex == 1) {
			for (int i = 0; i < gv->colliders[colIndex].patternCount * 4; i += 4) {
				gv->patternNx = (gv->colliders[colIndex].y_pattern_tf[i + 1] - gv->colliders[colIndex].y_pattern_tf[i]) * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]) - (gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) * (gv->colliders[colIndex].y_pattern_tf[i + 3] - gv->colliders[colIndex].y_pattern_tf[i]);
				gv->patternNy = (gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) * (gv->colliders[colIndex].x_pattern_tf[i + 3] - gv->colliders[colIndex].x_pattern_tf[i]) - (gv->colliders[colIndex].x_pattern_tf[i + 1] - gv->colliders[colIndex].x_pattern_tf[i]) * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]);
				gv->patternNz = (gv->colliders[colIndex].x_pattern_tf[i + 1] - gv->colliders[colIndex].x_pattern_tf[i]) * (gv->colliders[colIndex].y_pattern_tf[i + 3] - gv->colliders[colIndex].y_pattern_tf[i]) - (gv->colliders[colIndex].y_pattern_tf[i + 1] - gv->colliders[colIndex].y_pattern_tf[i]) * (gv->colliders[colIndex].x_pattern_tf[i + 3] - gv->colliders[colIndex].x_pattern_tf[i]);

				gv->patternNx /= (cd->patternSize * cd->patternSize);
				gv->patternNy /= (cd->patternSize * cd->patternSize);
				gv->patternNz /= (cd->patternSize * cd->patternSize);

				if (abs(gv->patternNz) < cd->epsilon3 && abs((gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) / cd->patternSize * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]) / cd->patternSize) <= cd->patternMaxTurnVal) {
					gv->centerRay[0] = 0.25f * (gv->colliders[colIndex].x_pattern_tf[i] + gv->colliders[colIndex].x_pattern_tf[i + 1] + gv->colliders[colIndex].x_pattern_tf[i + 2] + gv->colliders[colIndex].x_pattern_tf[i + 3]);
					gv->centerRay[1] = 0.25f * (gv->colliders[colIndex].y_pattern_tf[i] + gv->colliders[colIndex].y_pattern_tf[i + 1] + gv->colliders[colIndex].y_pattern_tf[i + 2] + gv->colliders[colIndex].y_pattern_tf[i + 3]);
					gv->centerRay[2] = 0.25f * (gv->colliders[colIndex].z_pattern_tf[i] + gv->colliders[colIndex].z_pattern_tf[i + 1] + gv->colliders[colIndex].z_pattern_tf[i + 2] + gv->colliders[colIndex].z_pattern_tf[i + 3]);
					gv->centerRay[0] = -gv->centerRay[0] + gv->camVectorsMT1[0]; gv->centerRay[1] = -gv->centerRay[1] + gv->camVectorsMT1[1]; gv->centerRay[2] = -gv->centerRay[2] + gv->camVectorsMT1[2];
					gv->centerRayLen = sqrt(gv->centerRay[0] * gv->centerRay[0] + gv->centerRay[1] * gv->centerRay[1] + gv->centerRay[2] * gv->centerRay[2]);

					if (gv->centerRayLen > cd->epsilon3) {
						gv->patternAngle = abs(acos(gv->patternNx * gv->centerRay[0] / gv->centerRayLen + gv->patternNy * gv->centerRay[1] / gv->centerRayLen + gv->patternNz * gv->centerRay[2] / gv->centerRayLen) * 180.0f / M_PI);

						if (gv->patternAngle <= cd->patternMaxAngle) {
							if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], gv->colliders[colIndex].x_pattern_tf[i], gv->colliders[colIndex].y_pattern_tf[i], gv->colliders[colIndex].z_pattern_tf[i], colIndex, 15, gv->camVectorsMT1) == true) {
								if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], gv->colliders[colIndex].x_pattern_tf[i + 1], gv->colliders[colIndex].y_pattern_tf[i + 1], gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 15, gv->camVectorsMT1) == true) {
									if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], gv->colliders[colIndex].x_pattern_tf[i + 2], gv->colliders[colIndex].y_pattern_tf[i + 2], gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 15, gv->camVectorsMT1) == true) {
										if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], gv->colliders[colIndex].x_pattern_tf[i + 3], gv->colliders[colIndex].y_pattern_tf[i + 3], gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 15, gv->camVectorsMT1) == true) {

											if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 15, gv->camVectorsMT1) == true) {
												if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 15, gv->camVectorsMT1) == true) {
													if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 15, gv->camVectorsMT1) == true) {
														if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i], colIndex, 15, gv->camVectorsMT1) == true) {

															if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 15, gv->camVectorsMT1) == true) {
																if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 15, gv->camVectorsMT1) == true) {
																	if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 15, gv->camVectorsMT1) == true) {
																		if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i], colIndex, 15, gv->camVectorsMT1) == true) {
																			return i;
																		}
																	}
																}
															}

														}
													}
												}
											}

										}
									}
								}
							}
						}
					}
				}
			}
			return -1;
		}
		else {
			for (int i = 0; i < gv->colliders[colIndex].patternCount * 4; i += 4) {
				gv->patternNx = (gv->colliders[colIndex].y_pattern_tf[i + 1] - gv->colliders[colIndex].y_pattern_tf[i]) * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]) - (gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) * (gv->colliders[colIndex].y_pattern_tf[i + 3] - gv->colliders[colIndex].y_pattern_tf[i]);
				gv->patternNy = (gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) * (gv->colliders[colIndex].x_pattern_tf[i + 3] - gv->colliders[colIndex].x_pattern_tf[i]) - (gv->colliders[colIndex].x_pattern_tf[i + 1] - gv->colliders[colIndex].x_pattern_tf[i]) * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]);
				gv->patternNz = (gv->colliders[colIndex].x_pattern_tf[i + 1] - gv->colliders[colIndex].x_pattern_tf[i]) * (gv->colliders[colIndex].y_pattern_tf[i + 3] - gv->colliders[colIndex].y_pattern_tf[i]) - (gv->colliders[colIndex].y_pattern_tf[i + 1] - gv->colliders[colIndex].y_pattern_tf[i]) * (gv->colliders[colIndex].x_pattern_tf[i + 3] - gv->colliders[colIndex].x_pattern_tf[i]);

				gv->patternNx /= (cd->patternSize * cd->patternSize);
				gv->patternNy /= (cd->patternSize * cd->patternSize);
				gv->patternNz /= (cd->patternSize * cd->patternSize);

				if (abs(gv->patternNz) < cd->epsilon3 && abs((gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) / cd->patternSize * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]) / cd->patternSize) <= cd->patternMaxTurnVal) {
					gv->centerRay[0] = 0.25f * (gv->colliders[colIndex].x_pattern_tf[i] + gv->colliders[colIndex].x_pattern_tf[i + 1] + gv->colliders[colIndex].x_pattern_tf[i + 2] + gv->colliders[colIndex].x_pattern_tf[i + 3]);
					gv->centerRay[1] = 0.25f * (gv->colliders[colIndex].y_pattern_tf[i] + gv->colliders[colIndex].y_pattern_tf[i + 1] + gv->colliders[colIndex].y_pattern_tf[i + 2] + gv->colliders[colIndex].y_pattern_tf[i + 3]);
					gv->centerRay[2] = 0.25f * (gv->colliders[colIndex].z_pattern_tf[i] + gv->colliders[colIndex].z_pattern_tf[i + 1] + gv->colliders[colIndex].z_pattern_tf[i + 2] + gv->colliders[colIndex].z_pattern_tf[i + 3]);
					gv->centerRay[0] = -gv->centerRay[0] + gv->camVectorsMT2[0]; gv->centerRay[1] = -gv->centerRay[1] + gv->camVectorsMT2[1]; gv->centerRay[2] = -gv->centerRay[2] + gv->camVectorsMT2[2];
					gv->centerRayLen = sqrt(gv->centerRay[0] * gv->centerRay[0] + gv->centerRay[1] * gv->centerRay[1] + gv->centerRay[2] * gv->centerRay[2]);

					if (gv->centerRayLen > cd->epsilon3) {
						gv->patternAngle = abs(acos(gv->patternNx * gv->centerRay[0] / gv->centerRayLen + gv->patternNy * gv->centerRay[1] / gv->centerRayLen + gv->patternNz * gv->centerRay[2] / gv->centerRayLen) * 180.0f / M_PI);

						if (gv->patternAngle <= cd->patternMaxAngle) {
							if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], gv->colliders[colIndex].x_pattern_tf[i], gv->colliders[colIndex].y_pattern_tf[i], gv->colliders[colIndex].z_pattern_tf[i], colIndex, 16, gv->camVectorsMT2) == true) {
								if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], gv->colliders[colIndex].x_pattern_tf[i + 1], gv->colliders[colIndex].y_pattern_tf[i + 1], gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 16, gv->camVectorsMT2) == true) {
									if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], gv->colliders[colIndex].x_pattern_tf[i + 2], gv->colliders[colIndex].y_pattern_tf[i + 2], gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 16, gv->camVectorsMT2) == true) {
										if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], gv->colliders[colIndex].x_pattern_tf[i + 3], gv->colliders[colIndex].y_pattern_tf[i + 3], gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 16, gv->camVectorsMT2) == true) {

											if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 16, gv->camVectorsMT2) == true) {
												if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 16, gv->camVectorsMT2) == true) {
													if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 16, gv->camVectorsMT2) == true) {
														if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i], colIndex, 16, gv->camVectorsMT2) == true) {

															if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 16, gv->camVectorsMT2) == true) {
																if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 16, gv->camVectorsMT2) == true) {
																	if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 16, gv->camVectorsMT2) == true) {
																		if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i], colIndex, 16, gv->camVectorsMT2) == true) {
																			return i;
																		}
																	}
																}
															}

														}
													}
												}
											}

										}
									}
								}
							}
						}
					}
				}
			}
			return -1;
		}
	}
}



bool MonitoringToolModels::isPatternVisible(int patternIndex, int mtIndex, bool detectionMode) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	GlobalVariables *gv = GlobalVariables::get_instance();
	if ((mtIndex == 1 && !gv->MT1currentlyActive) || (mtIndex == 2 && !gv->MT2currentlyActive)) return false;

	int colIndex = objectIndicesFromPattern[patternIndex];

	if (!detectionMode) {
		gv->objectInOff = gv->colliders[colIndex].offsets[0] > 45.0f || gv->colliders[colIndex].offsets[0] < -45.0f || gv->colliders[colIndex].offsets[1] > 45.0f;
	}
	else {
		gv->objectInOff = false;
	}

	if (gv->objectInOff) {
		return false;
	}
	else {
		int i = patternIndicesFromPattern[patternIndex];

		if (mtIndex == 1) {
			gv->patternNx = (gv->colliders[colIndex].y_pattern_tf[i + 1] - gv->colliders[colIndex].y_pattern_tf[i]) * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]) - (gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) * (gv->colliders[colIndex].y_pattern_tf[i + 3] - gv->colliders[colIndex].y_pattern_tf[i]);
			gv->patternNy = (gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) * (gv->colliders[colIndex].x_pattern_tf[i + 3] - gv->colliders[colIndex].x_pattern_tf[i]) - (gv->colliders[colIndex].x_pattern_tf[i + 1] - gv->colliders[colIndex].x_pattern_tf[i]) * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]);
			gv->patternNz = (gv->colliders[colIndex].x_pattern_tf[i + 1] - gv->colliders[colIndex].x_pattern_tf[i]) * (gv->colliders[colIndex].y_pattern_tf[i + 3] - gv->colliders[colIndex].y_pattern_tf[i]) - (gv->colliders[colIndex].y_pattern_tf[i + 1] - gv->colliders[colIndex].y_pattern_tf[i]) * (gv->colliders[colIndex].x_pattern_tf[i + 3] - gv->colliders[colIndex].x_pattern_tf[i]);

			gv->patternNx /= (cd->patternSize * cd->patternSize);
			gv->patternNy /= (cd->patternSize * cd->patternSize);
			gv->patternNz /= (cd->patternSize * cd->patternSize);

			if (abs(gv->patternNz) < cd->epsilon3 && abs((gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) / cd->patternSize * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]) / cd->patternSize) <= cd->patternMaxTurnVal) {
				gv->centerRay[0] = 0.25f * (gv->colliders[colIndex].x_pattern_tf[i] + gv->colliders[colIndex].x_pattern_tf[i + 1] + gv->colliders[colIndex].x_pattern_tf[i + 2] + gv->colliders[colIndex].x_pattern_tf[i + 3]);
				gv->centerRay[1] = 0.25f * (gv->colliders[colIndex].y_pattern_tf[i] + gv->colliders[colIndex].y_pattern_tf[i + 1] + gv->colliders[colIndex].y_pattern_tf[i + 2] + gv->colliders[colIndex].y_pattern_tf[i + 3]);
				gv->centerRay[2] = 0.25f * (gv->colliders[colIndex].z_pattern_tf[i] + gv->colliders[colIndex].z_pattern_tf[i + 1] + gv->colliders[colIndex].z_pattern_tf[i + 2] + gv->colliders[colIndex].z_pattern_tf[i + 3]);
				gv->centerRay[0] = -gv->centerRay[0] + gv->camVectorsMT1[0]; gv->centerRay[1] = -gv->centerRay[1] + gv->camVectorsMT1[1]; gv->centerRay[2] = -gv->centerRay[2] + gv->camVectorsMT1[2];
				gv->centerRayLen = sqrt(gv->centerRay[0] * gv->centerRay[0] + gv->centerRay[1] * gv->centerRay[1] + gv->centerRay[2] * gv->centerRay[2]);

				if (gv->centerRayLen > cd->epsilon3) {
					gv->patternAngle = abs(acos(gv->patternNx * gv->centerRay[0] / gv->centerRayLen + gv->patternNy * gv->centerRay[1] / gv->centerRayLen + gv->patternNz * gv->centerRay[2] / gv->centerRayLen) * 180.0f / M_PI);

					if (gv->patternAngle <= cd->patternMaxAngle) {
						if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], gv->colliders[colIndex].x_pattern_tf[i], gv->colliders[colIndex].y_pattern_tf[i], gv->colliders[colIndex].z_pattern_tf[i], colIndex, 15, gv->camVectorsMT1) == true) {
							if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], gv->colliders[colIndex].x_pattern_tf[i + 1], gv->colliders[colIndex].y_pattern_tf[i + 1], gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 15, gv->camVectorsMT1) == true) {
								if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], gv->colliders[colIndex].x_pattern_tf[i + 2], gv->colliders[colIndex].y_pattern_tf[i + 2], gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 15, gv->camVectorsMT1) == true) {
									if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], gv->colliders[colIndex].x_pattern_tf[i + 3], gv->colliders[colIndex].y_pattern_tf[i + 3], gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 15, gv->camVectorsMT1) == true) {

										if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 15, gv->camVectorsMT1) == true) {
											if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 15, gv->camVectorsMT1) == true) {
												if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 15, gv->camVectorsMT1) == true) {
													if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i], colIndex, 15, gv->camVectorsMT1) == true) {

														if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 15, gv->camVectorsMT1) == true) {
															if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 15, gv->camVectorsMT1) == true) {
																if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 15, gv->camVectorsMT1) == true) {
																	if (checkForFreeSight(gv->camVectorsMT1[0], gv->camVectorsMT1[1], gv->camVectorsMT1[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i], colIndex, 15, gv->camVectorsMT1) == true) {
																		return true;
																	}
																}
															}
														}

													}
												}
											}
										}

									}
								}
							}
						}
					}
				}
			}
			return false;
		}
		else {
			gv->patternNx = (gv->colliders[colIndex].y_pattern_tf[i + 1] - gv->colliders[colIndex].y_pattern_tf[i]) * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]) - (gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) * (gv->colliders[colIndex].y_pattern_tf[i + 3] - gv->colliders[colIndex].y_pattern_tf[i]);
			gv->patternNy = (gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) * (gv->colliders[colIndex].x_pattern_tf[i + 3] - gv->colliders[colIndex].x_pattern_tf[i]) - (gv->colliders[colIndex].x_pattern_tf[i + 1] - gv->colliders[colIndex].x_pattern_tf[i]) * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]);
			gv->patternNz = (gv->colliders[colIndex].x_pattern_tf[i + 1] - gv->colliders[colIndex].x_pattern_tf[i]) * (gv->colliders[colIndex].y_pattern_tf[i + 3] - gv->colliders[colIndex].y_pattern_tf[i]) - (gv->colliders[colIndex].y_pattern_tf[i + 1] - gv->colliders[colIndex].y_pattern_tf[i]) * (gv->colliders[colIndex].x_pattern_tf[i + 3] - gv->colliders[colIndex].x_pattern_tf[i]);

			gv->patternNx /= (cd->patternSize * cd->patternSize);
			gv->patternNy /= (cd->patternSize * cd->patternSize);
			gv->patternNz /= (cd->patternSize * cd->patternSize);

			if (abs(gv->patternNz) < cd->epsilon3 && abs((gv->colliders[colIndex].z_pattern_tf[i + 1] - gv->colliders[colIndex].z_pattern_tf[i]) / cd->patternSize * (gv->colliders[colIndex].z_pattern_tf[i + 3] - gv->colliders[colIndex].z_pattern_tf[i]) / cd->patternSize) <= cd->patternMaxTurnVal) {
				gv->centerRay[0] = 0.25f * (gv->colliders[colIndex].x_pattern_tf[i] + gv->colliders[colIndex].x_pattern_tf[i + 1] + gv->colliders[colIndex].x_pattern_tf[i + 2] + gv->colliders[colIndex].x_pattern_tf[i + 3]);
				gv->centerRay[1] = 0.25f * (gv->colliders[colIndex].y_pattern_tf[i] + gv->colliders[colIndex].y_pattern_tf[i + 1] + gv->colliders[colIndex].y_pattern_tf[i + 2] + gv->colliders[colIndex].y_pattern_tf[i + 3]);
				gv->centerRay[2] = 0.25f * (gv->colliders[colIndex].z_pattern_tf[i] + gv->colliders[colIndex].z_pattern_tf[i + 1] + gv->colliders[colIndex].z_pattern_tf[i + 2] + gv->colliders[colIndex].z_pattern_tf[i + 3]);
				gv->centerRay[0] = -gv->centerRay[0] + gv->camVectorsMT2[0]; gv->centerRay[1] = -gv->centerRay[1] + gv->camVectorsMT2[1]; gv->centerRay[2] = -gv->centerRay[2] + gv->camVectorsMT2[2];
				gv->centerRayLen = sqrt(gv->centerRay[0] * gv->centerRay[0] + gv->centerRay[1] * gv->centerRay[1] + gv->centerRay[2] * gv->centerRay[2]);

				if (gv->centerRayLen > cd->epsilon3) {
					gv->patternAngle = abs(acos(gv->patternNx * gv->centerRay[0] / gv->centerRayLen + gv->patternNy * gv->centerRay[1] / gv->centerRayLen + gv->patternNz * gv->centerRay[2] / gv->centerRayLen) * 180.0f / M_PI);

					if (gv->patternAngle <= cd->patternMaxAngle) {
						if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], gv->colliders[colIndex].x_pattern_tf[i], gv->colliders[colIndex].y_pattern_tf[i], gv->colliders[colIndex].z_pattern_tf[i], colIndex, 16, gv->camVectorsMT2) == true) {
							if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], gv->colliders[colIndex].x_pattern_tf[i + 1], gv->colliders[colIndex].y_pattern_tf[i + 1], gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 16, gv->camVectorsMT2) == true) {
								if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], gv->colliders[colIndex].x_pattern_tf[i + 2], gv->colliders[colIndex].y_pattern_tf[i + 2], gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 16, gv->camVectorsMT2) == true) {
									if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], gv->colliders[colIndex].x_pattern_tf[i + 3], gv->colliders[colIndex].y_pattern_tf[i + 3], gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 16, gv->camVectorsMT2) == true) {

										if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 16, gv->camVectorsMT2) == true) {
											if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 16, gv->camVectorsMT2) == true) {
												if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 16, gv->camVectorsMT2) == true) {
													if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i], 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i], 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3] + 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i], colIndex, 16, gv->camVectorsMT2) == true) {

														if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1], colIndex, 16, gv->camVectorsMT2) == true) {
															if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 1] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2], colIndex, 16, gv->camVectorsMT2) == true) {
																if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 2] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3], colIndex, 16, gv->camVectorsMT2) == true) {
																	if (checkForFreeSight(gv->camVectorsMT2[0], gv->camVectorsMT2[1], gv->camVectorsMT2[2], 2.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].x_pattern_tf[i], 2.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].y_pattern_tf[i], 2.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i + 3] + 1.0f / 3.0f * gv->colliders[colIndex].z_pattern_tf[i], colIndex, 16, gv->camVectorsMT2) == true) {
																		return true;
																	}
																}
															}
														}

													}
												}
											}
										}

									}
								}
							}
						}
					}
				}
			}
			return false;
		}
	}
}
