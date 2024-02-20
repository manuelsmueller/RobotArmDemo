/*
 * HelperFunctions.cpp
 *
 *  Created on: 05.12.2022
 *      Author: manuel
 */

#include "HelperFunctions.h"

HelperFunctions::HelperFunctions() {
	gv = GlobalVariables::get_instance();
	pp = PathPlanner::get_instance();
}

HelperFunctions::~HelperFunctions() {
}

/*
 * singelton pattern
 */
HelperFunctions* HelperFunctions::hf = 0;
HelperFunctions* HelperFunctions::get_instance(){
	static bool isInit=false;

//	InverseKinematic *ik;
	if(!isInit){
		hf  = new HelperFunctions();
		isInit=true;
	}
	return hf;
}



void HelperFunctions::getColorIndex(int edgeIndex) {
	for (int i = 0; i < gv->colorsCount; i++) {
		if (edgeIndex >= gv->colorStartPoints[i]) {
			gv->colorIndex = i;
		}
		else {
			break;
		}
	}
}

void HelperFunctions::calcDisplayPos(float xPos, float yPos, float zPos) {
	gv->xDiffDisp = (xPos - gv->camPos[0]) / 5.0f;
	gv->yDiffDisp = (yPos - gv->camPos[1]) / 5.0f;
	gv->zDiffDisp = (zPos - gv->camPos[2]) / 5.0f;

	gv->xDisp = gv->xDiffDisp * gv->camDir[0] + gv->yDiffDisp * gv->camDir[1] + gv->zDiffDisp * gv->camDir[2];
	gv->yDisp = gv->xDiffDisp * gv->camHorDir[0] + gv->yDiffDisp * gv->camHorDir[1];
	gv->zDisp = gv->xDiffDisp * gv->camVerDir[0] + gv->yDiffDisp * gv->camVerDir[1] + gv->zDiffDisp * gv->camVerDir[2];

	gv->displayPos[0] = gv->yDisp / gv->xDisp;
	gv->displayPos[1] = gv->zDisp / gv->xDisp;
	gv->displayPos[2] = ((50.0f + 0.1f) / (50.0f - 0.1f) * gv->xDisp - 2 * 50.0f * 0.1f / (50.0f - 0.1f)) / gv->xDisp;
	gv->displayPos[3] = gv->xDisp;
}

void HelperFunctions::updateTransformationMatrix() {
	gv->camDir[0] = cos(gv->camAngles[0]) * cos(gv->camAngles[1]);
	gv->camDir[1] = sin(gv->camAngles[0]) * cos(gv->camAngles[1]);
	gv->camDir[2] = sin(gv->camAngles[1]);
	gv->camHorDir[0] = sin(gv->camAngles[0]);
	gv->camHorDir[1] = -cos(gv->camAngles[0]);
	gv->camVerDir[0] = -cos(gv->camAngles[0]) * sin(gv->camAngles[1]);
	gv->camVerDir[1] = -sin(gv->camAngles[0]) * sin(gv->camAngles[1]);
	gv->camVerDir[2] = cos(gv->camAngles[1]);
}

int HelperFunctions::calcEdgesForConvexHull(collider::Edge* edges, collider* col, int counter) {
	for (int i = 0; i < col->facesTimes3; i++) {
		edges[counter].x = col->x_coll[col->connections[i]];
		edges[counter].y = col->y_coll[col->connections[i]];
		edges[counter].z = col->z_coll[col->connections[i]];
		counter++;
	}
	return counter;
}

int HelperFunctions::calcEdgesForMesh(collider::Edge* edges, collider* col, int counter) {
	for (int i = 0; i < col->facesTimes3; i++) {
		if (i % 3 == 0) {
			edges[counter].x = col->x_coll[col->connections[i]]; edges[counter].y = col->y_coll[col->connections[i]]; edges[counter].z = col->z_coll[col->connections[i]];
			edges[counter + 1].x = col->x_coll[col->connections[i + 1]]; edges[counter + 1].y = col->y_coll[col->connections[i + 1]]; edges[counter + 1].z = col->z_coll[col->connections[i + 1]];
			edges[counter + 2].x = col->x_coll[col->connections[i + 1]]; edges[counter + 2].y = col->y_coll[col->connections[i + 1]]; edges[counter + 2].z = col->z_coll[col->connections[i + 1]];
			edges[counter + 3].x = col->x_coll[col->connections[i + 2]]; edges[counter + 3].y = col->y_coll[col->connections[i + 2]]; edges[counter + 3].z = col->z_coll[col->connections[i + 2]];
			edges[counter + 4].x = col->x_coll[col->connections[i + 2]]; edges[counter + 4].y = col->y_coll[col->connections[i + 2]]; edges[counter + 4].z = col->z_coll[col->connections[i + 2]];
			edges[counter + 5].x = col->x_coll[col->connections[i]]; edges[counter + 5].y = col->y_coll[col->connections[i]]; edges[counter + 5].z = col->z_coll[col->connections[i]];

			counter += 6;
		}
	}

	return counter;
}

int HelperFunctions::calcEdgesForPatterns(collider::Edge* edges, collider* col, int counter) {
	for (int i = 0; i < col->patternCount * 4; i++) {
		edges[counter].x = col->x_pattern_tf[i];
		edges[counter].y = col->y_pattern_tf[i];
		edges[counter].z = col->z_pattern_tf[i];
		counter++;
	}
	return counter;
}

void HelperFunctions::calcEdgesForWorkpieceArrows(collider::Edge* edges, int workpieceIndex) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	if (workpieceIndex == 1) {
		edges[0].x = cd->workpieceArrowPosition1[0]; edges[0].y = cd->workpieceArrowPosition1[1]; edges[0].z = cd->workpieceArrowPosition1[2];
		edges[1].x = cd->workpieceArrowPosition1[3]; edges[1].y = cd->workpieceArrowPosition1[4]; edges[1].z = cd->workpieceArrowPosition1[5];
		edges[2].x = cd->workpieceArrowPosition1[3]; edges[2].y = cd->workpieceArrowPosition1[4]; edges[2].z = cd->workpieceArrowPosition1[5];
		edges[3].x = cd->workpieceArrowPosition1[6]; edges[3].y = cd->workpieceArrowPosition1[7]; edges[3].z = cd->workpieceArrowPosition1[8];
		edges[4].x = cd->workpieceArrowPosition1[6]; edges[4].y = cd->workpieceArrowPosition1[7]; edges[4].z = cd->workpieceArrowPosition1[8];
		edges[5].x = cd->workpieceArrowPosition1[0]; edges[5].y = cd->workpieceArrowPosition1[1]; edges[5].z = cd->workpieceArrowPosition1[2];
	}
	else if (workpieceIndex == 2) {
		edges[6].x = cd->workpieceArrowPosition2[0]; edges[6].y = cd->workpieceArrowPosition2[1]; edges[6].z = cd->workpieceArrowPosition2[2];
		edges[7].x = cd->workpieceArrowPosition2[3]; edges[7].y = cd->workpieceArrowPosition2[4]; edges[7].z = cd->workpieceArrowPosition2[5];
		edges[8].x = cd->workpieceArrowPosition2[3]; edges[8].y = cd->workpieceArrowPosition2[4]; edges[8].z = cd->workpieceArrowPosition2[5];
		edges[9].x = cd->workpieceArrowPosition2[6]; edges[9].y = cd->workpieceArrowPosition2[7]; edges[9].z = cd->workpieceArrowPosition2[8];
		edges[10].x = cd->workpieceArrowPosition2[6]; edges[10].y = cd->workpieceArrowPosition2[7]; edges[10].z = cd->workpieceArrowPosition2[8];
		edges[11].x = cd->workpieceArrowPosition2[0]; edges[11].y = cd->workpieceArrowPosition2[1]; edges[11].z = cd->workpieceArrowPosition2[2];
	}
	else if (workpieceIndex == 3) {
		edges[12].x = cd->workpieceArrowPosition3[0]; edges[12].y = cd->workpieceArrowPosition3[1]; edges[12].z = cd->workpieceArrowPosition3[2];
		edges[13].x = cd->workpieceArrowPosition3[3]; edges[13].y = cd->workpieceArrowPosition3[4]; edges[13].z = cd->workpieceArrowPosition3[5];
		edges[14].x = cd->workpieceArrowPosition3[3]; edges[14].y = cd->workpieceArrowPosition3[4]; edges[14].z = cd->workpieceArrowPosition3[5];
		edges[15].x = cd->workpieceArrowPosition3[6]; edges[15].y = cd->workpieceArrowPosition3[7]; edges[15].z = cd->workpieceArrowPosition3[8];
		edges[16].x = cd->workpieceArrowPosition3[6]; edges[16].y = cd->workpieceArrowPosition3[7]; edges[16].z = cd->workpieceArrowPosition3[8];
		edges[17].x = cd->workpieceArrowPosition3[0]; edges[17].y = cd->workpieceArrowPosition3[1]; edges[17].z = cd->workpieceArrowPosition3[2];
	}
	else if (workpieceIndex == 4) {
		edges[18].x = cd->workpieceArrowPosition4[0]; edges[18].y = cd->workpieceArrowPosition4[1]; edges[18].z = cd->workpieceArrowPosition4[2];
		edges[19].x = cd->workpieceArrowPosition4[3]; edges[19].y = cd->workpieceArrowPosition4[4]; edges[19].z = cd->workpieceArrowPosition4[5];
		edges[20].x = cd->workpieceArrowPosition4[3]; edges[20].y = cd->workpieceArrowPosition4[4]; edges[20].z = cd->workpieceArrowPosition4[5];
		edges[21].x = cd->workpieceArrowPosition4[6]; edges[21].y = cd->workpieceArrowPosition4[7]; edges[21].z = cd->workpieceArrowPosition4[8];
		edges[22].x = cd->workpieceArrowPosition4[6]; edges[22].y = cd->workpieceArrowPosition4[7]; edges[22].z = cd->workpieceArrowPosition4[8];
		edges[23].x = cd->workpieceArrowPosition4[0]; edges[23].y = cd->workpieceArrowPosition4[1]; edges[23].z = cd->workpieceArrowPosition4[2];
	}
}

void HelperFunctions::calcEdgesForViewFunnel(collider::Edge* edges, int mtIndex){
	CollisionDetection *cd = CollisionDetection::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();


	if (mtIndex == 1) {
		if (gv->MT1currentlyActive) {
			edges[0].x = gv->camVectorsMT1[0]; edges[0].y = gv->camVectorsMT1[1]; edges[0].z = gv->camVectorsMT1[2];
			edges[1].x = gv->camVectorsMT1[0] + 3.0f * gv->camVectorsMT1[3] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[6] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[9];
			edges[1].y = gv->camVectorsMT1[1] + 3.0f * gv->camVectorsMT1[4] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[7] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[10];
			edges[1].z = gv->camVectorsMT1[2] + 3.0f * gv->camVectorsMT1[5] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[8] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[11];
			edges[2].x = gv->camVectorsMT1[0]; edges[2].y = gv->camVectorsMT1[1]; edges[2].z = gv->camVectorsMT1[2];
			edges[3].x = gv->camVectorsMT1[0] + 3.0f * gv->camVectorsMT1[3] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[6] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[9];
			edges[3].y = gv->camVectorsMT1[1] + 3.0f * gv->camVectorsMT1[4] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[7] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[10];
			edges[3].z = gv->camVectorsMT1[2] + 3.0f * gv->camVectorsMT1[5] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[8] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[11];
			edges[4].x = gv->camVectorsMT1[0]; edges[4].y = gv->camVectorsMT1[1]; edges[4].z = gv->camVectorsMT1[2];
			edges[5].x = gv->camVectorsMT1[0] + 3.0f * gv->camVectorsMT1[3] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[6] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[9];
			edges[5].y = gv->camVectorsMT1[1] + 3.0f * gv->camVectorsMT1[4] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[7] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[10];
			edges[5].z = gv->camVectorsMT1[2] + 3.0f * gv->camVectorsMT1[5] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[8] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[11];
			edges[6].x = gv->camVectorsMT1[0]; edges[6].y = gv->camVectorsMT1[1]; edges[6].z = gv->camVectorsMT1[2];
			edges[7].x = gv->camVectorsMT1[0] + 3.0f * gv->camVectorsMT1[3] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[6] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[9];
			edges[7].y = gv->camVectorsMT1[1] + 3.0f * gv->camVectorsMT1[4] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[7] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[10];
			edges[7].z = gv->camVectorsMT1[2] + 3.0f * gv->camVectorsMT1[5] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[8] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT1[11];

			edges[8].x = edges[1].x; edges[8].y = edges[1].y; edges[8].z = edges[1].z;
			edges[9].x = edges[3].x; edges[9].y = edges[3].y; edges[9].z = edges[3].z;
			edges[10].x = edges[5].x; edges[10].y = edges[5].y; edges[10].z = edges[5].z;
			edges[11].x = edges[3].x; edges[11].y = edges[3].y; edges[11].z = edges[3].z;
			edges[12].x = edges[5].x; edges[12].y = edges[5].y; edges[12].z = edges[5].z;
			edges[13].x = edges[7].x; edges[13].y = edges[7].y; edges[13].z = edges[7].z;
			edges[14].x = edges[1].x; edges[14].y = edges[1].y; edges[14].z = edges[1].z;
			edges[15].x = edges[7].x; edges[15].y = edges[7].y; edges[15].z = edges[7].z;
		}
		else {
			for (int i = 0; i < 16; i++) {
				edges[i].x = 0.0f; edges[i].y = 0.0f; edges[i].z = 10.0e5;
			}
		}
	}
	else {
		if (gv->MT2currentlyActive) {
			edges[16].x = gv->camVectorsMT2[0]; edges[16].y = gv->camVectorsMT2[1]; edges[16].z = gv->camVectorsMT2[2];
			edges[17].x = gv->camVectorsMT2[0] + 3.0f * gv->camVectorsMT2[3] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[6] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[9];
			edges[17].y = gv->camVectorsMT2[1] + 3.0f * gv->camVectorsMT2[4] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[7] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[10];
			edges[17].z = gv->camVectorsMT2[2] + 3.0f * gv->camVectorsMT2[5] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[8] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[11];
			edges[18].x = gv->camVectorsMT2[0]; edges[18].y = gv->camVectorsMT2[1]; edges[18].z = gv->camVectorsMT2[2];
			edges[19].x = gv->camVectorsMT2[0] + 3.0f * gv->camVectorsMT2[3] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[6] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[9];
			edges[19].y = gv->camVectorsMT2[1] + 3.0f * gv->camVectorsMT2[4] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[7] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[10];
			edges[19].z = gv->camVectorsMT2[2] + 3.0f * gv->camVectorsMT2[5] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[8] + tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[11];
			edges[20].x = gv->camVectorsMT2[0]; edges[20].y = gv->camVectorsMT2[1]; edges[20].z = gv->camVectorsMT2[2];
			edges[21].x = gv->camVectorsMT2[0] + 3.0f * gv->camVectorsMT2[3] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[6] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[9];
			edges[21].y = gv->camVectorsMT2[1] + 3.0f * gv->camVectorsMT2[4] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[7] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[10];
			edges[21].z = gv->camVectorsMT2[2] + 3.0f * gv->camVectorsMT2[5] - tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[8] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[11];
			edges[22].x = gv->camVectorsMT2[0]; edges[22].y = gv->camVectorsMT2[1]; edges[22].z = gv->camVectorsMT2[2];
			edges[23].x = gv->camVectorsMT2[0] + 3.0f * gv->camVectorsMT2[3] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[6] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[9];
			edges[23].y = gv->camVectorsMT2[1] + 3.0f * gv->camVectorsMT2[4] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[7] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[10];
			edges[23].z = gv->camVectorsMT2[2] + 3.0f * gv->camVectorsMT2[5] + tan(cd->vertViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[8] - tan(cd->horViewAngle / 360.0f * M_PI) * 3.0f * gv->camVectorsMT2[11];

			edges[24].x = edges[1 + 16].x; edges[24].y = edges[1 + 16].y; edges[24].z = edges[1 + 16].z;
			edges[25].x = edges[3 + 16].x; edges[25].y = edges[3 + 16].y; edges[25].z = edges[3 + 16].z;
			edges[26].x = edges[5 + 16].x; edges[26].y = edges[5 + 16].y; edges[26].z = edges[5 + 16].z;
			edges[27].x = edges[3 + 16].x; edges[27].y = edges[3 + 16].y; edges[27].z = edges[3 + 16].z;
			edges[28].x = edges[5 + 16].x; edges[28].y = edges[5 + 16].y; edges[28].z = edges[5 + 16].z;
			edges[29].x = edges[7 + 16].x; edges[29].y = edges[7 + 16].y; edges[29].z = edges[7 + 16].z;
			edges[30].x = edges[1 + 16].x; edges[30].y = edges[1 + 16].y; edges[30].z = edges[1 + 16].z;
			edges[31].x = edges[7 + 16].x; edges[31].y = edges[7 + 16].y; edges[31].z = edges[7 + 16].z;
		}
		else {
			for (int i = 16; i < 32; i++) {
				edges[i].x = 0.0f; edges[i].y = 0.0f; edges[i].z = 10.0e5;
			}
		}
	}
}

void HelperFunctions::calcVerticesForObjects(collider::Vertex* vertices, collider::Edge* edges, int numPoints, int excludeStart) {
	CollisionDetection *cd = CollisionDetection::get_instance();

	gv->pointValidCounter = 0;
	for (int i = 0; i < numPoints; i++) {
		if (excludeStart != -1 && i >= excludeStart && i < excludeStart + gv->colliders[0].facesTimes3) {
			vertices[i].x = 2.0f;
			vertices[i].y = 0.0f;
			vertices[i].z = 0.0f;
		}
		else {
			getColorIndex(i);
			vertices[i].r = gv->colors[gv->colorIndex].x;
			vertices[i].g = gv->colors[gv->colorIndex].y;
			vertices[i].b = gv->colors[gv->colorIndex].z;

			if (gv->colorIndex == 14 && !(abs(gv->currentDetectedObsPosX[0]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[0]) > cd->epsilon3)) {
				vertices[i].r *= 0.75f;
				vertices[i].g *= 0.75f;
				vertices[i].b *= 0.75f;
			}
			if (gv->colorIndex == 15 && !(abs(gv->currentDetectedObsPosX[1]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[1]) > cd->epsilon3)) {
				vertices[i].r *= 0.75f;
				vertices[i].g *= 0.75f;
				vertices[i].b *= 0.75f;
			}
			if (gv->colorIndex == 16 && !(abs(gv->currentDetectedObsPosX[2]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[2]) > cd->epsilon3)) {
				vertices[i].r *= 0.75f;
				vertices[i].g *= 0.75f;
				vertices[i].b *= 0.75f;
			}

			calcDisplayPos(edges[i].x, edges[i].y, edges[i].z);
			if (gv->displayPos[3] > 0.1f) {
				vertices[i].x = gv->displayPos[0];
				vertices[i].y = gv->displayPos[1];
				vertices[i].z = gv->displayPos[2];
				gv->pointValidCounter++;
			}

			if ((i + 1) % 3 == 0) {
				if (gv->pointValidCounter != 3) {
					for (int j = i - 2; j <= i; j++) {
						vertices[j].x = 2.0f;
						vertices[j].y = 0.0f;
						vertices[j].z = 0.0f;
					}
				}
				else {
					gv->cross1X = edges[i - 1].x - edges[i].x;
					gv->cross1Y = edges[i - 1].y - edges[i].y;
					gv->cross1Z = edges[i - 1].z - edges[i].z;

					gv->cross2X = edges[i - 2].x - edges[i].x;
					gv->cross2Y = edges[i - 2].y - edges[i].y;
					gv->cross2Z = edges[i - 2].z - edges[i].z;

					gv->normX = gv->cross1Y * gv->cross2Z - gv->cross1Z * gv->cross2Y;
					gv->normY = gv->cross1Z * gv->cross2X - gv->cross1X * gv->cross2Z;
					gv->normZ = gv->cross1X * gv->cross2Y - gv->cross1Y * gv->cross2X;

					gv->angleLight = acos((gv->normX * gv->lightDir[0] + gv->normY * gv->lightDir[1] + gv->normZ * gv->lightDir[2]) / sqrt(gv->normX * gv->normX + gv->normY * gv->normY + gv->normZ * gv->normZ));
					for (int j = i - 2; j <= i; j++) {
						vertices[j].r *= 1.0f - gv->angleLight / (2 * M_PI);
						vertices[j].g *= 1.0f - gv->angleLight / (2 * M_PI);
						vertices[j].b *= 1.0f - gv->angleLight / (2 * M_PI);
					}
				}
				gv->pointValidCounter = 0;
			}
		}
	}
}

void HelperFunctions::calcVerticesForLines(collider::Vertex* vertices, collider::Edge* edges, int numPoints) {
	gv->pointValidCounter = 0;
	for (int i = 0; i < numPoints; i++) {
		calcDisplayPos(edges[i].x, edges[i].y, edges[i].z);
		if (gv->displayPos[3] > 0.1f) {
			vertices[i].x = gv->displayPos[0];
			vertices[i].y = gv->displayPos[1];
			vertices[i].z = gv->displayPos[2];
			gv->pointValidCounter++;
		}

		if ((i + 1) % 2 == 0) {
			if (gv->pointValidCounter != 2) {
				for (int j = i - 1; j <= i; j++) {
					vertices[j].x = 2.0f;
					vertices[j].y = 0.0f;
					vertices[j].z = 0.0f;
				}
			}
			gv->pointValidCounter = 0;
		}
	}
}

void HelperFunctions::calcVerticesForPoints(collider::Vertex* vertices, collider::Edge* edges, int numPoints) {
	for (int i = 0; i < numPoints; i++) {
		calcDisplayPos(edges[i].x, edges[i].y, edges[i].z);
		if (gv->displayPos[3] > 0.1f) {
			vertices[i].x = gv->displayPos[0];
			vertices[i].y = gv->displayPos[1];
			vertices[i].z = gv->displayPos[2];
		}
		else {
			vertices[i].x = 2.0f;
			vertices[i].y = 0.0f;
			vertices[i].z = 0.0f;
		}
	}
}

bool HelperFunctions::checkStateChange() {
	CollisionDetection *cd = CollisionDetection::get_instance();
	if (abs(gv->lastDetectedObsPosX[0]) <= cd->epsilon3 && abs(gv->lastDetectedObsPosY[0]) <= cd->epsilon3 && (abs(gv->currentDetectedObsPosX[0]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[0]) > cd->epsilon3)) return true;
	if (abs(gv->lastDetectedObsPosX[1]) <= cd->epsilon3 && abs(gv->lastDetectedObsPosY[1]) <= cd->epsilon3 && (abs(gv->currentDetectedObsPosX[1]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[1]) > cd->epsilon3)) return true;
	if (abs(gv->lastDetectedObsPosX[2]) <= cd->epsilon3 && abs(gv->lastDetectedObsPosY[2]) <= cd->epsilon3 && (abs(gv->currentDetectedObsPosX[2]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[2]) > cd->epsilon3)) return true;
	if (abs(gv->currentDetectedObsPosX[0]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[0]) > cd->epsilon3) {
		if (abs(gv->currentDetectedObsPosX[0] - gv->lastDetectedObsPosX[0]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[0] - gv->lastDetectedObsPosY[0]) > cd->epsilon3 || abs(gv->currentDetectedObsPosA[0] - gv->lastDetectedObsPosA[0]) > cd->epsilon3) return true;
	}

	if (abs(gv->currentDetectedObsPosX[1]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[1]) > cd->epsilon3) {
		if (abs(gv->currentDetectedObsPosX[1] - gv->lastDetectedObsPosX[1]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[1] - gv->lastDetectedObsPosY[1]) > cd->epsilon3 || abs(gv->currentDetectedObsPosA[1] - gv->lastDetectedObsPosA[1]) > cd->epsilon3) return true;
	}

	if (abs(gv->currentDetectedObsPosX[2]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[2]) > cd->epsilon3) {
		if (abs(gv->currentDetectedObsPosX[2] - gv->lastDetectedObsPosX[2]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[2] - gv->lastDetectedObsPosY[2]) > cd->epsilon3 || abs(gv->currentDetectedObsPosA[2] - gv->lastDetectedObsPosA[2]) > cd->epsilon3) return true;
	}
	return false;
}

void HelperFunctions::openGLDebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam) {
	cout << "[OpenGL Error] " << message << endl;
}

void HelperFunctions::requestRealSystemState() {
//#if defined(WIN32)
	 sendMessageMT1("state_request");
	 sendMessageMT2("state_request");
	 cout << "debug: helperFunc " << __LINE__ <<endl;
	 sendMessageSTATE("state_request");

	thread mt1Thread(receiveMessageMT1);
	thread mt2Thread(receiveMessageMT2);
	thread stateThread(receiveMessageSTATE);

	mt1Thread.join();
	mt2Thread.join();
	stateThread.join();
//#endif
}


void HelperFunctions::storePathRelatedData(int storageIndex) {
	if (storageIndex == 0) {
		gv->storedPathRelatedData1[0] = float(gv->objectToTransport);
		gv->storedPathRelatedData1[1] = gv->objectOffset;
		gv->storedPathRelatedData1[2] = gv->objectHeight;
		gv->storedPathRelatedData1[3] = gv->objectGripDepth;

		gv->storedPathRelatedData1[4] = gv->pathStartPos.x;
		gv->storedPathRelatedData1[5] = gv->pathStartPos.y;
		gv->storedPathRelatedData1[6] = gv->pathStartPos.z;
		gv->storedPathRelatedData1[7] = gv->startObjectAngle;

		gv->storedPathRelatedData1[8] = gv->qValuesObjectStart[0];
		gv->storedPathRelatedData1[9] = gv->qValuesObjectStart[1];
		gv->storedPathRelatedData1[10] = gv->qValuesObjectStart[2];
		gv->storedPathRelatedData1[11] = gv->qValuesObjectStart[3];
		gv->storedPathRelatedData1[12] = gv->qValuesObjectStart[4];

		gv->storedPathRelatedData1[13] = gv->pathEndPos.x;
		gv->storedPathRelatedData1[14] = gv->pathEndPos.y;
		gv->storedPathRelatedData1[15] = gv->pathEndPos.z;
		gv->storedPathRelatedData1[16] = gv->endObjectAngle;

		gv->storedPathRelatedData1[17] = gv->qValuesObjectEnd[0];
		gv->storedPathRelatedData1[18] = gv->qValuesObjectEnd[1];
		gv->storedPathRelatedData1[19] = gv->qValuesObjectEnd[2];
		gv->storedPathRelatedData1[20] = gv->qValuesObjectEnd[3];
		gv->storedPathRelatedData1[21] = gv->qValuesObjectEnd[4];

		gv->storedPathRelatedData1[22] = gv->grippingAngleDiff;
		gv->storedPathRelatedData1[23] = gv->grippingWidthOpen;
		gv->storedPathRelatedData1[24] = gv->grippingWidthFixed;

		gv->storedPathRelatedData1[25] = gv->safetyCollider->x_stat[5];
		gv->storedPathRelatedData1[26] = gv->safetyCollider->y_stat[5];
		gv->storedPathRelatedData1[27] = gv->safetyCollider->z_stat[5];
	}
	else {
		gv->storedPathRelatedData2[0] = float(gv->objectToTransport);
		gv->storedPathRelatedData2[1] = gv->objectOffset;
		gv->storedPathRelatedData2[2] = gv->objectHeight;
		gv->storedPathRelatedData2[3] = gv->objectGripDepth;

		gv->storedPathRelatedData2[4] = gv->pathStartPos.x;
		gv->storedPathRelatedData2[5] = gv->pathStartPos.y;
		gv->storedPathRelatedData2[6] = gv->pathStartPos.z;
		gv->storedPathRelatedData2[7] = gv->startObjectAngle;

		gv->storedPathRelatedData2[8] = gv->qValuesObjectStart[0];
		gv->storedPathRelatedData2[9] = gv->qValuesObjectStart[1];
		gv->storedPathRelatedData2[10] = gv->qValuesObjectStart[2];
		gv->storedPathRelatedData2[11] = gv->qValuesObjectStart[3];
		gv->storedPathRelatedData2[12] = gv->qValuesObjectStart[4];

		gv->storedPathRelatedData2[13] = gv->pathEndPos.x;
		gv->storedPathRelatedData2[14] = gv->pathEndPos.y;
		gv->storedPathRelatedData2[15] = gv->pathEndPos.z;
		gv->storedPathRelatedData2[16] = gv->endObjectAngle;

		gv->storedPathRelatedData2[17] = gv->qValuesObjectEnd[0];
		gv->storedPathRelatedData2[18] = gv->qValuesObjectEnd[1];
		gv->storedPathRelatedData2[19] = gv->qValuesObjectEnd[2];
		gv->storedPathRelatedData2[20] = gv->qValuesObjectEnd[3];
		gv->storedPathRelatedData2[21] = gv->qValuesObjectEnd[4];

		gv->storedPathRelatedData2[22] = gv->grippingAngleDiff;
		gv->storedPathRelatedData2[23] = gv->grippingWidthOpen;
		gv->storedPathRelatedData2[24] = gv->grippingWidthFixed;

		gv->storedPathRelatedData2[25] = gv->safetyCollider->x_stat[5];
		gv->storedPathRelatedData2[26] = gv->safetyCollider->y_stat[5];
		gv->storedPathRelatedData2[27] = gv->safetyCollider->z_stat[5];
	}
}

void HelperFunctions::reloadPathRelatedData(int storageIndex) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	if (storageIndex == 0) {
		gv->objectToTransport = int(gv->storedPathRelatedData1[0]);
		gv->objectOffset = gv->storedPathRelatedData1[1];
		gv->objectHeight = gv->storedPathRelatedData1[2];
		gv->objectGripDepth = gv->storedPathRelatedData1[3];

		gv->pathStartPos.x = gv->storedPathRelatedData1[4];
		gv->pathStartPos.y = gv->storedPathRelatedData1[5];
		gv->pathStartPos.z = gv->storedPathRelatedData1[6];
		gv->startObjectAngle = gv->storedPathRelatedData1[7];

		gv->qValuesObjectStart[0] = gv->storedPathRelatedData1[8];
		gv->qValuesObjectStart[1] = gv->storedPathRelatedData1[9];
		gv->qValuesObjectStart[2] = gv->storedPathRelatedData1[10];
		gv->qValuesObjectStart[3] = gv->storedPathRelatedData1[11];
		gv->qValuesObjectStart[4] = gv->storedPathRelatedData1[12];

		gv->pathEndPos.x = gv->storedPathRelatedData1[13];
		gv->pathEndPos.y = gv->storedPathRelatedData1[14];
		gv->pathEndPos.z = gv->storedPathRelatedData1[15];
		gv->endObjectAngle = gv->storedPathRelatedData1[16];

		gv->qValuesObjectEnd[0] = gv->storedPathRelatedData1[17];
		gv->qValuesObjectEnd[1] = gv->storedPathRelatedData1[18];
		gv->qValuesObjectEnd[2] = gv->storedPathRelatedData1[19];
		gv->qValuesObjectEnd[3] = gv->storedPathRelatedData1[20];
		gv->qValuesObjectEnd[4] = gv->storedPathRelatedData1[21];

		gv->grippingAngleDiff = gv->storedPathRelatedData1[22];
		gv->grippingWidthOpen = gv->storedPathRelatedData1[23];
		gv->grippingWidthFixed = gv->storedPathRelatedData1[24];

		cd->updateColliderSize(gv->safetyCollider, gv->storedPathRelatedData1[25], gv->storedPathRelatedData1[26], gv->storedPathRelatedData1[27]);
	}
	else {
		gv->objectToTransport = int(gv->storedPathRelatedData2[0]);
		gv->objectOffset = gv->storedPathRelatedData2[1];
		gv->objectHeight = gv->storedPathRelatedData2[2];
		gv->objectGripDepth = gv->storedPathRelatedData2[3];

		gv->pathStartPos.x = gv->storedPathRelatedData2[4];
		gv->pathStartPos.y = gv->storedPathRelatedData2[5];
		gv->pathStartPos.z = gv->storedPathRelatedData2[6];
		gv->startObjectAngle = gv->storedPathRelatedData2[7];

		gv->qValuesObjectStart[0] = gv->storedPathRelatedData2[8];
		gv->qValuesObjectStart[1] = gv->storedPathRelatedData2[9];
		gv->qValuesObjectStart[2] = gv->storedPathRelatedData2[10];
		gv->qValuesObjectStart[3] = gv->storedPathRelatedData2[11];
		gv->qValuesObjectStart[4] = gv->storedPathRelatedData2[12];

		gv->pathEndPos.x = gv->storedPathRelatedData2[13];
		gv->pathEndPos.y = gv->storedPathRelatedData2[14];
		gv->pathEndPos.z = gv->storedPathRelatedData2[15];
		gv->endObjectAngle = gv->storedPathRelatedData2[16];

		gv->qValuesObjectEnd[0] = gv->storedPathRelatedData2[17];
		gv->qValuesObjectEnd[1] = gv->storedPathRelatedData2[18];
		gv->qValuesObjectEnd[2] = gv->storedPathRelatedData2[19];
		gv->qValuesObjectEnd[3] = gv->storedPathRelatedData2[20];
		gv->qValuesObjectEnd[4] = gv->storedPathRelatedData2[21];

		gv->grippingAngleDiff = gv->storedPathRelatedData2[22];
		gv->grippingWidthOpen = gv->storedPathRelatedData2[23];
		gv->grippingWidthFixed = gv->storedPathRelatedData2[24];

		cd->updateColliderSize(gv->safetyCollider, gv->storedPathRelatedData2[25], gv->storedPathRelatedData2[26], gv->storedPathRelatedData2[27]);
	}
}

void HelperFunctions::getMTPositionData(int mtIndex) {
	GlobalVariables *gv = GlobalVariables::get_instance();

	if (mtIndex == 1) {
		distortedRelativePosition[0] = -sin(gv->colliders[15].angles[0]) * gv->centerRay[0] + cos(gv->colliders[15].angles[0]) * gv->centerRay[1];
		distortedRelativePosition[1] = -cos(gv->colliders[15].angles[0]) * gv->centerRay[0] - sin(gv->colliders[15].angles[0]) * gv->centerRay[1];
		distortedRelativePosition[2] = gv->camVectorsMT2[2] - gv->centerRay[2];
		distortedRelativePosition[3] = -atan2(-cos(gv->colliders[15].angles[0]) * gv->patternNy + sin(gv->colliders[15].angles[0]) * gv->patternNx, -cos(gv->colliders[15].angles[0]) * gv->patternNx - sin(gv->colliders[15].angles[0]) * gv->patternNy);
	}
	else {
		distortedRelativePosition[0] = -sin(gv->colliders[16].angles[0]) * gv->centerRay[0] + cos(gv->colliders[16].angles[0]) * gv->centerRay[1];
		distortedRelativePosition[1] = -cos(gv->colliders[16].angles[0]) * gv->centerRay[0] - sin(gv->colliders[16].angles[0]) * gv->centerRay[1];
		distortedRelativePosition[2] = gv->camVectorsMT2[2] - gv->centerRay[2];
		distortedRelativePosition[3] = -atan2(-cos(gv->colliders[16].angles[0]) * gv->patternNy + sin(gv->colliders[16].angles[0]) * gv->patternNx, -cos(gv->colliders[16].angles[0]) * gv->patternNx - sin(gv->colliders[16].angles[0]) * gv->patternNy);
	}
}

void HelperFunctions::increaseSimSpeed() {
	if (gv->simSpeed == 6) {
		gv->simSpeed = 1;
	}
	else {
		gv->simSpeed += 1;
	}

	if (gv->simSpeed == 1) {
		cout << "Info: simulation speed changed to: slow mode" << endl;
	}
	else if (gv->simSpeed == 6) {
		cout << "Info: simulation speed changed to: unlimited" << endl;
	}
	else {
		cout << "Info: simulation speed changed to: " << gv->simSpeed - 1 << endl;
	}
}

void HelperFunctions::resetCamPos() {
	gv->camPos[0] = 75.0f;
	gv->camPos[1] = 0.0f;
	gv->camPos[2] = 10.0f;
	gv->camAngles[0] = M_PI;
	gv->camAngles[1] = 0.0f;
	updateTransformationMatrix();
}

void HelperFunctions::resetSelectionMarkerPos() {
	CollisionDetection *cd = CollisionDetection::get_instance();
	gv->targetPos = { gv->standbyPos.x, gv->standbyPos.y, gv->standbyPos.z };
	gv->rotationValue = 0.0f;
	cd->recalculateCollider(&gv->colliders[0], gv->targetPos.x, gv->targetPos.y, gv->targetPos.z, 0.0f, 0.0f, 0.0f);
	calcEdgesForConvexHull(gv->object_edges, &gv->colliders[0], gv->totalPlatePoints);
}

bool HelperFunctions::processConfirmButtonPress() {
	CollisionDetection *cd = CollisionDetection::get_instance();
	if (gv->synMode == 2) {
		gv->found = false;
		for (int i = 0; i < 4; i++) {
			if (cd->checkForCollision(&gv->colliders[gv->transportableObjects[i]], &gv->colliders[0])) {
				gv->found = true;
				if (gv->selectedWorkpieceIndex == -1) {
					gv->selectedWorkpieceIndex = i;
					cout << "Info: workpiece " << i + 1 << " selected for transport simulation" << endl;
					break;
				}
				else {
					cout << "Warning: transportation object already selected. Now select the target position!" << endl;
					break;
				}
			}
		}

		if (!gv->found) {
			if (gv->selectedWorkpieceIndex != -1) {
				gv->selectedTargetPosition[0] = gv->targetPos.x;
				gv->selectedTargetPosition[1] = gv->targetPos.y;
				gv->selectedTargetPosition[2] = gv->targetPos.z;
				gv->selectedTargetPosition[3] = gv->rotationValue;
				cout << "Info: target position and rotation selected for transport simulation" << endl;
			}
			else {
				cout << "Warning: select transportation object first before selecting target position!" << endl;
			}
		}

		cout << "debug@HF l. "<<__LINE__<<endl;

		if (gv->selectedWorkpieceIndex != -1 && abs(gv->selectedTargetPosition[0]) < 1000.0f) {
			gv->selectionMarkerMode = false;
			gv->simTargetSelection = false;
			gv->simTransport = true;
			return true;
		}
	}
	else if (gv->synMode == 4 && gv->currentlyTransporting) {
		gv->found = false;
		for (int i = 0; i < 6; i++) {
			if (gv->transportableObjects[i] != gv->objectToTransport) {
				if (cd->checkForCollision(&gv->colliders[gv->transportableObjects[i]], &gv->colliders[0])) {
					gv->found = true;
					break;
				}
			}
		}

		if (!gv->found) {
			gv->selectedTargetPosition[0] = gv->targetPos.x;
			gv->selectedTargetPosition[1] = gv->targetPos.y;
			gv->selectedTargetPosition[2] = gv->targetPos.z;
			gv->selectedTargetPosition[3] = gv->rotationValue;
			cout << "Info: target position and rotation selected for object drop" << endl;
		}
		else {
			cout << "Warning: select a position outside a transportable object as target position instead!" << endl;
		}

		if (abs(gv->selectedTargetPosition[0]) < 1000.0f) {
			gv->selectionMarkerMode = false;
			gv->simTargetSelection = false;
			gv->simTransport = true;
			gv->manualMode = 0;
			gv->realExecSubPhase = 2;
			return true;
		}
	}
	else if (gv->synMode == 4 && !gv->currentlyTransporting) {
		gv->found = false;
		for (int i = 0; i < 6; i++) {
			if (cd->checkForCollision(&gv->colliders[gv->transportableObjects[i]], &gv->colliders[0])) {
				gv->selectedWorkpieceIndex = i;
				if (i == 4) cout << "Info: Monitoring Tool 1 selected for manual transportation" << endl;
				else if (i == 5) cout << "Info: Monitoring Tool 2 selected for manual transportation" << endl;
				else cout << "Info: workpiece " << i + 1 << " selected for manual transportation" << endl;

				gv->found = true;
				break;
			}
		}

		if (!gv->found) {
			gv->selectedTargetPosition[0] = gv->targetPos.x;
			gv->selectedTargetPosition[1] = gv->targetPos.y;
			gv->selectedTargetPosition[2] = gv->targetPos.z;
			gv->selectedTargetPosition[3] = gv->rotationValue;

			if (gv->selectedWorkpieceIndex != -1) cout << "Info: target position and rotation selected for object drop" << endl;
			else cout << "Info: target position and last joint rotation selected" << endl;
		}

		if (abs(gv->selectedTargetPosition[0]) < 1000.0f) {
			gv->selectionMarkerMode = false;
			gv->simTargetSelection = false;
			gv->simTransport = true;
			if (gv->selectedWorkpieceIndex != -1) gv->manualMode = 1;
			else gv->manualMode = 2;
			gv->realExecSubPhase = 0;
			return true;
		}
	}
	return false;
}

void HelperFunctions::selectionMarkerModeSwitch() {
	gv->selectionMarkerMode = !gv->selectionMarkerMode;

	if (gv->selectionMarkerMode) {
		cout << "Info: entering control mode" << endl;
		cout << "---------------------------" << endl;
	}
	else {
		cout << "Info: leaving control mode" << endl;
		cout << "--------------------------" << endl;
	}
}

void HelperFunctions::syncVisualization(vector<string> splitList) {
	CollisionDetection *cd = CollisionDetection::get_instance(); InverseKinematic *ik = InverseKinematic::get_instance();
	pp->nodeIndex = 1;
	for (int i = 0; i < gv->realNodeIndex; i++) {
		pp->nodeIndex = pp->nodeList[pp->nodeIndex].previous;
	}

	if (pp->nodeIndex != 1) {
		gv->qValuesStartPoint[0] = gv->q0ValsTempSeg[gv->realNodeIndex - 1];
		gv->qValuesStartPoint[1] = gv->q1ValsTempSeg[gv->realNodeIndex - 1];
		gv->qValuesStartPoint[2] = gv->q2ValsTempSeg[gv->realNodeIndex - 1];
		gv->qValuesStartPoint[3] = gv->q3ValsTempSeg[gv->realNodeIndex - 1];
		gv->qValuesStartPoint[4] = gv->q4ValsTempSeg[gv->realNodeIndex - 1];

		if (pp->nodeIndex != -1) {
			gv->qValuesEndPoint[0] = gv->q0ValsTempSeg[gv->realNodeIndex];
			gv->qValuesEndPoint[1] = gv->q1ValsTempSeg[gv->realNodeIndex];
			gv->qValuesEndPoint[2] = gv->q2ValsTempSeg[gv->realNodeIndex];
			gv->qValuesEndPoint[3] = gv->q3ValsTempSeg[gv->realNodeIndex];
			gv->qValuesEndPoint[4] = gv->q4ValsTempSeg[gv->realNodeIndex];

			gv->maxValue = -1.0f;
			gv->maxValueIndex = -1;
			for (int i = 0; i < 5; i++) {
				if (abs(gv->qValuesEndPoint[i] - gv->qValuesStartPoint[i]) > gv->maxValue) {
					gv->maxValue = abs(gv->qValuesEndPoint[i] - gv->qValuesStartPoint[i]);
					gv->maxValueIndex = i;
				}
			}

			gv->pathInterpolationFactor = (stof(splitList[gv->maxValueIndex]) - gv->qValuesStartPoint[gv->maxValueIndex]) / (gv->qValuesEndPoint[gv->maxValueIndex] - gv->qValuesStartPoint[gv->maxValueIndex]);

			if (gv->pathInterpolationFactor > 1.0f) gv->pathInterpolationFactor = 1.0f;
			if (gv->pathInterpolationFactor < 0.0f) gv->pathInterpolationFactor = 0.0f;

			for (int i = 0; i < 5; i++) {
				gv->qValuesCurrent[i] = (gv->qValuesEndPoint[i] - gv->qValuesStartPoint[i]) * gv->pathInterpolationFactor + gv->qValuesStartPoint[i];
			}
		}
		else {
			gv->pathInterpolationFactor = 0.0f;
			for (int i = 0; i < 5; i++) {
				gv->qValuesCurrent[i] = gv->qValuesStartPoint[i];
			}
		}

		gv->q0Sin = sin(gv->qValuesCurrent[0] / 180.0f * M_PI);
		gv->q0Cos = cos(gv->qValuesCurrent[0] / 180.0f * M_PI);
		gv->q1Sin = sin(gv->qValuesCurrent[1] / 180.0f * M_PI);
		gv->q1Cos = cos(gv->qValuesCurrent[1] / 180.0f * M_PI);
		gv->q12Sin = sin((gv->qValuesCurrent[1] + gv->qValuesCurrent[2]) / 180.0f * M_PI);
		gv->q12Cos = cos((gv->qValuesCurrent[1] + gv->qValuesCurrent[2]) / 180.0f * M_PI);
		gv->q123Sin = sin((gv->qValuesCurrent[1] + gv->qValuesCurrent[2] + gv->qValuesCurrent[3]) / 180.0f * M_PI);
		gv->q123Cos = cos((gv->qValuesCurrent[1] + gv->qValuesCurrent[2] + gv->qValuesCurrent[3]) / 180.0f * M_PI);

		if (abs(gv->pathLastPos.x) > cd->epsilon3 || abs(gv->pathLastPos.y) > cd->epsilon3 || abs(gv->pathLastPos.z) > cd->epsilon3) {
			gv->real_path_edges = ik->increaseSize(gv->real_path_edges, gv->numPointsRealPath, 2);
			gv->real_path_vertices = ik->increaseSize(gv->real_path_vertices, gv->numPointsRealPath, 2);

			gv->real_path_edges[gv->numPointsRealPath].x = gv->pathLastPos.x;
			gv->real_path_edges[gv->numPointsRealPath].y = gv->pathLastPos.y;
			gv->real_path_edges[gv->numPointsRealPath].z = gv->pathLastPos.z;
			gv->real_path_vertices[gv->numPointsRealPath] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->pathColors[1].x, gv->pathColors[1].y, gv->pathColors[1].z, 1.0f };
			gv->numPointsRealPath++;

			gv->real_path_edges[gv->numPointsRealPath].x = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Sin;
			gv->real_path_edges[gv->numPointsRealPath].y = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Cos;
			gv->real_path_edges[gv->numPointsRealPath].z = ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->len4 * gv->q123Cos;
			gv->real_path_vertices[gv->numPointsRealPath] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->pathColors[1].x, gv->pathColors[1].y, gv->pathColors[1].z, 1.0f };
			gv->numPointsRealPath++;
		}

		gv->pathLastPos.x = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Sin;
		gv->pathLastPos.y = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Cos;
		gv->pathLastPos.z = ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->len4 * gv->q123Cos;

		cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, 0.0f, 0.0f);
		cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Sin + 4.5f * gv->q0Cos, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Cos - 4.5f * gv->q0Sin, ik->len0 + ik->len2 / 2.0f * gv->q1Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (90.0f - gv->qValuesCurrent[1]) / 180.0f * M_PI, 0);
		cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Sin - 4.155f * gv->q0Cos, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Cos + 4.155f * gv->q0Sin, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 / 2.0f * gv->q12Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (90.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]) / 180.0f * M_PI, 0);
		cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->lastSegmentMid * gv->q123Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (90.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, 0);
		cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (90.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, gv->qValuesCurrent[4] / 180.0f * M_PI);

		cd->updateMatricesTransport((-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, gv->qValuesCurrent[4] / 180.0f * M_PI, gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
		cd->rotationMatrix(gv->q0Cos * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
		gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

		cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Cos + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos + gv->grippingDiffVector[2], (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, gv->qValuesCurrent[4] / 180.0f * M_PI);
		cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Cos - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos - gv->grippingDiffVector[2], (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, gv->qValuesCurrent[4] / 180.0f * M_PI);

		gv->tempCounter = gv->totalPlatePoints + gv->colliders[0].facesTimes3 + gv->colliders[1].facesTimes3 + gv->colliders[2].facesTimes3;
		gv->tempCounter = calcEdgesForConvexHull(gv->object_edges, &gv->colliders[3], gv->tempCounter);
		gv->tempCounter = calcEdgesForConvexHull(gv->object_edges, &gv->colliders[4], gv->tempCounter);
		gv->tempCounter = calcEdgesForConvexHull(gv->object_edges, &gv->colliders[5], gv->tempCounter);
		gv->tempCounter = calcEdgesForConvexHull(gv->object_edges, &gv->colliders[6], gv->tempCounter);
		gv->tempCounter = calcEdgesForConvexHull(gv->object_edges, &gv->colliders[7], gv->tempCounter);

		if (gv->transportPhase == 1) {
			cd->updateMatricesTransport((-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, (gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
			if (gv->objectToTransport == 15 || gv->objectToTransport == 16) {
				cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, (gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
				if (gv->objectToTransport == 15) calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
				else if (gv->objectToTransport == 16) calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
			}
			else {
				cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, (gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI, true);
			}
			cd->recalculateColliderTransport(gv->safetyCollider, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, (gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI);

			for (int i = 8; i < gv->objectToTransport; i++) {
				gv->tempCounter += gv->colliders[i].facesTimes3;
			}
			calcEdgesForConvexHull(gv->object_edges, &gv->colliders[gv->objectToTransport], gv->tempCounter);
			calcEdgesForMesh(gv->safety_col_edges, gv->safetyCollider, 0);

			if (gv->objectToTransport != 15 && gv->objectToTransport != 16) {
				gv->tempCounter = 0;
				for (int i = 8; i < gv->objectToTransport; i++) {
					gv->tempCounter += gv->colliders[i].patternCount * 4;
				}
				calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[gv->objectToTransport], gv->tempCounter);
			}
		}

		gv->tempCounter = gv->totalPlatePoints;
		for (int i = 0; i < gv->collidersCount; i++) {
			gv->tempCounter += gv->colliders[i].facesTimes3;
		}
		gv->tempCounter = calcEdgesForConvexHull(gv->object_edges, &gv->grippers[0], gv->tempCounter);
		calcEdgesForConvexHull(gv->object_edges, &gv->grippers[1], gv->tempCounter);
	}
}




/*
 * This function initializes the arrays handed over to graphic card for visualization.
 *
 */
void HelperFunctions::init_arrays_for_graphic_card() {
InverseKinematic *ik = InverseKinematic::get_instance();
HelperFunctions *hf = HelperFunctions::get_instance();
	//monitoring tool
	for (int j = 0; j < 32; j++) {
		gv->mt_viewfields_edges[j] = collider::Edge { 0.0f, 0.0f, 0.0f };
		gv->mt_viewfields_vertices[j] = collider::Vertex { 0.0f, 0.0f, 0.0f, gv->otherColors[0].x,
				gv->otherColors[0].y, gv->otherColors[0].z, 1.0f };
	}
	// Richtungsmarkierung der Werkstücke (wegen QR-Codes)
	for (int j = 0; j < 24; j++) {
		gv->workpiece_arrows_edges[j] = collider::Edge { 0.0f, 0.0f, 0.0f };
		gv->workpiece_arrows_vertices[j] = collider::Vertex { 0.0f, 0.0f, 0.0f,
				gv->otherColors[6].x, gv->otherColors[6].y, gv->otherColors[6].z, 1.0f };
	}

	for (int j = 0; j < gv->numPointsObject; j++) {
		gv->object_edges = ik->increaseSize(gv->object_edges, j, 1);
		gv->object_vertices = ik->increaseSize(gv->object_vertices, j, 1);
		gv->object_edges[j] = collider::Edge { 0.0f, 0.0f, 0.0f };
		hf->getColorIndex(j);
		gv->object_vertices[j] = collider::Vertex { 0.0f, 0.0f, 0.0f, gv->colors[gv->colorIndex].x,
				gv->colors[gv->colorIndex].y, gv->colors[gv->colorIndex].z, 1.0f };
	}
	gv->numPointsPattern = 0;
	for (int j = 8; j <= 14; j++) {
		gv->numPointsPattern += 4 * gv->colliders[j].patternCount;
	}
	for (int j = 0; j < gv->numPointsPattern; j++) {
		gv->object_pattern_edges = ik->increaseSize(gv->object_pattern_edges, j, 1);
		gv->object_pattern_vertices = ik->increaseSize(gv->object_pattern_vertices, j, 1);
		gv->object_pattern_edges[j] = collider::Edge { 0.0f, 0.0f, 0.0f };
		gv->object_pattern_vertices[j] = collider::Vertex { 0.0f, 0.0f, 0.0f,
				gv->otherColors[1].x, gv->otherColors[1].y, gv->otherColors[1].z, 1.0f };
	}
	for (int j = 0; j < 72 * 3; j++) {
		gv->distorted_obs_edges[j] = collider::Edge { 0.0f, 0.0f, 0.0f };
		gv->distorted_obs_vertices[j] = collider::Vertex { 0.0f, 0.0f, 0.0f, gv->otherColors[2].x,
				gv->otherColors[2].y, gv->otherColors[2].z, 1.0f };
		gv->ground_truth_obs_edges[j] = collider::Edge { 0.0f, 0.0f, 0.0f };
		gv->distorted_obs_vertices[j] = collider::Vertex { 0.0f, 0.0f, 0.0f,
				gv->otherColors[5].x, gv->otherColors[5].y, gv->otherColors[5].z, 1.0f };
	}
	for (int j = 0; j < 72; j++) {
		gv->safety_col_edges[j] = collider::Edge { 0.0f, 0.0f, 0.0f };
		gv->safety_col_vertices[j] = collider::Vertex { 0.0f, 0.0f, 0.0f, gv->otherColors[3].x,
				gv->otherColors[3].y, gv->otherColors[3].z, 1.0f };
		gv->workpiece_mesh_edges[j] = collider::Edge { 0.0f, 0.0f, 0.0f };
		gv->workpiece_mesh_vertices[j] = collider::Vertex { 0.0f, 0.0f, 0.0f,
				gv->otherColors[3].x, gv->otherColors[3].y, gv->otherColors[3].z, 1.0f };
	}

	// ground surface representing working area.
	gv->tempCounter = 0;
	for (int i = 0; i < 18; i++) {
		for (int j = 3; j < 18; j++) {
			gv->object_edges[gv->tempCounter].x = (float) ((i - 8)) * 5;
			gv->object_edges[gv->tempCounter].y = (float) ((j - 9)) * 5;
			gv->object_edges[gv->tempCounter].z = 0.0f;
			gv->object_edges[gv->tempCounter + 1].x = (float) ((i - 9)) * 5;
			gv->object_edges[gv->tempCounter + 1].y = (float) ((j - 9)) * 5;
			gv->object_edges[gv->tempCounter + 1].z = 0.0f;
			gv->object_edges[gv->tempCounter + 2].x = (float) ((i - 9)) * 5;
			gv->object_edges[gv->tempCounter + 2].y = (float) ((j - 8)) * 5;
			gv->object_edges[gv->tempCounter + 2].z = 0.0f;
			gv->object_edges[gv->tempCounter + 3].x = (float) ((i - 9)) * 5;
			gv->object_edges[gv->tempCounter + 3].y = (float) ((j - 8)) * 5;
			gv->object_edges[gv->tempCounter + 3].z = 0.0f;
			gv->object_edges[gv->tempCounter + 4].x = (float) ((i - 8)) * 5;
			gv->object_edges[gv->tempCounter + 4].y = (float) ((j - 8)) * 5;
			gv->object_edges[gv->tempCounter + 4].z = 0.0f;
			gv->object_edges[gv->tempCounter + 5].x = (float) ((i - 8)) * 5;
			gv->object_edges[gv->tempCounter + 5].y = (float) ((j - 9)) * 5;
			gv->object_edges[gv->tempCounter + 5].z = 0.0f;
			gv->tempCounter += 6;
		}
	}
	for (int i = 0; i < 22; i++) {
		for (int j = 5; j < 22; j++) {
			if (i == 0 || i == 21 || j == 21 || i == 1 || i == 20 || j == 20) {
				gv->object_edges[gv->tempCounter].x = (float) ((i - 10)) * 5;
				gv->object_edges[gv->tempCounter].y = (float) ((j - 11)) * 5;
				gv->object_edges[gv->tempCounter].z = 0.0f;
				gv->object_edges[gv->tempCounter + 1].x = (float) ((i - 11)) * 5;
				gv->object_edges[gv->tempCounter + 1].y = (float) ((j - 11)) * 5;
				gv->object_edges[gv->tempCounter + 1].z = 0.0f;
				gv->object_edges[gv->tempCounter + 2].x = (float) ((i - 11)) * 5;
				gv->object_edges[gv->tempCounter + 2].y = (float) ((j - 10)) * 5;
				gv->object_edges[gv->tempCounter + 2].z = 0.0f;
				gv->object_edges[gv->tempCounter + 3].x = (float) ((i - 11)) * 5;
				gv->object_edges[gv->tempCounter + 3].y = (float) ((j - 10)) * 5;
				gv->object_edges[gv->tempCounter + 3].z = 0.0f;
				gv->object_edges[gv->tempCounter + 4].x = (float) ((i - 10)) * 5;
				gv->object_edges[gv->tempCounter + 4].y = (float) ((j - 10)) * 5;
				gv->object_edges[gv->tempCounter + 4].z = 0.0f;
				gv->object_edges[gv->tempCounter + 5].x = (float) ((i - 10)) * 5;
				gv->object_edges[gv->tempCounter + 5].y = (float) ((j - 11)) * 5;
				gv->object_edges[gv->tempCounter + 5].z = 0.0f;
				gv->tempCounter += 6;
			}
		}
	}
}


/*
 *
 */
void HelperFunctions::recalculatesCollidersOfRobot() {
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik=InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

	ik->doesThetaExist(gv->standbyPos.x, gv->standbyPos.y, gv->standbyPos.z);
	ik->writeQValues(gv->qValuesStandby);
	gv->q0Sin = sinf(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q0Cos = cosf(gv->qValuesStandby[0] / 180.0f * M_PI);
	gv->q1Sin = sinf(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q1Cos = cosf(gv->qValuesStandby[1] / 180.0f * M_PI);
	gv->q12Sin = sinf((gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q12Cos = cosf((gv->qValuesStandby[1] + gv->qValuesStandby[2]) / 180.0f * M_PI);
	gv->q123Sin = sinf(
			(gv->qValuesStandby[1] + gv->qValuesStandby[2] + gv->qValuesStandby[3])
					/ 180.0f* M_PI);
	gv->q123Cos = cosf(
			(gv->qValuesStandby[1] + gv->qValuesStandby[2] + gv->qValuesStandby[3])
					/ 180.0f* M_PI);
	cd->recalculateCollider(&gv->colliders[0], gv->targetPos.x, gv->targetPos.y, gv->targetPos.z,
			0.0f, 0.0f, 0.0f);
	cd->recalculateCollider(&gv->colliders[1], 0, 0, 2.3f, 0.0f, 0.0f, 0.0f);
	cd->recalculateCollider(&gv->colliders[2], 0, 0, 8.5f, 0.0f, 0.0f, 0.0f);
	cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f,
			(-gv->qValuesStandby[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
	cd->recalculateCollider(&gv->colliders[4],
			(ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Sin + 4.5f * gv->q0Cos,
			(ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Cos - 4.5f * gv->q0Sin,
			ik->len0 + ik->len2 / 2.0f * gv->q1Cos,
			(-gv->qValuesStandby[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesStandby[1]) / 180.0f * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[5],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Sin
					- 4.155f * gv->q0Cos,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Cos
					+ 4.155f * gv->q0Sin,
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 / 2.0f * gv->q12Cos,
			(-gv->qValuesStandby[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesStandby[1] - gv->qValuesStandby[2]) / 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[6],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin)
					* gv->q0Sin,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin)
					* gv->q0Cos,
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->lastSegmentMid * gv->q123Cos,
			(-gv->qValuesStandby[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3])
					/ 180 * M_PI, 0);
	cd->recalculateCollider(&gv->colliders[7],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin)
					* gv->q0Sin,
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin)
					* gv->q0Cos,
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos
					+ (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos,
			(-gv->qValuesStandby[0] + 90) / 180.0f * M_PI,
			(90 - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3])
					/ 180 * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);
	cd->recalculateCollider(&gv->colliders[8], -25, 16, gv->objectOffsets[0], M_PI / 8.0f,
			0, 0, true);
	cd->recalculateCollider(&gv->colliders[9], -10, 25, gv->objectOffsets[1], 0, 0, 0,
			true);
	cd->recalculateCollider(&gv->colliders[10], 8, 21.5, gv->objectOffsets[2], M_PI / 4.0f,
			0, 0, true);
	cd->recalculateCollider(&gv->colliders[11], 21, 16.5f, gv->objectOffsets[3], 0, 0, 0,
			true);
	cd->recalculateCollider(&gv->colliders[12], 25.0f, 8.0f, 4.0f, M_PI / 12.0f, 0, 0,
			true);
	cd->recalculateCollider(&gv->colliders[13], -5.0f, 25.4f, 3.0f, 0.0f, 0, 0, true);
	cd->recalculateCollider(&gv->colliders[14], -23.0f, 30.0f, 6.0f, M_PI / 6.0f, 0, 0,
			true);
	cd->recalculateCollider(&gv->colliders[15], 10.0f, 30.4f, gv->objectOffsets[4],
			M_PI / 6.0f, 0, 0, false, true, gv->camVectorsMT1);
	hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
	cd->recalculateCollider(&gv->colliders[16], -23.0f, 3.0f, gv->objectOffsets[5],
			M_PI / 2, 0, 0, false, true, gv->camVectorsMT2);
	hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
	cd->recalculateCollider(&gv->distortedObstacles[0],
			gv->colliders[12].offsets[0] + gv->distortions[0],
			gv->colliders[12].offsets[1] + gv->distortions[1], gv->colliders[12].offsets[2],
			gv->colliders[12].angles[0] + gv->distortions[2], 0, 0);
	cd->recalculateCollider(&gv->distortedObstacles[1],
			gv->colliders[13].offsets[0] + gv->distortions[3],
			gv->colliders[13].offsets[1] + gv->distortions[4], gv->colliders[13].offsets[2],
			gv->colliders[13].angles[0] + gv->distortions[5], 0, 0);
	cd->recalculateCollider(&gv->distortedObstacles[2],
			gv->colliders[14].offsets[0] + gv->distortions[6],
			gv->colliders[14].offsets[1] + gv->distortions[7], gv->colliders[14].offsets[2],
			gv->colliders[14].angles[0] + gv->distortions[8], 0, 0);
	cd->updateMatricesTransport((-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3])
					/ 180.0f * M_PI, (gv->qValuesStandby[4] + 0) / 180.0f * M_PI,
			gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
	cd->rotationMatrix(gv->q0Cos * (gv->grippingWidth / 2.0f + 0.45f),
			-gv->q0Sin * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
	gv->grippingDiffVector[0] = cd->xVal_cd;
	gv->grippingDiffVector[1] = cd->yVal_cd;
	gv->grippingDiffVector[2] = cd->zVal_cd;
	cd->recalculateColliderTransport(&gv->grippers[0],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin)
					* gv->q0Sin + gv->grippingDiffVector[0],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin)
					* gv->q0Cos + gv->grippingDiffVector[1],
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos
					+ (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos
					+ gv->grippingDiffVector[2],
			(-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3])
					/ 180.0f * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);
	cd->recalculateColliderTransport(&gv->grippers[1],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin)
					* gv->q0Sin - gv->grippingDiffVector[0],
			(ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin
					+ (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin)
					* gv->q0Cos - gv->grippingDiffVector[1],
			ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos
					+ (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos
					- gv->grippingDiffVector[2],
			(-gv->qValuesStandby[0] + 90.0f) / 180.0f * M_PI,
			(180.0f - gv->qValuesStandby[1] - gv->qValuesStandby[2] - gv->qValuesStandby[3])
					/ 180.0f * M_PI, gv->qValuesStandby[4] / 180.0f * M_PI);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[0],
			gv->totalPlatePoints);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[1],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[2],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[3],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[4],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[5],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[6],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[7],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[8],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[9],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[10],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[11],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[12],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[13],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[14],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[15],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[16],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[0],
			gv->tempCounter);
	hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[1], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[8], 0);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[9],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[10],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[11],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[12],
			gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[13],
			gv->tempCounter);
	hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[14], gv->tempCounter);
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[0],
			0);
	gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[1],
			gv->tempCounter);
	hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2], gv->tempCounter);
}


char HelperFunctions::wait_for_user_input_yn(){
	char c_in = '-';
	SDL_Event event_sub;
	do {
		while (SDL_PollEvent(&event_sub)) {
			if (event_sub.type == SDL_KEYUP) {
				if (event_sub.key.keysym.sym == SDLK_y) {
					c_in = 'y';
				}
				else if (event_sub.key.keysym.sym == SDLK_n) {
					c_in = 'n';
				}
			}
		}
	} while (c_in == '-');

	return c_in;
}

char HelperFunctions::wait_for_user_input_space(){
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

	return c_in;
}
