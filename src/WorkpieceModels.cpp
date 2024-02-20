/*
 * WorkpieceModels.cpp
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#include "WorkpieceModels.h"

WorkpieceModels::WorkpieceModels() {

}

WorkpieceModels::~WorkpieceModels() {
}


/*
 * this function initializes the firs workpiece.
 * it is the box of lengthly shape. size =
 */
void WorkpieceModels::init_workpiece1(collider* colliders) {
	colliders[8].points_stat = 9;
	colliders[8].x_stat = new float[colliders[8].points_stat] { 0, 2.5f, 2.5f,
			-2.5f, -2.5f, 2.5f, 2.5f, -2.5f, -2.5f };
	colliders[8].y_stat = new float[colliders[8].points_stat] { 0, 1.15f,
			-1.15f, 1.15f, -1.15f, 1.15f, -1.15f, 1.15f, -1.15f };
	colliders[8].z_stat = new float[colliders[8].points_stat] { 0, -1.5f, -1.5f,
			-1.5f, -1.5f, 1.5f, 1.5f, 1.5f, 1.5f };
	colliders[8].color = collider::Edge { 1.0f, 0.0f, 1.0f };
	colliders[8].patternCount = 2;
	colliders[8].pattern_indices = new int[colliders[8].patternCount] { 1, 0 };
	colliders[8].x_pattern_stat = new float[colliders[8].patternCount * 4] {
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f };
	colliders[8].y_pattern_stat =
			new float[colliders[8].patternCount * 4] { 1.151f, 1.151f, 1.151f,
					1.151f, -1.151f, -1.151f, -1.151f, -1.151f };
	colliders[8].z_pattern_stat = new float[colliders[8].patternCount * 4] {
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, -patternSize / 2.0f };
	colliders[8].x_pattern_tf = new float[colliders[8].patternCount * 4];
	colliders[8].y_pattern_tf = new float[colliders[8].patternCount * 4];
	colliders[8].z_pattern_tf = new float[colliders[8].patternCount * 4];
}

/*
 * this function initializes the 2nd workpiece.
 * the shape is also a box. size =
 */
void WorkpieceModels::init_workpiece2(collider* colliders) {
	colliders[9].points_stat = 9;
	colliders[9].x_stat = new float[colliders[9].points_stat] { 0, 1.5f, 1.5f,
			-1.5f, -1.5f, 1.5f, 1.5f, -1.5f, -1.5f };
	colliders[9].y_stat = new float[colliders[9].points_stat] { 0, 2.0f, -2.0f,
			2.0f, -2.0f, 2.0f, -2.0f, 2.0f, -2.0f };
	colliders[9].z_stat = new float[colliders[9].points_stat] { 0, -1.5f, -1.5f,
			-1.5f, -1.5f, 1.5f, 1.5f, 1.5f, 1.5f };
	colliders[9].color = collider::Edge { 1.0f, 0.0f, 1.0f };
	colliders[9].patternCount = 4;
	colliders[9].pattern_indices = new int[colliders[9].patternCount] { 3, 5, 4,
			2 };
	colliders[9].x_pattern_stat = new float[colliders[9].patternCount * 4] {
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, 1.501f, 1.501f, 1.501f,
			1.501f, -1.501f, -1.501f, -1.501f, -1.501f };
	colliders[9].y_pattern_stat = new float[colliders[9].patternCount * 4] {
			2.001f, 2.001f, 2.001f, 2.001f, -2.001f, -2.001f, -2.001f, -2.001f,
			patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f,
			patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f,
			patternSize / 2.0f, -patternSize / 2.0f };
	colliders[9].z_pattern_stat = new float[colliders[9].patternCount * 4] {
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f,
			patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f,
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f };
	colliders[9].x_pattern_tf = new float[colliders[9].patternCount * 4];
	colliders[9].y_pattern_tf = new float[colliders[9].patternCount * 4];
	colliders[9].z_pattern_tf = new float[colliders[9].patternCount * 4];
}

/*
 * this function initializes the 2nd workpiece.
 * the shape is a prism.
 */
void WorkpieceModels::init_workpiece3(collider* colliders) {
	colliders[10].points_stat = 9;
	colliders[10].x_stat = new float[colliders[10].points_stat] { 0, 1.5f, 1.5f,
			-1.5f, -1.5f, 1.5f, 1.5f, -1.5f, -1.5f };
	colliders[10].y_stat = new float[colliders[10].points_stat] { 0, 1.25f,
			-1.25f, 1.75f, -1.75f, 1.25f, -1.25f, 1.75f, -1.75f };
	colliders[10].z_stat = new float[colliders[10].points_stat] { 0, -1.5f,
			-1.5f, -1.5f, -1.5f, 1.5f, 1.5f, 1.5f, 1.5f };
	colliders[10].color = collider::Edge { 1.0f, 0.0f, 1.0f };
	colliders[10].patternCount = 3;
	colliders[10].pattern_indices = new int[colliders[10].patternCount] { 6, 7,
			8 };
	colliders[10].x_pattern_stat = new float[colliders[10].patternCount * 4] {
			-1.501f, -1.501f, -1.501f, -1.501f, pattern7SurfaceVec[0]
					* patternSize / 2.0f, -pattern7SurfaceVec[0] * patternSize
					/ 2.0f, -pattern7SurfaceVec[0] * patternSize / 2.0f,
			pattern7SurfaceVec[0] * patternSize / 2.0f, -pattern8SurfaceVec[0]
					* patternSize / 2.0f, pattern8SurfaceVec[0] * patternSize
					/ 2.0f, pattern8SurfaceVec[0] * patternSize / 2.0f,
			-pattern8SurfaceVec[0] * patternSize / 2.0f };
	colliders[10].y_pattern_stat = new float[colliders[10].patternCount * 4] {
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, 1.501f
					+ pattern7SurfaceVec[1] * patternSize / 2.0f, 1.501f
					- pattern7SurfaceVec[1] * patternSize / 2.0f, 1.501f
					- pattern7SurfaceVec[1] * patternSize / 2.0f, 1.501f
					+ pattern7SurfaceVec[1] * patternSize / 2.0f, -1.501f
					- pattern8SurfaceVec[1] * patternSize / 2.0f, -1.501f
					+ pattern8SurfaceVec[1] * patternSize / 2.0f, -1.501f
					+ pattern8SurfaceVec[1] * patternSize / 2.0f, -1.501f
					- pattern8SurfaceVec[1] * patternSize / 2.0f };
	colliders[10].z_pattern_stat = new float[colliders[10].patternCount * 4] {
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f,
			patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f };
	colliders[10].x_pattern_tf = new float[colliders[10].patternCount * 4];
	colliders[10].y_pattern_tf = new float[colliders[10].patternCount * 4];
	colliders[10].z_pattern_tf = new float[colliders[10].patternCount * 4];
}
/*
 * this function initializes the 2nd workpiece.
 * the shape is a triangle zylinder.
 */
void WorkpieceModels::init_workpiece4(collider* colliders) {
	colliders[11].points_stat = 7;
	colliders[11].x_stat = new float[colliders[11].points_stat] { 0, -1.5f,
			-1.5f, 1.5f, -1.5f, -1.5f, 1.5f };
	colliders[11].y_stat = new float[colliders[11].points_stat] { 0, 2.0f,
			-2.0f, 0.0f, 2.0f, -2.0f, 0.0f };
	colliders[11].z_stat = new float[colliders[11].points_stat] { 0, -1.5f,
			-1.5f, -1.5f, 1.5f, 1.5f, 1.5f };
	colliders[11].color = collider::Edge { 1.0f, 0.0f, 1.0f };
	colliders[11].patternCount = 3;
	colliders[11].pattern_indices = new int[colliders[11].patternCount] { 9, 10,
			11 };
	colliders[11].x_pattern_stat = new float[colliders[11].patternCount * 4] {
			-1.501f, -1.501f, -1.501f, -1.501f, pattern10SurfaceVec[0]
					* patternSize / 2.0f, -pattern10SurfaceVec[0] * patternSize
					/ 2.0f, -pattern10SurfaceVec[0] * patternSize / 2.0f,
			pattern10SurfaceVec[0] * patternSize / 2.0f, -pattern11SurfaceVec[0]
					* patternSize / 2.0f, pattern11SurfaceVec[0] * patternSize
					/ 2.0f, pattern11SurfaceVec[0] * patternSize / 2.0f,
			-pattern11SurfaceVec[0] * patternSize / 2.0f };
	colliders[11].y_pattern_stat = new float[colliders[11].patternCount * 4] {
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, 1.001f
					+ pattern10SurfaceVec[1] * patternSize / 2.0f, 1.001f
					- pattern10SurfaceVec[1] * patternSize / 2.0f, 1.001f
					- pattern10SurfaceVec[1] * patternSize / 2.0f, 1.001f
					+ pattern10SurfaceVec[1] * patternSize / 2.0f, -1.001f
					- pattern11SurfaceVec[1] * patternSize / 2.0f, -1.001f
					+ pattern11SurfaceVec[1] * patternSize / 2.0f, -1.001f
					+ pattern11SurfaceVec[1] * patternSize / 2.0f, -1.001f
					- pattern11SurfaceVec[1] * patternSize / 2.0f };
	colliders[11].z_pattern_stat = new float[colliders[11].patternCount * 4] {
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f,
			patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f };
	colliders[11].x_pattern_tf = new float[colliders[11].patternCount * 4];
	colliders[11].y_pattern_tf = new float[colliders[11].patternCount * 4];
	colliders[11].z_pattern_tf = new float[colliders[11].patternCount * 4];
}

void WorkpieceModels::init_workpiceMesh(collider* workpieceMesh) {
	workpieceMesh->points_stat = 9;
	workpieceMesh->x_stat = new float[workpieceMesh->points_stat] { 0, 1.0f,
			1.0f, -1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f };
	workpieceMesh->y_stat = new float[workpieceMesh->points_stat] { 0, 1.0f,
			-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f };
	workpieceMesh->z_stat = new float[workpieceMesh->points_stat] { 0, -1.0f,
			-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
}
