/*
 * Obstacles.cpp
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#include "ObstacleModels.h"

ObstacleModels::ObstacleModels() {
	// TODO Auto-generated constructor stub

}

ObstacleModels::~ObstacleModels() {
	// TODO Auto-generated destructor stub
}

/*
 * this function inits the obstacle 1.
 * box with shape of a dice
 */
void ObstacleModels::init_obstacle1(collider* colliders) {
	colliders[12].points_stat = 9;
	colliders[12].x_stat = new float[colliders[12].points_stat] { 0, 4.0f, 4.0f,
			-4.0f, -4.0f, 4.0f, 4.0f, -4.0f, -4.0f };
	colliders[12].y_stat = new float[colliders[12].points_stat] { 0, 4.0f,
			-4.0f, 4.0f, -4.0f, 4.0f, -4.0f, 4.0f, -4.0f };
	colliders[12].z_stat = new float[colliders[12].points_stat] { 0, -4.0f,
			-4.0f, -4.0f, -4.0f, 4.0f, 4.0f, 4.0f, 4.0f };
	colliders[12].color = collider::Edge { 1.0f, 0.0f, 0.0f };
	colliders[12].patternCount = 8;
	colliders[12].pattern_indices = new int[colliders[12].patternCount] { 16,
			17, 13, 12, 15, 14, 18, 19 };
	colliders[12].x_pattern_stat = new float[colliders[12].patternCount * 4] {
			4.001f, 4.001f, 4.001f, 4.001f, 4.001f, 4.001f, 4.001f, 4.001f,
			-4.001f, -4.001f, -4.001f, -4.001f, -4.001f, -4.001f, -4.001f,
			-4.001f, 2.0f - patternSize / 2.0f, 2.0f + patternSize / 2.0f, 2.0f
					+ patternSize / 2.0f, 2.0f - patternSize / 2.0f, -2.0f
					- patternSize / 2.0f, -2.0f + patternSize / 2.0f, -2.0f
					+ patternSize / 2.0f, -2.0f - patternSize / 2.0f, 2.0f
					+ patternSize / 2.0f, 2.0f - patternSize / 2.0f, 2.0f
					- patternSize / 2.0f, 2.0f + patternSize / 2.0f, -2.0f
					+ patternSize / 2.0f, -2.0f - patternSize / 2.0f, -2.0f
					- patternSize / 2.0f, -2.0f + patternSize / 2.0f };
	colliders[12].y_pattern_stat = new float[colliders[12].patternCount * 4] {
			2.0f + patternSize / 2.0f, 2.0f - patternSize / 2.0f, 2.0f
					- patternSize / 2.0f, 2.0f + patternSize / 2.0f, -2.0f
					+ patternSize / 2.0f, -2.0f - patternSize / 2.0f, -2.0f
					- patternSize / 2.0f, -2.0f + patternSize / 2.0f, 2.0f
					- patternSize / 2.0f, 2.0f + patternSize / 2.0f, 2.0f
					+ patternSize / 2.0f, 2.0f - patternSize / 2.0f, -2.0f
					- patternSize / 2.0f, -2.0f + patternSize / 2.0f, -2.0f
					+ patternSize / 2.0f, -2.0f - patternSize / 2.0f, 4.001f,
			4.001f, 4.001f, 4.001f, 4.001f, 4.001f, 4.001f, 4.001f, -4.001f,
			-4.001f, -4.001f, -4.001f, -4.001f, -4.001f, -4.001f, -4.001f };
	colliders[12].z_pattern_stat = new float[colliders[12].patternCount * 4] {
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f,
			patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f,
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f,
			patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f,
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, -patternSize / 2.0f };
	colliders[12].x_pattern_tf = new float[colliders[12].patternCount * 4];
	colliders[12].y_pattern_tf = new float[colliders[12].patternCount * 4];
	colliders[12].z_pattern_tf = new float[colliders[12].patternCount * 4];
}

/*
 * TODO: explain what these colliders are for!
 */
void ObstacleModels::init_distoredObstacle1(collider * distortedObstacles) {
	distortedObstacles[0].points_stat = 9;
	distortedObstacles[0].x_stat =
			new float[distortedObstacles[0].points_stat] { 0, 4.0f, 4.0f, -4.0f,
					-4.0f, 4.0f, 4.0f, -4.0f, -4.0f };
	distortedObstacles[0].y_stat =
			new float[distortedObstacles[0].points_stat] { 0, 4.0f, -4.0f, 4.0f,
					-4.0f, 4.0f, -4.0f, 4.0f, -4.0f };
	distortedObstacles[0].z_stat =
			new float[distortedObstacles[0].points_stat] { 0, -4.0f, -4.0f,
					-4.0f, -4.0f, 4.0f, 4.0f, 4.0f, 4.0f };
}

/*
 * initializes th obstacle 2.
 * Todo: figure out the shape of the obstacle.
 */
void ObstacleModels::init_obstacle2(collider* colliders)
{
	colliders[13].points_stat = 9;
	colliders[13].x_stat = new float[colliders[13].points_stat] {0, 2.5f, 2.5f, -2.5f, -2.5f, 2.5f, 2.5f, -2.5f, -2.5f};
	colliders[13].y_stat = new float[colliders[13].points_stat] {0, 5.0f, -5.0f, 5.0f, -5.0f, 5.0f, -5.0f, 5.0f, -5.0f};
	colliders[13].z_stat = new float[colliders[13].points_stat] {0, -3.0f, -3.0f, -3.0f, -3.0f, 3.0f, 3.0f, 3.0f, 3.0f};
	colliders[13].color = collider::Edge {1.0f, 0.0f, 0.0f};
	colliders[13].patternCount = 6;
	colliders[13].pattern_indices = new int[colliders[13].patternCount] {23, 24, 21, 20, 22, 25};
	colliders[13].x_pattern_stat = new float[colliders[13].patternCount * 4] {2.501f, 2.501f, 2.501f, 2.501f, 2.501f, 2.501f, 2.501f, 2.501f, -2.501f, -2.501f, -2.501f, -2.501f, -2.501f, -2.501f, -2.501f, -2.501f, -patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f};
	colliders[13].y_pattern_stat = new float[colliders[13].patternCount * 4] {2.5f + patternSize / 2.0f, 2.5f - patternSize / 2.0f, 2.5f - patternSize / 2.0f, 2.5f + patternSize / 2.0f, -2.5f + patternSize / 2.0f, -2.5f - patternSize / 2.0f, -2.5f - patternSize / 2.0f, -2.5f + patternSize / 2.0f, 2.5f - patternSize / 2.0f, 2.5f + patternSize / 2.0f, 2.5f + patternSize / 2.0f, 2.5f - patternSize / 2.0f, -2.5f - patternSize / 2.0f, -2.5f + patternSize / 2.0f, -2.5f + patternSize / 2.0f, -2.5f - patternSize / 2.0f, 5.001f, 5.001f, 5.001f, 5.001f, -5.001f, -5.001f, -5.001f, -5.001f};
	colliders[13].z_pattern_stat = new float[colliders[13].patternCount * 4] {patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f};
	colliders[13].x_pattern_tf = new float[colliders[13].patternCount * 4];
	colliders[13].y_pattern_tf = new float[colliders[13].patternCount * 4];
	colliders[13].z_pattern_tf = new float[colliders[13].patternCount * 4];
}

void ObstacleModels::init_distortedObstacle2(collider* distortedObstacles) {
	distortedObstacles[1].points_stat = 9;
	distortedObstacles[1].x_stat =
			new float[distortedObstacles[1].points_stat] { 0, 2.5f, 2.5f, -2.5f,
					-2.5f, 2.5f, 2.5f, -2.5f, -2.5f };
	distortedObstacles[1].y_stat =
			new float[distortedObstacles[1].points_stat] { 0, 5.0f, -5.0f, 5.0f,
					-5.0f, 5.0f, -5.0f, 5.0f, -5.0f };
	distortedObstacles[1].z_stat =
			new float[distortedObstacles[1].points_stat] { 0, -3.0f, -3.0f,
					-3.0f, -3.0f, 3.0f, 3.0f, 3.0f, 3.0f };
}

void ObstacleModels::init_obstacle3(collider* colliders) {
	colliders[14].points_stat = 9;
	colliders[14].x_stat = new float[colliders[14].points_stat] { 0, 3.0f, 3.0f,
			-3.0f, -3.0f, 3.0f, 3.0f, -3.0f, -3.0f };
	colliders[14].y_stat = new float[colliders[14].points_stat] { 0, 3.0f,
			-3.0f, 3.0f, -3.0f, 3.0f, -3.0f, 3.0f, -3.0f };
	colliders[14].z_stat = new float[colliders[14].points_stat] { 0, -6.0f,
			-6.0f, -6.0f, -6.0f, 6.0f, 6.0f, 6.0f, 6.0f };
	colliders[14].color = collider::Edge { 1.0f, 0.0f, 0.0f };
	colliders[14].patternCount = 8;
	colliders[14].pattern_indices = new int[colliders[14].patternCount] { 31,
			30, 27, 26, 29, 28, 32, 33 };
	colliders[14].x_pattern_stat = new float[colliders[14].patternCount * 4] {
			3.001f, 3.001f, 3.001f, 3.001f, 3.001f, 3.001f, 3.001f, 3.001f,
			-3.001f, -3.001f, -3.001f, -3.001f, -3.001f, -3.001f, -3.001f,
			-3.001f, -patternSize / 2.0f, patternSize / 2.0f, patternSize
					/ 2.0f, -patternSize / 2.0f, -patternSize / 2.0f,
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f,
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f };
	colliders[14].y_pattern_stat = new float[colliders[14].patternCount * 4] {
			patternSize / 2.0f, -patternSize / 2.0f, -patternSize / 2.0f,
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			patternSize / 2.0f, patternSize / 2.0f, -patternSize / 2.0f,
			-patternSize / 2.0f, patternSize / 2.0f, patternSize / 2.0f,
			-patternSize / 2.0f, 3.001f, 3.001f, 3.001f, 3.001f, 3.001f, 3.001f,
			3.001f, 3.001f, -3.001f, -3.001f, -3.001f, -3.001f, -3.001f,
			-3.001f, -3.001f, -3.001f };
	colliders[14].z_pattern_stat = new float[colliders[14].patternCount * 4] {
			-3.0f + patternSize / 2.0f, -3.0f + patternSize / 2.0f, -3.0f
					- patternSize / 2.0f, -3.0f - patternSize / 2.0f, 3.0f
					+ patternSize / 2.0f, 3.0f + patternSize / 2.0f, 3.0f
					- patternSize / 2.0f, 3.0f - patternSize / 2.0f, -3.0f
					+ patternSize / 2.0f, -3.0f + patternSize / 2.0f, -3.0f
					- patternSize / 2.0f, -3.0f - patternSize / 2.0f, 3.0f
					+ patternSize / 2.0f, 3.0f + patternSize / 2.0f, 3.0f
					- patternSize / 2.0f, 3.0f - patternSize / 2.0f, -3.0f
					+ patternSize / 2.0f, -3.0f + patternSize / 2.0f, -3.0f
					- patternSize / 2.0f, -3.0f - patternSize / 2.0f, 3.0f
					+ patternSize / 2.0f, 3.0f + patternSize / 2.0f, 3.0f
					- patternSize / 2.0f, 3.0f - patternSize / 2.0f, -3.0f
					+ patternSize / 2.0f, -3.0f + patternSize / 2.0f, -3.0f
					- patternSize / 2.0f, -3.0f - patternSize / 2.0f, 3.0f
					+ patternSize / 2.0f, 3.0f + patternSize / 2.0f, 3.0f
					- patternSize / 2.0f, 3.0f - patternSize / 2.0f };
	colliders[14].x_pattern_tf = new float[colliders[14].patternCount * 4];
	colliders[14].y_pattern_tf = new float[colliders[14].patternCount * 4];
	colliders[14].z_pattern_tf = new float[colliders[14].patternCount * 4];
}

void ObstacleModels::init_distortedObstacle3(collider* distortedObstacles) {
	distortedObstacles[2].points_stat = 9;
	distortedObstacles[2].x_stat =
			new float[distortedObstacles[2].points_stat] { 0, 3.0f, 3.0f, -3.0f,
					-3.0f, 3.0f, 3.0f, -3.0f, -3.0f };
	distortedObstacles[2].y_stat =
			new float[distortedObstacles[2].points_stat] { 0, 3.0f, -3.0f, 3.0f,
					-3.0f, 3.0f, -3.0f, 3.0f, -3.0f };
	distortedObstacles[2].z_stat =
			new float[distortedObstacles[2].points_stat] { 0, -6.0f, -6.0f,
					-6.0f, -6.0f, 6.0f, 6.0f, 6.0f, 6.0f };
}
