/*
 * AdversaryCollider.cpp
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#include "AdversaryCollider.h"

AdversaryCollider::AdversaryCollider() {

}

AdversaryCollider::~AdversaryCollider() {

}


/*
 * @brief initialize the adversaries' colliders
 * the colliders are convex hull segments in 3d that represent the obstacles???
 */
void AdversaryCollider::init_adversary_colliders(collider* adversaryCollider, collider* colliders) {
	adversaryCollider[0].points_stat = 59;
	adversaryCollider[0].x_stat = new float[adversaryCollider[0].points_stat];
	adversaryCollider[0].y_stat = new float[adversaryCollider[0].points_stat];
	adversaryCollider[0].z_stat = new float[adversaryCollider[0].points_stat];
	for (int i = 0; i < adversaryCollider[0].points_stat; i++) {
		if (colliders[7].x_stat[i] > 0.5f) {
			adversaryCollider[0].x_stat[i] = colliders[7].x_stat[i] - 0.5f;
		} else if (colliders[7].x_stat[i] < -0.5f) {
			adversaryCollider[0].x_stat[i] = colliders[7].x_stat[i] + 0.5f;
		} else {
			adversaryCollider[0].x_stat[i] = colliders[7].x_stat[i] * 0.9f;
		}

		if (colliders[7].y_stat[i] > 0.5f) {
			adversaryCollider[0].y_stat[i] = colliders[7].y_stat[i] - 0.5f;
		} else if (colliders[7].y_stat[i] < -0.5f) {
			adversaryCollider[0].y_stat[i] = colliders[7].y_stat[i] + 0.5f;
		} else {
			adversaryCollider[0].y_stat[i] = colliders[7].y_stat[i] * 0.9f;
		}

		if (colliders[7].z_stat[i] > 0.5f) {
			adversaryCollider[0].z_stat[i] = colliders[7].z_stat[i] - 0.5f;
		} else if (colliders[7].z_stat[i] < -0.5f) {
			adversaryCollider[0].z_stat[i] = colliders[7].z_stat[i] + 0.5f;
		} else {
			adversaryCollider[0].z_stat[i] = colliders[7].z_stat[i] * 0.9f;
		}
	}
}


/*
 * Do something with an adversary collider and a gripper
 * Todo: figure out what is going on here!!!
 */
void AdversaryCollider::init_adversaryColliderAndGripper(collider* adversaryCollider, collider* grippers ) {
	adversaryCollider[3].points_stat = 9;
	adversaryCollider[3].x_stat = new float[adversaryCollider[3].points_stat] {
			0, 1.0f, 1.0f, -1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f };
	adversaryCollider[3].y_stat = new float[adversaryCollider[3].points_stat] {
			0, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f };
	adversaryCollider[3].z_stat = new float[adversaryCollider[3].points_stat] {
			0, -1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
	grippers[0].points_stat = 11;
	grippers[0].x_stat = new float[grippers[0].points_stat] { 0, 0.7f, 0.7f,
			-0.7f, -0.7f, 0.7f, 0.7f, -0.7f, -0.7f, -0.7f, 0.7f };
	grippers[0].y_stat = new float[grippers[0].points_stat] { 0, 0.45f, -0.45f,
			0.45f, -0.45f, 0.45f, -0.45f - 0.5f, 0.45f, -0.45f - 0.5f, -0.45f
					- 0.5f, -0.45f - 0.5f };
	grippers[0].z_stat = new float[grippers[0].points_stat] { 0, -2.25f, -2.25f,
			-2.25f, -2.25f, 2.25f, 2.25f, 2.25f, 2.25f, 0.75f, 0.75f };
	grippers[0].color = collider::Edge { 0.5f, 0.5f, 0.5f };
	adversaryCollider[1].points_stat = 11;
	adversaryCollider[1].x_stat = new float[adversaryCollider[1].points_stat];
	adversaryCollider[1].y_stat = new float[adversaryCollider[1].points_stat];
	adversaryCollider[1].z_stat = new float[adversaryCollider[1].points_stat];
	for (int i = 0; i < adversaryCollider[1].points_stat; i++) {
		if (grippers[0].x_stat[i] > 0.5f) {
			adversaryCollider[1].x_stat[i] = grippers[0].x_stat[i] - 0.5f;
		} else if (grippers[0].x_stat[i] < -0.5f) {
			adversaryCollider[1].x_stat[i] = grippers[0].x_stat[i] + 0.5f;
		} else {
			adversaryCollider[1].x_stat[i] = grippers[0].x_stat[i] * 0.9f;
		}

		if (grippers[0].y_stat[i] > 0.5f) {
			adversaryCollider[1].y_stat[i] = grippers[0].y_stat[i] - 0.5f;
		} else if (grippers[0].y_stat[i] < -0.5f) {
			adversaryCollider[1].y_stat[i] = grippers[0].y_stat[i] + 0.5f;
		} else {
			adversaryCollider[1].y_stat[i] = grippers[0].y_stat[i] * 0.9f;
		}

		if (grippers[0].z_stat[i] > 0.5f) {
			adversaryCollider[1].z_stat[i] = grippers[0].z_stat[i] - 0.5f;
		} else if (grippers[0].z_stat[i] < -0.5f) {
			adversaryCollider[1].z_stat[i] = grippers[0].z_stat[i] + 0.5f;
		} else {
			adversaryCollider[1].z_stat[i] = grippers[0].z_stat[i] * 0.9f;
		}
	}
	grippers[1].points_stat = 11;
	grippers[1].x_stat = new float[grippers[1].points_stat] { 0, 0.7f, 0.7f,
			-0.7f, -0.7f, 0.7f, 0.7f, -0.7f, -0.7f, -0.7f, 0.7f };
	grippers[1].y_stat = new float[grippers[1].points_stat] { 0, 0.45f, -0.45f,
			0.45f, -0.45f, 0.45f + 0.5f, -0.45f, 0.45f + 0.5f, -0.45f, 0.45f
					+ 0.5f, 0.45f + 0.5f };
	grippers[1].z_stat = new float[grippers[1].points_stat] { 0, -2.25f, -2.25f,
			-2.25f, -2.25f, 2.25f, 2.25f, 2.25f, 2.25f, 0.75f, 0.75f };
	grippers[1].color = collider::Edge { 0.5f, 0.5f, 0.5f };
	adversaryCollider[2].points_stat = 11;
	adversaryCollider[2].x_stat = new float[adversaryCollider[2].points_stat];
	adversaryCollider[2].y_stat = new float[adversaryCollider[2].points_stat];
	adversaryCollider[2].z_stat = new float[adversaryCollider[2].points_stat];
	for (int i = 0; i < adversaryCollider[2].points_stat; i++) {
		if (grippers[1].x_stat[i] > 0.5f) {
			adversaryCollider[2].x_stat[i] = grippers[1].x_stat[i] - 0.5f;
		} else if (grippers[1].x_stat[i] < -0.5f) {
			adversaryCollider[2].x_stat[i] = grippers[1].x_stat[i] + 0.5f;
		} else {
			adversaryCollider[2].x_stat[i] = grippers[1].x_stat[i] * 0.9f;
		}

		if (grippers[1].y_stat[i] > 0.5f) {
			adversaryCollider[2].y_stat[i] = grippers[1].y_stat[i] - 0.5f;
		} else if (grippers[1].y_stat[i] < -0.5f) {
			adversaryCollider[2].y_stat[i] = grippers[1].y_stat[i] + 0.5f;
		} else {
			adversaryCollider[2].y_stat[i] = grippers[1].y_stat[i] * 0.9f;
		}

		if (grippers[1].z_stat[i] > 0.5f) {
			adversaryCollider[2].z_stat[i] = grippers[1].z_stat[i] - 0.5f;
		} else if (grippers[1].z_stat[i] < -0.5f) {
			adversaryCollider[2].z_stat[i] = grippers[1].z_stat[i] + 0.5f;
		} else {
			adversaryCollider[2].z_stat[i] = grippers[1].z_stat[i] * 0.9f;
		}
	}
}
