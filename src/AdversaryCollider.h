/*
 * AdversaryCollider.h
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#ifndef ADVERSARYCOLLIDER_H_
#define ADVERSARYCOLLIDER_H_

#include "collider.h"

class AdversaryCollider {
private:
	static constexpr float patternSize=2.5f;

public:
	AdversaryCollider();
	virtual ~AdversaryCollider();

	static void init_adversaryColliderAndGripper(collider* adversaryCollider, collider* grippers );
	static void init_adversary_colliders(collider* adversaryCollider, collider* colliders);
};

#endif /* ADVERSARYCOLLIDER_H_ */
