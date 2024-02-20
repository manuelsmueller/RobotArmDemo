/*
 * WorkpieceModels.h
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#ifndef WORKPIECEMODELS_H_
#define WORKPIECEMODELS_H_

#include "collider.h"

class WorkpieceModels {
private:
	static constexpr float patternSize=2.5f;
	static constexpr float pattern7SurfaceVec[2]{ -0.9863939238f, 0.1643989873f }, pattern8SurfaceVec[2]{ -0.9863939238f, -0.1643989873f };
	static constexpr float pattern10SurfaceVec[2]{ -0.8320503f, 0.5547002f }, pattern11SurfaceVec[2]{ -0.8320503f, -0.5547002f };

public:
	WorkpieceModels();
	virtual ~WorkpieceModels();

	static void init_workpiece1(collider* colliders);
	static void init_workpiece2(collider* colliders);
	static void init_workpiece3(collider* colliders);
	static void init_workpiece4(collider* colliders);
	static void init_workpiceMesh(collider* workpieceMesh);
};

#endif /* WORKPIECEMODELS_H_ */
