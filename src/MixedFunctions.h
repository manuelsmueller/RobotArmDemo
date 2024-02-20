/*
 * MixedFunctions.h
 *
 *  Created on: 08.01.2023
 *      Author: manuel
 */

#ifndef MIXEDFUNCTIONS_H_
#define MIXEDFUNCTIONS_H_

#include <iostream>
#include <fstream>

#include "collider.h"
#include "shader.h"
#include "HelperFunctions.h"
#include "GlobalVariables.h"
#include "FrequentOps.h"
#include "SimulationControl.h"

#include "RobotModel.h"
#include "ObstacleModels.h"
#include "WorkpieceModels.h"
#include "AdversaryCollider.h"
#include "MonitoringToolModels.h"
#include "MiscModels.h"

class MixedFunctions {
private:
	static MixedFunctions *mf;
	MixedFunctions();

public:
	static MixedFunctions* get_instance();
	virtual ~MixedFunctions();

	void update_shaders(bool repaint, Shader &shader,
			collider::VertexBuffer &vertexBufferObjects,
			collider::VertexBuffer &vertexBufferPatterns,
			collider::VertexBuffer &vertexBufferMTviewfields,
			collider::VertexBuffer &vertexBufferDistObjects);
	void log_action_space_data();
	void log_transport_result();
	void select_active_agents();

	void init_colliders(GlobalVariables *gv);
};

#endif /* MIXEDFUNCTIONS_H_ */
