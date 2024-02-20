/*
 * MixedFunctions.cpp
 *
 *  Created on: 08.01.2023
 *      Author: manuel
 */

#include "MixedFunctions.h"

MixedFunctions::MixedFunctions() {
	// TODO Auto-generated constructor stub

}

MixedFunctions::~MixedFunctions() {
	// TODO Auto-generated destructor stub
}

/*
 * singelton pattern
 */
MixedFunctions* MixedFunctions::mf = 0;
MixedFunctions* MixedFunctions::get_instance(){
	static bool isInit=false;
	if(!isInit){
		mf  = new MixedFunctions();
		isInit=true;
	}
	return mf;
}


// ---------------------------------------------------------------
// ---------------------- Shaders --------------------------------
// ---------------------------------------------------------------
void MixedFunctions::update_shaders(bool repaint, Shader &shader,
		collider::VertexBuffer &vertexBufferObjects,
		collider::VertexBuffer &vertexBufferPatterns,
		collider::VertexBuffer &vertexBufferMTviewfields,
		collider::VertexBuffer &vertexBufferDistObjects) {

	HelperFunctions *hf = HelperFunctions::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();

	shader.bind();
	vertexBufferObjects.bind();
	if (repaint) {
		hf->calcVerticesForObjects(gv->object_vertices, gv->object_edges,
				gv->numPointsObject, gv->totalPlatePoints);
		vertexBufferObjects.update(gv->object_vertices, gv->numPointsObject);
	}
	glDrawArrays(GL_TRIANGLES, 0, gv->numPointsObject);
	vertexBufferObjects.unbind();
	vertexBufferPatterns.bind();
	if (repaint) {
		hf->calcVerticesForPoints(gv->object_pattern_vertices,
				gv->object_pattern_edges, gv->numPointsPattern);
		vertexBufferPatterns.update(gv->object_pattern_vertices,
				gv->numPointsPattern);
	}
	glDrawArrays(GL_POINTS, 0, gv->numPointsPattern);
	vertexBufferPatterns.unbind();
	vertexBufferMTviewfields.bind();
	if (repaint) {
		hf->calcVerticesForLines(gv->mt_viewfields_vertices,
				gv->mt_viewfields_edges, 32);
		vertexBufferMTviewfields.update(gv->mt_viewfields_vertices, 32);
	}
	glDrawArrays(GL_LINES, 0, 32);
	vertexBufferMTviewfields.unbind();
	vertexBufferDistObjects.bind();
	if (repaint) {
		hf->calcVerticesForLines(gv->distorted_obs_vertices,
				gv->distorted_obs_edges,
				gv->distortedObstacles[0].facesTimes3 * 6);
		vertexBufferDistObjects.update(gv->distorted_obs_vertices,
				gv->distortedObstacles[0].facesTimes3 * 6);
	}
	glDrawArrays(GL_LINES, 0, gv->distortedObstacles[0].facesTimes3 * 6);
	vertexBufferDistObjects.unbind();
	shader.unbind();
}


void MixedFunctions::log_action_space_data(){
	GlobalVariables *gv=GlobalVariables::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	if (gv->writeDataIntoFiles) {
		fstream file;
		file.open("action_spaces.csv", std::ios_base::app);
		if (file) {
			file << to_string(gv->simCounter) + ";" + to_string(fo->actionSpaceDist) + ";" + to_string(fo->actionSpaceAng * 180.0f / M_PI) + ";cm/degree\n";
		}
		else {
			cout << "Error: writing into file not possible!" << endl;
		}
		file.close();
	}
}

void MixedFunctions::log_transport_result() {
	GlobalVariables *gv=GlobalVariables::get_instance();
	if (gv->writeDataIntoFiles) {
		fstream file;
		file.open("transport_result.csv", std::ios_base::app);
		if (file) {
			if (gv->advancedControlMode)
				file
						<< to_string(gv->simCounter)
								+ ";second_gv->path_plannig_failed;"
								+ to_string(gv->agentMode) + "\n";
			else
				file
						<< to_string(gv->simCounter)
								+ ";second_gv->path_plannig_failed\n";
		} else {
			cout << "Error: writing into file not possible!" << endl;
		}
		file.close();
	}
}



void MixedFunctions::select_active_agents() {
	GlobalVariables *gv=GlobalVariables::get_instance();
	SimulationControl *sc= SimulationControl::get_instance();

	for (int i = 0; i < 5; i++) {
		if (gv->receivedAvailableAgents[i] == '1') {
			gv->activeAgents[i] = true;
		} else {
			gv->activeAgents[i] = false;
		}
	}
	if (!gv->activeAgents[0]){
		cout
		<< "Sim. info: safety area protagonist not avilable -> using default size 1.5cm!"
		<< endl;
	}else{
		cout
		<< "Sim. info: safety area protagonist enabled!"
		<< endl;
	}

	if (!gv->activeAgents[1])
	{
		cout
		<< "Sim. info: Monitoring Tool protagonist not avilable -> MTs will not be moved!"
		<< endl;
	}else{
		cout
		<< "Sim. info: Monitoring Tool protagonist active!"
		<< endl;
	}

	if (!gv->activeAgents[2]){
		cout
		<< "Sim. info: localization error adversary not avilable -> adversary will not impact transport!"
		<< endl;
	}else{
		cout
		<< "Sim. info: localization error adversary active!"
		<< endl;
	}

	if (!gv->activeAgents[3])
	{
		cout
		<< "Sim. info: gv->path manipulation adversary not avilable -> adversary will not impact transport!"
		<< endl;
	}else{
		cout
		<< "Sim. info: path manipulation adversary active!"
		<< endl;
	}
	if (!gv->activeAgents[4])
	{
		cout
		<< "Sim. info: obstacle adversary not avilable -> adversary will not impact transport!"
		<< endl;
	}else{
		cout
		<< "Sim. info: obstacle adversary active!"
		<< endl;
	}

	if (gv->advancedControlMode) {
		sc->sim_control_select_agents();
	}
}


void MixedFunctions::init_colliders(GlobalVariables *gv) {
	RobotModel::initialize_colliders_of_robot(gv->colliders);
	AdversaryCollider::init_adversary_colliders(gv->adversaryCollider,
			gv->colliders);
	WorkpieceModels::init_workpiece1(gv->colliders);
	WorkpieceModels::init_workpiece2(gv->colliders);
	WorkpieceModels::init_workpiece3(gv->colliders);
	WorkpieceModels::init_workpiece4(gv->colliders);
	ObstacleModels::init_obstacle1(gv->colliders);
	ObstacleModels::init_distoredObstacle1(gv->distortedObstacles);
	ObstacleModels::init_obstacle2(gv->colliders);
	ObstacleModels::init_distortedObstacle2(gv->distortedObstacles);
	ObstacleModels::init_obstacle3(gv->colliders);
	ObstacleModels::init_distortedObstacle3(gv->distortedObstacles);
	MonitoringToolModels::init_monitoringTool1(gv->colliders);
	MonitoringToolModels::init_safetyCollider4MT1(gv->safetyCollidersForMTs);
	MonitoringToolModels::init_monitoringTool2(gv->colliders);
	MonitoringToolModels::init_safetyCollider4MT2(gv->safetyCollidersForMTs);
	MiscModels::init_safetyCollider(gv->safetyCollider);
	AdversaryCollider::init_adversaryColliderAndGripper(gv->adversaryCollider,
			gv->grippers);
	WorkpieceModels::init_workpiceMesh(gv->workpieceMesh);
	MiscModels::calibrate_gripper(gv->objectHeights, gv->objectOffsets,
			gv->objectGripDepths, gv->gripperInstallationOffset,
			gv->transportableObjectsCount);
}


