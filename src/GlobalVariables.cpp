/*
 * GlobalVariables.cpp
 *
 *  Created on: 26.11.2022
 *      Author: manuel
 */

#include "GlobalVariables.h"

GlobalVariables::GlobalVariables() {


	camVectorsMT1 = new float[12];
	camVectorsMT2 = new float[12];
	camPos = new float[3]{ 75.0f, 0.0f, 10.0f };
	camAngles = new float[2]{ M_PI , 0.0f };
	lightDir = new float[3]{ -1.0f, 0.85f, -2.0f };
	camDir = new float[3]{ cos(camAngles[0]) * cos(camAngles[1]), sin(camAngles[0]) * cos(camAngles[1]), sin(camAngles[1]) };
	camHorDir = new float[3]{ sin(camAngles[0]), -cos(camAngles[0]), 0.0f };
	camVerDir = new float[3]{ -cos(camAngles[0]) * sin(camAngles[1]), -sin(camAngles[0]) * sin(camAngles[1]), cos(camAngles[1]) };
	displayPos = new float[4]{ 0.0f, 0.0f, 0.0f, 0.0f };

	MT1currentlyActive = true;
	MT2currentlyActive = true;

	simCounter = 0;
	timeCounter = 0;
	colorsCount = 2;

	asynMode = -1;
	synMode = 0;
	modeActive = false;
	initSpacePressed = false;
	initProcessed = false;
	initConfirmation = false;
	initWaitForFinish = false;
	simConfirmation = false;
	manualConfirmation = false;
	initPreviouslyProcessed = false;
	initPhase = 0;


	rotationValue = 0.0f;
	simTargetSelection = false;
	simTransport = false;
	simAskUser = false;

	rotationValue = 0.0f;

	userAbort = false;
	atStandby = true;
	currentlyTransporting = false;



	colorStartPoints = new int[colorsCount] { 0, innerPlatePoints };

	savedRot = 0.0f;

	virtSim = false;
	syncSim = false;
	paused = false;
	selectionMarkerMode = false;

	standbyPos = collider::Edge{ 0.0f, 25.0f, 20.0f };
	savedPos = collider::Edge{ standbyPos.x, standbyPos.y, standbyPos.z };

	pathAltStartPos = collider::Edge{ 0.0f, 0.0f, 0.0f };
	targetPos = collider::Edge{ standbyPos.x, standbyPos.y, standbyPos.z };
	pathStartPos = collider::Edge{ 0.0f, 0.0f, 0.0f };
	pathEndPos = collider::Edge{ 0.0f, 0.0f, 0.0f };
	pathLastPos = collider::Edge{ 0.0f, 0.0f, 0.0f };

	simSpeed = 2;


	tempCounter = 0;
	numPointsObject = totalPlatePoints;
	numPointsPattern = 0; numPointsPlannedPath = 0; numPointsRealPath = 0; numPointsMTrays = 0;


	q0ValsFirstSeg = new float[0];   q1ValsFirstSeg = new float[0];    q2ValsFirstSeg = new float[0];  q3ValsFirstSeg = new float[0];  q4ValsFirstSeg = new float[0];
	q0ValsSecondSeg = new float[0];  q1ValsSecondSeg = new float[0];   q2ValsSecondSeg = new float[0]; q3ValsSecondSeg = new float[0]; q4ValsSecondSeg = new float[0];
	q0ValsThirdSeg = new float[0];   q1ValsThirdSeg = new float[0];    q2ValsThirdSeg = new float[0];  q3ValsThirdSeg = new float[0];  q4ValsThirdSeg = new float[0];
	q0ValsTempSeg = new float[0];    q1ValsTempSeg = new float[0];     q2ValsTempSeg = new float[0];   q3ValsTempSeg = new float[0];   q4ValsTempSeg = new float[0];
	firstSegSize = 0; secondSegSize = 0; thirdSegSize = 0; tempSegSize = 0;
	secondSegLengths = new float[0];

//	activeAgents[5]={ false, false, false, false, false }; // safety, MT, loc, path, obs
//	fpsMax[6]={ 8, 20, 40, 60, 80, 160 };
//	groundTruthObstaclePositions[9]={ 0.0f, -25.0f, 0.0f, 0.0f, -25.0f, 0.0f, 0.0f, -25.0f, 0.0f };
//	distortions[9]={ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
//	colors = new collider::Edge[colorsCount]{ collider::Edge{ 0.7f, 0.7f, 0.7f }, collider::Edge{ 0.5f, 0.5f, 0.5f } };

	reward = 0.0f; reward1 = 0.0f; reward2 = 0.0f; reward3 = 0.0f;

	adv_dev_x = 0.0f; adv_dev_y = 0.0f; adv_dev_z = 0.0f;
	path_adv_counter = 0; obs_adv_counter = 0;

	object_vertices           = new collider::Vertex[0];
	object_pattern_vertices   = new collider::Vertex[0];
	planned_path_vertices     = new collider::Vertex[0];
	real_path_vertices        = new collider::Vertex[0];
	mt_viewfields_vertices    = new collider::Vertex[32];
	mt_rays_vertices          = new collider::Vertex[0];
	distorted_obs_vertices    = new collider::Vertex[72 * 3];
	ground_truth_obs_vertices = new collider::Vertex[72 * 3];
	safety_col_vertices       = new collider::Vertex[72];
	workpiece_mesh_vertices   = new collider::Vertex[72];
	workpiece_arrows_vertices = new collider::Vertex[24];

	object_edges                = new collider::Edge[0];
	object_pattern_edges        = new collider::Edge[0];
	planned_path_edges          = new collider::Edge[0];
	real_path_edges             = new collider::Edge[0];
	mt_viewfields_edges         = new collider::Edge[32];
	mt_rays_edges               = new collider::Edge[0];
	distorted_obs_edges         = new collider::Edge[72 * 3];
	ground_truth_obs_edges      = new collider::Edge[72 * 3];
	safety_col_edges            = new collider::Edge[72];
	workpiece_mesh_edges        = new collider::Edge[72];
	workpiece_arrows_edges      = new collider::Edge[24];

	qValuesCurrent     = new float[5];
	qValuesTarget      = new float[5];
	qValuesStandby     = new float[5];

	qValuesDiff        = new float[5];
	qValuesObjectStart = new float[5];
	qValuesObjectEnd   = new float[5];
	qValuesStartPoint  = new float[5];
	qValuesEndPoint    = new float[5];

	colors = new collider::Edge[colorsCount]{ collider::Edge{ 0.7f, 0.7f, 0.7f }, collider::Edge{ 0.5f, 0.5f, 0.5f } };

	colliders             = new collider[collidersCount];
	grippers              = new collider[2];
	safetyCollider        = new collider;
	distortedObstacles    = new collider[3];
	adversaryCollider     = new collider[4];
	safetyCollidersForMTs = new collider[2];
	workpieceMesh         = new collider;
}

GlobalVariables::~GlobalVariables() {
	// TODO Auto-generated destructor stub
}

/*
 * singelton pattern
 */
GlobalVariables* GlobalVariables::gv = 0;
GlobalVariables* GlobalVariables::get_instance(){
	static bool isInit=false;

//	InverseKinematic *ik;
	if(!isInit){
		gv  = new GlobalVariables();
		isInit=true;
	}
	return gv;
}
