/*
 * visualization.cpp
 *
 *  Created on: XX.YY.ZZ
 *
 *      Author: Manuel Müller, Rishika Agawal, Lennard Hettich
 *      (C) IAS University of Stuttgart
 *
 * visualization.cpp
 * this module contains the main function of the simulation.
 * It is ment to summarize all the requirements for the simulation.
 *
 */
#include <iostream>
#include <fstream>
#include <random>
#include <thread>
#include <sstream>
#include <set>
#include <string>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#pragma warning(disable:4996)

using namespace std;

#undef UNICODE

#if defined(WIN32)
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#endif

#pragma comment (lib, "Ws2_32.lib")

#define GLEW_STATIC
#include <GL/glew.h>
#define SDL_MAIN_HANDLED

#if defined(WIN32)
#include <SDL.h>
#else
#include <SDL2/SDL.h>
#endif

#include "glm/glm.hpp"
#include "glm/ext/matrix_transform.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "src/Constants.h"
#include "src/shader.h"
#include "src/collider.h"
#include "src/CollisionDetection.h"

#include "src/InverseKinematic.h"
#include "src/GlobalVariables.h"
#include "src/PathPlanner.h"
#include "src/ConvHull3d.h"
#include "src/FrequentOps.h"
#include "src/Protagonists.h"
#include "src/Adversaries.h"
#include "src/SimulationControl.h"
#include "src/SimInit.h"
#include "src/MixedFunctions.h"
#include "src/RealRobotExecution.h"
#include "src/AutomateSimState.h"


// #include "src/serverObs.h"
// #include "src/serverML.h"
// #include "src/serverMT1.h"
// #include "src/serverMT2.h"
// #include "src/serverPath.h"
// #include "src/serverState.h"
#include "src/parser.h"

#define CONVHULL_3D_ENABLE

#pragma comment(lib, "SDL2.lib")
#pragma comment(lib, "glew32s.lib")
#pragma comment(lib, "opengl32.lib")

//#include "index_buffer_old.h"
#include "src/font.h"

#if defined(WIN32)
#include "server_ml.h"
#include "server_mt1.h"
#include "server_mt2.h"
#include "server_path.h"
#include "server_state.h"
#endif
// ---------------------------------------------------------
// ------------Global Variables                -------------
// ---------------------------------------------------------

default_random_engine generator1;
normal_distribution<float> distribution1(0.0f, 6.0f);

default_random_engine generator2;
normal_distribution<float> distribution2(0.0f, 2.5f);

default_random_engine generator3;
normal_distribution<float> distribution3(0.0f, 0.2f);



// ----------------------------------------------------------------------------------
// ----------------------------  MAIN                --------------------------------
// ----------------------------------------------------------------------------------

int main()
{
	// ---------------------------------------------------------
	// ------------ Variable Containers                -------------
	// ---------------------------------------------------------
	GlobalVariables *gv=GlobalVariables::get_instance();
	PathPlanner *pp = PathPlanner::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	Protagonists *prot =Protagonists::get_instance();
	Adversaries *adv = Adversaries::get_instance();
	MonitoringToolModels *mt = MonitoringToolModels::get_instance();
	SimulationControl *sc= SimulationControl::get_instance();
	SimInit *si=SimInit::get_instance();
	CollisionDetection * cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	MixedFunctions *mf = MixedFunctions::get_instance();

	RealRobotExecution *rre=RealRobotExecution::get_instance();
	AutoSim *autoss = AutoSim::get_instance();

	//---------------------------------------------------------
	//--------------Initialize random distributions -----------
	//---------------------------------------------------------
	srand((unsigned)time(NULL));
	generator1.seed((unsigned)time(NULL));
	generator2.seed((unsigned)time(NULL));
	generator3.seed((unsigned)time(NULL));

	//---------------------------------------------------------
	//------------  Initialize 3d simulation elements  --------
	//---------------------------------------------------------

	ik->initInverseKin();
	pp->initNodeList();

	mf->init_colliders(gv);
	cd->init_colliders4collision_detection_and_visualization();

	// setup the light for the scene
	MiscModels::init_light(gv->lengthLight, gv->lightDir);

	hf->init_arrays_for_graphic_card();
	hf->recalculatesCollidersOfRobot();
	mt->updateMonitoredSpace(true, true);

	//--------------------------------------------------------------------
	//-------------- Initialize simulation object's position -------------
	//--------------------------------------------------------------------
	bool writeWorkpieces =true;
	bool writeObstacles=true;
	bool boDbgPrintWPposition =false;
	bool boWaitForSpace=false;
	gv -> stateSM = "unint";
	float* obs_pos;
	//storing x, y and rotation of workpieces and monitoring tools
	float workpiecesPosition[6][3]={{31.3949 , -3.93702,  9.88307}, {25.75997, -10.9935, 263.183}, 
									{34.5 , 3.5, 312.237},{1.5, 38.75,  9.88307},
									{-26.0, 10.0, 50.0}, {26  , 10,  130.0}};

	// Create an output filestream object
    std::ofstream myfile;
	myfile.open ("test.csv",ios::out|ios::app);
	
	// Setup the obstacle server
	// setupServerObs();

	// definition of workpieces and monitoring tools.
	ObjectPositionInterface workpieces[6];
	// setting workpieces in the loop
	for(int i=0;i<6;i++){

		string coord = "";

		workpieces[i].init(i, workpiecesPosition[i][0], workpiecesPosition[i][1], 5.65 ,workpiecesPosition[i][2]);
		si->set_workpiece_position(workpieces[i]);
		
		// convert the array of coorinates into string to send it to the genetic algorithm client
		for(int j = 0; j < (sizeof(workpiecesPosition[i]) / sizeof(float)); j++)
		{
			coord += to_string(workpiecesPosition[i][j]);
			if (j < (sizeof(workpiecesPosition[i])/ sizeof(float) -1))
			{
				coord += ",";
			}

		}
	}

	//-----------------------------------------------
	//    setup the simulation window and catch the joistick
	//-----------------------------------------------

	SDL_Window* window;
	SDL_Init(SDL_INIT_EVERYTHING);

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0)
	{
		fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
	}

	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 32);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetSwapInterval(1);

	uint32_t flags = SDL_WINDOW_OPENGL;

	window = SDL_CreateWindow("Visualization", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 800, flags);
	SDL_GLContext glContext = SDL_GL_CreateContext(window);

	GLenum err = glewInit();
	if (err != GLEW_OK) {
		cout << "Error: " << glewGetErrorString(err) << endl;
		cin.get();
		return -1;
	}
	cout << "OpenGL version info: " << glGetString(GL_VERSION) << endl;

	SDL_Joystick* joystick;
	SDL_JoystickEventState(SDL_ENABLE);
	joystick = SDL_JoystickOpen(0);

	bool mouseControl;
	if (joystick) {
		cout << "Info: controller " << SDL_JoystickName(joystick) << " connected." << endl;
		mouseControl = false;
	}
	else {
		cout << "Warning: no controller connected! Switched to mouse control." << endl;
		mouseControl = true;
	}
	cout << "========================\n" << endl;

	//---------------------------------------------------------
	//--------------Local Definitions            --------------
	//---------------------------------------------------------
	Font font, fontBig, fontUI;
	font.initFont("OpenSans-Regular.ttf", 32.0f);
	fontBig.initFont("OpenSans-Regular.ttf", 64.0f);
	fontUI.initFont("OpenSans-Regular.ttf", 26.0f);

	collider::VertexBuffer vertexBufferObjects(gv->object_vertices, gv->numPointsObject);
	collider::VertexBuffer vertexBufferPatterns(gv->object_pattern_vertices, gv->numPointsPattern);
	collider::VertexBuffer vertexBufferPlannedPath(gv->planned_path_vertices, 0);
	collider::VertexBuffer vertexBufferRealPath(gv->real_path_vertices, 0);
	collider::VertexBuffer vertexBufferMTviewfields(gv->mt_viewfields_vertices, 0);
	collider::VertexBuffer vertexBufferMTrays(gv->mt_rays_vertices, 0);
	collider::VertexBuffer vertexBufferDistObjects(gv->distorted_obs_vertices, 0);
	collider::VertexBuffer vertexBufferGroundTruths(gv->distorted_obs_vertices, 0);
	collider::VertexBuffer vertexBufferSafetyCol(gv->safety_col_vertices, 0);
	collider::VertexBuffer vertexBufferWorkpieceMesh(gv->workpiece_mesh_vertices, 0);
	collider::VertexBuffer vertexBufferWorkpieceArrows(gv->workpiece_arrows_vertices, 0);

	Shader fontShader("font.vs", "font.fs");
	Shader shader("basic.vs", "basic.fs");
	shader.bind();

	uint64_t perfCounterFrequency = SDL_GetPerformanceFrequency();
	float delta = 0.0f, fps = 0.0f;

	uint64_t startCounter, endCounter, fpsInit, fpsEnd;
	bool close = false, repaint = true, showSafetySpace = false;
	bool forwards = false, backwards = false, left = false, right = false, up = false, down = false, rotationUp = false, rotationDown = false;

	float pausedPixelOffsetsX, pausedPixelOffsetsY;
	fontBig.getStringPixelOffset("PAUSED");
	pausedPixelOffsetsX = fontCenter[0];  pausedPixelOffsetsY = fontCenter[1];
	int frameCounter = 0;
	//---------------------------------------------------------
	//--------------Initialize Interface         --------------
	//---------------------------------------------------------
	

	if (mouseControl) {
		std::cerr << "Mouse Conrol not implemented! Return..."<<std::endl;
		//  return 1;
		//ShowCursor(false);
	}else{
		std::cout << "config done..."<<std::endl;
	}

	// setupServerPATH(gv->virtualMode);
	// setupServerSTATE(gv->virtualMode);
//#endif



	//---------------------------------------------------------
	//--------------Actual simulation loop       --------------
	//---------------------------------------------------------
	while (!close) {
		startCounter = SDL_GetPerformanceCounter();
		if (frameCounter == 0) {
			fpsInit = SDL_GetPerformanceCounter();
		}

		// if (mouseControl) {
		// 	gv->axisX = 0; gv->axisY = 0;
		// 	rotationUp = false;
		// 	rotationDown = false;
		// }

//		-------------------------------------modeActive
//		Simulation environment control initial state.
//		-------------------------------------
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT) {
				close = sc->sim_control_close();
			}

			if (event.type == SDL_KEYUP) {
				repaint=sc->sim_control_keyup(event,repaint,boWaitForSpace);
			}

			// if (mouseControl) {
			// 	sc->sim_control_mouse_ctrl(event, window, repaint,forwards,backwards,left,right, up, down, rotationDown, rotationUp);
			// }
			else {
				sc->sim_control_joystick(event, repaint,forwards,backwards,left,right, up, down, rotationDown, rotationUp);
			}
		}
		//		-------------------------------------
		//		take action automatically
		//		-------------------------------------
		if(gv -> autoFlag == 0)// clear workpieces / Initialise
		{
			// get the obstacle coordinates
			// receiveMessageObs();

			// if(gv -> stateMessageObs == "connection_error")
			// {
			// 	cout<<"Info: Could not receive the obstacle position"<< endl;
			// 	myfile.close();
			// 	break;

			// }

			// // stop the simulator once generation limit has reached
			// else if(gv -> stateMessageObs == "Completed generation")
			// {
			// 	cout<<"Info: Generation Complete"<< endl;
			// 	// stop logging the data
			// 	myfile.close();
			// 	sendMessageObs("process complete");
			// 	break;


			// }
			// else
			// {
			// 	//parse the string
			// 	obs_pos = string_parser(gv ->stateMessageObs);
				

			// }

				//obs_pos = string_parser(gv ->stateMessageObs);

				// gv->obstacles[0].init(0,*(obs_pos + 0),*(obs_pos + 1),4,*(obs_pos + 2)) ;
				// gv->obstacles[1].init(1,*(obs_pos + 3),*(obs_pos + 4),3,*(obs_pos + 5))  ;
				// gv->obstacles[2].init(2,*(obs_pos + 6),*(obs_pos + 7),6,*(obs_pos + 8))  ;

				// obstacle position hardcoded for showcase
				gv->obstacles[0].init(0, 25.0,27.0,0.0,0.0) ;
				gv->obstacles[1].init(1,-22.0,-15.0,0.0,0.0)  ;
				gv->obstacles[2].init(2,18.0,5.0,0.0,0.0)  ;
				
				gv->obstacles[0].print();
				gv->obstacles[1].print();
				gv->obstacles[2].print();

				// storing obstacle positions in csv file
				// std::string obs1_pos = std::to_string(*(obs_pos + 0)) + ";" + std::to_string(*(obs_pos + 1)) + ";" + std::to_string(*(obs_pos + 2));
				// std::string obs2_pos = std::to_string(*(obs_pos + 3)) + ";" + std::to_string(*(obs_pos + 4)) + ";" + std::to_string(*(obs_pos + 5));
				// std::string obs3_pos = std::to_string(*(obs_pos + 6)) + ";" + std::to_string(*(obs_pos + 7)) + ";" + std::to_string(*(obs_pos + 8));
				// myfile << obs1_pos;
				// myfile << ",";
				// myfile << obs2_pos;
				// myfile << ",";
				// myfile << obs3_pos;
				// myfile << ",";

			autoss -> automateInit();
			gv -> autoFlag = 1;

		}
		else if(gv -> autoFlag ==1) // set wokpieces
		{
			autoss -> automateSetWorkpiece();
			cout<< "auto.info: workpiece set" <<endl;
			if(gv -> initProcessed)
			{
				gv -> autoFlag = 2;
			
			}	

		}

		else if(gv -> autoFlag == 2) // simulate / obstacles placement
		{
			cout<< "auto.info: place obstacles" <<endl;
			autoss -> automateSimulation();
			gv -> autoFlag = 3;

		}

		else if((gv -> autoFlag == 3) && (gv -> simTargetSelection) ) //workpiece and target position selection
		{
			repaint = true;
			cout << " auto.info: Start button pressed in joystick" <<endl;
			if(autoss -> automateTransportation() )
			{
				gv -> autoFlag = 4;

			}
			
		}

		else if(gv -> autoFlag == 5) // no execution on real robot
		{
			cout << "auto.info: no execution on real robot!" << endl;
			gv->modeActive = false;

			// storing collision result, object ID with which robot collided (ID = -1, if no collision)
			// and ligh class in csv file for the respective obstacle position
			myfile << gv->collision;
			myfile << ",";
			myfile << gv->collisionObsIndex;
			myfile << ",";
			myfile << gv->light_intensity;
			myfile << "\n";
			gv -> autoFlag = 0;

			cout<< "-----------------------------------------------"<< endl;
			cout<< "collision :" << gv ->collision<< endl;
			cout<< "-----------------------------------------------"<< endl;
			

			// sendMessageObs(std::to_string(gv ->collision));
			gv->collisionObsIndex = -1;
		}
		//		-------------------------------------
		//		take action to the user input
		//		-------------------------------------
		glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		sc->sim_control_take_action_to_user_input(repaint,forwards,backwards,left,right, up, down, rotationDown, rotationUp);

		//		-------------------------------------
		//		state machine
		//		-------------------------------------
		if (!gv->modeActive) {
			gv->synMode = gv->asynMode;
		}
		if (gv->initWaitForFinish) {
			// cout << "debug: main " << __LINE__ <<endl;
			sc->sim_control_wait_for_completed_init(boWaitForSpace);
		}

		if (!gv->virtSim && gv->synMode == 0 && gv->transportPhase == 0) {
			// cout << "debug: main " << __LINE__ <<endl;
			sc->train_agents(repaint,
					generator1,distribution1,
					generator2,distribution2,
					generator3,distribution3);
		}
		else if (!gv->virtSim && gv->synMode == 0 && gv->transportPhase == 1) {
			// cout << "debug: main " << __LINE__ <<endl;
			rre->sim_control_transportPhase1(repaint,
					generator1,distribution1,
					generator2,distribution2,
					generator3,distribution3);
		}
		else if (!gv->virtSim && gv->synMode == 0 && gv->transportPhase == 2) {
			// cout << "debug: main " << __LINE__ <<endl;
			rre->sim_control_transportPhas2(
					repaint,
					generator1,distribution1,
					generator2,distribution2,
					generator3,distribution3);
		}
		/*
		 * Init Config
		 */
		else if (gv->synMode == 1 && gv->initPhase == 0 && gv->initConfirmation) {

			cout <<"Init Config next"<<endl;
			sc->sim_control_clean_objects(repaint);
		}
		/*
		 * Main Init State
		 */
		else if (gv->synMode == 1 && gv->initSpacePressed && gv->initPhase != 0) {
			// init positions of WPs and MTs
			// not every combination works.
			si->sim_control_simple_add_workpiece_on_init(
					repaint, writeWorkpieces, boWaitForSpace,
					generator1,distribution1,
					generator2,distribution2,
					generator3,distribution3);

			if(boDbgPrintWPposition){
				ObjectPositionInterface * workpieces2 = si->get_workpiecePositions();
				cout << "------ workpiece positions -----"<<endl;
				for(int i=0;i<6;i++){
					workpieces2[i].print();
				}
				cout << "------ ------ -----"<<endl;
			}

		}
		/*
		 * Simulation Mode Start
		 */
		else if (gv->synMode == 2 && !gv->simTargetSelection && !gv->simTransport && gv->simConfirmation)
		{
			// cout << "debug: main " << __LINE__ <<endl;

			if (!gv->MT1currentlyActive || !gv->MT2currentlyActive) {
				cout << "Error: a MT is not active!" << endl;
			}

			if (gv->advancedControlMode) {
				sc->sim_control_enter_seed(generator1, distribution1, generator2, distribution2, generator3, distribution3);
			}

			gv->modeActive = true;
			gv->collision = false;
			gv->grippingWidth = 0.0f;

			sc->sim_control_print_prepare_models();
			//mf->select_active_agents();

			// deactivate all agents
			gv->activeAgents[0]=false;
			gv->activeAgents[1]=false;
			gv->activeAgents[2]=false;
			gv->activeAgents[3]=false;
			gv->activeAgents[4]=false;

			if (gv->advancedControlMode) {
				sc->sim_control_select_agents();
			}

			sc->sim_control_do_something2();

			if (gv->virtualMode) {
				sc->sim_control_virtual_mode1();
			}
			else {
				rre->sim_control_physical_mode();
			}

			sc->sim_control_apply_changes_from_init();

			gv->simCounter++;
			gv->timeCounter = 0;

			cout << endl;
			cout << "ACTION SPACE INFO: " << fo->actionSpaceSamples << " samples, " << fo->actionSpaceDist << "cm dist. range, " << fo->actionSpaceAng * 180.0f / M_PI << " degree angle range" << endl;
			mf->log_action_space_data();

			sc->sim_control_apply_agents_actions();

			cout << "DISCRETIZATION: " << fo->currentActionSpaceDist << "cm dist. range, " << fo->currentActionSpaceAng * 180.0f / M_PI << " degree angle range" << endl;
			cout << "DETECTIONS COUNTS: " << fo->detectionCounts[0] << ", " << fo->detectionCounts[1] << ", " << fo->detectionCounts[2] << ", " << fo->detectionCounts[3] << ", " << fo->detectionCounts[4] << ", " << fo->detectionCounts[5] << ", " << fo->detectionCounts[6] << endl;
			

			gv->valid = true;
			gv->fittingPatternsMax = -1;

			// this step is relevant for placing the obstacles
			// -------------------------------------------------

			if(sc->sim_control_position_obstacles(gv->obstacles, writeObstacles)){
				cout << "E: invalid result @ mail l. " << __LINE__ <<endl;

			}else{
				// cout << "debug: main " << __LINE__ <<endl;
			}

			std::cout<<"----- FINAL2 obstacle positions ----"<<std::endl;
			for(int i=0;i<3;i++){
				std::cout<<"("<<gv->obstacles[i].index<<","<<gv->obstacles[i].x<<","<<gv->obstacles[i].y<<","<<gv->obstacles[i].z<<","<<gv->obstacles[i].angle<<")"<<std::endl;
			}
			std::cout<<"----- ----- ----"<<std::endl;

			cout << "FITTING PATTERNS: " << gv->fittingPatternsMax << " / " << totalPatternCount * 2 << endl;

			sc->sim_control_recalculate_colliders();
			mt->updateMonitoredSpace(true, true);

			if (gv->valid) {
				for (int i = 0; i < 3; i++) {
					gv->tempDetectedObsPosX[i] = gv->currentDetectedObsPosX[i];
					gv->tempDetectedObsPosY[i] = gv->currentDetectedObsPosY[i];
				}
			}

			sc->sim_control_calc_edges();
			repaint = true;
		}
		else if (gv->synMode == 2 && !gv->simTargetSelection && gv->simTransport && gv->transportPhase == 0 && !gv->virtSim)
		{
			// transport phase 0
			// cout << "debug: main " << __LINE__ <<endl;
			sc->sim_control_do_something4(repaint,
					generator1,distribution1,
					generator2,distribution2,
					generator3,distribution3);
		}
		else if (gv->synMode == 2 && !gv->simTargetSelection && gv->simTransport && gv->transportPhase == 1 && !gv->virtSim)
		{
			// transport phase 1
			repaint = sc->do_something5(repaint,
					generator1, distribution1,
					generator2, distribution2,
					generator3, distribution3);
		}
		else if (gv->synMode == 2 && !gv->simTargetSelection && gv->simTransport && gv->transportPhase == 2 && !gv->virtSim)
		{
			// transport phase 2
			repaint = sc->do_something6(repaint,
					generator1, distribution1,
					generator2, distribution2,
					generator3, distribution3);
		}
		else if (gv->synMode == 3 && gv->realExecPhase == 0) {
			// real transport (synMode ==3), execution phase 0
			repaint = rre->real_execution_phase0(repaint);
		}
		else if (gv->synMode == 3 && gv->realExecPhase == 1) {
			// position monitoring tools
			// -------------------------
			repaint = rre->position_monitoring_tools1(repaint,
					generator1, distribution1,
					generator2, distribution2,
					generator3, distribution3);
		}
		else if (gv->synMode == 3 && gv->realExecPhase == 2) {

			repaint = rre->execute_real_phase2(repaint,
					generator1, distribution1,
					generator2, distribution2,
					generator3, distribution3);
		}
		else if (gv->synMode == 3 && gv->realExecPhase == 3) {
			repaint = rre->real_exec_phase3(repaint,
					generator1, distribution1,
					generator2, distribution2,
					generator3, distribution3);
		}
		else if (gv->synMode == 3 && gv->realExecPhase == 4) {
			repaint = rre->real_exec_phase4(repaint,
					generator1, distribution1,
					generator2, distribution2,
					generator3, distribution3);
		}
		else if (gv->synMode == 3 && gv->realExecPhase == 5) {
			repaint = rre->real_exec_phase5(repaint,
					generator1, distribution1,
					generator2, distribution2,
					generator3, distribution3);
		}
		else if (gv->synMode == 3 && gv->realExecPhase == 6) {
			repaint = rre->real_exec_phase6(repaint);
		}
		else if (gv->synMode == 3 && gv->realExecPhase == 7) {
			repaint = rre->real_exec_phase7(repaint,
					generator1, distribution1,
					generator2, distribution2,
					generator3, distribution3);
		}
		else if (gv->synMode == 4 && !gv->simTargetSelection && !gv->simTransport && gv->manualConfirmation)
		{
			gv->userAbort = false;
			gv->modeActive = true;
			gv->collision = false;

			if (gv->atStandby) {
				gv->MT1currentlyActive = true; gv->MT2currentlyActive = true;
				for (int i = 0; i < 5; i++) {
					gv->qValuesCurrent[i] = gv->qValuesStandby[i];
				}
				gv->pathAltStartPos.x = gv->standbyPos.x; gv->pathAltStartPos.y = gv->standbyPos.y; gv->pathAltStartPos.z = gv->standbyPos.z;
			}
			else {
				for (int i = 0; i < 5; i++) {
					gv->qValuesCurrent[i] = gv->robotPoseAtAbort[i + 3];
				}
				gv->pathAltStartPos.x = gv->robotPoseAtAbort[0]; gv->pathAltStartPos.y = gv->robotPoseAtAbort[1]; gv->pathAltStartPos.z = gv->robotPoseAtAbort[2];

				if (gv->currentlyTransporting) {
					hf->reloadPathRelatedData(1);
					gv->MT1currentlyActive = gv->MT1currentlyActiveTemp; gv->MT2currentlyActive = gv->MT2currentlyActiveTemp;
				}
				else {
					gv->MT1currentlyActive = true; gv->MT2currentlyActive = true;
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

			cd->recalculateCollider(&gv->colliders[3], 0, 0, 17.4f, (-gv->qValuesCurrent[0] + 90) / 180.0f * M_PI, 0.0f, 0.0f);
			cd->recalculateCollider(&gv->colliders[4], (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Sin + 4.5f * gv->q0Cos, (ik->len1 + ik->len2 / 2.0f * gv->q1Sin) * gv->q0Cos - 4.5f * gv->q0Sin, ik->len0 + ik->len2 / 2.0f * gv->q1Cos, (-gv->qValuesCurrent[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesCurrent[1]) / 180.0f * M_PI, 0);
			cd->recalculateCollider(&gv->colliders[5], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Sin - 4.155f * gv->q0Cos, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 / 2.0f * gv->q12Sin) * gv->q0Cos + 4.155f * gv->q0Sin, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 / 2.0f * gv->q12Cos, (-gv->qValuesCurrent[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesCurrent[1] - gv->qValuesCurrent[2]) / 180 * M_PI, 0);
			cd->recalculateCollider(&gv->colliders[6], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->lastSegmentMid * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->lastSegmentMid * gv->q123Cos, (-gv->qValuesCurrent[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180 * M_PI, 0);
			cd->recalculateCollider(&gv->colliders[7], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos, (-gv->qValuesCurrent[0] + 90) / 180.0f * M_PI, (90 - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180 * M_PI, gv->qValuesCurrent[4] / 180.0f * M_PI);

			gv->grippingWidth = gv->realGripperWidth;

			cd->updateMatricesTransport((-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, (gv->qValuesCurrent[4] + 0) / 180.0f * M_PI, gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
			cd->rotationMatrix(gv->q0Cos * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
			gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

			cd->recalculateColliderTransport(&gv->grippers[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin) * gv->q0Cos + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos + gv->grippingDiffVector[2], (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, gv->qValuesCurrent[4] / 180.0f * M_PI);
			cd->recalculateColliderTransport(&gv->grippers[1], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin) * gv->q0Sin - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Sin) * gv->q0Cos - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->gripperInstallationOffset + 2.25f) * gv->q123Cos - gv->grippingDiffVector[2], (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, gv->qValuesCurrent[4] / 180.0f * M_PI);

			gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
			gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
			gv->numPointsPlannedPath = 0;

			gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
			gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
			gv->numPointsRealPath = 0;

			if (gv->currentlyTransporting) {
				cd->updateMatricesTransport((-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, (gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin * gv->q0Sin, gv->q123Sin * gv->q0Cos, gv->q123Cos);
				if (gv->objectToTransport == 15 || gv->objectToTransport == 16) {
					cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, (gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
				}
				else {
					cd->recalculateColliderTransport(&gv->colliders[gv->objectToTransport], (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, (gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI, true);
				}
				cd->recalculateColliderTransport(gv->safetyCollider, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Sin, (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin) * gv->q0Cos, ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos, (-gv->qValuesCurrent[0] + 90.0f) / 180.0f * M_PI, (180.0f - gv->qValuesCurrent[1] - gv->qValuesCurrent[2] - gv->qValuesCurrent[3]) / 180.0f * M_PI, (gv->qValuesCurrent[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
				hf->calcEdgesForMesh(gv->safety_col_edges, gv->safetyCollider, 0);

				for (int i = 0; i < gv->transportableObjectsCount; i++) {
					if (gv->transportableObjects[i] != gv->objectToTransport) {
						if (i < gv->transportableObjectsCount - 2) {
							cd->recalculateCollider(&gv->colliders[gv->transportableObjects[i]], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, true);
						}
						else if (gv->transportableObjects[i] == 15) {
							cd->recalculateCollider(&gv->colliders[15], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, false, true, gv->camVectorsMT1);
						}
						else  if (gv->transportableObjects[i] == 16) {
							cd->recalculateCollider(&gv->colliders[16], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, false, true, gv->camVectorsMT2);
						}
					}
				}
			}
			else {
				for (int i = 0; i < gv->transportableObjectsCount; i++) {
					if (i < gv->transportableObjectsCount - 2) {
						cd->recalculateCollider(&gv->colliders[gv->transportableObjects[i]], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, true);
					}
					else if (gv->transportableObjects[i] == 15) {
						cd->recalculateCollider(&gv->colliders[15], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, false, true, gv->camVectorsMT1);
					}
					else  if (gv->transportableObjects[i] == 16) {
						cd->recalculateCollider(&gv->colliders[16], gv->initPositionsX[i], gv->initPositionsY[i], gv->objectOffsets[i], gv->initPositionsAngles[i], 0, 0, false, true, gv->camVectorsMT2);
					}
				}
			}
			for (int i = 12; i <= 14; i++) {
				cd->recalculateCollider(&gv->colliders[i], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
			}

			gv->tempCounter = gv->totalPlatePoints;
			for (int i = 0; i < 3; i++) {
				gv->tempCounter += gv->colliders[i].facesTimes3;
			}
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[3], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[4], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[5], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[6], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[7], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[8], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[9], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[10], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[11], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[12], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[13], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[14], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[15], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[16], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[0], gv->tempCounter);
			hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[1], gv->tempCounter);

			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
			hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);

			gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[8], 0);
			gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[9], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[10], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[11], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[12], gv->tempCounter);
			gv->tempCounter = hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[13], gv->tempCounter);
			hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[14], gv->tempCounter);

			gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges, gv->numPointsMTrays, 0);
			gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices, gv->numPointsMTrays, 0);
			gv->numPointsMTrays = 0;

			if (gv->virtualMode) {

				if (gv->groundTruthObstaclePositions[1] > -25.0f + cd->epsilon3) {
					cd->recalculateCollider(&gv->colliders[12], gv->groundTruthObstaclePositions[0], gv->groundTruthObstaclePositions[1], 4.0f, gv->groundTruthObstaclePositions[2], 0, 0, true);
					cd->recalculateCollider(&gv->colliders[13], gv->groundTruthObstaclePositions[3], gv->groundTruthObstaclePositions[4], 3.0f, gv->groundTruthObstaclePositions[5], 0, 0, true);
					cd->recalculateCollider(&gv->colliders[14], gv->groundTruthObstaclePositions[6], gv->groundTruthObstaclePositions[7], 6.0f, gv->groundTruthObstaclePositions[8], 0, 0, true);

					gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[12], 0);
					gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[13], gv->tempCounter);
					hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[14], gv->tempCounter);
				}
				else {
					do {
						gv->collision_init = false;
						cd->recalculateCollider(&gv->colliders[12], ((float)rand() / RAND_MAX) * 110.0f - 55.0f, ((float)rand() / RAND_MAX) * 67.5f - 12.5f, 4.0f, (2 * M_PI - cd->epsilon5) * ((float)rand() / RAND_MAX), 0, 0, true);

						if (cd->checkForCollision(&gv->colliders[12], &gv->colliders[8])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[9])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[10])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[11])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[1])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[15])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[12], &gv->colliders[16])) {
							gv->collision_init = true;
						}
					} while (gv->collision_init);

					do {
						gv->collision_init = false;
						cd->recalculateCollider(&gv->colliders[13], ((float)rand() / RAND_MAX) * 110.0f - 55.0f, ((float)rand() / RAND_MAX) * 67.5f - 12.5f, 3.0f, (2 * M_PI - cd->epsilon5) * ((float)rand() / RAND_MAX), 0, 0, true);

						if (cd->checkForCollision(&gv->colliders[13], &gv->colliders[8])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[9])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[10])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[11])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[12])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[1])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[15])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[13], &gv->colliders[16])) {
							gv->collision_init = true;
						}
					} while (gv->collision_init);

					do {
						gv->collision_init = false;
						cd->recalculateCollider(&gv->colliders[14], ((float)rand() / RAND_MAX) * 110.0f - 55.0f, ((float)rand() / RAND_MAX) * 67.5f - 12.5f, 6.0f, (2 * M_PI - cd->epsilon5) * ((float)rand() / RAND_MAX), 0, 0, true);

						if (cd->checkForCollision(&gv->colliders[14], &gv->colliders[8])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[9])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[10])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[11])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[12])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[13])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[1])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[15])) {
							gv->collision_init = true;
						}
						if (!gv->collision_init && cd->checkForCollision(&gv->colliders[14], &gv->colliders[16])) {
							gv->collision_init = true;
						}
					} while (gv->collision_init);

					gv->groundTruthObstaclePositions[0] = gv->colliders[12].offsets[0]; gv->groundTruthObstaclePositions[1] = gv->colliders[12].offsets[1]; gv->groundTruthObstaclePositions[2] = gv->colliders[12].angles[0];
					gv->groundTruthObstaclePositions[3] = gv->colliders[13].offsets[0]; gv->groundTruthObstaclePositions[4] = gv->colliders[13].offsets[1]; gv->groundTruthObstaclePositions[5] = gv->colliders[13].angles[0];
					gv->groundTruthObstaclePositions[6] = gv->colliders[14].offsets[0]; gv->groundTruthObstaclePositions[7] = gv->colliders[14].offsets[1]; gv->groundTruthObstaclePositions[8] = gv->colliders[14].angles[0];

					gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[12], 0);
					gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[13], gv->tempCounter);
					hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[14], gv->tempCounter);
				}

				fo->resetDetectedPatterns();
				for (int i = 0; i < totalPatternCount; i++) {
					if (mt->isPatternVisible(i, 1, true)) {
						hf->getMTPositionData(1);
						fo->calcPosFromMTData(&gv->colliders[15], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], i);
						fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);

						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

						gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;

						gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}

					if (mt->isPatternVisible(i, 2, true)) {
						hf->getMTPositionData(2);
						fo->calcPosFromMTData(&gv->colliders[16], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], i);
						fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);

						gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

						gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;

						gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
						gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
						gv->numPointsMTrays++;
					}
				}
				fo->evaluateDetections(false);
			}
			else {
				if (gv->MT1currentlyActive && gv->MT2currentlyActive) {
//#if defined(WIN32)
					 sendMessageMT1("state_request");
					 sendMessageMT2("state_request");
					//thread mt1Thread( receiveMessageMT1);
					//thread mt2Thread(receiveMessageMT2);
					receiveMessageMT1();
					receiveMessageMT2();
					//mt1Thread.join();
					//mt2Thread.join();
//#endif
				}
				else if (gv->MT1currentlyActive && !gv->MT2currentlyActive) {
//#if defined(WIN32)
					sendMessageMT1("state_request");
					receiveMessageMT1();
//#endif
				}
				else if (!gv->MT1currentlyActive && gv->MT2currentlyActive) {
//#if defined(WIN32)
					 sendMessageMT2("state_request");
					 receiveMessageMT2();
//#endif
				}
				else {
					cout << "Real exec. error: both MTs inactive!" << endl;
				}

				string splitted;

				fo->resetDetectedPatterns();
				if (gv->MT1currentlyActive) {
					vector<string> splitList1;
					std::stringstream string_stream_1;
					string_stream_1.str(gv->stateMessageMT1);
					while (getline(string_stream_1, splitted, ',')) {
						splitList1.push_back(splitted);
					}

					if (splitList1.size() > 0) {
						for (int i = 0; i < stoi(splitList1[splitList1.size() - 1]) * 5; i += 5) {
							fo->distortedRelativePosition[0] = stof(splitList1[i + 1]);
							fo->distortedRelativePosition[1] = stof(splitList1[i + 2]);
							fo->distortedRelativePosition[2] = stof(splitList1[i + 3]);
							fo->distortedRelativePosition[3] = stof(splitList1[i + 4]);
							fo->calcPosFromMTData(&gv->colliders[15], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], stoi(splitList1[i]));
							fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList1[i])] - 8);

							gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

							gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
							gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
							gv->numPointsMTrays++;

							gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
							gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
							gv->numPointsMTrays++;
						}
					}
				}

				if (gv->MT2currentlyActive) {
					vector<string> splitList2;
					std::stringstream string_stream_2;
					string_stream_2.str(gv->stateMessageMT2);
					while (getline(string_stream_2, splitted, ',')) {
						splitList2.push_back(splitted);
					}

					if (splitList2.size() > 0) {
						for (int i = 0; i < stoi(splitList2[splitList2.size() - 1]) * 5; i += 5) {
							fo->distortedRelativePosition[0] = stof(splitList2[i + 1]);
							fo->distortedRelativePosition[1] = stof(splitList2[i + 2]);
							fo->distortedRelativePosition[2] = stof(splitList2[i + 3]);
							fo->distortedRelativePosition[3] = stof(splitList2[i + 4]);
							fo->calcPosFromMTData(&gv->colliders[16], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], stoi(splitList2[i]));
							fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList2[i])] - 8);

							gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

							gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
							gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
							gv->numPointsMTrays++;

							gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
							gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
							gv->numPointsMTrays++;
						}
					}
				}
				fo->evaluateDetections(false);
			}

			if (fo->detectionCounts[4] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[0] = 0.0f; gv->currentDetectedObsPosY[0] = 0.0f; gv->currentDetectedObsPosA[0] = 0.0f;
			}
			else {
				cd->recalculateCollider(&gv->distortedObstacles[0], fo->evaluatedDetections[4].x, fo->evaluatedDetections[4].y, 4.0f, fo->evaluatedDetections[4].z, 0, 0);
				gv->currentDetectedObsPosX[0] = fo->evaluatedDetections[4].x; gv->currentDetectedObsPosY[0] = fo->evaluatedDetections[4].y; gv->currentDetectedObsPosA[0] = fo->evaluatedDetections[4].z;
			}
			if (fo->detectionCounts[5] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[1] = 0.0f; gv->currentDetectedObsPosY[1] = 0.0f; gv->currentDetectedObsPosA[1] = 0.0f;
			}
			else {
				cd->recalculateCollider(&gv->distortedObstacles[1], fo->evaluatedDetections[5].x, fo->evaluatedDetections[5].y, 3.0f, fo->evaluatedDetections[5].z, 0, 0);
				gv->currentDetectedObsPosX[1] = fo->evaluatedDetections[5].x; gv->currentDetectedObsPosY[1] = fo->evaluatedDetections[5].y; gv->currentDetectedObsPosA[1] = fo->evaluatedDetections[5].z;
			}
			if (fo->detectionCounts[6] == 0) {
				cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
				gv->currentDetectedObsPosX[2] = 0.0f; gv->currentDetectedObsPosY[2] = 0.0f; gv->currentDetectedObsPosA[2] = 0.0f;
			}
			else {
				cd->recalculateCollider(&gv->distortedObstacles[2], fo->evaluatedDetections[6].x, fo->evaluatedDetections[6].y, 6.0f, fo->evaluatedDetections[6].z, 0, 0);
				gv->currentDetectedObsPosX[2] = fo->evaluatedDetections[6].x; gv->currentDetectedObsPosY[2] = fo->evaluatedDetections[6].y; gv->currentDetectedObsPosA[2] = fo->evaluatedDetections[6].z;
			}

			gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[0], 0);
			gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[1], gv->tempCounter);
			hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2], gv->tempCounter);

			gv->selectedWorkpieceIndex = -1;
			gv->selectedTargetPosition[0] = 10000.0f; gv->selectedTargetPosition[1] = 0.0f; gv->selectedTargetPosition[2] = 0.0f; gv->selectedTargetPosition[3] = 0.0f;
			gv->simTargetSelection = true;

			repaint = true;
		}
		else if (gv->synMode == 4 && !gv->simTargetSelection && gv->simTransport)
		{
			if (gv->manualMode == 0) {
				if (gv->realExecSubPhase == 2) {
					if (fo->checkUnfinishedTransport(generator1,distribution1, generator2,distribution2,generator3,distribution3)) {
						gv->transportPhase = 1;
						gv->grippingWidth = gv->grippingWidthFixed;

						gv->objectPosReachable = false;
						for (int i = 0; i < 10; i++) {
							if (pp->calcPath(gv->qValuesObjectEnd, gv->qValuesCurrent, &gv->pathEndPos, &gv->pathAltStartPos, false, generator1,distribution1, generator2,distribution2,generator3,distribution3)) {
								gv->objectPosReachable = true;
								break;
							}
						}

						if (gv->objectPosReachable) {
							cd->recalculateCollider(gv->safetyCollider, 0.0f, 0.0f, 10.0e5, 0, 0, 0);
							hf->calcEdgesForMesh(gv->safety_col_edges, gv->safetyCollider, 0);
							pp->optimizePath();

							gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
							gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
							gv->numPointsPlannedPath = 0;
							gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
							gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
							gv->numPointsRealPath = 0;

							fo->drawPathOnly(2);
							gv->pathLastPos = collider::Edge{ 0.0f, 0.0f, 0.0f };

							fo->writeIntoTempSegments();
							gv->pathDataForKuka = "normal_no_gripper,";
							for (int i = 0; i < gv->tempSegSize; i++) {
								gv->pathDataForKuka += hf->to_string_with_precision(gv->q0ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4) + ",";
							}
							gv->pathDataForKuka += hf->to_string_with_precision(0.0f, 4);

//#if defined(WIN32)		
							// cout << "debug: main " << __LINE__ <<endl;
							 sendMessagePATH(gv->pathDataForKuka);
							 receiveMessagePATH();
//#endif

							gv->realGripperWidth = gv->grippingWidthFixed;

							gv->realExecSubPhase++;

							repaint = true;
						}
						else {
							cout << "Real exec. info: second gv->path planning and real transport failed!" << endl;
							gv->manualConfirmation = false;
							gv->simTransport = false;
							gv->modeActive = false;
							gv->transportPhase = 0;
							repaint = false;
						}
					}
					else {
						cout << "Real exec. info: complete gv->path planning and real transport failed!" << endl;
						gv->manualConfirmation = false;
						gv->simTransport = false;
						gv->modeActive = false;
						gv->transportPhase = 0;
					}
				}
				else if (gv->realExecSubPhase == 4) {
					gv->transportPhase = 2;
					gv->grippingWidth = gv->grippingWidthOpen;

					gv->objectPosReachable = false;
					for (int i = 0; i < 10; i++) {
						if (pp->calcPath(gv->qValuesStandby, gv->qValuesObjectEnd, &gv->standbyPos, &gv->pathEndPos, false, generator1,distribution1, generator2,distribution2,generator3,distribution3)) {
							gv->objectPosReachable = true;
							break;
						}
					}

					if (gv->objectPosReachable) {
						gv->currentlyTransporting = false;

						if (gv->objectToTransport == 15) {
							gv->MT1currentlyActive = true;
							hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
							repaint = true;
						}
						else if (gv->objectToTransport == 16) {
							gv->MT2currentlyActive = true;
							hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
							repaint = true;
						}

						pp->optimizePath();

						gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
						gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
						gv->numPointsPlannedPath = 0;
						gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
						gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
						gv->numPointsRealPath = 0;

						fo->drawPathOnly(2);
						gv->pathLastPos = collider::Edge{ 0.0f, 0.0f, 0.0f };

						fo->writeIntoTempSegments();
						gv->pathDataForKuka = "normal,";
						for (int i = 0; i < gv->tempSegSize; i++) {
							gv->pathDataForKuka += hf->to_string_with_precision(gv->q0ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4) + ",";
						}
						gv->pathDataForKuka += hf->to_string_with_precision(gv->grippingWidth, 4);
						// cout << "debug: main " << __LINE__ <<endl;
//#if defined(WIN32)
						//  sendMessagePATH(gv->pathDataForKuka);
						//  receiveMessagePATH();
//endif

						gv->realGripperWidth = gv->grippingWidthOpen;

						gv->realExecSubPhase++;

						repaint = true;
					}
					else {
						gv->robotPoseAtAbort[0] = gv->pathEndPos.x;
						gv->robotPoseAtAbort[1] = gv->pathEndPos.y;
						gv->robotPoseAtAbort[2] = gv->pathEndPos.z;
						gv->robotPoseAtAbort[3] = gv->qValuesObjectEnd[0];
						gv->robotPoseAtAbort[4] = gv->qValuesObjectEnd[1];
						gv->robotPoseAtAbort[5] = gv->qValuesObjectEnd[2];
						gv->robotPoseAtAbort[6] = gv->qValuesObjectEnd[3];
						gv->robotPoseAtAbort[7] = gv->qValuesObjectEnd[4];
						hf->storePathRelatedData(1);
						gv->MT1currentlyActiveTemp = gv->MT1currentlyActive; gv->MT2currentlyActiveTemp = gv->MT2currentlyActive;

						cout << "Real exec. info: third gv->path planning and real transport failed!" << endl;
						gv->manualConfirmation = false;
						gv->simTransport = false;
						gv->modeActive = false;
						gv->transportPhase = 0;
						repaint = false;
					}
				}
				else if (gv->realExecSubPhase == 6) {
					cout << "Real exec. info: end reached, transport finished!" << endl;
					gv->atStandby = true;
					gv->manualConfirmation = false;
					gv->simTransport = false;
					gv->modeActive = false;
					gv->transportPhase = 0;
				}
				else {
					if (gv->virtualMode) {
//#if defined(WIN32)
						//  cout << "debug: main " << __LINE__ <<endl;
						//  sendMessageSTATE("state_request");
						//  receiveMessageSTATE();
//#endif
					}
					else {
						if (gv->MT1currentlyActive && gv->MT2currentlyActive) {
							std::cout<<"W: Socket communication to ML part currently deactivated!"<<std::endl;
//#if defined(WIN32)
							//  sendMessageMT1("state_request");
							//  sendMessageMT2("state_request");
							//  cout << "debug: main " << __LINE__ <<endl;
							//  sendMessageSTATE("state_request");

							// thread mt1Thread( receiveMessageMT1);
							// thread mt2Thread( receiveMessageMT2);
							// thread stateThread(receiveMessageSTATE);

							// mt1Thread.join();
							// mt2Thread.join();
							// stateThread.join();
//#endif
						}
						else if (gv->MT1currentlyActive && !gv->MT2currentlyActive) {
							std::cout<<"W: Socket communication to ML part currently deactivated!"<<std::endl;

//#if defined(WIN32)
							//  sendMessageMT1("state_request");
							//  cout << "debug: main " << __LINE__ <<endl;
							//  sendMessageSTATE("state_request");

							// thread mt1Thread( receiveMessageMT1);
							// thread stateThread( receiveMessageSTATE);

							// mt1Thread.join();
							// stateThread.join();
//#endif
						}
						else if (!gv->MT1currentlyActive && gv->MT2currentlyActive) {
							std::cout<<"W: Socket communication to ML part currently deactivated!"<<std::endl;
//#if defined(WIN32)
							//  sendMessageMT2("state_request");
							//  cout << "debug: main " << __LINE__ <<endl;
							//  sendMessageSTATE("state_request");

							// thread mt2Thread(receiveMessageMT2);
							// thread stateThread(receiveMessageSTATE);

							// mt2Thread.join();
							// stateThread.join();
//#endif
						}
						else {
							cout << "Real exec. error: both MTs inactive!" << endl;
						}
					}

					vector<string> splitList;
					std::stringstream string_stream;
					string splitted;

					string_stream.str(gv->stateMessageSTATE);
					while (getline(string_stream, splitted, ',')) {
						splitList.push_back(splitted);
					}
					gv->realNodeIndex = stoi(splitList[splitList.size() - 1]);

					hf->syncVisualization(splitList);

					if (pp->nodeIndex != 1) {
						gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges, gv->numPointsMTrays, 0);
						gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices, gv->numPointsMTrays, 0);
						gv->numPointsMTrays = 0;

						if (gv->virtualMode) {
							for (int i = 0; i <= 6; i += 3) {
								gv->groundTruthObstaclePositions[i] += ((rand() / (float)RAND_MAX) - 0.5f) / 2.5f;
								gv->groundTruthObstaclePositions[i + 1] += ((rand() / (float)RAND_MAX) - 0.5f) / 2.5f;

								if (gv->groundTruthObstaclePositions[i] > 55.0f) gv->groundTruthObstaclePositions[i] = 55.0f;
								if (gv->groundTruthObstaclePositions[i] < -55.0f) gv->groundTruthObstaclePositions[i] = -55.0f;

								if (gv->groundTruthObstaclePositions[i + 1] > 55.0f) gv->groundTruthObstaclePositions[i + 1] = 55.0f;
								if (gv->groundTruthObstaclePositions[i + 1] < -12.5f) gv->groundTruthObstaclePositions[i + 1] = -12.5f;
							}

							cd->recalculateCollider(&gv->colliders[12], gv->groundTruthObstaclePositions[0], gv->groundTruthObstaclePositions[1], 4.0f, gv->groundTruthObstaclePositions[2], 0, 0, true);
							cd->recalculateCollider(&gv->colliders[13], gv->groundTruthObstaclePositions[3], gv->groundTruthObstaclePositions[4], 3.0f, gv->groundTruthObstaclePositions[5], 0, 0, true);
							cd->recalculateCollider(&gv->colliders[14], gv->groundTruthObstaclePositions[6], gv->groundTruthObstaclePositions[7], 6.0f, gv->groundTruthObstaclePositions[8], 0, 0, true);

							gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[12], 0);
							gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[13], gv->tempCounter);
							hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[14], gv->tempCounter);

							fo->resetDetectedPatterns();
							for (int i = 0; i < totalPatternCount; i++) {
								if (mt->isPatternVisible(i, 1, true)) {
									hf->getMTPositionData(1);
									fo->calcPosFromMTData(&gv->colliders[15], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], i);
									fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);

									gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

									gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;

									gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;
								}

								if (mt->isPatternVisible(i, 2, true)) {
									hf->getMTPositionData(2);
									fo->calcPosFromMTData(&gv->colliders[16], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], i);
									fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);

									gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

									gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;

									gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;
								}
							}
							fo->evaluateDetections(false);
						}
						else {
							fo->resetDetectedPatterns();
							if (gv->MT1currentlyActive) {
								vector<string> splitList1;
								std::stringstream string_stream_1;
								string_stream_1.str(gv->stateMessageMT1);
								while (getline(string_stream_1, splitted, ',')) {
									splitList1.push_back(splitted);
								}

								if (splitList1.size() > 0) {
									for (int i = 0; i < stoi(splitList1[splitList1.size() - 1]) * 5; i += 5) {
										fo->distortedRelativePosition[0] = stof(splitList1[i + 1]);
										fo->distortedRelativePosition[1] = stof(splitList1[i + 2]);
										fo->distortedRelativePosition[2] = stof(splitList1[i + 3]);
										fo->distortedRelativePosition[3] = stof(splitList1[i + 4]);
										fo->calcPosFromMTData(&gv->colliders[15], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], stoi(splitList1[i]));
										fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList1[i])] - 8);

										gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

										gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
										gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
										gv->numPointsMTrays++;

										gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
										gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
										gv->numPointsMTrays++;
									}
								}
							}

							if (gv->MT2currentlyActive) {
								vector<string> splitList2;
								std::stringstream string_stream_2;
								string_stream_2.str(gv->stateMessageMT2);
								while (getline(string_stream_2, splitted, ',')) {
									splitList2.push_back(splitted);
								}

								if (splitList2.size() > 0) {
									for (int i = 0; i < stoi(splitList2[splitList2.size() - 1]) * 5; i += 5) {
										fo->distortedRelativePosition[0] = stof(splitList2[i + 1]);
										fo->distortedRelativePosition[1] = stof(splitList2[i + 2]);
										fo->distortedRelativePosition[2] = stof(splitList2[i + 3]);
										fo->distortedRelativePosition[3] = stof(splitList2[i + 4]);
										fo->calcPosFromMTData(&gv->colliders[16], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], stoi(splitList2[i]));
										fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList2[i])] - 8);

										gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

										gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
										gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
										gv->numPointsMTrays++;

										gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
										gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
										gv->numPointsMTrays++;
									}
								}
							}
							fo->evaluateDetections(false);
						}

						if (fo->detectionCounts[4] == 0) {
							cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
							gv->currentDetectedObsPosX[0] = 0.0f; gv->currentDetectedObsPosY[0] = 0.0f; gv->currentDetectedObsPosA[0] = 0.0f;
						}
						else {
							cd->recalculateCollider(&gv->distortedObstacles[0], fo->evaluatedDetections[4].x, fo->evaluatedDetections[4].y, 4.0f, fo->evaluatedDetections[4].z, 0, 0);
							gv->currentDetectedObsPosX[0] = fo->evaluatedDetections[4].x; gv->currentDetectedObsPosY[0] = fo->evaluatedDetections[4].y; gv->currentDetectedObsPosA[0] = fo->evaluatedDetections[4].z;
						}
						if (fo->detectionCounts[5] == 0) {
							cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
							gv->currentDetectedObsPosX[1] = 0.0f; gv->currentDetectedObsPosY[1] = 0.0f; gv->currentDetectedObsPosA[1] = 0.0f;
						}
						else {
							cd->recalculateCollider(&gv->distortedObstacles[1], fo->evaluatedDetections[5].x, fo->evaluatedDetections[5].y, 3.0f, fo->evaluatedDetections[5].z, 0, 0);
							gv->currentDetectedObsPosX[1] = fo->evaluatedDetections[5].x; gv->currentDetectedObsPosY[1] = fo->evaluatedDetections[5].y; gv->currentDetectedObsPosA[1] = fo->evaluatedDetections[5].z;
						}
						if (fo->detectionCounts[6] == 0) {
							cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
							gv->currentDetectedObsPosX[2] = 0.0f; gv->currentDetectedObsPosY[2] = 0.0f; gv->currentDetectedObsPosA[2] = 0.0f;
						}
						else {
							cd->recalculateCollider(&gv->distortedObstacles[2], fo->evaluatedDetections[6].x, fo->evaluatedDetections[6].y, 6.0f, fo->evaluatedDetections[6].z, 0, 0);
							gv->currentDetectedObsPosX[2] = fo->evaluatedDetections[6].x; gv->currentDetectedObsPosY[2] = fo->evaluatedDetections[6].y; gv->currentDetectedObsPosA[2] = fo->evaluatedDetections[6].z;
						}

						gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[0], 0);
						gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[1], gv->tempCounter);
						hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2], gv->tempCounter);

						if (gv->realNodeIndex == gv->tempSegSize) {
							gv->syncSim = false;

							if (gv->realExecSubPhase == 3) {
								if (gv->objectToTransport < 15) {
									gv->initPositionsX[gv->objectToTransport - 8] = gv->pathEndPos.x; gv->initPositionsY[gv->objectToTransport - 8] = gv->pathEndPos.y; gv->initPositionsAngles[gv->objectToTransport - 8] = gv->endObjectAngle / 180.0f * M_PI;
									cd->recalculateCollider(&gv->colliders[gv->objectToTransport], gv->pathEndPos.x, gv->pathEndPos.y, gv->objectOffset, gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true);

									gv->tempCounter = 0;
									for (int i = 8; i < gv->objectToTransport; i++) {
										gv->tempCounter += gv->colliders[i].patternCount * 4;
									}
									hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[gv->objectToTransport], gv->tempCounter);

									gv->tempCounter = gv->totalPlatePoints;
									for (int i = 0; i < gv->objectToTransport; i++) {
										gv->tempCounter += gv->colliders[i].facesTimes3;
									}
									hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[gv->objectToTransport], gv->tempCounter);
								}
								else if (gv->objectToTransport == 15) {
									gv->initPositionsX[4] = gv->pathEndPos.x; gv->initPositionsY[4] = gv->pathEndPos.y; gv->initPositionsAngles[4] = gv->endObjectAngle / 180.0f * M_PI;
									cd->recalculateCollider(&gv->colliders[15], gv->pathEndPos.x, gv->pathEndPos.y, gv->objectOffset, gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, false, true, gv->camVectorsMT1);

									gv->tempCounter = gv->totalPlatePoints;
									for (int i = 0; i < 15; i++) {
										gv->tempCounter += gv->colliders[i].facesTimes3;
									}
									hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[15], gv->tempCounter);
								}
								else if (gv->objectToTransport == 16) {
									gv->initPositionsX[5] = gv->pathEndPos.x; gv->initPositionsY[5] = gv->pathEndPos.y; gv->initPositionsAngles[5] = gv->endObjectAngle / 180.0f * M_PI;
									cd->recalculateCollider(&gv->colliders[16], gv->pathEndPos.x, gv->pathEndPos.y, gv->objectOffset, gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, false, true, gv->camVectorsMT2);

									gv->tempCounter = gv->totalPlatePoints;
									for (int i = 0; i < 16; i++) {
										gv->tempCounter += gv->colliders[i].facesTimes3;
									}
									hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[16], gv->tempCounter);
								}
							}

							gv->realExecSubPhase++;
						}
						else if (gv->realNodeIndex > 0) {
							if (!gv->syncSim) gv->syncSim = true;
						}
						repaint = true;
					}
					else gv->syncSim = false;
				}
			}
			else if (gv->manualMode == 1) {
				if (gv->realExecSubPhase == 0) {
					if (fo->checkNewTransport(generator1,distribution1, generator2,distribution2,generator3,distribution3)) {
						gv->transportPhase = 0;
						gv->grippingWidth = gv->grippingWidthOpen;

						gv->objectPosReachable = false;
						for (int i = 0; i < 10; i++) {
							if (gv->atStandby) {
								if (pp->calcPath(gv->qValuesObjectStart, gv->qValuesStandby, &gv->pathStartPos, &gv->standbyPos, false, generator1,distribution1, generator2,distribution2,generator3,distribution3)) {
									gv->objectPosReachable = true;
									break;
								}
							}
							else {
								if (pp->calcPath(gv->qValuesObjectStart, gv->qValuesCurrent, &gv->pathStartPos, &gv->pathAltStartPos, false, generator1,distribution1, generator2,distribution2,generator3,distribution3)) {
									gv->objectPosReachable = true;
									break;
								}
							}
						}

						if (gv->objectPosReachable) {
							if (!gv->atStandby) cout << "Real exec. warning: gripper zero init. can cause gv->collisions when robot arm is not in standby position!" << endl;
//#if defined(WIN32)
							// cout << "debug: main " << __LINE__ <<endl;

							//  sendMessagePATH("zero");
							//  receiveMessagePATH();
//#endif

							gv->atStandby = false;
							pp->optimizePath();

							gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
							gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
							gv->numPointsPlannedPath = 0;
							gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
							gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
							gv->numPointsRealPath = 0;

							fo->drawPathOnly(2);
							gv->pathLastPos = collider::Edge{ 0.0f, 0.0f, 0.0f };

							fo->writeIntoTempSegments();
							gv->pathDataForKuka = "normal,";
							for (int i = 0; i < gv->tempSegSize; i++) {
								gv->pathDataForKuka += hf->to_string_with_precision(gv->q0ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4) + ",";
							}
							gv->pathDataForKuka += hf->to_string_with_precision(gv->grippingWidth, 4);
//#if defined(WIN32)
							//  cout << "debug: main " << __LINE__ <<endl;
							//  sendMessagePATH(gv->pathDataForKuka);
							//  receiveMessagePATH();
//#endif

							gv->realGripperWidth = gv->grippingWidthOpen;

							gv->realExecSubPhase++;

							repaint = true;
						}
						else {
							cout << "Real exec. info: first gv->path planning and real transport failed!" << endl;
							gv->manualConfirmation = false;
							gv->simTransport = false;
							gv->modeActive = false;
							repaint = false;
						}
					}
					else {
						cout << "Real exec. info: complete gv->path planning and real transport failed!" << endl;
						gv->manualConfirmation = false;
						gv->simTransport = false;
						gv->modeActive = false;
					}
				}
				else if (gv->realExecSubPhase == 2) {
					gv->transportPhase = 1;
					gv->grippingWidth = gv->grippingWidthFixed;

					gv->objectPosReachable = false;
					for (int i = 0; i < 10; i++) {
						if (
								pp->calcPath(gv->qValuesObjectEnd, gv->qValuesObjectStart, &gv->pathEndPos, &gv->pathStartPos, false,
										generator1,distribution1,generator2,distribution2,generator3,distribution3)


						) {
							gv->objectPosReachable = true;
							break;
						}
					}

					if (gv->objectPosReachable) {
						gv->currentlyTransporting = true;
						cd->recalculateCollider(gv->safetyCollider, 0.0f, 0.0f, 10.0e5, 0, 0, 0);
						hf->calcEdgesForMesh(gv->safety_col_edges, gv->safetyCollider, 0);

						if (gv->objectToTransport == 15) {
							gv->MT1currentlyActive = false;
							hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
							repaint = true;
						}
						else if (gv->objectToTransport == 16) {
							gv->MT2currentlyActive = false;
							hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
							repaint = true;
						}

						pp->optimizePath();

						gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
						gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
						gv->numPointsPlannedPath = 0;
						gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
						gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
						gv->numPointsRealPath = 0;

						fo->drawPathOnly(2);
						gv->pathLastPos = collider::Edge{ 0.0f, 0.0f, 0.0f };

						fo->writeIntoTempSegments();
						gv->pathDataForKuka = "normal,";
						for (int i = 0; i < gv->tempSegSize; i++) {
							gv->pathDataForKuka += hf->to_string_with_precision(gv->q0ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4) + ",";
						}
						gv->pathDataForKuka += hf->to_string_with_precision(gv->grippingWidth, 4);
// //#if defined(WIN32)
// 						 cout << "debug: main " << __LINE__ <<endl;
// 						 sendMessagePATH(gv->pathDataForKuka);
// 						 receiveMessagePATH();
//#endif

						gv->realGripperWidth = gv->grippingWidthFixed;

						gv->realExecSubPhase++;

						repaint = true;
					}
					else {
						gv->robotPoseAtAbort[0] = gv->pathStartPos.x;
						gv->robotPoseAtAbort[1] = gv->pathStartPos.y;
						gv->robotPoseAtAbort[2] = gv->pathStartPos.z;
						gv->robotPoseAtAbort[3] = gv->qValuesObjectStart[0];
						gv->robotPoseAtAbort[4] = gv->qValuesObjectStart[1];
						gv->robotPoseAtAbort[5] = gv->qValuesObjectStart[2];
						gv->robotPoseAtAbort[6] = gv->qValuesObjectStart[3];
						gv->robotPoseAtAbort[7] = gv->qValuesObjectStart[4];

						cout << "Real exec. info: second gv->path planning and real transport failed!" << endl;
						gv->manualConfirmation = false;
						gv->simTransport = false;
						gv->modeActive = false;
						gv->transportPhase = 0;
						repaint = false;
					}
				}
				else if (gv->realExecSubPhase == 4) {
					gv->transportPhase = 2;
					gv->grippingWidth = gv->grippingWidthOpen;

					gv->objectPosReachable = false;
					for (int i = 0; i < 10; i++) {
						if (pp->calcPath(gv->qValuesStandby, gv->qValuesObjectEnd, &gv->standbyPos, &gv->pathEndPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3)) {
							gv->objectPosReachable = true;
							break;
						}
					}

					if (gv->objectPosReachable) {
						gv->currentlyTransporting = false;
						if (gv->objectToTransport == 15) {
							gv->MT1currentlyActive = true;
							hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 1);
							repaint = true;
						}
						else if (gv->objectToTransport == 16) {
							gv->MT2currentlyActive = true;
							hf->calcEdgesForViewFunnel(gv->mt_viewfields_edges, 2);
							repaint = true;
						}

						pp->optimizePath();

						gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
						gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
						gv->numPointsPlannedPath = 0;
						gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
						gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
						gv->numPointsRealPath = 0;

						fo->drawPathOnly(2);
						gv->pathLastPos = collider::Edge{ 0.0f, 0.0f, 0.0f };

						fo->writeIntoTempSegments();
						gv->pathDataForKuka = "normal,";
						for (int i = 0; i < gv->tempSegSize; i++) {
							gv->pathDataForKuka += hf->to_string_with_precision(gv->q0ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4) + ",";
						}
						gv->pathDataForKuka += hf->to_string_with_precision(gv->grippingWidth, 4);
//#if defined(WIN32)
						//  cout << "debug: main " << __LINE__ <<endl;
						//  sendMessagePATH(gv->pathDataForKuka);
						//  receiveMessagePATH();
//#endif

						gv->realGripperWidth = gv->grippingWidthOpen;

						gv->realExecSubPhase++;

						repaint = true;
					}
					else {
						gv->robotPoseAtAbort[0] = gv->pathEndPos.x;
						gv->robotPoseAtAbort[1] = gv->pathEndPos.y;
						gv->robotPoseAtAbort[2] = gv->pathEndPos.z;
						gv->robotPoseAtAbort[3] = gv->qValuesObjectEnd[0];
						gv->robotPoseAtAbort[4] = gv->qValuesObjectEnd[1];
						gv->robotPoseAtAbort[5] = gv->qValuesObjectEnd[2];
						gv->robotPoseAtAbort[6] = gv->qValuesObjectEnd[3];
						gv->robotPoseAtAbort[7] = gv->qValuesObjectEnd[4];
						hf->storePathRelatedData(1);
						gv->MT1currentlyActiveTemp = gv->MT1currentlyActive; gv->MT2currentlyActiveTemp = gv->MT2currentlyActive;

						cout << "Real exec. info: third gv->path planning and real transport failed!" << endl;
						gv->manualConfirmation = false;
						gv->simTransport = false;
						gv->modeActive = false;
						gv->transportPhase = 0;
						repaint = false;
					}
				}
				else if (gv->realExecSubPhase == 6) {
					cout << "Real exec. info: end reached, transport finished!" << endl;
					gv->atStandby = true;
					gv->manualConfirmation = false;
					gv->simTransport = false;
					gv->modeActive = false;
					gv->transportPhase = 0;
				}
				else {
					if (gv->virtualMode) {
//#if defined(WIN32)
						//  cout << "debug: main " << __LINE__ <<endl;
						//  sendMessageSTATE("state_request");
						//  receiveMessageSTATE();
//#endif
					}
					else {
						if (gv->MT1currentlyActive && gv->MT2currentlyActive) hf->requestRealSystemState();
						else if (gv->MT1currentlyActive && !gv->MT2currentlyActive) {
//#if defined(WIN32)
							//  sendMessageMT1("state_request");
							//  cout << "debug: main " << __LINE__ <<endl;
							// //  sendMessageSTATE("state_request");

							// thread mt1Thread( receiveMessageMT1);
							// thread stateThread(receiveMessageSTATE);

							// mt1Thread.join();
							// stateThread.join();
//#endif
						}
						else if (!gv->MT1currentlyActive && gv->MT2currentlyActive) {
//#if defined(WIN32)
							//  sendMessageMT2("state_request");
							//  cout << "debug: main " << __LINE__ <<endl;
							//  sendMessageSTATE("state_request");

							// thread mt2Thread( receiveMessageMT2);
							// thread stateThread(receiveMessageSTATE);

							// mt2Thread.join();
							// stateThread.join();
//#endif
						}
						else {
							cout << "Real exec. error: both MTs inactive!" << endl;
						}
					}

					vector<string> splitList;
					std::stringstream string_stream;
					string splitted;

					string_stream.str(gv->stateMessageSTATE);
					while (getline(string_stream, splitted, ',')) {
						splitList.push_back(splitted);
					}
					gv->realNodeIndex = stoi(splitList[splitList.size() - 1]);

					hf->syncVisualization(splitList);

					if (pp->nodeIndex != 1) {
						gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges, gv->numPointsMTrays, 0);
						gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices, gv->numPointsMTrays, 0);
						gv->numPointsMTrays = 0;

						if (gv->virtualMode) {
							for (int i = 0; i <= 6; i += 3) {
								gv->groundTruthObstaclePositions[i] += ((rand() / (float)RAND_MAX) - 0.5f) / 2.5f;
								gv->groundTruthObstaclePositions[i + 1] += ((rand() / (float)RAND_MAX) - 0.5f) / 2.5f;

								if (gv->groundTruthObstaclePositions[i] > 55.0f) gv->groundTruthObstaclePositions[i] = 55.0f;
								if (gv->groundTruthObstaclePositions[i] < -55.0f) gv->groundTruthObstaclePositions[i] = -55.0f;

								if (gv->groundTruthObstaclePositions[i + 1] > 55.0f) gv->groundTruthObstaclePositions[i + 1] = 55.0f;
								if (gv->groundTruthObstaclePositions[i + 1] < -12.5f) gv->groundTruthObstaclePositions[i + 1] = -12.5f;
							}

							cd->recalculateCollider(&gv->colliders[12], gv->groundTruthObstaclePositions[0], gv->groundTruthObstaclePositions[1], 4.0f, gv->groundTruthObstaclePositions[2], 0, 0, true);
							cd->recalculateCollider(&gv->colliders[13], gv->groundTruthObstaclePositions[3], gv->groundTruthObstaclePositions[4], 3.0f, gv->groundTruthObstaclePositions[5], 0, 0, true);
							cd->recalculateCollider(&gv->colliders[14], gv->groundTruthObstaclePositions[6], gv->groundTruthObstaclePositions[7], 6.0f, gv->groundTruthObstaclePositions[8], 0, 0, true);

							gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[12], 0);
							gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[13], gv->tempCounter);
							hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[14], gv->tempCounter);

							fo->resetDetectedPatterns();
							for (int i = 0; i < totalPatternCount; i++) {
								if (mt->isPatternVisible(i, 1, true)) {
									hf->getMTPositionData(1);
									fo->calcPosFromMTData(&gv->colliders[15], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], i);
									fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);

									gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

									gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;

									gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;
								}

								if (mt->isPatternVisible(i, 2, true)) {
									hf->getMTPositionData(2);
									fo->calcPosFromMTData(&gv->colliders[16], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], i);
									fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);

									gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

									gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;

									gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;
								}
							}
							fo->evaluateDetections(false);
						}
						else {
							fo->resetDetectedPatterns();
							if (gv->MT1currentlyActive) {
								vector<string> splitList1;
								std::stringstream string_stream_1;

								string_stream_1.str(gv->stateMessageMT1);
								while (getline(string_stream_1, splitted, ',')) {
									splitList1.push_back(splitted);
								}

								if (splitList1.size() > 0) {
									for (int i = 0; i < stoi(splitList1[splitList1.size() - 1]) * 5; i += 5) {
										fo->distortedRelativePosition[0] = stof(splitList1[i + 1]);
										fo->distortedRelativePosition[1] = stof(splitList1[i + 2]);
										fo->distortedRelativePosition[2] = stof(splitList1[i + 3]);
										fo->distortedRelativePosition[3] = stof(splitList1[i + 4]);
										fo->calcPosFromMTData(&gv->colliders[15], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], stoi(splitList1[i]));
										fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList1[i])] - 8);

										gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

										gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
										gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
										gv->numPointsMTrays++;

										gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
										gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
										gv->numPointsMTrays++;
									}
								}
							}

							if (gv->MT2currentlyActive) {
								vector<string> splitList2;
								std::stringstream string_stream_2;

								string_stream_2.str(gv->stateMessageMT2);
								while (getline(string_stream_2, splitted, ',')) {
									splitList2.push_back(splitted);
								}

								if (splitList2.size() > 0) {
									for (int i = 0; i < stoi(splitList2[splitList2.size() - 1]) * 5; i += 5) {
										fo->distortedRelativePosition[0] = stof(splitList2[i + 1]);
										fo->distortedRelativePosition[1] = stof(splitList2[i + 2]);
										fo->distortedRelativePosition[2] = stof(splitList2[i + 3]);
										fo->distortedRelativePosition[3] = stof(splitList2[i + 4]);
										fo->calcPosFromMTData(&gv->colliders[16], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], stoi(splitList2[i]));
										fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList2[i])] - 8);

										gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

										gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
										gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
										gv->numPointsMTrays++;

										gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
										gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
										gv->numPointsMTrays++;
									}
								}
							}
							fo->evaluateDetections(false);
						}

						if (fo->detectionCounts[4] == 0) {
							cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
							gv->currentDetectedObsPosX[0] = 0.0f; gv->currentDetectedObsPosY[0] = 0.0f; gv->currentDetectedObsPosA[0] = 0.0f;
						}
						else {
							cd->recalculateCollider(&gv->distortedObstacles[0], fo->evaluatedDetections[4].x, fo->evaluatedDetections[4].y, 4.0f, fo->evaluatedDetections[4].z, 0, 0);
							gv->currentDetectedObsPosX[0] = fo->evaluatedDetections[4].x; gv->currentDetectedObsPosY[0] = fo->evaluatedDetections[4].y; gv->currentDetectedObsPosA[0] = fo->evaluatedDetections[4].z;
						}
						if (fo->detectionCounts[5] == 0) {
							cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
							gv->currentDetectedObsPosX[1] = 0.0f; gv->currentDetectedObsPosY[1] = 0.0f; gv->currentDetectedObsPosA[1] = 0.0f;
						}
						else {
							cd->recalculateCollider(&gv->distortedObstacles[1], fo->evaluatedDetections[5].x, fo->evaluatedDetections[5].y, 3.0f, fo->evaluatedDetections[5].z, 0, 0);
							gv->currentDetectedObsPosX[1] = fo->evaluatedDetections[5].x; gv->currentDetectedObsPosY[1] = fo->evaluatedDetections[5].y; gv->currentDetectedObsPosA[1] = fo->evaluatedDetections[5].z;
						}
						if (fo->detectionCounts[6] == 0) {
							cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
							gv->currentDetectedObsPosX[2] = 0.0f; gv->currentDetectedObsPosY[2] = 0.0f; gv->currentDetectedObsPosA[2] = 0.0f;
						}
						else {
							cd->recalculateCollider(&gv->distortedObstacles[2], fo->evaluatedDetections[6].x, fo->evaluatedDetections[6].y, 6.0f, fo->evaluatedDetections[6].z, 0, 0);
							gv->currentDetectedObsPosX[2] = fo->evaluatedDetections[6].x; gv->currentDetectedObsPosY[2] = fo->evaluatedDetections[6].y; gv->currentDetectedObsPosA[2] = fo->evaluatedDetections[6].z;
						}

						gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[0], 0);
						gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[1], gv->tempCounter);
						hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2], gv->tempCounter);

						if (gv->realNodeIndex == gv->tempSegSize) {
							gv->syncSim = false;

							if (gv->realExecSubPhase == 3) {
								if (gv->objectToTransport < 15) {
									gv->initPositionsX[gv->objectToTransport - 8] = gv->pathEndPos.x; gv->initPositionsY[gv->objectToTransport - 8] = gv->pathEndPos.y; gv->initPositionsAngles[gv->objectToTransport - 8] = gv->endObjectAngle / 180.0f * M_PI;
									cd->recalculateCollider(&gv->colliders[gv->objectToTransport], gv->pathEndPos.x, gv->pathEndPos.y, gv->objectOffset, gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true);

									gv->tempCounter = 0;
									for (int i = 8; i < gv->objectToTransport; i++) {
										gv->tempCounter += gv->colliders[i].patternCount * 4;
									}
									hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[gv->objectToTransport], gv->tempCounter);

									gv->tempCounter = gv->totalPlatePoints;
									for (int i = 0; i < gv->objectToTransport; i++) {
										gv->tempCounter += gv->colliders[i].facesTimes3;
									}
									hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[gv->objectToTransport], gv->tempCounter);
								}
								else if (gv->objectToTransport == 15) {
									gv->initPositionsX[4] = gv->pathEndPos.x; gv->initPositionsY[4] = gv->pathEndPos.y; gv->initPositionsAngles[4] = gv->endObjectAngle / 180.0f * M_PI;
									cd->recalculateCollider(&gv->colliders[15], gv->pathEndPos.x, gv->pathEndPos.y, gv->objectOffset, gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, false, true, gv->camVectorsMT1);

									gv->tempCounter = gv->totalPlatePoints;
									for (int i = 0; i < 15; i++) {
										gv->tempCounter += gv->colliders[i].facesTimes3;
									}
									hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[15], gv->tempCounter);
								}
								else if (gv->objectToTransport == 16) {
									gv->initPositionsX[5] = gv->pathEndPos.x; gv->initPositionsY[5] = gv->pathEndPos.y; gv->initPositionsAngles[5] = gv->endObjectAngle / 180.0f * M_PI;
									cd->recalculateCollider(&gv->colliders[16], gv->pathEndPos.x, gv->pathEndPos.y, gv->objectOffset, gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, false, true, gv->camVectorsMT2);

									gv->tempCounter = gv->totalPlatePoints;
									for (int i = 0; i < 16; i++) {
										gv->tempCounter += gv->colliders[i].facesTimes3;
									}
									hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[16], gv->tempCounter);
								}
							}

							gv->realExecSubPhase++;
						}
						else if (gv->realNodeIndex > 0) {
							if (!gv->syncSim) gv->syncSim = true;
						}
						repaint = true;
					}
					else gv->syncSim = false;
				}
			}
			else {
				if (gv->realExecSubPhase == 0) {
					gv->pathStartPos.x = gv->selectedTargetPosition[0];
					gv->pathStartPos.y = gv->selectedTargetPosition[1];
					gv->pathStartPos.z = gv->selectedTargetPosition[2];

					if (abs(gv->standbyPos.x - gv->pathStartPos.x) < cd->epsilon3 && abs(gv->standbyPos.y - gv->pathStartPos.y) < cd->epsilon3 && abs(gv->standbyPos.z - gv->pathStartPos.z) < cd->epsilon3 && abs(gv->selectedTargetPosition[3]) < cd->epsilon3) gv->movingToStandby = true;
					else gv->movingToStandby = false;

					if (ik->doesThetaExist(gv->pathStartPos.x, gv->pathStartPos.y, gv->pathStartPos.z)) {
						ik->writeQValues(gv->qValuesObjectStart);
						gv->qValuesObjectStart[4] = gv->selectedTargetPosition[3];
						if (gv->qValuesObjectStart[4] > 180) gv->qValuesObjectStart[4] = -(360 - gv->qValuesObjectStart[4]);
						gv->transportPhase = 0;

						gv->objectPosReachable = false;
						for (int i = 0; i < 10; i++) {
							if (pp->calcPath(gv->qValuesObjectStart, gv->qValuesCurrent, &gv->pathStartPos, &gv->pathAltStartPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3)) {
								gv->objectPosReachable = true;
								break;
							}
						}

						if (gv->objectPosReachable) {
							if (!gv->movingToStandby) gv->atStandby = false;
							pp->optimizePath();

							gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
							gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
							gv->numPointsPlannedPath = 0;
							gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
							gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
							gv->numPointsRealPath = 0;

							fo->drawPathOnly(2);
							gv->pathLastPos = collider::Edge{ 0.0f, 0.0f, 0.0f };

							fo->writeIntoTempSegments();
							gv->pathDataForKuka = "normal_no_gripper,";
							for (int i = 0; i < gv->tempSegSize; i++) {
								gv->pathDataForKuka += hf->to_string_with_precision(gv->q0ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q1ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q2ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q3ValsTempSeg[i], 4) + "," + hf->to_string_with_precision(gv->q4ValsTempSeg[i], 4) + ",";
							}
							gv->pathDataForKuka += hf->to_string_with_precision(0, 4);
//#if defined(WIN32)
							//  cout << "debug: main " << __LINE__ <<endl;
							//  sendMessagePATH(gv->pathDataForKuka);
							//  receiveMessagePATH();
//#endif

							gv->realExecSubPhase++;

							repaint = true;
						}
						else {
							cout << "Real exec. info: gv->path planning failed and real movement is not possible!" << endl;
							gv->manualConfirmation = false;
							gv->simTransport = false;
							gv->modeActive = false;
						}
					}
					else {
						cout << "Real exec. info: target position is not reachable!" << endl;
						gv->manualConfirmation = false;
						gv->simTransport = false;
						gv->modeActive = false;
					}
				}
				else if (gv->realExecSubPhase == 2) {
					cout << "Real exec. info: target position reached successfully!" << endl;
					if (gv->movingToStandby) gv->atStandby = true;
					else {
						gv->robotPoseAtAbort[0] = gv->pathStartPos.x;
						gv->robotPoseAtAbort[1] = gv->pathStartPos.y;
						gv->robotPoseAtAbort[2] = gv->pathStartPos.z;
						gv->robotPoseAtAbort[3] = gv->qValuesObjectStart[0];
						gv->robotPoseAtAbort[4] = gv->qValuesObjectStart[1];
						gv->robotPoseAtAbort[5] = gv->qValuesObjectStart[2];
						gv->robotPoseAtAbort[6] = gv->qValuesObjectStart[3];
						gv->robotPoseAtAbort[7] = gv->qValuesObjectStart[4];
					}
					gv->manualConfirmation = false;
					gv->simTransport = false;
					gv->modeActive = false;
				}
				else {
					if (gv->virtualMode) {
//#if defined(WIN32)
						//  cout << "debug: main " << __LINE__ <<endl;
						//  sendMessageSTATE("state_request");
						 
						//  receiveMessageSTATE();
//#endif
					}
					else hf->requestRealSystemState();

					vector<string> splitList;
					std::stringstream string_stream;
					string splitted;

					string_stream.str(gv->stateMessageSTATE);
					while (getline(string_stream, splitted, ',')) {
						splitList.push_back(splitted);
					}
					gv->realNodeIndex = stoi(splitList[splitList.size() - 1]);

					hf->syncVisualization(splitList);

					if (pp->nodeIndex != 1) {
						gv->mt_rays_edges = ik->cutOff(gv->mt_rays_edges, gv->numPointsMTrays, 0);
						gv->mt_rays_vertices = ik->cutOff(gv->mt_rays_vertices, gv->numPointsMTrays, 0);
						gv->numPointsMTrays = 0;

						if (gv->virtualMode) {
							for (int i = 0; i <= 6; i += 3) {
								gv->groundTruthObstaclePositions[i] += ((rand() / (float)RAND_MAX) - 0.5f) / 2.5f;
								gv->groundTruthObstaclePositions[i + 1] += ((rand() / (float)RAND_MAX) - 0.5f) / 2.5f;

								if (gv->groundTruthObstaclePositions[i] > 55.0f) gv->groundTruthObstaclePositions[i] = 55.0f;
								if (gv->groundTruthObstaclePositions[i] < -55.0f) gv->groundTruthObstaclePositions[i] = -55.0f;

								if (gv->groundTruthObstaclePositions[i + 1] > 55.0f) gv->groundTruthObstaclePositions[i + 1] = 55.0f;
								if (gv->groundTruthObstaclePositions[i + 1] < -12.5f) gv->groundTruthObstaclePositions[i + 1] = -12.5f;
							}

							cd->recalculateCollider(&gv->colliders[12], gv->groundTruthObstaclePositions[0], gv->groundTruthObstaclePositions[1], 4.0f, gv->groundTruthObstaclePositions[2], 0, 0, true);
							cd->recalculateCollider(&gv->colliders[13], gv->groundTruthObstaclePositions[3], gv->groundTruthObstaclePositions[4], 3.0f, gv->groundTruthObstaclePositions[5], 0, 0, true);
							cd->recalculateCollider(&gv->colliders[14], gv->groundTruthObstaclePositions[6], gv->groundTruthObstaclePositions[7], 6.0f, gv->groundTruthObstaclePositions[8], 0, 0, true);

							gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[12], 0);
							gv->tempCounter = hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[13], gv->tempCounter);
							hf->calcEdgesForMesh(gv->ground_truth_obs_edges, &gv->colliders[14], gv->tempCounter);

							fo->resetDetectedPatterns();
							for (int i = 0; i < totalPatternCount; i++) {
								if (mt->isPatternVisible(i, 1, true)) {
									hf->getMTPositionData(1);
									fo->calcPosFromMTData(&gv->colliders[15], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], i);
									fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);

									gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

									gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;

									gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;
								}

								if (mt->isPatternVisible(i, 2, true)) {
									hf->getMTPositionData(2);
									fo->calcPosFromMTData(&gv->colliders[16], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], i);
									fo->insertDetectedPattern(objectIndicesFromPattern[i] - 8);

									gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

									gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;

									gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;
								}
							}
							fo->evaluateDetections(false);
						}
						else {
							vector<string> splitList1;
							vector<string> splitList2;
							std::stringstream string_stream_1;
							std::stringstream string_stream_2;

							string_stream_1.str(gv->stateMessageMT1);
							while (getline(string_stream_1, splitted, ',')) {
								splitList1.push_back(splitted);
							}

							string_stream_2.str(gv->stateMessageMT2);
							while (getline(string_stream_2, splitted, ',')) {
								splitList2.push_back(splitted);
							}

							fo->resetDetectedPatterns();
							if (splitList1.size() > 0) {
								for (int i = 0; i < stoi(splitList1[splitList1.size() - 1]) * 5; i += 5) {
									fo->distortedRelativePosition[0] = stof(splitList1[i + 1]);
									fo->distortedRelativePosition[1] = stof(splitList1[i + 2]);
									fo->distortedRelativePosition[2] = stof(splitList1[i + 3]);
									fo->distortedRelativePosition[3] = stof(splitList1[i + 4]);
									fo->calcPosFromMTData(&gv->colliders[15], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], stoi(splitList1[i]));
									fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList1[i])] - 8);

									gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

									gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;

									gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT1[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT1[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT1[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;
								}
							}

							if (splitList2.size() > 0) {
								for (int i = 0; i < stoi(splitList2[splitList2.size() - 1]) * 5; i += 5) {
									fo->distortedRelativePosition[0] = stof(splitList2[i + 1]);
									fo->distortedRelativePosition[1] = stof(splitList2[i + 2]);
									fo->distortedRelativePosition[2] = stof(splitList2[i + 3]);
									fo->distortedRelativePosition[3] = stof(splitList2[i + 4]);
									fo->calcPosFromMTData(&gv->colliders[16], fo->distortedRelativePosition[0], fo->distortedRelativePosition[1], fo->distortedRelativePosition[2], fo->distortedRelativePosition[3], stoi(splitList2[i]));
									fo->insertDetectedPattern(objectIndicesFromPattern[stoi(splitList2[i])] - 8);

									gv->mt_rays_edges = ik->increaseSize(gv->mt_rays_edges, gv->numPointsMTrays, 2); gv->mt_rays_vertices = ik->increaseSize(gv->mt_rays_vertices, gv->numPointsMTrays, 2);

									gv->mt_rays_edges[gv->numPointsMTrays].x = fo->distortedPatternCenter[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = fo->distortedPatternCenter[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = fo->distortedPatternCenter[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;

									gv->mt_rays_edges[gv->numPointsMTrays].x = gv->camVectorsMT2[0]; gv->mt_rays_edges[gv->numPointsMTrays].y = gv->camVectorsMT2[1]; gv->mt_rays_edges[gv->numPointsMTrays].z = gv->camVectorsMT2[2];
									gv->mt_rays_vertices[gv->numPointsMTrays] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->otherColors[4].x, gv->otherColors[4].y, gv->otherColors[4].z, 1.0f };
									gv->numPointsMTrays++;
								}
							}
							fo->evaluateDetections(false);
						}

						if (fo->detectionCounts[4] == 0) {
							cd->recalculateCollider(&gv->distortedObstacles[0], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
							gv->currentDetectedObsPosX[0] = 0.0f; gv->currentDetectedObsPosY[0] = 0.0f; gv->currentDetectedObsPosA[0] = 0.0f;
						}
						else {
							cd->recalculateCollider(&gv->distortedObstacles[0], fo->evaluatedDetections[4].x, fo->evaluatedDetections[4].y, 4.0f, fo->evaluatedDetections[4].z, 0, 0);
							gv->currentDetectedObsPosX[0] = fo->evaluatedDetections[4].x; gv->currentDetectedObsPosY[0] = fo->evaluatedDetections[4].y; gv->currentDetectedObsPosA[0] = fo->evaluatedDetections[4].z;
						}
						if (fo->detectionCounts[5] == 0) {
							cd->recalculateCollider(&gv->distortedObstacles[1], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
							gv->currentDetectedObsPosX[1] = 0.0f; gv->currentDetectedObsPosY[1] = 0.0f; gv->currentDetectedObsPosA[1] = 0.0f;
						}
						else {
							cd->recalculateCollider(&gv->distortedObstacles[1], fo->evaluatedDetections[5].x, fo->evaluatedDetections[5].y, 3.0f, fo->evaluatedDetections[5].z, 0, 0);
							gv->currentDetectedObsPosX[1] = fo->evaluatedDetections[5].x; gv->currentDetectedObsPosY[1] = fo->evaluatedDetections[5].y; gv->currentDetectedObsPosA[1] = fo->evaluatedDetections[5].z;
						}
						if (fo->detectionCounts[6] == 0) {
							cd->recalculateCollider(&gv->distortedObstacles[2], 0.0f, 0.0f, 10.0e5, 0, 0, 0);
							gv->currentDetectedObsPosX[2] = 0.0f; gv->currentDetectedObsPosY[2] = 0.0f; gv->currentDetectedObsPosA[2] = 0.0f;
						}
						else {
							cd->recalculateCollider(&gv->distortedObstacles[2], fo->evaluatedDetections[6].x, fo->evaluatedDetections[6].y, 6.0f, fo->evaluatedDetections[6].z, 0, 0);
							gv->currentDetectedObsPosX[2] = fo->evaluatedDetections[6].x; gv->currentDetectedObsPosY[2] = fo->evaluatedDetections[6].y; gv->currentDetectedObsPosA[2] = fo->evaluatedDetections[6].z;
						}

						gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[0], 0);
						gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[1], gv->tempCounter);
						hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2], gv->tempCounter);

						if (gv->realNodeIndex == gv->tempSegSize) {
							gv->syncSim = false;

							gv->realExecSubPhase++;
						}
						else if (gv->realNodeIndex > 0) {
							if (!gv->syncSim) gv->syncSim = true;
						}
						repaint = true;
					}
					else gv->syncSim = false;
				}
			}
		}


		if (gv->virtSim && !gv->paused)
		{
			if (abs(gv->qValuesTarget[gv->maxValueIndex] - gv->qValuesCurrent[gv->maxValueIndex]) < cd->epsilon3) {
				for (int i = 0; i < 5; i++) {
					gv->qValuesCurrent[i] = gv->qValuesTarget[i];
				}

				if (pp->nodeIndex == 0) {
					if (gv->transportPhase == 0) {
						gv->transportPhase = 1;
						cd->recalculateCollider(gv->safetyCollider, gv->colliders[gv->objectToTransport].offsets[0], gv->colliders[gv->objectToTransport].offsets[1], gv->colliders[gv->objectToTransport].offsets[2], gv->colliders[gv->objectToTransport].angles[0], 0.0f, 0.0f);
						hf->calcEdgesForMesh(gv->safety_col_edges, gv->safetyCollider, 0);
						repaint = true;
					}
					else if (gv->transportPhase == 1) {
						gv->transportPhase = 2;
						cd->recalculateCollider(&gv->colliders[gv->objectToTransport], gv->pathEndPos.x, gv->pathEndPos.y, gv->objectOffset, gv->endObjectAngle / 180.0f * M_PI, 0.0f, 0.0f, true);
						gv->tempCounter = 0;
						for (int i = 8; i < gv->objectToTransport; i++) {
							gv->tempCounter += gv->colliders[i].patternCount * 4;
						}
						hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[gv->objectToTransport], gv->tempCounter);

						gv->tempCounter = gv->totalPlatePoints;
						for (int i = 0; i < gv->objectToTransport; i++) {
							gv->tempCounter += gv->colliders[i].facesTimes3;
						}
						hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[gv->objectToTransport], gv->tempCounter);
						repaint = true;
					}
					else if (gv->transportPhase == 2) {
						gv->transportPhase = 0;

						if (gv->synMode == 0) {
							gv->modeActive = false;
							if (gv->activeAgents[0]) {
//#if defined(WIN32)
								 sendMessageML("safety_reward,1000");
								 receiveMessageML();
								 cout<<"------------------ACTIVE0--------------"<< endl;
//#endif
							}

							if (gv->activeAgents[1]) {
								if (gv->MT1posChanged) {
//#if defined(WIN32)
									 sendMessageML("mt1_reward,1000");
									 receiveMessageML();
									  cout<<"------------------ACTIVE11--------------"<< endl;
//#endif
								}
								if (gv->MT2posChanged) {
//#if defined(WIN32)
									 sendMessageML("mt2_reward,1000");
									 receiveMessageML();
									  cout<<"------------------ACTIVE12--------------"<< endl;
//#endif
								}
							}

							if (gv->activeAgents[2]) {
								if (gv->obsLocError[0] == true) {
//#if defined(WIN32)
									 sendMessageML("loc1_reward,-1000");
									 receiveMessageML();
									  cout<<"------------------ACTIVE21--------------"<< endl;
//#endif
								}
								if (gv->obsLocError[1] == true) {
//#if defined(WIN32)
									 sendMessageML("loc2_reward,-1000");
									 receiveMessageML();
									  cout<<"------------------ACTIVE22--------------"<< endl;
//#endif
								}
								if (gv->obsLocError[2] == true) {
//#if defined(WIN32)
									 sendMessageML("loc3_reward,-1000");
									 receiveMessageML();
									  cout<<"------------------ACTIVE23-------------"<< endl;
//#endif
								}
							}

							if (gv->activeAgents[3]) {
								if (gv->path_adv_counter > 0) {
									adv->path_adversary(false, true, gv->objectIndex, false);
//#if defined(WIN32)
									 sendMessageML("gv->path_reward,-1000");
									 receiveMessageML();
//#endif
									gv->path_adv_counter = 0;
									 cout<<"------------------ACTIVE3--------------"<< endl;
								}
							}

							if (gv->activeAgents[4])
							{
								if (gv->obs_adv_counter > 0) {
									adv->obs_preparation(gv->objectIndex);
									adv->obs_adversary(false, true, 12, false);
									adv->obs_adversary(false, true, 13, false);
									adv->obs_adversary(false, true, 14, false);
//#if defined(WIN32)
									 sendMessageML("obs1_reward,-1000");
									 receiveMessageML();
									 sendMessageML("obs2_reward,-1000");
									 receiveMessageML();
									 sendMessageML("obs3_reward,-1000");
									 receiveMessageML();
//#endif
									gv->obs_adv_counter = 0;
									 cout<<"------------------ACTIVE4--------------"<< endl;
								}
							}
						}
						else if (gv->synMode == 2) {
							gv->simTransport = false;
							gv->simConfirmation = false;

							if (gv->activeAgents[3]) {
								if (gv->path_adv_counter > 0) {
									adv->path_adversary(false, true, gv->objectIndex, true);
									gv->path_adv_counter = 0;
								}
							}

							if (gv->activeAgents[4])
							{
								if (gv->obs_adv_counter > 0) {
									adv->obs_preparation(gv->objectIndex);
									adv->obs_adversary(false, true, 12, true);
									adv->obs_adversary(false, true, 13, true);
									adv->obs_adversary(false, true, 14, true);
									gv->obs_adv_counter = 0;
									gv->obs_adv_counter = 0;
								}
							}

							cout << "Sim. info: transport was successful and is safe!" << endl;

							if (gv->writeDataIntoFiles) {
								fstream file;
								file.open("transport_result.csv", std::ios_base::app);
								if (file) {
									if (gv->advancedControlMode) file << to_string(gv->simCounter) + ";transport_successful;" + to_string(gv->agentMode) + "\n";
									else file << to_string(gv->simCounter) + ";transport_successful\n";
								}
								else {
									cout << "Error: writing into file not possible!" << endl;
								}
								file.close();
							}

							cout << "\nExecute on real robot? (press y/n)" << endl;
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
								if(gv -> autoFlag == 4){
									gv -> autoFlag =5;
									break;
								}
							} while (c_in == '-');

							if (c_in == 'y') {
								gv->synMode = 3;
								gv->asynMode = 3;
								gv->realExecPhase = 0;
							}
							else {
								cout << "Sim. info: no execution on real robot!" << endl;
								gv->modeActive = false;
							}

							if (gv->advancedControlMode) {
								srand((unsigned)time(NULL));
								generator1.seed((unsigned)time(NULL));
								generator2.seed((unsigned)time(NULL));
								generator3.seed((unsigned)time(NULL));
							}
						}
					}

					gv->virtSim = false;
				}
				else {
					gv->adv_dev_x = 0.0f; gv->adv_dev_y = 0.0f; gv->adv_dev_z = 0.0;

					pp->nodeIndex = pp->nodeList[pp->nodeIndex].previous;
					for (int i = 0; i < 5; i++) {
						gv->qValuesTarget[i] = pp->nodeList[pp->nodeIndex].nodeQVal[i];
						gv->qValuesDiff[i] = gv->qValuesTarget[i] - gv->qValuesCurrent[i];
					}

					gv->maxValue = -1.0f;
					for (int i = 0; i < 5; i++) {
						if (abs(gv->qValuesDiff[i]) > gv->maxValue) {
							gv->maxValue = abs(gv->qValuesDiff[i]);
							gv->maxValueIndex = i;
						}
					}

					if (gv->maxValue >= pp->simulationStepSize) {
						for (int i = 0; i < 5; i++) {
							gv->qValuesDiff[i] /= ceil(gv->maxValue / pp->simulationStepSize);
						}
					}
				}
			}
			else {
				gv->real_path_edges = ik->increaseSize(gv->real_path_edges, gv->numPointsRealPath, 2);
				gv->real_path_vertices = ik->increaseSize(gv->real_path_vertices, gv->numPointsRealPath, 2);

				gv->real_path_edges[gv->numPointsRealPath].x = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Sin;
				gv->real_path_edges[gv->numPointsRealPath].y = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Cos;
				gv->real_path_edges[gv->numPointsRealPath].z = ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->len4 * gv->q123Cos;
				gv->real_path_vertices[gv->numPointsRealPath] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->pathColors[1].x, gv->pathColors[1].y, gv->pathColors[1].z, 1.0f };
				gv->numPointsRealPath++;

				for (int i = 0; i < 5; i++) {
					gv->qValuesCurrent[i] += gv->qValuesDiff[i];
				}

				gv->q0Sin = sin(gv->qValuesCurrent[0] / 180.0f * M_PI);
				gv->q0Cos = cos(gv->qValuesCurrent[0] / 180.0f * M_PI);
				gv->q1Sin = sin(gv->qValuesCurrent[1] / 180.0f * M_PI);
				gv->q1Cos = cos(gv->qValuesCurrent[1] / 180.0f * M_PI);
				gv->q12Sin = sin((gv->qValuesCurrent[1] + gv->qValuesCurrent[2]) / 180.0f * M_PI);
				gv->q12Cos = cos((gv->qValuesCurrent[1] + gv->qValuesCurrent[2]) / 180.0f * M_PI);
				gv->q123Sin = sin((gv->qValuesCurrent[1] + gv->qValuesCurrent[2] + gv->qValuesCurrent[3]) / 180.0f * M_PI);
				gv->q123Cos = cos((gv->qValuesCurrent[1] + gv->qValuesCurrent[2] + gv->qValuesCurrent[3]) / 180.0f * M_PI);

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
				gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[3], gv->tempCounter);
				gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[4], gv->tempCounter);
				gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[5], gv->tempCounter);
				gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[6], gv->tempCounter);
				gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[7], gv->tempCounter);

				gv->collision = false;
				if (gv->transportPhase == 1) {
					sc->transport_phase1();
				}
				else {
					// Mit sich selbst:
					for (int i = 1; i <= 4; i++) {
						if (cd->checkForCollision(&gv->colliders[i], &gv->colliders[7])) {
							gv->collision = true;
							gv->collisionObsIndex = i;
							break;
						}
						if (cd->checkForCollision(&gv->colliders[i], &gv->grippers[0])) {
							gv->collision = true;
							gv->collisionObsIndex = i;
							break;
						}
						if (cd->checkForCollision(&gv->colliders[i], &gv->grippers[1])) {
							gv->collision = true;
							gv->collisionObsIndex = i;
							break;
						}
					}

					// Arm mit Umgebung:
					if (!gv->collision) {
						for (int i = 5; i <= 7; i++) {
							for (int j = 8; j < gv->collidersCount; j++) {
								if (cd->checkForCollision(&gv->colliders[i], &gv->colliders[j])) {
									gv->collision = true;
									cout<< "collision with9 :"<< i<< endl;
									gv -> collisionObsIndex = j;
									break;
								}
							}
							if (gv->collision) break;
						}
					}

					// Greifer mit Umgebung:
					if (!gv->collision) {
						for (int i = 8; i < gv->collidersCount; i++) {
							if (i != gv->objectToTransport) {
								if (cd->checkForCollision(&gv->grippers[0], &gv->colliders[i])) {
									gv->collision = true;
									cout<< "collision with10 :"<< i<< endl;
									gv -> collisionObsIndex = i;
									break;
								}
								if (cd->checkForCollision(&gv->grippers[1], &gv->colliders[i])) {
									gv->collision = true;
									cout<< "collision with11 :"<< i<< endl;
									gv -> collisionObsIndex = i;
									break;
								}
							}
						}
					}

					// Mit Boden!
					if (!gv->collision) {
						if (cd->checkForCollisionWithGround(&gv->colliders[7], -cd->epsilon2) || cd->checkForCollisionWithGround(&gv->grippers[0], -cd->epsilon2) || cd->checkForCollisionWithGround(&gv->grippers[0], -cd->epsilon2)) {
							gv->collision = true;
						}
					}
				}

				gv->real_path_edges[gv->numPointsRealPath].x = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Sin;
				gv->real_path_edges[gv->numPointsRealPath].y = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Cos;
				gv->real_path_edges[gv->numPointsRealPath].z = ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->len4 * gv->q123Cos;
				gv->real_path_vertices[gv->numPointsRealPath] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->pathColors[1].x, gv->pathColors[1].y, gv->pathColors[1].z, 1.0f };
				gv->numPointsRealPath++;

				gv->tempCounter = gv->totalPlatePoints;
				for (int i = 0; i < gv->collidersCount; i++) {
					gv->tempCounter += gv->colliders[i].facesTimes3;
				}
				gv->tempCounter = hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[0], gv->tempCounter);
				hf->calcEdgesForConvexHull(gv->object_edges, &gv->grippers[1], gv->tempCounter);

				if (gv->collision) {
					gv->aborting = true;
					gv->replanning = false;
				}
				else {
					gv->aborting = false;
					gv->replanning = false;

					if (gv->calculateDistanceMetric) {
						fo->calcMetricValue();
//if defined(WIN32)
						 sendMessageML("metric," + hf->to_string_with_precision(float(fo->minDistancesForMetric[0]), 4) + "," + hf->to_string_with_precision(float(fo->minDistancesForMetric[1]), 4) + "," + hf->to_string_with_precision(float(fo->minDistancesForMetric[2]), 4)
							+ "," + hf->to_string_with_precision(float(fo->minDistancesForMetric[3]), 4) + "," + hf->to_string_with_precision(float(fo->minDistancesForMetric[4]), 4) + "," + hf->to_string_with_precision(float(fo->minDistancesForMetric[5]), 4)
							+ "," + hf->to_string_with_precision(float(fo->minDistancesForMetric[6]), 4) + "," + hf->to_string_with_precision(float(fo->minDistancesForMetric[7]), 4) + "," + hf->to_string_with_precision(float(fo->minDistancesForMetric[8]), 4)
							+ "," + hf->to_string_with_precision(float(fo->minDistancesForMetric[9]), 4) + "," + hf->to_string_with_precision(float(fo->minDistancesForMetric[10]), 4));
						 receiveMessageML();
//#endif
					}

					mt->updateMonitoredSpace(false, true);
					if (hf->checkStateChange()) {
						if (!fo->checkIfPathIsCollisionFree()) {
							gv->pathAltStartPos.x = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Sin;
							gv->pathAltStartPos.y = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Cos;
							gv->pathAltStartPos.z = ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->len4 * gv->q123Cos;

							gv->objectPosReachable = false;
							if (gv->transportPhase == 0) {
								for (int i = 0; i < 10; i++) {
									if (pp->calcPath(gv->qValuesObjectStart, gv->qValuesCurrent, &gv->pathStartPos, &gv->pathAltStartPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3)) {
										gv->objectPosReachable = true;
										break;
									}
								}
							}
							else if (gv->transportPhase == 1) {
								for (int i = 0; i < 10; i++) {
									if (pp->calcPath(gv->qValuesObjectEnd, gv->qValuesCurrent, &gv->pathEndPos, &gv->pathAltStartPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3)) {
										gv->objectPosReachable = true;
										break;
									}
								}
							}
							else {
								for (int i = 0; i < 10; i++) {
									if (pp->calcPath(gv->qValuesStandby, gv->qValuesCurrent, &gv->standbyPos, &gv->pathAltStartPos, false, generator1,distribution1,generator2,distribution2,generator3,distribution3)) {
										gv->objectPosReachable = true;
										break;
									}
								}
							}

							if (gv->objectPosReachable) {
								cout << "Info: gv->path gv->replanning..." << endl;
								pp->optimizePath();

								gv->planned_path_edges = ik->cutOff(gv->planned_path_edges, gv->numPointsPlannedPath, 0);
								gv->planned_path_vertices = ik->cutOff(gv->planned_path_vertices, gv->numPointsPlannedPath, 0);
								gv->numPointsPlannedPath = 0;

								gv->real_path_edges = ik->cutOff(gv->real_path_edges, gv->numPointsRealPath, 0);
								gv->real_path_vertices = ik->cutOff(gv->real_path_vertices, gv->numPointsRealPath, 0);
								gv->numPointsRealPath = 0;

								pp->nodeIndex = 1;
								while (pp->nodeIndex != 0) {
									pp->nodeIndexTemp = pp->nodeList[pp->nodeIndex].previous;
									gv->planned_path_edges = ik->increaseSize(gv->planned_path_edges, gv->numPointsPlannedPath, 2);
									gv->planned_path_vertices = ik->increaseSize(gv->planned_path_vertices, gv->numPointsPlannedPath, 2);

									gv->planned_path_edges[gv->numPointsPlannedPath].x = pp->nodeList[pp->nodeIndex].x;
									gv->planned_path_edges[gv->numPointsPlannedPath].y = pp->nodeList[pp->nodeIndex].y;
									gv->planned_path_edges[gv->numPointsPlannedPath].z = pp->nodeList[pp->nodeIndex].z;
									gv->planned_path_vertices[gv->numPointsPlannedPath] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->pathColors[2].x, gv->pathColors[2].y, gv->pathColors[2].z, 1.0f };

									gv->numPointsPlannedPath++;

									gv->planned_path_edges[gv->numPointsPlannedPath].x = pp->nodeList[pp->nodeIndexTemp].x;
									gv->planned_path_edges[gv->numPointsPlannedPath].y = pp->nodeList[pp->nodeIndexTemp].y;
									gv->planned_path_edges[gv->numPointsPlannedPath].z = pp->nodeList[pp->nodeIndexTemp].z;
									gv->planned_path_vertices[gv->numPointsPlannedPath] = collider::Vertex{ 0.0f, 0.0f, 0.0f, gv->pathColors[2].x, gv->pathColors[2].y, gv->pathColors[2].z, 1.0f };

									gv->numPointsPlannedPath++;

									pp->nodeIndex = pp->nodeIndexTemp;
								}

								pp->nodeIndex = pp->nodeList[1].previous;

								for (int i = 0; i < 5; i++) {
									gv->qValuesTarget[i] = pp->nodeList[pp->nodeIndex].nodeQVal[i];
									gv->qValuesDiff[i] = gv->qValuesTarget[i] - gv->qValuesCurrent[i];
								}

								gv->maxValue = -1.0f;
								for (int i = 0; i < 5; i++) {
									if (abs(gv->qValuesDiff[i]) > gv->maxValue) {
										gv->maxValue = abs(gv->qValuesDiff[i]);
										gv->maxValueIndex = i;
									}
								}

								if (gv->maxValue >= pp->simulationStepSize) {
									for (int i = 0; i < 5; i++) {
										gv->qValuesDiff[i] /= ceil(gv->maxValue / pp->simulationStepSize);
									}
								}

								gv->adv_dev_x = 0.0f; gv->adv_dev_y = 0.0f; gv->adv_dev_z = 0.0f;
								gv->replanning = true;
							}
							else {
								gv->aborting = true;
							}
						}
					}
				}

				if (!gv->aborting && !gv->replanning) {
					if (gv->synMode == 0) {
						if (gv->activeAgents[3]) {
							if (pp->nodeIndex != 0) {
								if ((gv->transportPhase == 0 && pp->nodeList[pp->nodeIndex].previous != 0 && min(cd->gjk(&gv->grippers[0], &gv->colliders[gv->objectToTransport]), cd->gjk(&gv->grippers[1], &gv->colliders[gv->objectToTransport])) >= 1.5f) || (gv->transportPhase == 2 && pp->nodeList[1].previous != pp->nodeIndex && min(cd->gjk(&gv->grippers[0], &gv->colliders[gv->objectToTransport]), cd->gjk(&gv->grippers[1], &gv->colliders[gv->objectToTransport])) >= 1.5f) || gv->transportPhase == 1) {
									if (abs(gv->qValuesTarget[gv->maxValueIndex] - gv->qValuesCurrent[gv->maxValueIndex]) >= 3.0f) {
										if (gv->path_adv_counter == 0) adv->path_adversary(true, false, gv->objectIndex, false);
										else adv->path_adversary(false, false, gv->objectIndex, false);
										gv->path_adv_counter++;
									}
									else if (abs(gv->qValuesTarget[gv->maxValueIndex] - gv->qValuesCurrent[gv->maxValueIndex]) < 3.0f) {
										if (gv->path_adv_counter > 0) {
											adv->path_adversary(false, true, gv->objectIndex, false);
											gv->path_adv_counter = 0;
										}
									}
								}
							}
						}

						if (gv->activeAgents[4]) {
							if (abs(gv->qValuesTarget[gv->maxValueIndex] - gv->qValuesCurrent[gv->maxValueIndex]) >= 1.5f) {
								if (gv->obs_adv_counter == 0) {
									adv->obs_preparation(gv->objectIndex);
									adv->obs_adversary(true, false, 12, false);
									adv->obs_adversary(true, false, 13, false);
									adv->obs_adversary(true, false, 14, false);
								}
								else {
									adv->obs_preparation(gv->objectIndex);
									adv->obs_adversary(false, false, 12, false);
									adv->obs_adversary(false, false, 13, false);
									adv->obs_adversary(false, false, 14, false);
								}
								gv->obs_adv_counter++;
							}
							else {
								if (gv->obs_adv_counter > 0) {
									adv->obs_preparation(gv->objectIndex);
									adv->obs_adversary(false, true, 12, false);
									adv->obs_adversary(false, true, 13, false);
									adv->obs_adversary(false, true, 14, false);
									gv->obs_adv_counter = 0;
								}
							}
						}
					}
					else if (gv->synMode == 2) {
						if (gv->activeAgents[3]) {
							if (pp->nodeIndex != 0) {
								if ((gv->transportPhase == 0 && pp->nodeList[pp->nodeIndex].previous != 0 && min(cd->gjk(&gv->grippers[0], &gv->colliders[gv->objectToTransport]), cd->gjk(&gv->grippers[1], &gv->colliders[gv->objectToTransport])) >= 1.5f) || (gv->transportPhase == 2 && pp->nodeList[1].previous != pp->nodeIndex && min(cd->gjk(&gv->grippers[0], &gv->colliders[gv->objectToTransport]), cd->gjk(&gv->grippers[1], &gv->colliders[gv->objectToTransport])) >= 1.5f) || gv->transportPhase == 1) {
									if (abs(gv->qValuesTarget[gv->maxValueIndex] - gv->qValuesCurrent[gv->maxValueIndex]) >= 3.0f) {
										if (gv->path_adv_counter == 0) adv->path_adversary(true, false, gv->objectIndex, true);
										else adv->path_adversary(false, false, gv->objectIndex, true);
										gv->path_adv_counter++;
									}
									else if (abs(gv->qValuesTarget[gv->maxValueIndex] - gv->qValuesCurrent[gv->maxValueIndex]) < 3.0f) {
										if (gv->path_adv_counter > 0) {
											adv->path_adversary(false, true, gv->objectIndex, true);
											gv->path_adv_counter = 0;
										}
									}
								}
							}
						}

						if (gv->activeAgents[4]) {
							if (abs(gv->qValuesTarget[gv->maxValueIndex] - gv->qValuesCurrent[gv->maxValueIndex]) >= 1.5f) {
								if (gv->obs_adv_counter == 0) {
									adv->obs_preparation(gv->objectIndex);
									adv->obs_adversary(true, false, 12, true);
									adv->obs_adversary(true, false, 13, true);
									adv->obs_adversary(true, false, 14, true);
								}
								else {
									adv->obs_preparation(gv->objectIndex);
									adv->obs_adversary(false, false, 12, true);
									adv->obs_adversary(false, false, 13, true);
									adv->obs_adversary(false, false, 14, true);
								}
								gv->obs_adv_counter++;
							}
							else {
								if (gv->obs_adv_counter > 0) {
									adv->obs_preparation(gv->objectIndex);
									adv->obs_adversary(false, true, 12, true);
									adv->obs_adversary(false, true, 13, true);
									adv->obs_adversary(false, true, 14, true);
									gv->obs_adv_counter = 0;
								}
							}
						}
					}
				}
				else if (gv->replanning && !gv->aborting) {
					if (gv->synMode == 0) {
						if (gv->activeAgents[3]) {
							if (gv->path_adv_counter > 0) {
								adv->path_adversary(false, true, gv->objectIndex, false);
								gv->path_adv_counter = 0;
							}
						}

						if (gv->activeAgents[4]) {
							if (gv->obs_adv_counter > 0) {
								adv->obs_preparation(gv->objectIndex);
								adv->obs_adversary(false, true, 12, false);
								adv->obs_adversary(false, true, 13, false);
								adv->obs_adversary(false, true, 14, false);
								gv->obs_adv_counter = 0;
							}
						}
					}
					else if (gv->synMode == 2) {
						if (gv->activeAgents[3]) {
							if (gv->path_adv_counter > 0) {
								adv->path_adversary(false, true, gv->objectIndex, true);
								gv->path_adv_counter = 0;
							}
						}

						if (gv->activeAgents[4]) {
							if (gv->obs_adv_counter > 0) {
								adv->obs_preparation(gv->objectIndex);
								adv->obs_adversary(false, true, 12, true);
								adv->obs_adversary(false, true, 13, true);
								adv->obs_adversary(false, true, 14, true);
								gv->obs_adv_counter = 0;
							}
						}
					}
				}
				else if (gv->aborting && !gv->replanning) {
					cout << "Info: gv->collision detected/gv->replanning failed -> abort..." << endl;
					if (gv->synMode == 0) {
						gv->modeActive = false;

						if (gv->activeAgents[0] && gv->collision) {
//#if defined(WIN32)
							 sendMessageML("safety_reward,-1000");
							 receiveMessageML();
							 cout<<"------------------ACTIVE0--------------"<< endl;
//#endif
						}

						if (gv->activeAgents[1] && gv->collision) {
							if (gv->MT1posChanged) {
//#if defined(WIN32)
								 sendMessageML("mt1_reward,-1000");
								 receiveMessageML();
								 cout<<"------------------ACTIVE11--------------"<< endl;
//#endif
							}
							if (gv->MT2posChanged) {
//#if defined(WIN32)
								 sendMessageML("mt2_reward,-1000");
								 receiveMessageML();
								 cout<<"------------------ACTIVE12-------------"<< endl;
//#endif
							}
						}

						if (gv->activeAgents[2] && gv->collision) {
							if (gv->obsLocError[0] == true) {
//#if defined(WIN32)
								 sendMessageML("loc1_reward,1000");
								 receiveMessageML();
								 cout<<"------------------ACTIVE21--------------"<< endl;
//#endif
							}
							if (gv->obsLocError[1] == true) {
//#if defined(WIN32)
								 sendMessageML("loc2_reward,1000");
								 receiveMessageML();
								 cout<<"------------------ACTIVE22-------------"<< endl;
//#endif
							}
							if (gv->obsLocError[2] == true) {
//#if defined(WIN32)
								 sendMessageML("loc3_reward,1000");
								 receiveMessageML();
								 cout<<"------------------ACTIVE23--------------"<< endl;
//#endif
							}
						}

						if (gv->activeAgents[3]) {
							if (gv->path_adv_counter > 0) {
								adv->path_adversary(false, true, gv->objectIndex, false);

								if (gv->collision) {
//#if defined(WIN32)
									 sendMessageML("gv->path_reward,1000");
									 receiveMessageML();
									 cout<<"------------------ACTIVE3--------------"<< endl;
//#endif
								}

								gv->path_adv_counter = 0;
							}
						}

						if (gv->activeAgents[4])
						{
							if (gv->obs_adv_counter > 0) {
								adv->obs_preparation(gv->objectIndex);
								adv->obs_adversary(false, true, 12, false);
								adv->obs_adversary(false, true, 13, false);
								adv->obs_adversary(false, true, 14, false);

								if (gv->collision) {
//#if defined(WIN32)
									 sendMessageML("obs1_reward,1000");
									 receiveMessageML();
									 sendMessageML("obs2_reward,1000");
									 receiveMessageML();
									 sendMessageML("obs3_reward,1000");
									 receiveMessageML();
									 cout<<"------------------ACTIVE4--------------"<< endl;
//#endif
								}

								gv->obs_adv_counter = 0;
							}
						}
					}
					else if (gv->synMode == 2) {
						gv->simTransport = false;
						gv->simConfirmation = false;

						if (gv->activeAgents[3]) {
							if (gv->path_adv_counter > 0) {
								adv->path_adversary(false, true, gv->objectIndex, true);
								gv->path_adv_counter = 0;
							}
						}

						if (gv->activeAgents[4])
						{
							if (gv->obs_adv_counter > 0) {
								adv->obs_preparation(gv->objectIndex);
								adv->obs_adversary(false, true, 12, true);
								adv->obs_adversary(false, true, 13, true);
								adv->obs_adversary(false, true, 14, true);
								gv->obs_adv_counter = 0;
							}
						}

						if (gv->collision) {
							cout << "Sim. info: gv->collision detected -> transport is NOT SAFE!" << endl;
						}
						else {
							cout << "Sim. info: gv->path gv->replanning failed -> no safety evaluation possible!" << endl;
						}

						gv->simAskUser = true;
					}

					if (gv->transportPhase == 1) showSafetySpace = true;

					gv->virtSim = false;
					gv->transportPhase = 0;
				}

				gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[0], 0);
				gv->tempCounter = hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[1], gv->tempCounter);
				hf->calcEdgesForMesh(gv->distorted_obs_edges, &gv->distortedObstacles[2], gv->tempCounter);

				gv->timeCounter++;

				repaint = true;
			}
		}
		else if (gv->syncSim) {
			sc->sim_control_sync_sim(
					generator1,distribution1,
					generator2,distribution2,
					generator3,distribution3);
		}

		shader.bind();
		vertexBufferObjects.bind();
		if (gv->selectionMarkerMode && repaint) {
			hf->calcVerticesForObjects(gv->object_vertices, gv->object_edges, gv->numPointsObject, -1);
			vertexBufferObjects.update(gv->object_vertices, gv->numPointsObject);
		}
		else if (repaint) {
			hf->calcVerticesForObjects(gv->object_vertices, gv->object_edges, gv->numPointsObject, gv->totalPlatePoints);
			vertexBufferObjects.update(gv->object_vertices, gv->numPointsObject);
		}
		glDrawArrays(GL_TRIANGLES, 0, gv->numPointsObject);
		vertexBufferObjects.unbind();

		vertexBufferPatterns.bind();
		if (repaint) {
			hf->calcVerticesForPoints(gv->object_pattern_vertices, gv->object_pattern_edges, gv->numPointsPattern);
			vertexBufferPatterns.update(gv->object_pattern_vertices, gv->numPointsPattern);
		}
		glDrawArrays(GL_POINTS, 0, gv->numPointsPattern);
		vertexBufferPatterns.unbind();

		if (gv->numPointsPlannedPath > 0) {
			vertexBufferPlannedPath.bind();
			if (repaint) {
				hf->calcVerticesForLines(gv->planned_path_vertices, gv->planned_path_edges, gv->numPointsPlannedPath);
				vertexBufferPlannedPath.update(gv->planned_path_vertices, gv->numPointsPlannedPath);
			}
			glDrawArrays(GL_LINES, 0, gv->numPointsPlannedPath);
			vertexBufferPlannedPath.unbind();
		}

		if (gv->numPointsRealPath > 0) {
			vertexBufferRealPath.bind();
			if (repaint) {
				hf->calcVerticesForLines(gv->real_path_vertices, gv->real_path_edges, gv->numPointsRealPath);
				vertexBufferRealPath.update(gv->real_path_vertices, gv->numPointsRealPath);
			}
			glDrawArrays(GL_LINES, 0, gv->numPointsRealPath);
			vertexBufferRealPath.unbind();
		}

		if (gv->synMode == 1) {
			vertexBufferWorkpieceArrows.bind();
			if (repaint) {
				hf->calcVerticesForLines(gv->workpiece_arrows_vertices, gv->workpiece_arrows_edges, 24);
				vertexBufferWorkpieceArrows.update(gv->workpiece_arrows_vertices, 24);
			}
			glDrawArrays(GL_LINES, 0, 24);
			vertexBufferWorkpieceArrows.unbind();
		}

		vertexBufferMTviewfields.bind();
		if (repaint) {
			hf->calcVerticesForLines(gv->mt_viewfields_vertices, gv->mt_viewfields_edges, 32);
			vertexBufferMTviewfields.update(gv->mt_viewfields_vertices, 32);
		}
		glDrawArrays(GL_LINES, 0, 32);
		vertexBufferMTviewfields.unbind();

		vertexBufferDistObjects.bind();
		if (repaint) {
			hf->calcVerticesForLines(gv->distorted_obs_vertices, gv->distorted_obs_edges, gv->distortedObstacles[0].facesTimes3 * 6);
			vertexBufferDistObjects.update(gv->distorted_obs_vertices, gv->distortedObstacles[0].facesTimes3 * 6);
		}
		glDrawArrays(GL_LINES, 0, gv->distortedObstacles[0].facesTimes3 * 6);
		vertexBufferDistObjects.unbind();

		if (showSafetySpace || (gv->transportPhase == 1 && gv->virtSim && gv->synMode == 0) || (gv->transportPhase == 1 && gv->virtSim && gv->synMode == 2) || (gv->transportPhase == 1 && gv->synMode == 3) || (gv->transportPhase == 1 && gv->synMode == 4)) {
			vertexBufferSafetyCol.bind();
			if (repaint) {
				hf->calcVerticesForLines(gv->safety_col_vertices, gv->safety_col_edges, gv->safetyCollider->facesTimes3 * 2);
				vertexBufferSafetyCol.update(gv->safety_col_vertices, gv->safetyCollider->facesTimes3 * 2);
			}
			glDrawArrays(GL_LINES, 0, gv->safetyCollider->facesTimes3 * 2);
			vertexBufferSafetyCol.unbind();
		}

		if (gv->synMode == 1 || (gv->synMode == 3 && gv->realExecPhase == 1) || (gv->synMode == 3 && gv->realExecPhase == 4)) {
			vertexBufferWorkpieceMesh.bind();
			if (repaint) {
				hf->calcVerticesForLines(gv->workpiece_mesh_vertices, gv->workpiece_mesh_edges, gv->workpieceMesh->facesTimes3 * 2);
				vertexBufferWorkpieceMesh.update(gv->workpiece_mesh_vertices, gv->workpieceMesh->facesTimes3 * 2);
			}
			glDrawArrays(GL_LINES, 0, gv->workpieceMesh->facesTimes3 * 2);
			vertexBufferWorkpieceMesh.unbind();
		}

		if (gv->virtualMode) {
			if ((gv->synMode == 2 && gv->simTargetSelection) || gv->synMode == 3 || gv->synMode == 4) {
				vertexBufferGroundTruths.bind();
				if (repaint) {
					hf->calcVerticesForLines(gv->distorted_obs_vertices, gv->ground_truth_obs_edges, gv->colliders[12].facesTimes3 * 6);
					vertexBufferGroundTruths.update(gv->distorted_obs_vertices, gv->colliders[12].facesTimes3 * 6);
				}
				glDrawArrays(GL_LINES, 0, gv->colliders[12].facesTimes3 * 6);
				vertexBufferGroundTruths.unbind();
			}
		}

		if (gv->numPointsMTrays > 0) {
			vertexBufferMTrays.bind();
			if (repaint) {
				hf->calcVerticesForLines(gv->mt_rays_vertices, gv->mt_rays_edges, gv->numPointsMTrays);
				vertexBufferMTrays.update(gv->mt_rays_vertices, gv->numPointsMTrays);
			}
			glDrawArrays(GL_LINES, 0, gv->numPointsMTrays);
			vertexBufferMTrays.unbind();
		}

		shader.unbind();

		fontShader.bind();
		glm::mat4 ortho = glm::ortho(0.0f, 800.0f, 800.0f, 0.0f);
		glUniformMatrix4fv(glGetUniformLocation(fontShader.getShaderId(), "u_modelViewProj"), 1, GL_FALSE, &ortho[0][0]);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_DEPTH_TEST);


		Font::shader_handle_text(fps, pausedPixelOffsetsX, pausedPixelOffsetsY, hf,
				font, fontShader, fontUI, fontBig);

		fontShader.unbind();

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);

		SDL_GL_SwapWindow(window);

		endCounter = SDL_GetPerformanceCounter();
		delta = ((float)(endCounter - startCounter)) / (float)perfCounterFrequency * 1000.0f; // in ms

		if (gv->simSpeed != 6) {
			if (gv->simSpeed >= 2) {
				if (delta < 16.666f / float(gv->simSpeed - 1)) {
					SDL_Delay((int)floor(16.666f / float(gv->simSpeed - 1) - delta));
				}
			}
			else {
				if (delta < 16.666f * 2.5f) {
					SDL_Delay((int)floor(16.666f * 2.5f - delta));
				}
			}
		}

		if (frameCounter >= gv->fpsMax[gv->simSpeed - 1] - 1) {
			fpsEnd = SDL_GetPerformanceCounter();
			fps = (float)perfCounterFrequency / (float)(fpsEnd - fpsInit) * float(gv->fpsMax[gv->simSpeed - 1]);
			fps = roundf(fps * float(gv->fpsMax[gv->simSpeed - 1])) / float(gv->fpsMax[gv->simSpeed - 1]);
			frameCounter = 0;
		}
		else {
			frameCounter++;
		}
		
		if (gv->simAskUser) {
			gv->simAskUser = false;

			if (gv->writeDataIntoFiles) {
				fstream file;
				file.open("transport_result.csv", std::ios_base::app);
				if (file) {
					if (gv->collision) {
						if (gv->advancedControlMode) file << to_string(gv->simCounter) << ";gv->collision;" + to_string(gv->agentMode) + "\n";
						else file << to_string(gv->simCounter) << ";gv->collision\n";
					}
					else {
						if (gv->advancedControlMode) file << to_string(gv->simCounter) + ";gv->path_replanning_failed;" + to_string(gv->agentMode) + "\n";
						else file << to_string(gv->simCounter) + ";gv->path_replanning_failed\n";
					}
				}
				else {
					cout << "Error: writing into file not possible!" << endl;
				}
				file.close();
			}

			cout << "\nExecute on real robot anyway? (press y/n)" << endl;
			//char c_in = hf->wait_for_user_input_yn();
			char c_in = '-';
			SDL_Event event_sub;
			do {
					while (SDL_PollEvent(&event_sub)) {
						if (event_sub.type == SDL_KEYUP) {
							if (event_sub.key.keysym.sym == SDLK_y) {
								c_in = 'y';
							} else if (event_sub.key.keysym.sym == SDLK_n) {
								c_in = 'n';
							}
						}
					}
					if(gv -> autoFlag == 4){
									gv -> autoFlag = 5;
									break;
								}
				} while (c_in == '-');
			if (c_in == 'y') {
				gv->synMode = 3;
				gv->asynMode = 3;
				gv->realExecPhase = 0;
			}
			else {
				cout << "Sim. info: no execution on real robot!" << endl;
				gv->modeActive = false;
			}

			if (gv->advancedControlMode) {
				srand((unsigned)time(NULL));
				generator1.seed((unsigned)time(NULL));
				generator2.seed((unsigned)time(NULL));
				generator3.seed((unsigned)time(NULL));
			}
		}

		repaint = false;
		showSafetySpace = false;
	}
	return 0;
}

