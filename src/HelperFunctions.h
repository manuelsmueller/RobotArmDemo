/*
 * HelperFunctions.h
 *
 *  Created on: 05.12.2022
 *      Author: manuel
 */

#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_

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

#if defined(WIN32)
#include <SDL.h>
#else
#include <SDL2/SDL.h>
#endif

#include "MonitoringToolModels.h"
#include "CollisionDetection.h"
#include "GlobalVariables.h"
#include "InverseKinematic.h"
#include "PathPlanner.h"
//#include "../frequent_operations.h"
//#include "../protagonists.h"
//#include "../adversaries.h"

class HelperFunctions {
private:
	HelperFunctions();
	GlobalVariables *gv;
	PathPlanner *pp;

	static HelperFunctions* hf;

	static constexpr int totalPatternCount = 8 + 6 + 8 + 2 + 4 + 3 + 3;
	const int objectIndicesFromPattern[totalPatternCount]{ 8,8, 9,9,9,9, 10,10,10, 11,11,11, 12,12,12,12,12,12,12,12, 13,13,13,13,13,13, 14,14,14,14,14,14,14,14 };
	const int patternIndicesFromPattern[totalPatternCount]{ 4,0, 12,0,8,4, 0,4,8, 0,4,8, 12,8,20,16,0,4,24,28, 12,8,16,0,4,20, 12,8,20,16,0,4,24,28 };
	float distortedRelativePosition[4];//ToDo: fill with data @frequent operations

public:
	virtual ~HelperFunctions();
	static HelperFunctions* get_instance();

	void getColorIndex(int edgeIndex);
	void calcDisplayPos(float xPos, float yPos, float zPos) ;
	void updateTransformationMatrix();
	int calcEdgesForConvexHull(collider::Edge* edges, collider* col, int counter);
	int calcEdgesForMesh(collider::Edge* edges, collider* col, int counter);
	int calcEdgesForPatterns(collider::Edge* edges, collider* col, int counter);
	void calcEdgesForWorkpieceArrows(collider::Edge* edges, int workpieceIndex);
	void calcEdgesForViewFunnel(collider::Edge* edges, int mtIndex);
	void calcVerticesForObjects(collider::Vertex* vertices, collider::Edge* edges, int numPoints, int excludeStart);
	void calcVerticesForLines(collider::Vertex* vertices, collider::Edge* edges, int numPoints);
	void calcVerticesForPoints(collider::Vertex* vertices, collider::Edge* edges, int numPoints);
//	bool checkForFreeSight(float startX, float startY, float startZ, float endX, float endY, float endZ, int obstacleIndex, int mtIndex, float* camVectors);
//	int isObjectVisible(int colIndex, int mtIndex);
	bool checkStateChange();

	void openGLDebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam);
	void requestRealSystemState();
//	bool isPatternVisible(int patternIndex, int mtIndex, bool detectionMode);
	void storePathRelatedData(int storageIndex);
	void reloadPathRelatedData(int storageIndex);
	void getMTPositionData(int mtIndex);
	void increaseSimSpeed() ;
	void resetCamPos();
	void resetSelectionMarkerPos() ;
	bool processConfirmButtonPress() ;
	void selectionMarkerModeSwitch();
	void syncVisualization(vector<string> splitList);

	void init_arrays_for_graphic_card();
	void recalculatesCollidersOfRobot();

	char wait_for_user_input_yn();
	char wait_for_user_input_space();


//	void train_agents(bool &repaint);

	template<class Type>
	std::string to_string_with_precision(Type value, int digits)
	{
		std::ostringstream out;
		out.precision(digits);
		out << std::fixed << value;
		return out.str();
	}

};
#endif /* HELPERFUNCTIONS_H_ */
