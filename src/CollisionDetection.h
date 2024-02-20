/*
 * CollisionDetection.h
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_

#include <cmath>
#include <iostream>
#include "collider.h"

#include "InverseKinematic.h"
#include "ConvHull3d.h"
#include "GlobalVariables.h"


using namespace std;

class CollisionDetection {

private:
	bool bInit;
	CollisionDetection();
	static CollisionDetection* cd;

public:
	static CollisionDetection* get_instance();
	void init();
	virtual ~CollisionDetection();

	const float epsilon2 = 0.01f;
	const float epsilon3 = 0.001f;
	const float epsilon4 = 0.0001f;
	const float epsilon5 = 0.00001f;

	const float nickAngleMT = 		0.0f;
	const float horViewAngle = 		64.4425f;
	const float vertViewAngle = 	50.5983f;
	const float minObjectDistance = 7.5f    ;
	const float maxObjectDistance = 65.0f   ;
	const float patternSize = 		2.5f    ;
	const float patternMaxAngle = 	65.0f   ;
	const float patternMaxTurnVal = 0.2f;

//	= new float[12]{ 2.57f, 0.0f, 6.1375f - 3.5f, cosf(nickAngleMT / 180.0f * M_PI), 0.0f, sinf(nickAngleMT / 180.0f * M_PI), -sinf(nickAngleMT / 180.0f * M_PI), 0.0f, cosf(nickAngleMT / 180.0f * M_PI), 0.0f, 1.0f, 0.0f };

	const int maxIterations = 64; // iteration limit for the GJK algorithm

	const float* camVectorsStatic ;
	const float* workpieceArrowPositionStatic;
	float* workpieceArrowPosition1;
	float* workpieceArrowPosition2;
	float* workpieceArrowPosition3;
	float* workpieceArrowPosition4;

	double* col1_s;
	double* col2_s;
	double* v_direction;

	struct simplex {
	    int    nvrtx;
	    double vrtx[4][3];
	    int    wids[4];
	    double lambdas[4];
	};

	simplex* gjk_simplex;

	double n[3], pq[3], pr[3], ntmp[3], vrt[3], s1[3], s2[3], s3[3], s4[3], s1s2[3], s1s3[3], s1s4[3], si[3], sj[3], sk[3], v[3], v_old[3], vminus[3], w[3];
	double s, maxs, tmp, det134, norm2Wmax, tesnorm, eps_rel, eps_rel2, eps_tot, exeedtol_rel;
	int better, hff1f_s12, hff1f_s13, hff2f_23, hff2f_32, hff2_ik, hff2_jk, hff2_ki, hff2_kj, i_gjk, j_gjk, k_gjk, t_gjk, hff1_tests[3];
	int testLineThree, testLineFour, testPlaneTwo, testPlaneThree, testPlaneFour, dotTotal, sss_gjk, absTestin, nullV, iters_gjk;

	float maxSizeTemp;
	// variables for temporally stroing the point positions after operations like translation or rotation:
	float xVal_cd, yVal_cd, zVal_cd;
	float xVal2_cd, yVal2_cd, zVal2_cd;


	// define three 3x3 matrices for realizing point rotations along three different axes:
	float firstMatrixValuesX[3], firstMatrixValuesY[3], firstMatrixValuesZ[3];
	float secondMatrixValuesX[3], secondMatrixValuesY[3], secondMatrixValuesZ[3];
	float thirdMatrixValuesX[3], thirdMatrixValuesY[3], thirdMatrixValuesZ[3];

	void   crossProduct(const double* a, const double* b, double* c);
	double  determinant(const double* p, const double* q, const double* r);
	void  projectOnLine(const double* p, const double* q, double* v);
	void  projectOnPlane(const double* p, const double* q, const double* r, double* v);
	int hff1(const double* p, const double* q);
	int hff2(const double* p, const double* q, const double* r);
	int hff3(const double* p, const double* q, const double* r);
	void support(const collider* col, const int bd_index, const double* v);
	void S1D(struct simplex* s, double* v);
	void S2D(struct simplex* s, double* v);
	void S3D(struct simplex* s, double* v);
	void subalgorithm(struct simplex* s, double* v);
	void initMaxSize(collider* col);
	void updateMatrices(float rotHor, float rotNick, float rotRoll);
	void updateMatricesInverse(float rotHor, float rotNick, float rotRoll);
	void updateMatricesTransport(float rotHor, float rotNick, float rotRoll, float axis_x, float axis_y, float axis_z);
	void rotationMatrix(float inX, float inY, float inZ, int index);
	void redoRotation(collider* col, float x_in, float y_in, float z_in);
	void undoRotation(collider* col, float x_in, float y_in, float z_in);
	void overrideCollider(collider* dest, collider* source);
	void updateColliderSize(collider* col, float x_size, float y_size, float z_size);
	void recalculateCollider(collider* col, float offX, float offY, float offZ, float rotHor, float rotNick, float rotRoll, bool recalcPattern = false, bool recalcMTCameraPos = false, float* camVectors = nullptr, bool recalcOrientationArrow = false, float* arrowPosition = nullptr);
	void recalculateColliderTransport(collider* col, float offX, float offY, float offZ, float rotHor, float rotNick, float rotRoll, bool recalcPattern = false);
	double gjk(collider* col1, collider* col2);
	bool checkForCollisionAxisAligned(collider* a, collider* b);

	bool checkForCollision(collider* col1, collider* col2);
	bool checkForCollisionWithGround(collider* col, float floorHeight);
	float distApproximation(collider* col1, collider* col2);

	void init_colliders4collision_detection_and_visualization();

};

#endif /* COLLISIONDETECTION_H_ */
