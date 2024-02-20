/*
 * CollisionDetection.cpp
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 */

#include "CollisionDetection.h"

#define eps_rel22  1e-10
#define eps_tot22  1e-12

#define dotProduct(a, b) (a[0]*b[0]+a[1]*b[1]+a[2]*b[2])
#define norm2(a) (a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

#define getvrtx(point, location)        point[0] = s->vrtx[location][0];\
		point[1] = s->vrtx[location][1];\
		point[2] = s->vrtx[location][2];

#define select_1ik()  s->nvrtx = 3;\
		for (t_gjk = 0; t_gjk < 3; t_gjk++)\
		s->vrtx[2][t_gjk] = s->vrtx[3][t_gjk];\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[1][t_gjk] = si[t_gjk];\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[0][t_gjk] = sk[t_gjk];

#define select_1ij()  s->nvrtx = 3;\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[2][t_gjk] = s->vrtx[3][t_gjk];\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[1][t_gjk] = si[t_gjk];\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[0][t_gjk] = sj[t_gjk];

#define select_1jk()  s->nvrtx = 3;\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[2][t_gjk] = s->vrtx[3][t_gjk];\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[1][t_gjk] = sj[t_gjk];\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[0][t_gjk] = sk[t_gjk];

#define select_1i()   s->nvrtx = 2;\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[1][t_gjk] = s->vrtx[3][t_gjk];\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[0][t_gjk] = si[t_gjk];

#define select_1j()   s->nvrtx = 2;\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[1][t_gjk] = s->vrtx[3][t_gjk];\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[0][t_gjk] = sj[t_gjk];

#define select_1k()   s->nvrtx = 2;\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[1][t_gjk] = s->vrtx[3][t_gjk];\
		for (t_gjk= 0; t_gjk< 3; t_gjk++)\
		s->vrtx[0][t_gjk] = sk[t_gjk];

#define S1Dregion1()                    v[0] = s->vrtx[1][0];\
		v[1] = s->vrtx[1][1];\
		v[2] = s->vrtx[1][2];\
		s->nvrtx = 1;\
		s->vrtx[0][0] = s->vrtx[1][0];\
		s->vrtx[0][1] = s->vrtx[1][1];\
		s->vrtx[0][2] = s->vrtx[1][2];

#define S2Dregion1()                    v[0] = s->vrtx[2][0];\
		v[1] = s->vrtx[2][1];\
		v[2] = s->vrtx[2][2];\
		s->nvrtx = 1;\
		s->vrtx[0][0] = s->vrtx[2][0];\
		s->vrtx[0][1] = s->vrtx[2][1];\
		s->vrtx[0][2] = s->vrtx[2][2];

#define S2Dregion12()                   s->nvrtx = 2;\
		s->vrtx[0][0] = s->vrtx[2][0];\
		s->vrtx[0][1] = s->vrtx[2][1];\
		s->vrtx[0][2] = s->vrtx[2][2];

#define S2Dregion13()                   s->nvrtx = 2;\
		s->vrtx[1][0] = s->vrtx[2][0];\
		s->vrtx[1][1] = s->vrtx[2][1];\
		s->vrtx[1][2] = s->vrtx[2][2];

#define S3Dregion1()                    v[0] = s1[0];\
		v[1] = s1[1];\
		v[2] = s1[2];\
		s->nvrtx = 1;\
		s->vrtx[0][0] = s1[0];\
		s->vrtx[0][1] = s1[1];\
		s->vrtx[0][2] = s1[2];

#define S3Dregion1234()                 v[0] = 0;\
		v[1] = 0;\
		v[2] = 0;\
		s->nvrtx = 4;

#define calculateEdgeVector(p1p2, p2)   p1p2[0] = p2[0] - s->vrtx[3][0];\
		p1p2[1] = p2[1] - s->vrtx[3][1];\
		p1p2[2] = p2[2] - s->vrtx[3][2];


CollisionDetection::CollisionDetection() {
	// TODO Auto-generated constructor stub
	bInit=false;
	init();

}

CollisionDetection::~CollisionDetection() {
}

void CollisionDetection::CollisionDetection::init(){
	if(!bInit){
	camVectorsStatic = new float[12]{ 2.57f, 0.0f, 6.1375f - 3.5f, cosf(nickAngleMT / 180.0f * M_PI), 0.0f, sinf(nickAngleMT / 180.0f * M_PI), -sinf(nickAngleMT / 180.0f * M_PI), 0.0f, cosf(nickAngleMT / 180.0f * M_PI), 0.0f, 1.0f, 0.0f };

	workpieceArrowPositionStatic = new float[9]{ 1.0f, 0.0f, 1.502f, 1.0f - 0.43333f, -0.25f, 1.502f, 1.0f - 0.43333f, 0.25f, 1.502f };
	workpieceArrowPosition1 = new float[9];
	workpieceArrowPosition2 = new float[9];
	workpieceArrowPosition3 = new float[9];
	workpieceArrowPosition4 = new float[9];

	col1_s = new double[3];
	col2_s = new double[3];
	v_direction = new double[3];

	gjk_simplex = new simplex;
	bInit = true;
	}
}

CollisionDetection* CollisionDetection::cd = 0;

CollisionDetection* CollisionDetection::get_instance(){
	static bool isInit=false;
//	CollisionDetection * temp;
	if(!isInit){
		cd  = new CollisionDetection();
//		cd = temp;
		isInit=true;
	}

	return cd; //CollisionDetection::cd;
}












void CollisionDetection::crossProduct(const double* a, const double* b, double* c)
{
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}

double CollisionDetection::determinant(const double* p, const double* q, const double* r) {
	return p[0] * ((q[1] * r[2]) - (r[1] * q[2])) - p[1] * (q[0] * r[2] - r[0] * q[2]) + p[2] * (q[0] * r[1] - r[0] * q[1]);
}

void CollisionDetection::projectOnLine(const double* p, const double* q, double* v)
{
	pq[0] = p[0] - q[0];
	pq[1] = p[1] - q[1];
	pq[2] = p[2] - q[2];

	if (dotProduct(pq, pq) < epsilon4) {
		cout << "Warning: low dot product value!" << endl;
	}

	tmp = dotProduct(p, pq) / dotProduct(pq, pq);

	for (int i = 0; i < 3; i++)
		v[i] = p[i] - pq[i] * tmp;
}

void CollisionDetection::projectOnPlane(const double* p, const double* q, const double* r, double* v)
{
	for (int i = 0; i < 3; i++)
		pq[i] = p[i] - q[i];

	for (int i = 0; i < 3; i++)
		pr[i] = p[i] - r[i];

	crossProduct(pq, pr, n);

	if (dotProduct(n, n) < epsilon4) {
		cout << "Warning: low dot product value!" << endl;
	}

	tmp = dotProduct(n, p) / dotProduct(n, n);

	for (int i = 0; i < 3; i++)
		v[i] = n[i] * tmp;

}

int CollisionDetection::hff1(const double* p, const double* q)
{
	tmp = 0;
	for (int i = 0; i < 3; i++)
		tmp += (p[i] * p[i] - p[i] * q[i]);

	if (tmp > 0)
		return 1;

	return 0;
}

int CollisionDetection::hff2(const double* p, const double* q, const double* r)
{
	tmp = 0;

	for (int i = 0; i < 3; i++)
		pq[i] = q[i] - p[i];

	for (int i = 0; i < 3; i++)
		pr[i] = r[i] - p[i];

	crossProduct(pq, pr, ntmp);
	crossProduct(pq, ntmp, n);

	for (int i = 0; i < 3; i++)
		tmp = tmp + (p[i] * n[i]);

	if (tmp < 0)
		return 1;

	return 0;
}

int CollisionDetection::hff3(const double* p, const double* q, const double* r)
{
	tmp = 0;
	for (int i = 0; i < 3; i++)
		pq[i] = q[i] - p[i];

	for (int i = 0; i < 3; i++)
		pr[i] = r[i] - p[i];

	crossProduct(pq, pr, n);

	for (int i = 0; i < 3; i++)
		tmp = tmp + (p[i] * n[i]);

	if (tmp > 0)
		return 0;

	return 1;
}

void CollisionDetection::support(const collider* col, const int bd_index, const double* v) {
	better = -1;

	if (bd_index == 1) {
		maxs = dotProduct(col1_s, v);

		for (int i = 0; i < col->points_stat; i++) {
			if (col->ignoredPoints[i] == false) {
				vrt[0] = col->x_coll[i];
				vrt[1] = col->y_coll[i];
				vrt[2] = col->z_coll[i];
				s = dotProduct(vrt, v);
				if (s > maxs) {
					maxs = s;
					better = i;
				}
			}
		}

		if (better != -1) {
			col1_s[0] = col->x_coll[better];
			col1_s[1] = col->y_coll[better];
			col1_s[2] = col->z_coll[better];
		}
	}
	else {
		maxs = dotProduct(col2_s, v);

		for (int i = 0; i < col->points_stat; i++) {
			if (col->ignoredPoints[i] == false) {
				vrt[0] = col->x_coll[i];
				vrt[1] = col->y_coll[i];
				vrt[2] = col->z_coll[i];
				s = dotProduct(vrt, v);
				if (s > maxs) {
					maxs = s;
					better = i;
				}
			}
		}

		if (better != -1) {
			col2_s[0] = col->x_coll[better];
			col2_s[1] = col->y_coll[better];
			col2_s[2] = col->z_coll[better];
		}
	}
}

void CollisionDetection::S1D(struct simplex* s, double* v)
{
	if (hff1(s->vrtx[1], s->vrtx[0])) {
		projectOnLine(s->vrtx[1], s->vrtx[0], v);
		return;
	}
	else {
		S1Dregion1();
		return;
	}
}

void CollisionDetection::S2D(struct simplex* s, double* v)
{
	hff1f_s12 = hff1(s->vrtx[2], s->vrtx[1]);
	hff1f_s13 = hff1(s->vrtx[2], s->vrtx[0]);
	hff2f_23 = !hff2(s->vrtx[2], s->vrtx[1], s->vrtx[0]);
	hff2f_32 = !hff2(s->vrtx[2], s->vrtx[0], s->vrtx[1]);

	if (hff1f_s12 == 1) {
		if (hff2f_23 == 1) {
			if (hff1f_s13 == 1) {
				if (hff2f_32 == 1) {
					projectOnPlane(s->vrtx[2], s->vrtx[1], s->vrtx[0], v);
					return;
				}
				else
				{
					projectOnLine(s->vrtx[2], s->vrtx[0], v);
					S2Dregion13();
					return;
				}
			}
			else
			{
				projectOnPlane(s->vrtx[2], s->vrtx[1], s->vrtx[0], v);
				return;
			}
		}
		else
		{
			projectOnLine(s->vrtx[2], s->vrtx[1], v);
			S2Dregion12();
			return;
		}
	}
	else if (hff1f_s13 == 1) {
		if (hff2f_32 == 1) {
			projectOnPlane(s->vrtx[2], s->vrtx[1], s->vrtx[0], v);
			return;
		}
		else
		{
			projectOnLine(s->vrtx[2], s->vrtx[0], v);
			S2Dregion13();
			return;
		}
	}
	else {
		S2Dregion1();
		return;
	}

}

void CollisionDetection::S3D(struct simplex* s, double* v) {
	getvrtx(s1, 3);
	getvrtx(s2, 2);
	getvrtx(s3, 1);
	getvrtx(s4, 0);
	calculateEdgeVector(s1s2, s2);
	calculateEdgeVector(s1s3, s3);
	calculateEdgeVector(s1s4, s4);

	hff1_tests[2] = hff1(s1, s2);
	hff1_tests[1] = hff1(s1, s3);
	hff1_tests[0] = hff1(s1, s4);
	testLineThree = hff1(s1, s3);
	testLineFour = hff1(s1, s4);

	dotTotal = hff1(s1, s2) + testLineThree + testLineFour;
	if (dotTotal == 0) {
		S3Dregion1();
		return;
	}

	det134 = determinant(s1s3, s1s4, s1s2);

	if (det134 > 0) {
		sss_gjk = 0;
	}
	else {
		sss_gjk = 1;
	}

	testPlaneTwo = hff3(s1, s3, s4) - sss_gjk;
	testPlaneTwo = testPlaneTwo * testPlaneTwo;
	testPlaneThree = hff3(s1, s4, s2) - sss_gjk;
	testPlaneThree = testPlaneThree * testPlaneThree;
	testPlaneFour = hff3(s1, s2, s3) - sss_gjk;
	testPlaneFour = testPlaneFour * testPlaneFour;

	switch (testPlaneTwo + testPlaneThree + testPlaneFour) {
	case 3:
		S3Dregion1234();
		break;

	case 2:
		s->nvrtx = 3;
		if (!testPlaneTwo) {
			for (int i = 0; i < 3; i++)
				s->vrtx[2][i] = s->vrtx[3][i];
		}
		else if (!testPlaneThree) {
			for (i_gjk = 0; i_gjk < 3; i_gjk++)
				s->vrtx[1][i_gjk] = s2[i_gjk];
			for (i_gjk = 0; i_gjk < 3; i_gjk++)
				s->vrtx[2][i_gjk] = s->vrtx[3][i_gjk];
		}
		else if (!testPlaneFour) {
			for (i_gjk = 0; i_gjk < 3; i_gjk++)
				s->vrtx[0][i_gjk] = s3[i_gjk];
			for (i_gjk = 0; i_gjk < 3; i_gjk++)
				s->vrtx[1][i_gjk] = s2[i_gjk];
			for (i_gjk = 0; i_gjk < 3; i_gjk++)
				s->vrtx[2][i_gjk] = s->vrtx[3][i_gjk];
		}
		S2D(s, v);
		break;
	case 1:
		s->nvrtx = 3;
		if (testPlaneTwo) {
			k_gjk = 2;
			i_gjk = 1;
			j_gjk = 0;
		}
		else if (testPlaneThree) {
			k_gjk = 1;
			i_gjk = 0;
			j_gjk = 2;
		}
		else {
			k_gjk = 0;
			i_gjk = 2;
			j_gjk = 1;
		}

		getvrtx(si, i_gjk);
		getvrtx(sj, j_gjk);
		getvrtx(sk, k_gjk);

		if (dotTotal == 1) {
			if (hff1_tests[k_gjk]) {
				if (!hff2(s1, sk, si)) {
					select_1ik();
					projectOnPlane(s1, si, sk, v);
				}
				else if (!hff2(s1, sk, sj)) {
					select_1jk();
					projectOnPlane(s1, sj, sk, v);
				}
				else {
					select_1k();
					projectOnLine(s1, sk, v);
				}
			}
			else if (hff1_tests[i_gjk]) {
				if (!hff2(s1, si, sk)) {
					select_1ik();
					projectOnPlane(s1, si, sk, v);
				}
				else {
					select_1i();
					projectOnLine(s1, si, v);
				}
			}
			else {
				if (!hff2(s1, sj, sk)) {
					select_1jk();
					projectOnPlane(s1, sj, sk, v);
				}
				else {
					select_1j();
					projectOnLine(s1, sj, v);
				}
			}
		}
		else if (dotTotal == 2) {
			if (hff1_tests[i_gjk]) {
				if (!hff2(s1, sk, si))
					if (!hff2(s1, si, sk)) {
						select_1ik();
						projectOnPlane(s1, si, sk, v);
					}
					else {
						select_1k();
						projectOnLine(s1, sk, v);
					}
				else {
					if (!hff2(s1, sk, sj)) {
						select_1jk();
						projectOnPlane(s1, sj, sk, v);
					}
					else {
						select_1k();
						projectOnLine(s1, sk, v);
					}
				}
			}
			else if (hff1_tests[j_gjk]) {
				if (!hff2(s1, sk, sj))
					if (!hff2(s1, sj, sk)) {
						select_1jk();
						projectOnPlane(s1, sj, sk, v);
					}
					else {
						select_1j();
						projectOnLine(s1, sj, v);
					}
				else {
					if (!hff2(s1, sk, si)) {
						select_1ik();
						projectOnPlane(s1, si, sk, v);
					}
					else {
						select_1k();
						projectOnLine(s1, sk, v);
					}
				}
			}
		}
		else if (dotTotal == 3) {
			hff2_ik = hff2(s1, si, sk);
			hff2_jk = hff2(s1, sj, sk);
			hff2_ki = hff2(s1, sk, si);
			hff2_kj = hff2(s1, sk, sj);

			if (hff2_ki == 0 && hff2_kj == 0) {
				std::cout << "Error: unexpected values!" << endl;
			}
			if (hff2_ki == 1 && hff2_kj == 1) {
				select_1k();
				projectOnLine(s1, sk, v);
			}
			else if (hff2_ki) {
				if (hff2_jk) {
					select_1j();
					projectOnLine(s1, sj, v);
				}
				else {
					select_1jk();
					projectOnPlane(s1, sk, sj, v);
				}
			}
			else {
				if (hff2_ik) {
					select_1i();
					projectOnLine(s1, si, v);
				}
				else {
					select_1ik();
					projectOnPlane(s1, sk, si, v);
				}
			}
		}
		break;

	case 0:
		if (dotTotal == 1) {
			if (testLineThree) {
				k_gjk = 2;
				i_gjk = 1;
				j_gjk = 0;
			}
			else if (testLineFour) {
				k_gjk = 1;
				i_gjk = 0;
				j_gjk = 2;
			}
			else {
				k_gjk = 0;
				i_gjk = 2;
				j_gjk = 1;
			}
			getvrtx(si, i_gjk);
			getvrtx(sj, j_gjk);
			getvrtx(sk, k_gjk);

			if (!hff2(s1, si, sj)) {
				select_1ij();
				projectOnPlane(s1, si, sj, v);
			}
			else if (!hff2(s1, si, sk)) {
				select_1ik();
				projectOnPlane(s1, si, sk, v);
			}
			else {
				select_1i();
				projectOnLine(s1, si, v);
			}
		}
		else if (dotTotal == 2) {
			s->nvrtx = 3;
			if (!testLineThree) {
				k_gjk = 2;
				i_gjk = 1;
				j_gjk = 0;
			}
			else if (!testLineFour) {
				k_gjk = 1;
				i_gjk = 0;
				j_gjk = 2;
			}
			else {
				k_gjk = 0;
				i_gjk = 2;
				j_gjk = 1;
			}
			getvrtx(si, i_gjk);
			getvrtx(sj, j_gjk);
			getvrtx(sk, k_gjk);

			if (!hff2(s1, sj, sk)) {
				if (!hff2(s1, sk, sj)) {
					select_1jk();
					projectOnPlane(s1, sj, sk, v);
				}
				else if (!hff2(s1, sk, si)) {
					select_1ik();
					projectOnPlane(s1, sk, si, v);
				}
				else {
					select_1k();
					projectOnLine(s1, sk, v);
				}
			}
			else if (!hff2(s1, sj, si)) {
				select_1ij();
				projectOnPlane(s1, si, sj, v);
			}
			else {
				select_1j();
				projectOnLine(s1, sj, v);
			}
		}
		break;
	default:
		std::cout << "Error: unhandled!" << endl;
	}

}

void CollisionDetection::subalgorithm(struct simplex* s, double* v) {
	switch (s->nvrtx) {
	case 4:
		S3D(s, v);
		break;
	case 3:
		S2D(s, v);
		break;
	case 2:
		S1D(s, v);
		break;
	default:
		std::cout << "Error: invalid simplex!" << std::endl;
	}
}



/*
The function initMaxSize() is one of the init. functions, that has to be called for every collider at the beginning.
For a transmitted collider col, the arihmetic distance of every point of the point cloud to the origin is calculated,
the max. distance is saved in the struct patameter maxSize.
 */
 void CollisionDetection::initMaxSize(collider* col) {
	 maxSizeTemp = 0;
	 for (int i = 0; i < col->points_stat; i++) {
		 xVal_cd = col->x_stat[i];
		 yVal_cd = col->y_stat[i];
		 zVal_cd = col->z_stat[i];

		 if (sqrt(xVal_cd * xVal_cd + yVal_cd * yVal_cd + zVal_cd * zVal_cd) > maxSizeTemp) {
			 maxSizeTemp = sqrt(xVal_cd * xVal_cd + yVal_cd * yVal_cd + zVal_cd * zVal_cd);
		 }
	 }
	 col->maxSize = maxSizeTemp;
 }

 // updateMatrices() fills those three matrices with the fitting values to perform horizontal, nick and roll rotations:
 void CollisionDetection::updateMatrices(float rotHor, float rotNick, float rotRoll) {
	 firstMatrixValuesX[0] = cos(rotHor);
	 firstMatrixValuesX[1] = -sin(rotHor);
	 firstMatrixValuesX[2] = 0.0f;
	 firstMatrixValuesY[0] = sin(rotHor);
	 firstMatrixValuesY[1] = cos(rotHor);
	 firstMatrixValuesY[2] = 0.0f;
	 firstMatrixValuesZ[0] = 0.0f;
	 firstMatrixValuesZ[1] = 0.0f;
	 firstMatrixValuesZ[2] = 1.0f;

	 secondMatrixValuesX[0] = cos(rotNick) + sin(rotHor) * sin(rotHor) * (1 - cos(rotNick));
	 secondMatrixValuesX[1] = sin(rotHor) * -cos(rotHor) * (1 - cos(rotNick));
	 secondMatrixValuesX[2] = -cos(rotHor) * sin(rotNick);
	 secondMatrixValuesY[0] = sin(rotHor) * -cos(rotHor) * (1 - cos(rotNick));
	 secondMatrixValuesY[1] = cos(rotNick) + cos(rotHor) * cos(rotHor) * (1 - cos(rotNick));
	 secondMatrixValuesY[2] = -sin(rotHor) * sin(rotNick);
	 secondMatrixValuesZ[0] = cos(rotHor) * sin(rotNick);
	 secondMatrixValuesZ[1] = sin(rotHor) * sin(rotNick);
	 secondMatrixValuesZ[2] = cos(rotNick);

	 thirdMatrixValuesX[0] = cos(rotRoll) + cos(rotHor) * cos(rotHor) * cos(rotNick) * cos(rotNick) * (1 - cos(rotRoll));
	 thirdMatrixValuesX[1] = cos(rotHor) * cos(rotNick) * sin(rotHor) * cos(rotNick) * (1 - cos(rotRoll)) + sin(rotNick) * sin(rotRoll);
	 thirdMatrixValuesX[2] = cos(rotHor) * cos(rotNick) * sin(rotNick) * (1 - cos(rotRoll)) - sin(rotHor) * cos(rotNick) * sin(rotRoll);
	 thirdMatrixValuesY[0] = cos(rotHor) * cos(rotNick) * sin(rotHor) * cos(rotNick) * (1 - cos(rotRoll)) - sin(rotNick) * sin(rotRoll);
	 thirdMatrixValuesY[1] = cos(rotRoll) + sin(rotHor) * cos(rotNick) * sin(rotHor) * cos(rotNick) * (1 - cos(rotRoll));
	 thirdMatrixValuesY[2] = sin(rotHor) * cos(rotNick) * sin(rotNick) * (1 - cos(rotRoll)) + cos(rotHor) * cos(rotNick) * sin(rotRoll);
	 thirdMatrixValuesZ[0] = cos(rotHor) * cos(rotNick) * sin(rotNick) * (1 - cos(rotRoll)) + sin(rotHor) * cos(rotNick) * sin(rotRoll);
	 thirdMatrixValuesZ[1] = sin(rotNick) * sin(rotHor) * cos(rotNick) * (1 - cos(rotRoll)) - cos(rotHor) * cos(rotNick) * sin(rotRoll);
	 thirdMatrixValuesZ[2] = cos(rotRoll) + sin(rotNick) * sin(rotNick) * (1 - cos(rotRoll));
 }

 // updateMatricesInverse() can be seen as the inverse operation, i.e. established rotations can be undone.
 // The inverse operation may become necessary if, for example, points of the minimum distance between colliders
 // are to be retraced:
 void CollisionDetection::updateMatricesInverse(float rotHor, float rotNick, float rotRoll) {
	 firstMatrixValuesX[0] = cos(-rotHor);
	 firstMatrixValuesX[1] = -sin(-rotHor);
	 firstMatrixValuesX[2] = 0.0f;
	 firstMatrixValuesY[0] = sin(-rotHor);
	 firstMatrixValuesY[1] = cos(-rotHor);
	 firstMatrixValuesY[2] = 0.0f;
	 firstMatrixValuesZ[0] = 0.0f;
	 firstMatrixValuesZ[1] = 0.0f;
	 firstMatrixValuesZ[2] = 1.0f;

	 secondMatrixValuesX[0] = cos(-rotNick) + sin(rotHor) * sin(rotHor) * (1 - cos(-rotNick));
	 secondMatrixValuesX[1] = sin(rotHor) * -cos(rotHor) * (1 - cos(-rotNick));
	 secondMatrixValuesX[2] = -cos(rotHor) * sin(-rotNick);
	 secondMatrixValuesY[0] = sin(rotHor) * -cos(rotHor) * (1 - cos(-rotNick));
	 secondMatrixValuesY[1] = cos(-rotNick) + cos(rotHor) * cos(rotHor) * (1 - cos(-rotNick));
	 secondMatrixValuesY[2] = -sin(rotHor) * sin(-rotNick);
	 secondMatrixValuesZ[0] = cos(rotHor) * sin(-rotNick);
	 secondMatrixValuesZ[1] = sin(rotHor) * sin(-rotNick);
	 secondMatrixValuesZ[2] = cos(-rotNick);

	 thirdMatrixValuesX[0] = cos(-rotRoll) + cos(rotHor) * cos(rotHor) * cos(rotNick) * cos(rotNick) * (1 - cos(-rotRoll));
	 thirdMatrixValuesX[1] = cos(rotHor) * cos(rotNick) * sin(rotHor) * cos(rotNick) * (1 - cos(-rotRoll)) + sin(rotNick) * sin(-rotRoll);
	 thirdMatrixValuesX[2] = cos(rotHor) * cos(rotNick) * sin(rotNick) * (1 - cos(-rotRoll)) - sin(rotHor) * cos(rotNick) * sin(-rotRoll);
	 thirdMatrixValuesY[0] = cos(rotHor) * cos(rotNick) * sin(rotHor) * cos(rotNick) * (1 - cos(-rotRoll)) - sin(rotNick) * sin(-rotRoll);
	 thirdMatrixValuesY[1] = cos(-rotRoll) + sin(rotHor) * cos(rotNick) * sin(rotHor) * cos(rotNick) * (1 - cos(-rotRoll));
	 thirdMatrixValuesY[2] = sin(rotHor) * cos(rotNick) * sin(rotNick) * (1 - cos(-rotRoll)) + cos(rotHor) * cos(rotNick) * sin(-rotRoll);
	 thirdMatrixValuesZ[0] = cos(rotHor) * cos(rotNick) * sin(rotNick) * (1 - cos(-rotRoll)) + sin(rotHor) * cos(rotNick) * sin(-rotRoll);
	 thirdMatrixValuesZ[1] = sin(rotNick) * sin(rotHor) * cos(rotNick) * (1 - cos(-rotRoll)) - cos(rotHor) * cos(rotNick) * sin(-rotRoll);
	 thirdMatrixValuesZ[2] = cos(-rotRoll) + sin(rotNick) * sin(rotNick) * (1 - cos(-rotRoll));
 }

 // updateMatricesTransport() is a special version, required only for objects that are currently transported with the gripper.
 // For corretly calculating the rotation of those objects caused by the rotation of the last arm joint, rotRoll specifies the
 // rotation around the axis (axis_x, axis_y, axis_z):
 void CollisionDetection::updateMatricesTransport(float rotHor, float rotNick, float rotRoll, float axis_x, float axis_y, float axis_z) {
	 firstMatrixValuesX[0] = cos(rotHor);
	 firstMatrixValuesX[1] = -sin(rotHor);
	 firstMatrixValuesX[2] = 0.0f;
	 firstMatrixValuesY[0] = sin(rotHor);
	 firstMatrixValuesY[1] = cos(rotHor);
	 firstMatrixValuesY[2] = 0.0f;
	 firstMatrixValuesZ[0] = 0.0f;
	 firstMatrixValuesZ[1] = 0.0f;
	 firstMatrixValuesZ[2] = 1.0f;

	 secondMatrixValuesX[0] = cos(rotNick) + sin(rotHor) * sin(rotHor) * (1 - cos(rotNick));
	 secondMatrixValuesX[1] = sin(rotHor) * -cos(rotHor) * (1 - cos(rotNick));
	 secondMatrixValuesX[2] = -cos(rotHor) * sin(rotNick);
	 secondMatrixValuesY[0] = sin(rotHor) * -cos(rotHor) * (1 - cos(rotNick));
	 secondMatrixValuesY[1] = cos(rotNick) + cos(rotHor) * cos(rotHor) * (1 - cos(rotNick));
	 secondMatrixValuesY[2] = -sin(rotHor) * sin(rotNick);
	 secondMatrixValuesZ[0] = cos(rotHor) * sin(rotNick);
	 secondMatrixValuesZ[1] = sin(rotHor) * sin(rotNick);
	 secondMatrixValuesZ[2] = cos(rotNick);

	 thirdMatrixValuesX[0] = cos(rotRoll) + axis_x * axis_x * (1 - cos(rotRoll));
	 thirdMatrixValuesX[1] = axis_x * axis_y * (1 - cos(rotRoll)) + axis_z * sin(rotRoll);
	 thirdMatrixValuesX[2] = axis_x * axis_z * (1 - cos(rotRoll)) - axis_y * sin(rotRoll);
	 thirdMatrixValuesY[0] = axis_x * axis_y * (1 - cos(rotRoll)) - axis_z * sin(rotRoll);
	 thirdMatrixValuesY[1] = cos(rotRoll) + axis_y * axis_y * (1 - cos(rotRoll));
	 thirdMatrixValuesY[2] = axis_y * axis_z * (1 - cos(rotRoll)) + axis_x * sin(rotRoll);
	 thirdMatrixValuesZ[0] = axis_x * axis_z * (1 - cos(rotRoll)) + axis_y * sin(rotRoll);
	 thirdMatrixValuesZ[1] = axis_y * axis_z * (1 - cos(rotRoll)) - axis_x * sin(rotRoll);
	 thirdMatrixValuesZ[2] = cos(rotRoll) + axis_z * axis_z * (1 - cos(rotRoll));
 }

 // the function rotationMatrix() processes the matrix-vector calculation for a point (inX, inY, inZ) and
 // the rotation matrix [index] and stores the output in (xVal_cd, yVal_cd, zVal_cd):
 void CollisionDetection::rotationMatrix(float inX, float inY, float inZ, int index) {
	 if (index == 0) { // index = 0: firstMatrix
		 xVal_cd = firstMatrixValuesX[0] * inX + firstMatrixValuesX[1] * inY + firstMatrixValuesX[2] * inZ;
		 yVal_cd = firstMatrixValuesY[0] * inX + firstMatrixValuesY[1] * inY + firstMatrixValuesY[2] * inZ;
		 zVal_cd = firstMatrixValuesZ[0] * inX + firstMatrixValuesZ[1] * inY + firstMatrixValuesZ[2] * inZ;
	 }
	 else if (index == 1) { // index = 1: secondMatrix
		 xVal_cd = secondMatrixValuesX[0] * inX + secondMatrixValuesX[1] * inY + secondMatrixValuesX[2] * inZ;
		 yVal_cd = secondMatrixValuesY[0] * inX + secondMatrixValuesY[1] * inY + secondMatrixValuesY[2] * inZ;
		 zVal_cd = secondMatrixValuesZ[0] * inX + secondMatrixValuesZ[1] * inY + secondMatrixValuesZ[2] * inZ;
	 }
	 else { // index = 2: thirdMatrix
		 xVal_cd = thirdMatrixValuesX[0] * inX + thirdMatrixValuesX[1] * inY + thirdMatrixValuesX[2] * inZ;
		 yVal_cd = thirdMatrixValuesY[0] * inX + thirdMatrixValuesY[1] * inY + thirdMatrixValuesY[2] * inZ;
		 zVal_cd = thirdMatrixValuesZ[0] * inX + thirdMatrixValuesZ[1] * inY + thirdMatrixValuesZ[2] * inZ;
	 }
 }

 // redoRotation() rotates and moves a point (x_in, y_in, z_in) by sequential multiplication with the three rotation
 // matrices and summation of the offset stored in collider col. Write the output into (xVal_cd, ...):
 void CollisionDetection::redoRotation(collider* col, float x_in, float y_in, float z_in) {
	 xVal2_cd = x_in; yVal2_cd = y_in; zVal2_cd = z_in;
	 rotationMatrix(xVal2_cd, yVal2_cd, zVal2_cd, 0);
	 xVal2_cd = xVal_cd; yVal2_cd = yVal_cd; zVal2_cd = zVal_cd; // use the multiplication output as input for the next matrix multiplication
	 rotationMatrix(xVal2_cd, yVal2_cd, zVal2_cd, 1);
	 xVal2_cd = xVal_cd; yVal2_cd = yVal_cd; zVal2_cd = zVal_cd;
	 rotationMatrix(xVal2_cd, yVal2_cd, zVal2_cd, 2);
	 xVal_cd = xVal_cd + col->offsets[0]; // add the offset values
	 yVal_cd = yVal_cd + col->offsets[1];
	 zVal_cd = zVal_cd + col->offsets[2];
 }

 // undoRotation() is the inverse process to redoRotation() and traces the output of the previous method back to the input:
 void CollisionDetection::undoRotation(collider* col, float x_in, float y_in, float z_in) {
	 xVal2_cd = x_in; yVal2_cd = y_in; zVal2_cd = z_in;
	 updateMatricesInverse(col->angles[0], col->angles[1], col->angles[2]);
	 xVal2_cd = xVal2_cd - col->offsets[0];
	 yVal2_cd = yVal2_cd - col->offsets[1];
	 zVal2_cd = zVal2_cd - col->offsets[2];

	 rotationMatrix(xVal2_cd, yVal2_cd, zVal2_cd, 2);
	 xVal2_cd = xVal_cd; yVal2_cd = yVal_cd; zVal2_cd = zVal_cd;
	 rotationMatrix(xVal2_cd, yVal2_cd, zVal2_cd, 1);
	 xVal2_cd = xVal_cd; yVal2_cd = yVal_cd; zVal2_cd = zVal_cd;
	 rotationMatrix(xVal2_cd, yVal2_cd, zVal2_cd, 0);
 }

 // overrideCollider() writes the geometric structure and all associated data of a source collider (source) into a target collider (dest):
 void CollisionDetection::overrideCollider(collider* dest, collider* source) {
	 dest->points_stat = source->points_stat;
	 dest->facesTimes3 = source->facesTimes3;
	 dest->maxSize = source->maxSize;
	 dest->firstSurfacePoint = source->firstSurfacePoint;

	 for (int i = 0; i < source->points_stat; i++) {
		 dest->x_stat[i] = source->x_stat[i];
		 dest->y_stat[i] = source->y_stat[i];
		 dest->z_stat[i] = source->z_stat[i];

		 dest->ignoredPoints[i] = source->ignoredPoints[i];
	 }

	 for (int i = 0; i < source->facesTimes3; i++) {
		 dest->connections[i] = source->connections[i];
	 }
 }

 // updateColliderSize() changes the size of a cuboid collider col. x_size, y_size and z_size define the half edge lengths in the direction of the respective axes:
 void CollisionDetection::updateColliderSize(collider* col, float x_size, float y_size, float z_size) {
	 col->x_stat[1] = x_size; col->x_stat[2] = x_size; col->x_stat[3] = -x_size; col->x_stat[4] = -x_size; col->x_stat[5] = x_size; col->x_stat[6] = x_size; col->x_stat[7] = -x_size; col->x_stat[8] = -x_size;
	 col->y_stat[1] = y_size; col->y_stat[2] = -y_size; col->y_stat[3] = y_size; col->y_stat[4] = -y_size; col->y_stat[5] = y_size; col->y_stat[6] = -y_size; col->y_stat[7] = y_size; col->y_stat[8] = -y_size;
	 col->z_stat[1] = -z_size; col->z_stat[2] = -z_size; col->z_stat[3] = -z_size; col->z_stat[4] = -z_size; col->z_stat[5] = z_size; col->z_stat[6] = z_size; col->z_stat[7] = z_size; col->z_stat[8] = z_size;
	 initMaxSize(col);
 }

 /*
recalculateCollider() is the actual translation and rotation operation for colliders. Besides a target
position vector (offX, offY, offZ) a target rotation vector (rotHor, rotNick, rotRoll) is passed as
input parameter. However, the recalculation of a point cloud alone is not sufficient. For objects with
localization patterns, their new spatial position and orientation must also be calculated equivalently
(recalcPattern = true). The same applies to the camera view direction of a Monitoring Tool (recalcMTCameraPosition = true) and the
orientation arrow on the top of workpieces displayed during initialization mode (recalcOrientationArrow = true).
  */
 void CollisionDetection::recalculateCollider(collider* col, float offX, float offY, float offZ, float rotHor, float rotNick, float rotRoll, bool recalcPattern, bool recalcMTCameraPos, float* camVectors, bool recalcOrientationArrow, float* arrowPosition) {
	 updateMatrices(rotHor, rotNick, rotRoll);

	 // set the init. values for the limits array:
	 for (int i = 0; i < 6; i++) {
		 if (i % 2 == 0) {
			 col->limits[i] = -1000.0f;
		 }
		 else {
			 col->limits[i] = 1000.0f;
		 }
	 }

	 // store the new position and rotation in the collider col:
	 col->offsets[0] = offX;
	 col->offsets[1] = offY;
	 col->offsets[2] = offZ;

	 col->angles[0] = rotHor;
	 col->angles[1] = rotNick;
	 col->angles[2] = rotRoll;

	 // make sure that the horizontal rotation is always in the range [0, 2*pi):
	 while (col->angles[0] >= M_PI * 2.0f) {
		 col->angles[0] -= M_PI * 2.0f;
	 }
	 while (col->angles[0] < 0.0f) {
		 col->angles[0] += M_PI * 2.0f;
	 }

	 // go through every point in the point cloud that lies on the surface and recalculate its new position with the rotation matrices.
	 // Meanwhile also calculate the lower and upper limits for each of the three coordinate system axes:
	 for (int i = 0; i < col->points_stat; i++) {
		 if (col->ignoredPoints[i] == false) {
			 rotationMatrix(col->x_stat[i], col->y_stat[i], col->z_stat[i], 0);
			 col->x_coll[i] = xVal_cd; col->y_coll[i] = yVal_cd; col->z_coll[i] = zVal_cd;

			 rotationMatrix(col->x_coll[i], col->y_coll[i], col->z_coll[i], 1);
			 col->x_coll[i] = xVal_cd; col->y_coll[i] = yVal_cd; col->z_coll[i] = zVal_cd;

			 rotationMatrix(col->x_coll[i], col->y_coll[i], col->z_coll[i], 2);
			 xVal_cd += offX;
			 yVal_cd += offY;
			 zVal_cd += offZ;
			 // store the new point positions in (x_coll, y_coll, z_coll):
            		col->x_coll[i] = xVal_cd; col->y_coll[i] = yVal_cd; col->z_coll[i] = zVal_cd;

            		if (xVal_cd > col->limits[0]) {
            			col->limits[0] = xVal_cd;
            		}

            		if (xVal_cd < col->limits[1]) {
            			col->limits[1] = xVal_cd;
            		}

            		if (yVal_cd > col->limits[2]) {
            			col->limits[2] = yVal_cd;
            		}

            		if (yVal_cd < col->limits[3]) {
            			col->limits[3] = yVal_cd;
            		}

            		if (zVal_cd > col->limits[4]) {
            			col->limits[4] = zVal_cd;
            		}

            		if (zVal_cd < col->limits[5]) {
            			col->limits[5] = zVal_cd;
            		}
		 }
	 }

	 if (recalcPattern && recalcMTCameraPos) cout << "Warning: mode error in collider update!" << endl;
	 if (recalcMTCameraPos && recalcOrientationArrow) cout << "Warning: mode error in collider update!" << endl;

	 if (recalcPattern) {
		 // if the collider has patterns, also recalculate their positions:
		 for (int i = 0; i < col->patternCount * 4; i++) {
			 rotationMatrix(col->x_pattern_stat[i], col->y_pattern_stat[i], col->z_pattern_stat[i], 0);
			 col->x_pattern_tf[i] = xVal_cd; col->y_pattern_tf[i] = yVal_cd; col->z_pattern_tf[i] = zVal_cd;

			 rotationMatrix(col->x_pattern_tf[i], col->y_pattern_tf[i], col->z_pattern_tf[i], 1);
			 col->x_pattern_tf[i] = xVal_cd; col->y_pattern_tf[i] = yVal_cd; col->z_pattern_tf[i] = zVal_cd;

			 rotationMatrix(col->x_pattern_tf[i], col->y_pattern_tf[i], col->z_pattern_tf[i], 2);
			 // save the new pattern edge positions in (x_pattern_tf, ...):
			 col->x_pattern_tf[i] = xVal_cd + offX; col->y_pattern_tf[i] = yVal_cd + offY; col->z_pattern_tf[i] = zVal_cd + offZ;
		 }
	 }
	 else if (recalcMTCameraPos) {
		 // if the collider is a MT, then the camera's center of projection and viewing direction is recalculated and stored in
		 // the input parameter camVectors. The starting point is the original viewing direction, which is stored in the constant camVectorsStatic:
		 for (int i = 0; i < 12; i += 3) {
			 rotationMatrix(camVectorsStatic[i], camVectorsStatic[i + 1], camVectorsStatic[i + 2], 0);
			 camVectors[i] = xVal_cd; camVectors[i + 1] = yVal_cd; camVectors[i + 2] = zVal_cd;

			 rotationMatrix(camVectors[i], camVectors[i + 1], camVectors[i + 2], 1);
			 camVectors[i] = xVal_cd; camVectors[i + 1] = yVal_cd; camVectors[i + 2] = zVal_cd;

			 rotationMatrix(camVectors[i], camVectors[i + 1], camVectors[i + 2], 2);
			 if (i == 0) { // the first three values in camVectors are the center of projection:
				 xVal_cd += offX;
				 yVal_cd += offY;
				 zVal_cd += offZ;
			 }
			 camVectors[i] = xVal_cd; camVectors[i + 1] = yVal_cd; camVectors[i + 2] = zVal_cd;
		 }
	 }

	 if (recalcOrientationArrow) {
		 // if the collider is a workpiece with an orientation arrow, recalculate its position with the constant workpieceArrowPositionStatic as
		 // starting point and the parameter arrowPosition as storage destination:
		 for (int i = 0; i < 9; i += 3) {
			 rotationMatrix(workpieceArrowPositionStatic[i], workpieceArrowPositionStatic[i + 1], workpieceArrowPositionStatic[i + 2], 0);
			 arrowPosition[i] = xVal_cd; arrowPosition[i + 1] = yVal_cd; arrowPosition[i + 2] = zVal_cd;

			 rotationMatrix(arrowPosition[i], arrowPosition[i + 1], arrowPosition[i + 2], 1);
			 arrowPosition[i] = xVal_cd; arrowPosition[i + 1] = yVal_cd; arrowPosition[i + 2] = zVal_cd;

			 rotationMatrix(arrowPosition[i], arrowPosition[i + 1], arrowPosition[i + 2], 2);

			 xVal_cd += offX;
			 yVal_cd += offY;
			 zVal_cd += offZ;

			 arrowPosition[i] = xVal_cd; arrowPosition[i + 1] = yVal_cd; arrowPosition[i + 2] = zVal_cd;
		 }
	 }
 }

 /*
recalulateColliderTransport() is mostly identical to the recalclateCollider() function, but specifically for objects that are
currently being transported with the gripper. The roll rotation in these cases must be performed along an axis along the last
arm segment to correctly represent the rotation of the last joint. Only workpieces and monitoring tools are transported with
the gripper arm. The latter are deactivated during transport, camera viewing directions do not have to be calculated. For this
reason, one mode parameter is sufficient, namely recalcPattern.
  */
 void CollisionDetection::recalculateColliderTransport(collider* col, float offX, float offY, float offZ, float rotHor, float rotNick, float rotRoll, bool recalcPattern) {
	 for (int i = 0; i < 6; i++) {
		 if (i % 2 == 0) {
			 col->limits[i] = -1000.0f;
		 }
		 else {
			 col->limits[i] = 1000.0f;
		 }
	 }

	 col->offsets[0] = offX;
	 col->offsets[1] = offY;
	 col->offsets[2] = offZ;

	 col->angles[0] = rotHor;
	 col->angles[1] = rotNick;
	 col->angles[2] = rotRoll;

	 for (int i = 0; i < col->points_stat; i++) {
		 if (col->ignoredPoints[i] == false) {
			 rotationMatrix(col->x_stat[i], col->y_stat[i], col->z_stat[i], 0);
			 col->x_coll[i] = xVal_cd; col->y_coll[i] = yVal_cd; col->z_coll[i] = zVal_cd;

			 rotationMatrix(col->x_coll[i], col->y_coll[i], col->z_coll[i], 1);
			 col->x_coll[i] = xVal_cd; col->y_coll[i] = yVal_cd; col->z_coll[i] = zVal_cd;

			 rotationMatrix(col->x_coll[i], col->y_coll[i], col->z_coll[i], 2);
			 xVal_cd += offX;
			 yVal_cd += offY;
			 zVal_cd += offZ;
			 col->x_coll[i] = xVal_cd; col->y_coll[i] = yVal_cd; col->z_coll[i] = zVal_cd;

			 if (xVal_cd > col->limits[0]) {
				 col->limits[0] = xVal_cd;
			 }

			 if (xVal_cd < col->limits[1]) {
				 col->limits[1] = xVal_cd;
			 }

			 if (yVal_cd > col->limits[2]) {
				 col->limits[2] = yVal_cd;
			 }

			 if (yVal_cd < col->limits[3]) {
				 col->limits[3] = yVal_cd;
			 }

			 if (zVal_cd > col->limits[4]) {
				 col->limits[4] = zVal_cd;
			 }

			 if (zVal_cd < col->limits[5]) {
				 col->limits[5] = zVal_cd;
			 }
		 }
	 }

	 if (recalcPattern) {
		 for (int i = 0; i < col->patternCount * 4; i++) {
			 rotationMatrix(col->x_pattern_stat[i], col->y_pattern_stat[i], col->z_pattern_stat[i], 0);
			 col->x_pattern_tf[i] = xVal_cd; col->y_pattern_tf[i] = yVal_cd; col->z_pattern_tf[i] = zVal_cd;

			 rotationMatrix(col->x_pattern_tf[i], col->y_pattern_tf[i], col->z_pattern_tf[i], 1);
			 col->x_pattern_tf[i] = xVal_cd; col->y_pattern_tf[i] = yVal_cd; col->z_pattern_tf[i] = zVal_cd;

			 rotationMatrix(col->x_pattern_tf[i], col->y_pattern_tf[i], col->z_pattern_tf[i], 2);
			 col->x_pattern_tf[i] = xVal_cd + offX; col->y_pattern_tf[i] = yVal_cd + offY; col->z_pattern_tf[i] = zVal_cd + offZ;
		 }
	 }
 }

 // gjk() returns the min. distance between two colliders col1 and col2. If a collision is happening, the returned float value is 0:
 double CollisionDetection::gjk(collider* col1, collider* col2) {
	 // FOR DETAILED EXPLANATION OF THE GJK ALGORITHM AND THE CODE, VISIT https://www.medien.ifi.lmu.de/lehre/ss10/ps/Ausarbeitung_Beispiel.pdf
	 // ---------------------------------------------------------------------------------------------------------------------------------------
	 int i;
	 norm2Wmax = 0;
	 eps_rel = eps_rel22;
	 eps_rel2 = eps_rel * eps_rel;
	 eps_tot = eps_tot22;
	 nullV = 0;

	 // pick one random point from each of the colliders as init:
	 col1_s[0] = col1->x_coll[col1->firstSurfacePoint];
	 col1_s[1] = col1->y_coll[col1->firstSurfacePoint];
	 col1_s[2] = col1->z_coll[col1->firstSurfacePoint];

	 col2_s[0] = col2->x_coll[col2->firstSurfacePoint];
	 col2_s[1] = col2->y_coll[col2->firstSurfacePoint];
	 col2_s[2] = col2->z_coll[col2->firstSurfacePoint];

	 v[0] = col1_s[0] - col2_s[0];
	 v[1] = col1_s[1] - col2_s[1];
	 v[2] = col1_s[2] - col2_s[2];

	 gjk_simplex->nvrtx = 1;
	 for (int t = 0; t < 3; ++t)
		 gjk_simplex->vrtx[0][t] = v[t];

	 iters_gjk = 0;
	 do {
		 iters_gjk++;
		 v_old[0] = v[0]; v_old[1] = v[1]; v_old[2] = v[2];
		 for (int t = 0; t < 3; ++t)
			 vminus[t] = -v[t];

		 support(col1, 1, vminus);
		 support(col2, 2, v);
		 for (int t = 0; t < 3; ++t)
			 w[t] = col1_s[t] - col2_s[t];

		 exeedtol_rel = (norm2(v) - dotProduct(v, w));
		 if (exeedtol_rel <= (eps_rel * norm2(v)) || exeedtol_rel < eps_tot22) {
			 break;
		 }

		 nullV = norm2(v) < eps_rel2;
		 if (nullV) {
			 break;
		 }

		 i = gjk_simplex->nvrtx;
		 for (int t = 0; t < 3; ++t)
			 gjk_simplex->vrtx[i][t] = w[t];
		 gjk_simplex->nvrtx++;

		 subalgorithm(gjk_simplex, v);

		 for (int jj = 0; jj < gjk_simplex->nvrtx; jj++) {
			 tesnorm = norm2(gjk_simplex->vrtx[jj]);
			 if (tesnorm > norm2Wmax) {
				 norm2Wmax = tesnorm;
			 }
		 }

		 absTestin = (norm2(v) <= (eps_tot * eps_tot * norm2Wmax));
		 if (absTestin) {
			 break;
		 }

		 if (iters_gjk > maxIterations) {
			 // cout << "Warning: early exit!" << endl;
			 break;
		 }
	 } while ((gjk_simplex->nvrtx != 4) && !(abs(v[0] - v_old[0]) < epsilon4 && abs(v[1] - v_old[1]) < epsilon4 && abs(v[2] - v_old[2]) < epsilon4));

	 // the vector representing the minimum distance is also saved under v_direction:
	 v_direction[0] = v[0];
	 v_direction[1] = v[1];
	 v_direction[2] = v[2];

	 return sqrt(norm2(v));
 }

 // checkForCollisionAxisAligned() does a axis aligned collision detection by comparing the limit values from both colliders.
 // This is a preprocessing step for the actual collision detection. If this function returns no collision, GJK will too:
 bool CollisionDetection::checkForCollisionAxisAligned(collider* a, collider* b) {
	 return (a->limits[1] <= b->limits[0] && a->limits[0] >= b->limits[1]) && (a->limits[3] <= b->limits[2] && a->limits[2] >= b->limits[3]) && (a->limits[5] <= b->limits[4] && a->limits[4] >= b->limits[5]);
 }

 /*
checkForCollision() uses the GJK algorithm to detect collisions between two colliders col1 and col2 and returns true
or false accordingly. In order to be able to recognize many cases in which no collision occurs without large computational
effort as such, a collision detection by means of spheres and maximum sizes as well as one along the three axes of the
coordinate system are carried out before the GJK algorithm.
  */
 bool CollisionDetection::checkForCollision(collider* col1, collider* col2) {
	 if (sqrt((col1->offsets[0] - col2->offsets[0]) * (col1->offsets[0] - col2->offsets[0]) + (col1->offsets[1] - col2->offsets[1]) * (col1->offsets[1] - col2->offsets[1]) + (col1->offsets[2] - col2->offsets[2]) * (col1->offsets[2] - col2->offsets[2])) > col1->maxSize + col2->maxSize) {
		 return false;
	 }

	 if (checkForCollisionAxisAligned(col1, col2)) {
		 if (gjk(col1, col2) > epsilon4) {
			 return false;
		 }
		 else {
			 return true;
		 }
	 }
	 else {
		 return false;
	 }
 }

 // for a collision detection of a collider col with the floor surface only a comparison of the lower limit value in z-direction with the floorHeight is necessary:
 bool CollisionDetection::checkForCollisionWithGround(collider* col, float floorHeight) {
	 if (col->limits[5] < floorHeight) return true;
	 else return false;
 }

 // distApproximation() returns a simple calculable approximation of the distance between two colliders, which is bigger or equal compared with the real distance:
 float CollisionDetection::distApproximation(collider* col1, collider* col2) {
	 // calculate the distance between the two centers of col1 and col2 and subtract both maxSizes:
	 return sqrt((col1->offsets[0] - col2->offsets[0]) * (col1->offsets[0] - col2->offsets[0]) + (col1->offsets[1] - col2->offsets[1]) * (col1->offsets[1] - col2->offsets[1]) + (col1->offsets[2] - col2->offsets[2]) * (col1->offsets[2] - col2->offsets[2])) - col1->maxSize - col2->maxSize;
 }


 /*
  * In this function we init the gv->collision detection and prepare stuff for visualization.
  *
  * detailed description...
  */
 void CollisionDetection::init_colliders4collision_detection_and_visualization() {

 	CollisionDetection *cd = CollisionDetection::get_instance();
 	InverseKinematic *ik = InverseKinematic::get_instance();
 	ConvHull3d *ch = ConvHull3d::get_instance();
 	GlobalVariables *gv = GlobalVariables::get_instance();

 	//init gv->collision detection
 	for (int i = 0; i < gv->collidersCount; i++) {
 		gv->colors = ik->increaseSize(gv->colors, gv->colorsCount, 1);
 		gv->colorStartPoints = ik->increaseSize(gv->colorStartPoints, gv->colorsCount, 1);
 		gv->colors[gv->colorsCount] = gv->colliders[i].color;
 		gv->colorStartPoints[gv->colorsCount] = gv->numPointsObject;
 		gv->colorsCount++;
 		cd->initMaxSize(&gv->colliders[i]);
 		ch->calcSurfaceAndReduced(&gv->colliders[i], false);
 		gv->numPointsObject += gv->colliders[i].facesTimes3;
 	}
 	// initialize gv->grippers. Different init becaus gv->gripping = gv->collision.
 	for (int i = 0; i < 2; i++) {
 		gv->colors = ik->increaseSize(gv->colors, gv->colorsCount, 1);
 		gv->colorStartPoints = ik->increaseSize(gv->colorStartPoints, gv->colorsCount, 1);
 		gv->colors[gv->colorsCount] = gv->grippers[i].color;
 		gv->colorStartPoints[gv->colorsCount] = gv->numPointsObject;
 		gv->colorsCount++;
 		cd->initMaxSize(&gv->grippers[i]);
 		ch->calcSurfaceAndReduced(&gv->grippers[i], false);
 		gv->numPointsObject += gv->grippers[i].facesTimes3;
 	}
 	// visualization of manipulated objects
 	for (int i = 0; i < 3; i++) {
 		cd->initMaxSize(&gv->distortedObstacles[i]);
 		ch->calcSurfaceAndReduced(&gv->distortedObstacles[i], false);
 	}
 	// safety margin for transport planning
 	for (int i = 0; i < 2; i++) {
 		cd->initMaxSize(&gv->safetyCollidersForMTs[i]);
 		ch->calcSurfaceAndReduced(&gv->safetyCollidersForMTs[i], false);
 	}
 	cd->initMaxSize(gv->safetyCollider);
 	ch->calcSurfaceAndReduced(gv->safetyCollider, false);

 	// ???
 	for (int i = 0; i < 4; i++) {
 		cd->initMaxSize(&gv->adversaryCollider[i]);
 		ch->calcSurfaceAndReduced(&gv->adversaryCollider[i], false);
 	}
 	cd->initMaxSize(gv->workpieceMesh);
 	ch->calcSurfaceAndReduced(gv->workpieceMesh, false);
 }
