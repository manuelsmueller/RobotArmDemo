/*
 * ConvHull3d.cpp
 *
 *  Created on: 30.11.2022
 *      Author: manuel
 */

/*
 * This content is from:
 *
filename: convhull_3d.h
version 1.0 2022-02-24

description:
The header file convhull_3d.h implements the quickhull algorithm, that calculates the minimal convex hull for a
finite set of points. In the threedimensional case, the quickhull algorithm returns which points create the
surface and defines a set of triangles, that form together the object surface. In this work, every element in the
environment is described as a convex hull and the quickhull algorithm is necessary for visualization. A further
advantage of convex hulls is simple and fast collision detection.

(c) 2003-2022 IAS, Universit�t Stuttgart
*/

// FOR DETAILED CODE EXPLANATION, VISIT https://github.com/leomccormack/convhull_3d/blob/master/convhull_3d.h
// ----------------------------------------------------------------------------------------------------------


#include "ConvHull3d.h"

#ifdef CONVHULL_3D_USE_SINGLE_PRECISION
#define CH_NOISE_VAL 0.00001f
#define ch_pow powf
#define ch_sqrt sqrtf
#else
#define CH_NOISE_VAL 0.0000001
#define ch_pow pow
#define ch_sqrt sqrt
#endif
#ifndef MIN
#define MIN(a,b) (( (a) < (b) ) ? (a) : (b) )
#endif
#ifndef MAX
#define MAX(a,b) (( (a) > (b) ) ? (a) : (b) )
#endif
#ifndef ch_malloc
#define ch_malloc malloc
#endif
#ifndef ch_calloc
#define ch_calloc calloc
#endif
#ifndef ch_realloc
#define ch_realloc realloc
#endif
#ifndef ch_free
#define ch_free free
#endif
#define CH_MAX_NUM_FACES 50000


ConvHull3d::ConvHull3d() {
	// TODO Auto-generated constructor stub

}

ConvHull3d::~ConvHull3d() {
	// TODO Auto-generated destructor stub
}
/*
 * singelton pattern
 */
ConvHull3d* ConvHull3d::ch = 0;
ConvHull3d* ConvHull3d::get_instance(){
	static bool isInit=false;

	//	InverseKinematic *ik;
		if(!isInit){
			ch  = new ConvHull3d();
			isInit=true;
		}
		return ch;
}

int ConvHull3d::cmp_asc_int(const void* a, const void* b) {
	//		 struct int_w_idx* a1 = (struct int_w_idx*)a;
	//		 struct int_w_idx* a2 = (struct int_w_idx*)b;
	int_w_idx* a1 = (int_w_idx*)a;
	int_w_idx* a2 = (int_w_idx*)b;
	if ((*a1).val < (*a2).val)return -1;
	else if ((*a1).val > (*a2).val)return 1;
	else return 0;
}

int ConvHull3d::cmp_desc_int(const void* a, const void* b) {
	//		 struct int_w_idx* a1 = (struct int_w_idx*)a;
	//		 struct int_w_idx* a2 = (struct int_w_idx*)b;
	int_w_idx* a1 = (int_w_idx*)a;
	int_w_idx* a2 = (int_w_idx*)b;
	if ((*a1).val > (*a2).val)return -1;
	else if ((*a1).val < (*a2).val)return 1;
	else return 0;
}

int ConvHull3d::cmp_asc_float(const void* a, const void* b) {
	//		 struct float_w_idx* a1 = (struct float_w_idx*)a;
	//		 struct float_w_idx* a2 = (struct float_w_idx*)b;
	int_w_idx* a1 = (int_w_idx*)a;
	int_w_idx* a2 = (int_w_idx*)b;
	if ((*a1).val < (*a2).val)return -1;
	else if ((*a1).val > (*a2).val)return 1;
	else return 0;
}

int ConvHull3d::cmp_desc_float(const void* a, const void* b) {
	//		 struct float_w_idx* a1 = (struct float_w_idx*)a;
	//		 struct float_w_idx* a2 = (struct float_w_idx*)b;
	int_w_idx* a1 = (int_w_idx*)a;
	int_w_idx* a2 = (int_w_idx*)b;
	if ((*a1).val > (*a2).val)return -1;
	else if ((*a1).val < (*a2).val)return 1;
	else return 0;
}



  void ConvHull3d::plane_3d(CH_FLOAT* p, CH_FLOAT* c, CH_FLOAT* d)
{
    int i, j, k, l;
    int r[3];
    CH_FLOAT sign, det, norm_c;
    CH_FLOAT pdiff[2][3], pdiff_s[2][2];

    for (i = 0; i < 2; i++)
        for (j = 0; j < 3; j++)
            pdiff[i][j] = p[(i + 1) * 3 + j] - p[i * 3 + j];
    memset(c, 0, 3 * sizeof(CH_FLOAT));
    sign = 1.0;
    for (i = 0; i < 3; i++)
        r[i] = i;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 2; j++) {
            for (k = 0, l = 0; k < 3; k++) {
                if (r[k] != i) {
                    pdiff_s[j][l] = pdiff[j][k];
                    l++;
                }
            }
        }
        det = pdiff_s[0][0] * pdiff_s[1][1] - pdiff_s[1][0] * pdiff_s[0][1];
        c[i] = sign * det;
        sign *= -1.0;
    }
    norm_c = (CH_FLOAT)0.0;
    for (i = 0; i < 3; i++)
        norm_c += (ch_pow(c[i], 2.0));
    norm_c = ch_sqrt(norm_c);
    for (i = 0; i < 3; i++)
        c[i] /= norm_c;
    (*d) = (CH_FLOAT)0.0;
    for (i = 0; i < 3; i++)
        (*d) += -p[i] * c[i];
}





  CH_FLOAT ConvHull3d::det_4x4(CH_FLOAT* m) {
    return
        m[3] * m[6] * m[9] * m[12] - m[2] * m[7] * m[9] * m[12] -
        m[3] * m[5] * m[10] * m[12] + m[1] * m[7] * m[10] * m[12] +
        m[2] * m[5] * m[11] * m[12] - m[1] * m[6] * m[11] * m[12] -
        m[3] * m[6] * m[8] * m[13] + m[2] * m[7] * m[8] * m[13] +
        m[3] * m[4] * m[10] * m[13] - m[0] * m[7] * m[10] * m[13] -
        m[2] * m[4] * m[11] * m[13] + m[0] * m[6] * m[11] * m[13] +
        m[3] * m[5] * m[8] * m[14] - m[1] * m[7] * m[8] * m[14] -
        m[3] * m[4] * m[9] * m[14] + m[0] * m[7] * m[9] * m[14] +
        m[1] * m[4] * m[11] * m[14] - m[0] * m[5] * m[11] * m[14] -
        m[2] * m[5] * m[8] * m[15] + m[1] * m[6] * m[8] * m[15] +
        m[2] * m[4] * m[9] * m[15] - m[0] * m[6] * m[9] * m[15] -
        m[1] * m[4] * m[10] * m[15] + m[0] * m[5] * m[10] * m[15];
}

  void ConvHull3d::sort_float(CH_FLOAT* in_vec, CH_FLOAT* out_vec, int* new_idices, int len, int descendFLAG)
{
    int i;
//    struct float_w_idx* data;
    float_w_idx* data;

    data = (float_w_idx*)ch_malloc(len * sizeof(float_w_idx));
    for (i = 0; i < len; i++) {
        data[i].val = in_vec[i];
        data[i].idx = i;
    }
    if (descendFLAG)
        qsort(data, len, sizeof(data[0]), cmp_desc_float);
    else
        qsort(data, len, sizeof(data[0]), cmp_asc_float);
    for (i = 0; i < len; i++) {
        if (out_vec != NULL)
            out_vec[i] = data[i].val;
        else
            in_vec[i] = data[i].val;
        if (new_idices != NULL)
            new_idices[i] = data[i].idx;
    }
    ch_free(data);
}

  void ConvHull3d::ismember(int* pLeft, int* pRight, int* pOut, int nLeftElements, int nRightElements)
{
    int i, j;
    memset(pOut, 0, nLeftElements * sizeof(int));
    for (i = 0; i < nLeftElements; i++)
        for (j = 0; j < nRightElements; j++)
            if (pLeft[i] == pRight[j])
                pOut[i] = 1;
}

  void ConvHull3d::sort_int(int* in_vec, int* out_vec, int* new_idices, int len, int descendFLAG)
{
    int i;
    int_w_idx* data;

    data = (int_w_idx*)ch_malloc(len * sizeof(int_w_idx));
    for (i = 0; i < len; i++) {
        data[i].val = in_vec[i];
        data[i].idx = i;
    }
    if (descendFLAG)
        qsort(data, len, sizeof(data[0]), cmp_desc_int);
    else
        qsort(data, len, sizeof(data[0]), cmp_asc_int);
    for (i = 0; i < len; i++) {
        if (out_vec != NULL)
            out_vec[i] = data[i].val;
        else
            in_vec[i] = data[i].val;
        if (new_idices != NULL)
            new_idices[i] = data[i].idx;
    }
    ch_free(data);
}

 void ConvHull3d::convhull_3d_build(ch_vertex* const in_vertices, const int nVert, int** out_faces, int* nOut_faces)
{
    int i, j, k, l, h;
    int nFaces, p, d;
    int* aVec, * faces;
    CH_FLOAT dfi, v, max_p, min_p;
    CH_FLOAT* points, * cf, * cfi, * df, * p_s, * span;

    if (nVert < 3 || in_vertices == NULL) {
        (*out_faces) = NULL;
        (*nOut_faces) = 0;
        return;
    }

    d = 3;

    points = (CH_FLOAT*)malloc(nVert * (d + 1) * sizeof(CH_FLOAT));
    for (i = 0; i < nVert; i++) {
        for (j = 0; j < d; j++)
            points[i * (d + 1) + j] = in_vertices[i].v[j] + CH_NOISE_VAL * rand() / (CH_FLOAT)RAND_MAX;
        points[i * (d + 1) + d] = 1.0f;
    }

    span = (CH_FLOAT*)malloc(d * sizeof(CH_FLOAT));
    for (j = 0; j < d; j++) {
        max_p = -2.23e+13; min_p = 2.23e+13;
        for (i = 0; i < nVert; i++) {
            max_p = MAX(max_p, points[i * (d + 1) + j]);
            min_p = MIN(min_p, points[i * (d + 1) + j]);
        }
        span[j] = max_p - min_p;
        assert(span[j] > 0.0000001f);
    }

    nFaces = (d + 1);
    faces = (int*)ch_calloc(nFaces * d, sizeof(int));
    aVec = (int*)ch_malloc(nFaces * sizeof(int));
    for (i = 0; i < nFaces; i++)
        aVec[i] = i;

    cf = (CH_FLOAT*)ch_malloc(nFaces * d * sizeof(CH_FLOAT));
    cfi = (CH_FLOAT*)ch_malloc(d * sizeof(CH_FLOAT));
    df = (CH_FLOAT*)ch_malloc(nFaces * sizeof(CH_FLOAT));
    p_s = (CH_FLOAT*)ch_malloc(d * d * sizeof(CH_FLOAT));
    for (i = 0; i < nFaces; i++) {
        for (j = 0, k = 0; j < (d + 1); j++) {
            if (aVec[j] != i) {
                faces[i * d + k] = aVec[j];
                k++;
            }
        }

        for (j = 0; j < d; j++)
            for (k = 0; k < d; k++)
                p_s[j * d + k] = points[(faces[i * d + j]) * (d + 1) + k];

        plane_3d(p_s, cfi, &dfi);
        for (j = 0; j < d; j++)
            cf[i * d + j] = cfi[j];
        df[i] = dfi;
    }
    CH_FLOAT* A;
    int* bVec, * fVec, * asfVec;
    int face_tmp[2];

    bVec = (int*)ch_malloc(4 * sizeof(int));
    for (i = 0; i < d + 1; i++)
        bVec[i] = i;

    A = (CH_FLOAT*)ch_calloc((d + 1) * (d + 1), sizeof(CH_FLOAT));
    fVec = (int*)ch_malloc((d + 1) * sizeof(int));
    asfVec = (int*)ch_malloc((d + 1) * sizeof(int));
    for (k = 0; k < (d + 1); k++) {
        for (i = 0; i < d; i++)
            fVec[i] = faces[k * d + i];
        sort_int(fVec, NULL, NULL, d, 0);
        p = k;
        for (i = 0; i < d; i++)
            for (j = 0; j < (d + 1); j++)
                A[i * (d + 1) + j] = points[(faces[k * d + i]) * (d + 1) + j];
        for (; i < (d + 1); i++)
            for (j = 0; j < (d + 1); j++)
                A[i * (d + 1) + j] = points[p * (d + 1) + j];

        v = det_4x4(A);

        if (v < 0) {
            for (j = 0; j < 2; j++)
                face_tmp[j] = faces[k * d + d - j - 1];
            for (j = 0; j < 2; j++)
                faces[k * d + d - j - 1] = face_tmp[1 - j];

            for (j = 0; j < d; j++)
                cf[k * d + j] = -cf[k * d + j];
            df[k] = -df[k];
            for (i = 0; i < d; i++)
                for (j = 0; j < (d + 1); j++)
                    A[i * (d + 1) + j] = points[(faces[k * d + i]) * (d + 1) + j];
            for (; i < (d + 1); i++)
                for (j = 0; j < (d + 1); j++)
                    A[i * (d + 1) + j] = points[p * (d + 1) + j];
        }
    }

    CH_FLOAT* meanp, * absdist, * reldist, * desReldist;
    meanp = (CH_FLOAT*)ch_calloc(d, sizeof(CH_FLOAT));
    for (i = d + 1; i < nVert; i++)
        for (j = 0; j < d; j++)
            meanp[j] += points[i * (d + 1) + j];
    for (j = 0; j < d; j++)
        meanp[j] = meanp[j] / (CH_FLOAT)(nVert - d - 1);

    absdist = (CH_FLOAT*)ch_malloc((nVert - d - 1) * d * sizeof(CH_FLOAT));
    for (i = d + 1, k = 0; i < nVert; i++, k++)
        for (j = 0; j < d; j++)
            absdist[k * d + j] = (points[i * (d + 1) + j] - meanp[j]) / span[j];

    reldist = (CH_FLOAT*)ch_calloc((nVert - d - 1), sizeof(CH_FLOAT));
    desReldist = (CH_FLOAT*)ch_malloc((nVert - d - 1) * sizeof(CH_FLOAT));
    for (i = 0; i < (nVert - d - 1); i++)
        for (j = 0; j < d; j++)
            reldist[i] += ch_pow(absdist[i * d + j], 2.0);

    int num_pleft, cnt;
    int* ind, * pleft;
    ind = (int*)ch_malloc((nVert - d - 1) * sizeof(int));
    pleft = (int*)ch_malloc((nVert - d - 1) * sizeof(int));
    sort_float(reldist, desReldist, ind, (nVert - d - 1), 1);

    num_pleft = (nVert - d - 1);
    for (i = 0; i < num_pleft; i++)
        pleft[i] = ind[i] + d + 1;

    memset(A, 0, (d + 1) * (d + 1) * sizeof(CH_FLOAT));

    cnt = 0;

    CH_FLOAT detA;
    CH_FLOAT* points_cf, * points_s;
    int* visible_ind, * visible, * nonvisible_faces, * f0, * face_s, * u, * gVec, * horizon, * hVec, * pp, * hVec_mem_face;
    int num_visible_ind, num_nonvisible_faces, n_newfaces, count, vis;
    int f0_sum, u_len, start, num_p, index, horizon_size1;
    int FUCKED;
    FUCKED = 0;
    u = horizon = NULL;
    nFaces = d + 1;
    visible_ind = (int*)ch_malloc(nFaces * sizeof(int));
    points_cf = (CH_FLOAT*)ch_malloc(nFaces * sizeof(CH_FLOAT));
    points_s = (CH_FLOAT*)ch_malloc(d * sizeof(CH_FLOAT));
    face_s = (int*)ch_malloc(d * sizeof(int));
    gVec = (int*)ch_malloc(d * sizeof(int));
    while ((num_pleft > 0)) {
        i = pleft[0];

        for (j = 0; j < num_pleft - 1; j++)
            pleft[j] = pleft[j + 1];
        num_pleft--;
        if (num_pleft == 0)
            ch_free(pleft);
        else
            pleft = (int*)ch_realloc(pleft, num_pleft * sizeof(int));

        cnt++;

        for (j = 0; j < d; j++)
            points_s[j] = points[i * (d + 1) + j];
        points_cf = (CH_FLOAT*)ch_realloc(points_cf, nFaces * sizeof(CH_FLOAT));
        visible_ind = (int*)ch_realloc(visible_ind, nFaces * sizeof(int));
#ifdef CONVHULL_3D_USE_CBLAS
#ifdef CONVHULL_3D_USE_SINGLE_PRECISION
        cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, 1, nFaces, d, 1.0f,
            points_s, d,
            cf, d, 0.0f,
            points_cf, nFaces);
#else
        cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, 1, nFaces, d, 1.0,
            points_s, d,
            cf, d, 0.0,
            points_cf, nFaces);
#endif
#else
        for (j = 0; j < nFaces; j++) {
            points_cf[j] = 0;
            for (k = 0; k < d; k++)
                points_cf[j] += points_s[k] * cf[j * d + k];
        }
#endif
        num_visible_ind = 0;
        for (j = 0; j < nFaces; j++) {
            if (points_cf[j] + df[j] > 0.0) {
                num_visible_ind++;
                visible_ind[j] = 1;
            }
            else
                visible_ind[j] = 0;
        }
        num_nonvisible_faces = nFaces - num_visible_ind;

        if (num_visible_ind != 0) {
            visible = (int*)ch_malloc(num_visible_ind * sizeof(int));
            for (j = 0, k = 0; j < nFaces; j++) {
                if (visible_ind[j] == 1) {
                    visible[k] = j;
                    k++;
                }
            }

            nonvisible_faces = (int*)ch_malloc(num_nonvisible_faces * d * sizeof(int));
            f0 = (int*)ch_malloc(num_nonvisible_faces * d * sizeof(int));
            for (j = 0, k = 0; j < nFaces; j++) {
                if (visible_ind[j] == 0) {
                    for (l = 0; l < d; l++)
                        nonvisible_faces[k * d + l] = faces[j * d + l];
                    k++;
                }
            }

            count = 0;
            for (j = 0; j < num_visible_ind; j++) {
                vis = visible[j];
                for (k = 0; k < d; k++)
                    face_s[k] = faces[vis * d + k];
                sort_int(face_s, NULL, NULL, d, 0);
                ismember(nonvisible_faces, face_s, f0, num_nonvisible_faces * d, d);
                u_len = 0;

                for (k = 0; k < num_nonvisible_faces; k++) {
                    f0_sum = 0;
                    for (l = 0; l < d; l++)
                        f0_sum += f0[k * d + l];
                    if (f0_sum == d - 1) {
                        u_len++;
                        if (u_len == 1)
                            u = (int*)ch_malloc(u_len * sizeof(int));
                        else
                            u = (int*)ch_realloc(u, u_len * sizeof(int));
                        u[u_len - 1] = k;
                    }
                }
                for (k = 0; k < u_len; k++) {
                    count++;
                    if (count == 1)
                        horizon = (int*)ch_malloc(count * (d - 1) * sizeof(int));
                    else
                        horizon = (int*)ch_realloc(horizon, count * (d - 1) * sizeof(int));
                    for (l = 0; l < d; l++)
                        gVec[l] = nonvisible_faces[u[k] * d + l];
                    for (l = 0, h = 0; l < d; l++) {
                        if (f0[u[k] * d + l]) {
                            horizon[(count - 1) * (d - 1) + h] = gVec[l];
                            h++;
                        }
                    }
                }
                if (u_len != 0)
                    ch_free(u);
            }
            horizon_size1 = count;
            for (j = 0, l = 0; j < nFaces; j++) {
                if (!visible_ind[j]) {
                    for (k = 0; k < d; k++)
                        faces[l * d + k] = faces[j * d + k];

                    for (k = 0; k < d; k++)
                        cf[l * d + k] = cf[j * d + k];
                    df[l] = df[j];
                    l++;
                }
            }

            nFaces = nFaces - num_visible_ind;
            faces = (int*)ch_realloc(faces, nFaces * d * sizeof(int));
            cf = (CH_FLOAT*)ch_realloc(cf, nFaces * d * sizeof(CH_FLOAT));
            df = (CH_FLOAT*)ch_realloc(df, nFaces * sizeof(CH_FLOAT));

            start = nFaces;

            n_newfaces = horizon_size1;
            for (j = 0; j < n_newfaces; j++) {
                nFaces++;
                faces = (int*)ch_realloc(faces, nFaces * d * sizeof(int));
                cf = (CH_FLOAT*)ch_realloc(cf, nFaces * d * sizeof(CH_FLOAT));
                df = (CH_FLOAT*)ch_realloc(df, nFaces * sizeof(CH_FLOAT));
                for (k = 0; k < d - 1; k++)
                    faces[(nFaces - 1) * d + k] = horizon[j * (d - 1) + k];
                faces[(nFaces - 1) * d + (d - 1)] = i;

                for (k = 0; k < d; k++)
                    for (l = 0; l < d; l++)
                        p_s[k * d + l] = points[(faces[(nFaces - 1) * d + k]) * (d + 1) + l];
                plane_3d(p_s, cfi, &dfi);
                for (k = 0; k < d; k++)
                    cf[(nFaces - 1) * d + k] = cfi[k];
                df[(nFaces - 1)] = dfi;
                if (nFaces > CH_MAX_NUM_FACES) {
                    FUCKED = 1;
                    nFaces = 0;
                    break;
                }
            }

            hVec = (int*)ch_malloc(nFaces * sizeof(int));
            hVec_mem_face = (int*)ch_malloc(nFaces * sizeof(int));
            for (j = 0; j < nFaces; j++)
                hVec[j] = j;
            for (k = start; k < nFaces; k++) {
                for (j = 0; j < d; j++)
                    face_s[j] = faces[k * d + j];
                sort_int(face_s, NULL, NULL, d, 0);
                ismember(hVec, face_s, hVec_mem_face, nFaces, d);
                num_p = 0;
                for (j = 0; j < nFaces; j++)
                    if (!hVec_mem_face[j])
                        num_p++;
                pp = (int*)ch_malloc(num_p * sizeof(int));
                for (j = 0, l = 0; j < nFaces; j++) {
                    if (!hVec_mem_face[j]) {
                        pp[l] = hVec[j];
                        l++;
                    }
                }
                index = 0;
                detA = 0.0;

                while (detA == 0.0) {
                    for (j = 0; j < d; j++)
                        for (l = 0; l < d + 1; l++)
                            A[j * (d + 1) + l] = points[(faces[k * d + j]) * (d + 1) + l];
                    for (; j < d + 1; j++)
                        for (l = 0; l < d + 1; l++)
                            A[j * (d + 1) + l] = points[pp[index] * (d + 1) + l];
                    index++;
                    detA = det_4x4(A);
                }

                if (detA < 0.0) {
                    for (j = 0; j < 2; j++)
                        face_tmp[j] = faces[k * d + d - j - 1];
                    for (j = 0; j < 2; j++)
                        faces[k * d + d - j - 1] = face_tmp[1 - j];

                    for (j = 0; j < d; j++)
                        cf[k * d + j] = -cf[k * d + j];
                    df[k] = -df[k];
                    for (l = 0; l < d; l++)
                        for (j = 0; j < d + 1; j++)
                            A[l * (d + 1) + j] = points[(faces[k * d + l]) * (d + 1) + j];
                    for (; l < d + 1; l++)
                        for (j = 0; j < d + 1; j++)
                            A[l * (d + 1) + j] = points[pp[index] * (d + 1) + j];

                }
                ch_free(pp);
            }
            if (horizon_size1 > 0)
                ch_free(horizon);
            ch_free(f0);
            ch_free(nonvisible_faces);
            ch_free(visible);
            ch_free(hVec);
            ch_free(hVec_mem_face);
        }
        if (FUCKED) {
            break;
        }
    }

    if (FUCKED) {
        (*out_faces) = NULL;
        (*nOut_faces) = 0;
    }
    else {
        (*out_faces) = (int*)ch_malloc(nFaces * d * sizeof(int));
        memcpy((*out_faces), faces, nFaces * d * sizeof(int));
        (*nOut_faces) = nFaces;
    }

    ch_free(visible_ind);
    ch_free(points_cf);
    ch_free(points_s);
    ch_free(face_s);
    ch_free(gVec);
    ch_free(meanp);
    ch_free(absdist);
    ch_free(reldist);
    ch_free(desReldist);
    ch_free(ind);
    ch_free(span);
    ch_free(points);
    ch_free(faces);
    ch_free(aVec);
    ch_free(cf);
    ch_free(cfi);
    ch_free(df);
    ch_free(p_s);
    ch_free(fVec);
    ch_free(asfVec);
    ch_free(bVec);
    ch_free(A);
}

/*
The calcSurfaceAndReduced() method should be considered as one of the steps necessary to initialize the collider.
It executes the Quickhull algorithm for a collider passed as input parameter. On the one hand, it determines the
points on the surface. On the other hand, the surface itself is calculated as a list of triangles in space (vertices),
which is the prerequisite for a graphical representation. Likewise, the shading of these vertices is important in order
to be able to recognize spatial conditions. The normal vector is used here. It must be ensured that this always points
outwards.
*/
 void ConvHull3d::calcSurfaceAndReduced(collider* col, bool debug) {
	CollisionDetection *cd = CollisionDetection::get_instance();
    int* faceIndices = NULL;
    int numberOfFaces;

    // prepare the input parameter vertices for the convhull_3d_build method by filling this array with the points that
    // describe the geometric structure:
    ch_vertex* point_cloud;
    point_cloud = (ch_vertex*)malloc(col->points_stat * sizeof(ch_vertex)); // points_stat are the number of points
    for (int i = 0; i < col->points_stat; i++) {
        point_cloud[i].x = col->x_stat[i];
        point_cloud[i].y = col->y_stat[i];
        point_cloud[i].z = col->z_stat[i];
    }

    convhull_3d_build(point_cloud, col->points_stat, &faceIndices, &numberOfFaces); // convhull_3d_build returns the number of
                                                                          // surface faces and a list of indices refering
                                                                          // to the list of points (x_stat, ...), that
                                                                          // describes the triangles form the surface

    col->connections = new int[numberOfFaces * 3]; // prepare the collider to store this surface informations
                                            // (nFaces * 3 because every vertex consists of 3 points)

    // necessary variables for calculating the normal vector:
    float x_center, y_center, z_center;
    float x_normal, y_normal, z_normal, d_val, t_val, normal_len;
    float x_outside, y_outside, z_outside;
    float x_a, y_a, z_a, x_b, y_b, z_b;

    col->facesTimes3 = numberOfFaces * 3; // save the number of surface faces

    std::set<int> pointsOnSurface;
    for (int i = 0; i < col->facesTimes3; i++) {
        pointsOnSurface.insert(faceIndices[i]); // save the points lying on the surface in a separate list

        // consider every surface face:
        if ((i + 1) % 3 == 0) {
            // calculate the normal vector (x_normal, y_normal, z_normal):
            x_center = (col->x_stat[faceIndices[i]] + col->x_stat[faceIndices[i - 1]] + col->x_stat[faceIndices[i - 2]]) / 3.0f;
            y_center = (col->y_stat[faceIndices[i]] + col->y_stat[faceIndices[i - 1]] + col->y_stat[faceIndices[i - 2]]) / 3.0f;
            z_center = (col->z_stat[faceIndices[i]] + col->z_stat[faceIndices[i - 1]] + col->z_stat[faceIndices[i - 2]]) / 3.0f;

            x_a = col->x_stat[faceIndices[i - 1]] - col->x_stat[faceIndices[i]];
            y_a = col->y_stat[faceIndices[i - 1]] - col->y_stat[faceIndices[i]];
            z_a = col->z_stat[faceIndices[i - 1]] - col->z_stat[faceIndices[i]];

            x_b = col->x_stat[faceIndices[i - 2]] - col->x_stat[faceIndices[i]];
            y_b = col->y_stat[faceIndices[i - 2]] - col->y_stat[faceIndices[i]];
            z_b = col->z_stat[faceIndices[i - 2]] - col->z_stat[faceIndices[i]];

            x_normal = y_a * z_b - z_a * y_b;
            y_normal = z_a * x_b - x_a * z_b;
            z_normal = x_a * y_b - y_a * x_b;
            normal_len = sqrt(x_normal * x_normal + y_normal * y_normal + z_normal * z_normal);

            if (normal_len > cd->epsilon4) {
                // normailze the normal vector to length 1:
                x_normal /= normal_len;
                y_normal /= normal_len;
                z_normal /= normal_len;

                // determine whether the normal vector points to the outside or inside:
                d_val = x_normal * col->x_stat[faceIndices[i]] + y_normal * col->y_stat[faceIndices[i]] + z_normal * col->z_stat[faceIndices[i]];

                x_outside = x_center + x_normal * cd->epsilon4;
                y_outside = y_center + y_normal * cd->epsilon4;
                z_outside = z_center + z_normal * cd->epsilon4;

                t_val = d_val / (x_normal * x_outside + y_normal * y_outside + z_normal * z_outside);

                // modify the sequence with which the indices are saved so that the vector always points outwards:
                if (t_val >= 0.9f && t_val < 1) {
                    col->connections[i] = faceIndices[i];
                    col->connections[i - 1] = faceIndices[i - 1];
                    col->connections[i - 2] = faceIndices[i - 2];
                }
                else if (t_val > 1 && t_val <= 1.1f) {
                    col->connections[i] = faceIndices[i - 2];
                    col->connections[i - 1] = faceIndices[i - 1];
                    col->connections[i - 2] = faceIndices[i];
                }
                else {
                    cout << "Hull error: normal value 1!" << endl;
                }
            }
            else {
                cout << "Hull error: normal value 2!" << endl;
            }
        }
    }

    // for every point in the structure, save if it is inside (ignored = true) or on the surface:
    col->ignoredPoints = new bool[col->points_stat];
    for (int i = 0; i < col->points_stat; i++) {
        col->ignoredPoints[i] = true;
    }

    if (debug) cout << "Info: set size: " << pointsOnSurface.size() << ", faces: " << col->facesTimes3 / 3 << endl;
    for (std::set<int>::iterator it = pointsOnSurface.begin(); it != pointsOnSurface.end(); ++it) {
        col->ignoredPoints[*it] = false;
        if (debug) cout << *it << " ";
    }
    if (debug) cout << endl;

    // determine the first point in the list, that is lying on the surface:
    for (int i = 0; i < col->points_stat; i++) {
        if (!col->ignoredPoints[i]) {
            col->firstSurfacePoint = i; // save it seperately
            break;
        }
    }

    if (col->firstSurfacePoint == -1) {
        cout << "Hull error: no surface point!" << endl;
    }

    // prepare the arrays, that will contain only the relevant points for the collision detection
    col->x_coll = new float[col->points_stat];
    col->y_coll = new float[col->points_stat];
    col->z_coll = new float[col->points_stat];

    // delete the in this function created pointers, because pointers do not delete automatically
    free(point_cloud);
    free(faceIndices);
}
