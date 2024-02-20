/*
 * ConvHull3d.h
 *
 *  Created on: 30.11.2022
 *      Author: manuel
 */

#ifndef CONVHULL3D_H_
#define CONVHULL3D_H_

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <set>

#include "CollisionDetection.h"


#ifdef CONVHULL_3D_USE_SINGLE_PRECISION
    typedef float CH_FLOAT;
#else
    typedef double CH_FLOAT;
#endif

    class ConvHull3d {
    private:
    	static ConvHull3d* ch;
    	ConvHull3d();
    public:
    	typedef struct _ch_vertex {
    		union {
    			CH_FLOAT v[3];
    			struct {
    				CH_FLOAT x, y, z;
    			};
    		};
    	} ch_vertex;
    	typedef ch_vertex ch_vec3;

    	typedef struct float_w_idx {
    		CH_FLOAT val;
    		int idx;
    	}float_w_idx;

    	typedef struct int_w_idx {
    		int val;
    		int idx;
    	}int_w_idx;

    	virtual ~ConvHull3d();
    	static ConvHull3d* get_instance();

    	void plane_3d(CH_FLOAT* p, CH_FLOAT* c, CH_FLOAT* d);
    	static int cmp_asc_int(const void* a, const void* b)   ;
    	static int cmp_desc_int(const void* a, const void* b)  ;
    	static int cmp_asc_float(const void* a, const void* b) ;
    	static int cmp_desc_float(const void* a, const void* b);
    	CH_FLOAT det_4x4(CH_FLOAT* m);
    	void sort_float(CH_FLOAT* in_vec, CH_FLOAT* out_vec, int* new_idices, int len, int descendFLAG);
    	void ismember(int* pLeft, int* pRight, int* pOut, int nLeftElements, int nRightElements);
    	void sort_int(int* in_vec, int* out_vec, int* new_idices, int len, int descendFLAG);
    	void convhull_3d_build(ch_vertex* const in_vertices, const int nVert, int** out_faces, int* nOut_faces);
    	void calcSurfaceAndReduced(collider* col, bool debug);
};

#endif /* CONVHULL3D_H_ */
