/*
 * collider.h
 *
 *  Created on: 21.11.2022
 *      Author: manuel
 *
 *  Note: This is a fundamental basis class. Be careful with moving stuff into it.
 *  Avoid dependancies to other classes.
 */

#ifndef COLLIDER_H_
#define COLLIDER_H_

#include <GL/glew.h>

// caution!
// May not include GlobalDefinitions.h since otherwhise, a circular definition structure is build.

class collider {
public:
	collider();
	virtual ~collider();

//	static void init_colliders4collision_detection_and_visualization();


	struct Vertex {
	    float x;
	    float y;
	    float z;

	    float r;
	    float g;
	    float b;
	    float a;
	};

	struct Edge {
	    float x;
	    float y;
	    float z;
	};


	struct VertexBuffer {
		VertexBuffer(void* data, uint32_t numVertices) {
			glGenVertexArrays(1, &vao);
			glBindVertexArray(vao);

			glGenBuffers(1, &bufferId);
			glBindBuffer(GL_ARRAY_BUFFER, bufferId);
			glBufferData(GL_ARRAY_BUFFER, numVertices * sizeof(Vertex), data, GL_DYNAMIC_DRAW);

			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(struct Vertex, x));

			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(struct Vertex, r));

			glBindVertexArray(0);
		}

		virtual ~VertexBuffer() {
			glDeleteBuffers(1, &bufferId);
		}

		void bind() {
			glBindVertexArray(vao);
		}

		void unbind() {
			glBindVertexArray(0);
		}

		void update(void* data, uint32_t numVertices) {
			glBindBuffer(GL_ARRAY_BUFFER, bufferId);
			glBufferData(GL_ARRAY_BUFFER, numVertices * sizeof(Vertex), data, GL_DYNAMIC_DRAW);
		}
	private:
		GLuint bufferId;
		GLuint vao;
	};

	/*
	The struct collider represents an arbitray convex geometric 3d structure and contains all necessary information for checking for collisions
	between colliders and similar tasks.

	At the beginning, a collider has to be initialized in several steps: it has to be specified which point cloud the convex hull is based on,
	i.e. which points (x_stat, y_stat, z_stat) the point cloud consists of and how many points are used (points_stat). The point cloud is
	initialized in such a way that in it the coordinate origin lies. Later rotation and translation operators can be used and the shifted structure
	(x_coll, y_coll, z_coll) can be generated. If it is for example an obstacle with localization patterns, their corner point positions are to be
	fixed likewise (x_pattern_stat, ...) and transformed later equivalently (x_pattern_tf, ...). For the graphical representation it has to be defined
	which points belong to the surface (ignoredPoints), which tuples of three points (i.e. vertices) form the surface (connections) and in which color
	it is to be displayed (color). The last step of the initialization is to determine the maximum extension of the hull along the three axes (limits)
	and the minimum radius of an imaginary sphere with center in the origin, so that it completely encloses the unshifted hull (maxSize), in order to
	accelerate the collision detection. The variables offsets and angles are necessary for storing the shifts and rotations during runtime.33
	*/
	int points_stat;
	int facesTimes3;
	int patternCount;

	// convex structure!
	float* x_stat;
	float* y_stat;
	float* z_stat;

	float* x_coll;
	float* y_coll;
	float* z_coll;

	bool* ignoredPoints;
	int firstSurfacePoint = -1;

	int* connections;
	Edge color;

	float* x_pattern_stat;
	float* y_pattern_stat;
	float* z_pattern_stat;

	float* x_pattern_tf;
	float* y_pattern_tf;
	float* z_pattern_tf;

	int* pattern_indices;

	float offsets[3]; //[0] =X, [1]=Y, [2]=Z
	float angles[3];
	float maxSize;
	float limits[6];

	static void recalculatesColliders();
};

#endif /* COLLIDER_H_ */
