/*
 * Constants.h
 *
 *  Created on: 05.12.2022
 *      Author: manuel
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

const int totalPatternCount = 8 + 6 + 8 + 2 + 4 + 3 + 3;
const int objectsWithPatternsCount = 7;

const int objectIndicesFromPattern[totalPatternCount]{ 8,8, 9,9,9,9, 10,10,10, 11,11,11, 12,12,12,12,12,12,12,12, 13,13,13,13,13,13, 14,14,14,14,14,14,14,14 };
const int patternIndicesFromPattern[totalPatternCount]{ 4,0, 12,0,8,4, 0,4,8, 0,4,8, 12,8,20,16,0,4,24,28, 12,8,16,0,4,20, 12,8,20,16,0,4,24,28 };

const float pattern7SurfaceVec[2]{ -0.9863939238f, 0.1643989873f }, pattern8SurfaceVec[2]{ -0.9863939238f, -0.1643989873f };
const float pattern10SurfaceVec[2]{ -0.8320503f, 0.5547002f }, pattern11SurfaceVec[2]{ -0.8320503f, -0.5547002f };

const float objectPatternOffsetsX[totalPatternCount] = { 0.0f, 0.0f,   0.0f, 0.0f, 0.0f, 0.0f,   0.0f, -1.5f * pattern7SurfaceVec[1], 1.5f * pattern7SurfaceVec[1],   0.0f, -0.5547002f, 0.5547002f,   -2.0f, 2.0f, -2.0f, 2.0f, -2.0f, 2.0f, -2.0f, 2.0f,   -2.5f, 2.5f, 0.0f, -2.5f, 2.5f, 0.0f,   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
const float objectPatternOffsetsY[totalPatternCount] = { 1.15f, 1.15f,   1.5f, 2.0f, 1.5f, 2.0f,   1.5f, -1.5f * pattern7SurfaceVec[0], -1.5f * pattern7SurfaceVec[0],   1.5f, 0.8320503f, 0.8320503f,   4.0f, 4.0f, 4.0f, 4.0f, 4.0f, 4.0f, 4.0f, 4.0f,   2.5f, 2.5f, 5.0f, 2.5f, 2.5f, 5.0f,   3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f };

const float patternDirRotationX[totalPatternCount] = { 1.0f, -1.0f,   0.0f, -1.0f, 0.0f, 1.0f,   0.0f, pattern7SurfaceVec[0], -pattern7SurfaceVec[0],   0.0f, -0.8320503f, 0.8320503f,   0.0f, 0.0f, -1.0f, -1.0f, 0.0f, 0.0f, 1.0f, 1.0f,   0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, -1.0f, -1.0f, 0.0f, 0.0f, 1.0f, 1.0f };
const float patternDirRotationY[totalPatternCount] = { 0.0f, 0.0f,   1.0f, 0.0f, -1.0f, 0.0f,   1.0f, -pattern7SurfaceVec[1], -pattern7SurfaceVec[1],   1.0f, -0.5547002f, -0.5547002f,   1.0f, 1.0f, 0.0f, 0.0f, -1.0f, -1.0f, 0.0f, 0.0f,    1.0f, 1.0f, 0.0f, -1.0f, -1.0f, 0.0f,   1.0f, 1.0f, 0.0f, 0.0f, -1.0f, -1.0f, 0.0f, 0.0f };



#endif /* CONSTANTS_H_ */
