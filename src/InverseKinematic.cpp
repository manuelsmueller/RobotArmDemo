/*
 * InverseKinematic.cpp
 *
 *  Created on: 26.11.2022
 *      Author: manuel
 */

#include "InverseKinematic.h"

InverseKinematic::InverseKinematic() {

}

InverseKinematic::~InverseKinematic() {
	// TODO Auto-generated destructor stub
}

/*
 * singelton pattern
 */
InverseKinematic* InverseKinematic::ik = 0;
InverseKinematic* InverseKinematic::get_instance(){
	static bool isInit=false;

//	InverseKinematic *ik;
	if(!isInit){
		ik  = new InverseKinematic();
		isInit=true;
	}
	return ik;
}


/*
initInverseKin() initializes the corresponding calculation and must be called once at the beginning.
The basis of these calculations are movement margins of individual arm segments. Variables that together
define these movement margins are filled with calculated values in the initialization method.
*/
void InverseKinematic::initInverseKin() {
    innerVector[0] = -len4 * cos((qUpperLimits[3] + 90.0f + qLowerLimits[1]) / 180.0f * M_PI) - len3 * cos(
        (90.0f + qLowerLimits[1]) / 180 * M_PI);
    innerVector[1] = len4 * sin((qUpperLimits[3] + 90.0f + qLowerLimits[1]) / 180.0f * M_PI) + len3 * sin(
        (90.0f + qLowerLimits[1]) / 180 * M_PI);

    innerRadius = sqrt(innerVector[0] * innerVector[0] + innerVector[1] * innerVector[1]);
    innerAngle = -atan2(-innerVector[0], innerVector[1]) / M_PI * 180.0f - qLowerLimits[1] + qUpperLimits[2] - 270.0f;

    offset1[0] = -len2 * cos((90.0f + qLowerLimits[1]) / 180.0f * M_PI);
    offset1[1] = len2 * sin((90.0f + qLowerLimits[1]) / 180.0f * M_PI);

    offset2[0] = offset1[0] / len2 * (len2 + len3);
    offset2[1] = offset1[1] / len2 * (len2 + len3);

    offset3[0] = len2 - len3 * cos((qUpperLimits[2] - 180.0f) / 180.0f * M_PI);
    offset3[1] = len3 * sin((qUpperLimits[2] - 180.0f) / 180.0f * M_PI);


    offset4[0] = -len2 * cos((90 + qLowerLimits[1]) / 180 * M_PI) - len3 * cos(
        (90 + qLowerLimits[1] + qUpperLimits[2]) / 180 * M_PI);

    offset4[1] = len2 * sin((90 + qLowerLimits[1]) / 180 * M_PI) + len3 * sin(
        (90 + qLowerLimits[1] + qUpperLimits[2]) / 180 * M_PI);

    innerRadius2 = sqrt(offset4[0] * offset4[0] + offset4[1] * offset4[1]);
    innerAngle2 = -atan2(-offset4[0], offset4[1]) / M_PI * 180;

    circleCentersX[0] = -len2 * cos((90 + qLowerLimits[1]) / 180 * M_PI);
    circleCentersY[0] = len2 * sin((90 + qLowerLimits[1]) / 180 * M_PI);

    circleCentersX[1] = 0.0f;
    circleCentersY[1] = 0.0f;

    circleCentersX[2] = len2 * sin(qUpperLimits[1] / 180 * M_PI);
    circleCentersY[2] = len2 * cos(qUpperLimits[1] / 180 * M_PI);

    circleCentersX[3] = 0.0f;
    circleCentersY[3] = 0.0f;

    circleRangesLower[0] = qLowerLimits[1];
    circleRangesUpper[0] = qLowerLimits[1] + qUpperLimits[2];

    circleRangesLower[1] = innerAngle2;
    circleRangesUpper[1] = innerAngle2 + qUpperLimits[1] - qLowerLimits[1];

    circleRangesLower[2] = qUpperLimits[1] + qUpperLimits[2] - 360;
    circleRangesUpper[2] = qUpperLimits[1];

    circleRangesLower[3] = qLowerLimits[1];
    circleRangesUpper[3] = qUpperLimits[1];

    circleRadius[0] = len3;
    circleRadius[1] = innerRadius2;
    circleRadius[2] = len3;
    circleRadius[3] = len2 + len3;
}

/*
In the following, the range of motion in the two-dimensional case is to be considered. This means
that the first joint at the foot of the robot is initially neglected. Likewise, the last joint at
the end effector is not relevant, which can rotate it. A point (x, y) is passed as input parameter
to the method isThirdJointValid(). For this point it is then checked whether it could be reached
by the gripper arm without violating angle limits. Accordingly, true or false is returned.
*/
bool InverseKinematic::isThirdJointValid(float x, float y) {
    inside[0] = false;
    inside[1] = false;
    inside[2] = false;

    outside[0] = false;
    outside[1] = false;
    outside[2] = false;
    outside[3] = false;
    outside[4] = false;

    pointRadius = sqrt(x * x + y * y);
    x_diff = x / pointRadius;
    y_diff = y / pointRadius;

    pointAngle = acos(y_diff) / M_PI * 180;
    if (x_diff < 0) {
        pointAngle = -pointAngle;
    }

    if (pointAngle >= qLowerLimits[1] && pointAngle <= 90 && pointRadius <= len2 + len3 + len4) {
        inside[0] = true;
    }

    if ((pointAngle <= -90 || pointAngle >= (180 + qLowerLimits[1])) && pointRadius <= innerRadius - len2) {
        outside[0] = true;
    }

    if (pointRadius <= (innerRadius - len2) / 2) {
        outside[4] = true;
    }

    pointRadius = sqrt((x - len2) * (x - len2) + y * y);
    x_diff = (x - len2) / pointRadius;
    y_diff = y / pointRadius;

    pointAngle = acos(y_diff) / M_PI * 180;
    if (x_diff < 0) {
        pointAngle = -pointAngle;
    }

    if ((pointAngle <= -(270 - qUpperLimits[2]) || pointAngle >= 90) && pointRadius <= len3 + len4) {
        inside[1] = true;
    }

    if (pointAngle >= -90 && pointAngle <= innerAngle && pointRadius <= innerRadius) {
        outside[3] = true;
    }

    pointRadius = sqrt((x - offset1[0]) * (x - offset1[0]) + (y - offset1[1]) * (y - offset1[1]));
    x_diff = (x - offset1[0]) / pointRadius;
    y_diff = (y - offset1[1]) / pointRadius;

    pointAngle = acos(y_diff) / M_PI * 180;
    if (x_diff < 0) {
        pointAngle = -pointAngle;
    }

    if (pointAngle >= qLowerLimits[1] && pointAngle <= (
        180 + qLowerLimits[1]) && pointRadius <= innerRadius) {
        outside[1] = true;
    }

    pointRadius = sqrt((x - offset2[0]) * (x - offset2[0]) + (y - offset2[1]) * (y - offset2[1]));
    x_diff = (x - offset2[0]) / pointRadius;
    y_diff = (y - offset2[1]) / pointRadius;

    pointAngle = acos(y_diff) / M_PI * 180;
    if (x_diff < 0) {
        pointAngle = -pointAngle;
    }

    if (pointAngle >= qLowerLimits[1] && pointAngle <= (180 + qLowerLimits[1]) && pointRadius <= len4) {
        outside[2] = true;
    }

    pointRadius = sqrt((x - offset3[0]) * (x - offset3[0]) + (y - offset3[1]) * (y - offset3[1]));
    x_diff = (x - offset3[0]) / pointRadius;
    y_diff = (y - offset3[1]) / pointRadius;

    pointAngle = acos(y_diff) / M_PI * 180;
    if (x_diff < 0) {
        pointAngle = -pointAngle;
    }

    if (pointAngle >= qUpperLimits[2] - 270 && pointAngle <= qUpperLimits[2] + qUpperLimits[3] - 270 && pointRadius <= len4) {
        inside[2] = true;
    }

    if ((inside[0] == true || inside[1] == true || inside[2] == true) && (!outside[0] && !outside[1] && !outside[2] && !outside[3] && !outside[4])) {
        return true;
    }
    else if (inside[2] && !outside[0] && !outside[3] && !outside[4]) {
        return true;
    }
    return false;
}

/*
isSecondJointValid() is similar to isThirdJointValid(), but ignores the last arm segment (the one to which the end
effector is attached). So the point (x, y) passed as parameter is checked for reachability ignoring the last arm
segment. Accordingly, true or false is returned again. Two special features set this method apart from isThirdJointValid():
if includeThird = true, then a theta angle must be passed. The theta angle specifies the angle between the ground plane and
the end effector. The point (x, y) in this case, as in the previous method, specifies the position of the end effector. At
the same time, the angle values of the three middle joints are calculated and stored in q_ik.
*/
inline bool InverseKinematic::isSecondJointValid(float x, float y, bool includeThird, float theta) {

    // in case of includeThird = true, return the point to the end position of the second joint:
    if (includeThird) {
        x_in = x + len4 * sin(theta / 180 * M_PI);
        y_in = y + len4 * cos(theta / 180 * M_PI);
    }
    else {
        x_in = x;
        y_in = y;
    }

    jointDistance = sqrt(x_in * x_in + y_in * y_in);

    // calculate the second, third and, in the case of includeThird also the fourth, joint angle and store them in q_ik. Return if the point is reachable or not:
    if (jointDistance <= len2 + len3 && jointDistance >= abs(len2 - len3)) {
        x_mid = (jointDistance * jointDistance + len2 * len2 - len3 * len3) / (2 * jointDistance);
        y_mid = sqrt(-jointDistance * jointDistance * jointDistance * jointDistance + 2 * jointDistance * jointDistance * (len2 * len2 + len3 * len3) - (len2 * len2 - len3 * len3) * (len2 * len2 - len3 * len3)) / (2 * jointDistance);
        q_ik[1] = acos(y_in / jointDistance) / M_PI * 180;
        if (x_in < 0) {
            q_ik[1] = -q_ik[1];
        }
        q_ik[1] += acos(y_mid / len2) / M_PI * 180 - 90;

        // transform q_ik[1], so that the angle is in range [-180, 180)
        if (q_ik[1] < -180) {
            while (q_ik[1] < -180) {
                q_ik[1] += 360;
            }
        }
        if (q_ik[1] >= 180) {
            while (q_ik[1] >= 180) {
                q_ik[1] -= 360;
            }
        }

        x_joint = len2 * sin(q_ik[1] / 180 * M_PI);
        y_joint = len2 * cos(q_ik[1] / 180 * M_PI);

        q_ik[2] = acos(x_joint / len2 * (x_in - x_joint) / len3 + y_joint / len2 * (y_in - y_joint) / len3) / M_PI * 180;

        if (includeThird) {
            q_ik[3] = acos((x_in - x_joint) / len3 * (x - x_in) / len4 + (y_in - y_joint) / len3 * (y - y_in) / len4) / M_PI * 180;

            if (qLowerLimits[1] <= q_ik[1] && q_ik[1] <= qUpperLimits[1] && qLowerLimits[2] <= q_ik[2] && q_ik[2] <= qUpperLimits[2] && qLowerLimits[3] <= q_ik[3] && q_ik[3] <= qUpperLimits[3]) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            if (qLowerLimits[1] <= q_ik[1] && q_ik[1] <= qUpperLimits[1] && qLowerLimits[2] <= q_ik[2] && q_ik[2] <= qUpperLimits[2]) {
                return true;
            }
            else {
                return false;
            }
        }
    }
    return false;
}

/*
The function doesThetaExist() finally determines the inverse kinematics for a three-dimensional point
(xCoord, yCoord, zCoord) and returns whether this calculation was successful or not. First the independently
computable setting angle of the first joint at the foot of the robot is determined and stored in q_ik[0].
Then the reachability of the specified point is checked with isThirdJointValid(). If this is not guaranteed,
then the method is terminated with the return false. The inverse kinematics can provide unique results only
under constraints. In this project, the constraint is that the last arm segment should point downwards as
steeply as possible, i.e. the theta angle should be as large as possible. In doesThetaExist() first the areas
of the theta angle are determined which are reachable according to isSecondJointValid(). Within these ranges the
largest possible theta angle is then determined with a numerical procedure, which can be set. This is how the
arm pose is determined
*/
bool InverseKinematic::doesThetaExist(float xCoord, float yCoord, float zCoord) {
    rad_ik = sqrt(xCoord * xCoord + yCoord * yCoord) - len1;
    zCoord -= len0;

    // calculate the angle of the first joint, beacuse the calculation can be done independent:
    q_ik[0] = acos(yCoord / (rad_ik + len1)) / M_PI * 180.0f;
    if (xCoord < 0) {
        q_ik[0] = -q_ik[0];
    }

    if (q_ik[0] < qLowerLimits[0] || q_ik[0] > qUpperLimits[0]) {
        return false;
    }

    q_ik[4] = 0.0f; // the angle value of the end effector joint is set to 0 for the time being.
                    // The actual calculation of the value takes place during path planning

    if (isThirdJointValid(rad_ik, zCoord)) { // check if point (xCoord, yCoord, zCoord) is reachable
        float* circleAngles = new float[0];
        intersectCounter = 0;

        // Calculate the intervals within which the theta angle can theoretically be set.
        // Store these intervals in the dynamic array circleAngles:
        for (int i = 0; i < 4; i++) {
            x_diff = circleCentersX[i] - rad_ik;
            y_diff = circleCentersY[i] - zCoord;

            jointDistance = sqrt(x_diff * x_diff + y_diff * y_diff);

            if (jointDistance <= circleRadius[i] + len4 && jointDistance >= abs(circleRadius[i] - len4)) {
                x_mid = (jointDistance * jointDistance + len4 * len4 - circleRadius[i] * circleRadius[i]) / (2 * jointDistance);
                y_mid = sqrt(-jointDistance * jointDistance * jointDistance * jointDistance + 2 * jointDistance * jointDistance * (len4 * len4 + circleRadius[i] * circleRadius[i]) - (len4 * len4 - circleRadius[i] * circleRadius[i]) * (len4 * len4 - circleRadius[i] * circleRadius[i])) / (2 * jointDistance);

                if (x_mid >= 0) {
                    angleDiff1 = 90 - acos(y_mid / len4) / M_PI * 180;
                }
                else {
                    angleDiff1 = 90 + acos(y_mid / len4) / M_PI * 180;
                }

                x_mid -= jointDistance;
                if (x_mid >= 0) {
                    angleDiff2 = 90 + acos(y_mid / circleRadius[i]) / M_PI * 180;
                }
                else {
                    angleDiff2 = 90 - acos(y_mid / circleRadius[i]) / M_PI * 180;
                }

                angleCircle1 = acos(y_diff / jointDistance) / M_PI * 180;
                if (x_diff < 0) {
                    angleCircle1 = -angleCircle1;
                }

                angleCircle2 = acos(-y_diff / jointDistance) / M_PI * 180;
                if (x_diff > 0) {
                    angleCircle2 = -angleCircle2;
                }

                intersectAngles[0] = angleCircle1 - angleDiff1;
                intersectAngles[1] = angleCircle2 + angleDiff2;
                intersectAngles[2] = angleCircle1 + angleDiff1;
                intersectAngles[3] = angleCircle2 - angleDiff2;

                for (int j = 0; j < 4; j++) {
                    if (intersectAngles[j] < -180) {
                        while (intersectAngles[j] < -180)
                        {
                            intersectAngles[j] += 360.0f;
                        }
                    }
                    else if (intersectAngles[j] >= 180) {
                        while (intersectAngles[j] >= 180)
                        {
                            intersectAngles[j] -= 360.0f;
                        }
                    }
                }

                if (i != 2) {
                    if (-180 <= intersectAngles[0] && intersectAngles[0] <= 0.0 && circleRangesLower[i] <= intersectAngles[1] && intersectAngles[1] <= circleRangesUpper[i]) {
                        circleAngles = increaseSize(circleAngles, intersectCounter, 1);
                        circleAngles[intersectCounter] = intersectAngles[0];
                        intersectCounter++;
                    }

                    if (-180 <= intersectAngles[2] && intersectAngles[2] <= 0.0 && circleRangesLower[i] <= intersectAngles[3] && intersectAngles[3] <= circleRangesUpper[i]) {
                        circleAngles = increaseSize(circleAngles, intersectCounter, 1);
                        circleAngles[intersectCounter] = intersectAngles[2];
                        intersectCounter++;
                    }
                }
                else {
                    if (-180 <= intersectAngles[0] && intersectAngles[0] <= 0.0 && !(circleRangesLower[i] < intersectAngles[1] && intersectAngles[1] < circleRangesUpper[i])) {
                        circleAngles = increaseSize(circleAngles, intersectCounter, 1);
                        circleAngles[intersectCounter] = intersectAngles[0];
                        intersectCounter++;
                    }

                    if (-180 <= intersectAngles[2] && intersectAngles[2] <= 0.0 && !(circleRangesLower[i] < intersectAngles[3] && intersectAngles[3] < circleRangesUpper[i])) {
                        circleAngles = increaseSize(circleAngles, intersectCounter, 1);
                        circleAngles[intersectCounter] = intersectAngles[2];
                        intersectCounter++;
                    }
                }
            }
        }
        circleAngles = sortArray(circleAngles, intersectCounter);

        firstAdded = false;
        if (intersectCounter == 0) {
            if (isSecondJointValid(rad_ik, zCoord + len4, false, 1.0)) {
                circleAngles = increaseSize(circleAngles, 0, 2);
                circleAngles[0] = 0.0f;
                circleAngles[1] = -180.0f;
            }
        }
        else {
            if (isSecondJointValid(rad_ik, zCoord + len4, false, 1.0)) {
                circleAngles = insertAtBegin(circleAngles, intersectCounter);
                circleAngles[0] = 0.0f;
                intersectCounter++;
                firstAdded = true;
            }
            if (isSecondJointValid(rad_ik, zCoord - len4, false, 1.0)) {
                circleAngles = increaseSize(circleAngles, intersectCounter, 1);
                circleAngles[intersectCounter] = 180.0f;
                intersectCounter++;
            }
        }

        if (intersectCounter > 0) {
            // now calculate the biggest valid Theta angle. This ensures that objects on the floor surface can be gripped in all possible cases:
            for (int i = 0; i < intersectCounter - 1; i++) {
                startAngle = circleAngles[i];
                endAngle = circleAngles[i + 1];
                if (abs(startAngle - endAngle) > 0.01f)
                {
                    if (isSecondJointValid(rad_ik + len4 * sin((startAngle + endAngle) / 360.0f * M_PI), zCoord + len4 * cos((startAngle + endAngle) / 360.0f * M_PI), false, 1.0f)) {
                        if (i > 0 || (i == 0 && !firstAdded)) {
//                            startAngle -= epsilon3;
                            startAngle -= 0.001f;
                        }
                        firstIteration = true;
                        while (startAngle >= endAngle) {
                            if (isSecondJointValid(rad_ik, zCoord, true, startAngle)) {
                                if (!firstIteration) {
                                    upperBorder = startAngle + stepSize;
                                    lowerBorder = startAngle;
                                    for (int j = 0; j <= 5; j++) {
                                        if (isSecondJointValid(rad_ik, zCoord, true, ((upperBorder + lowerBorder) / 2.0f))) {
                                            lowerBorder = (upperBorder + lowerBorder) / 2.0f;
                                        }
                                        else {
                                            upperBorder = (upperBorder + lowerBorder) / 2.0f;
                                        }
                                    }
                                    delete[] circleAngles;
                                    return isSecondJointValid(rad_ik, zCoord, true, lowerBorder);
                                }
                                else {
                                    delete[] circleAngles;
                                    return true;
                                }
                            }
                            startAngle -= stepSize;
                            firstIteration = false;
                        }
                    }
                }
            }
        }
        delete[] circleAngles;
    }
    return false;
}

// write the information in q_ik into the pointer qVal, which is the input patameter:
void InverseKinematic::writeQValues(float* qVal) {
    for (int i = 0; i < 5; i++) {
        qVal[i] = q_ik[i];
    }
}
