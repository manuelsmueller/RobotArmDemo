/*
 * Adversaries.cpp
 *
 *  Created on: 05.12.2022
 *      Author: manuel
 */

#include "Adversaries.h"

Adversaries::Adversaries() {
	// TODO Auto-generated constructor stub
	gv=GlobalVariables::get_instance();
	pp=PathPlanner::get_instance();
}

Adversaries::~Adversaries() {
	// TODO Auto-generated destructor stub
}

/*
 * singelton pattern
 */
Adversaries* Adversaries::adv = 0;
Adversaries* Adversaries::get_instance(){
	static bool isInit=false;

//	InverseKinematic *ik;
	if(!isInit){
		adv  = new Adversaries();
		isInit=true;
	}
	return adv;
}



void Adversaries::loc_adversary(float action_space_dist, float action_space_angle, int transportedWorkpiece, bool prognosis) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

    gv->distToStartColl[0] = cd->gjk(&gv->colliders[12], &gv->safetyCollidersForMTs[0]);
    gv->distToStartColl[1] = cd->gjk(&gv->colliders[13], &gv->safetyCollidersForMTs[0]);
    gv->distToStartColl[2] = cd->gjk(&gv->colliders[14], &gv->safetyCollidersForMTs[0]);
    gv->distToEndColl[0] = cd->gjk(&gv->colliders[12], &gv->safetyCollidersForMTs[1]);
    gv->distToEndColl[1] = cd->gjk(&gv->colliders[13], &gv->safetyCollidersForMTs[1]);
    gv->distToEndColl[2] = cd->gjk(&gv->colliders[14], &gv->safetyCollidersForMTs[1]);

    if (min(gv->distToStartColl[0], gv->distToEndColl[0]) < 25.0f) gv->obsLocError[0] = true;
    else gv->obsLocError[0] = false;

    if (min(gv->distToStartColl[1], gv->distToEndColl[1]) < 25.0f) gv->obsLocError[1] = true;
    else gv->obsLocError[1] = false;

    if (min(gv->distToStartColl[2], gv->distToEndColl[2]) < 25.0f) gv->obsLocError[2] = true;
    else gv->obsLocError[2] = false;

    for (int it = 0; it <= 550; it++) {
        for (int n = 0; n < 3; n++) {
            if (gv->obsLocError[n]) {
                if (!prognosis) {
                    if (n == 0) {
                        gv->messageToSendML = "loc1,";
                    }
                    else if (n == 1) {
                        gv->messageToSendML = "loc2,";
                    }
                    else {
                        gv->messageToSendML = "loc3,";
                    }
                }
                else {
                    if (n == 0) {
                        gv->messageToSendML = "loc1_p,";
                    }
                    else if (n == 1) {
                        gv->messageToSendML = "loc2_p,";
                    }
                    else {
                        gv->messageToSendML = "loc3_p,";
                    }
                }

                if (it == 0) {
                    gv->messageToSendML += "1,";
                }
                else {
                    gv->messageToSendML += "0,";
                }

                gv->messageToSendML += hf->to_string_with_precision(gv->colliders[8].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[8].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[8].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[8].angles[0]), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->colliders[9].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[9].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[9].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[9].angles[0]), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->colliders[10].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[10].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[10].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[10].angles[0]), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->colliders[11].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[11].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[11].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[11].angles[0]), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->colliders[12].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[12].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[12].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[12].angles[0]), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->colliders[13].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[13].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[13].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[13].angles[0]), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->colliders[14].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[14].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[14].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[14].angles[0]), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->colliders[15].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[15].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[15].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[15].angles[0]), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->colliders[16].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[16].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[16].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[16].angles[0]), 4) + ",";

                gv->messageToSendML += hf->to_string_with_precision(action_space_dist, 4) + "," + hf->to_string_with_precision(sin(action_space_angle), 4) + "," + hf->to_string_with_precision(cos(action_space_angle), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->distortions[0], 4) + "," + hf->to_string_with_precision(gv->distortions[1], 4) + "," + hf->to_string_with_precision(sin(gv->distortions[2]), 4) + "," + hf->to_string_with_precision(cos(gv->distortions[2]), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->distortions[3], 4) + "," + hf->to_string_with_precision(gv->distortions[4], 4) + "," + hf->to_string_with_precision(sin(gv->distortions[5]), 4) + "," + hf->to_string_with_precision(cos(gv->distortions[5]), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->distortions[6], 4) + "," + hf->to_string_with_precision(gv->distortions[7], 4) + "," + hf->to_string_with_precision(sin(gv->distortions[8]), 4) + "," + hf->to_string_with_precision(cos(gv->distortions[8]), 4) + ",";

                if (transportedWorkpiece == 0) {
                    gv->messageToSendML += "1,0,0,0,";
                }
                else if (transportedWorkpiece == 1) {
                    gv->messageToSendML += "0,1,0,0,";
                }
                else if (transportedWorkpiece == 2) {
                    gv->messageToSendML += "0,0,1,0,";
                }
                else if (transportedWorkpiece == 3) {
                    gv->messageToSendML += "0,0,0,1,";
                }
                gv->messageToSendML += hf->to_string_with_precision(sin(gv->grippingAngleDiff / 180.0f * M_PI), 4) + "," + hf->to_string_with_precision(cos(gv->grippingAngleDiff / 180.0f * M_PI), 4) + ",";
                gv->messageToSendML += hf->to_string_with_precision(gv->pathEndPos.x / 55.0f, 4) + "," + hf->to_string_with_precision(gv->pathEndPos.y / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->endObjectAngle), 4) + "," + hf->to_string_with_precision(cos(gv->endObjectAngle), 4) + ",";

                if (n == 0) {
                    gv->messageToSendML += "1,0,0,";
                }
                else if (n == 1) {
                    gv->messageToSendML += "0,1,0,";
                }
                else {
                    gv->messageToSendML += "0,0,1,";
                }

                if (gv->distToStartColl[n] <= gv->distToEndColl[n] && gv->distToStartColl[n] <= 25.0f) {
                    gv->locDiffMax = -10.0e5;
                    gv->iMax = 0; gv->jMax = 0, gv->angleMax = 0;

                    for (int i = -1; i <= 1; i++) {
                        for (int j = -1; j <= 1; j++) {
                            for (int k = -1; k <= 1; k++) {
                                if ((i == 0 && j == 0 && k != 0) || (i == 0 && j != 0 && k == 0) || (i != 0 && j == 0 && k == 0) || (i == 0 && j == 0 && k == 0)) {
                                    if (n == 0) {
                                        gv->locDiffX = gv->distortions[0] + i * 0.01f;
                                        gv->locDiffY = gv->distortions[1] + j * 0.01f;
                                        gv->locDiffAngle = gv->distortions[2] + k * 0.1f / 180.0f * M_PI;
                                    }
                                    else if (n == 1) {
                                        gv->locDiffX = gv->distortions[3] + i * 0.01f;
                                        gv->locDiffY = gv->distortions[4] + j * 0.01f;
                                        gv->locDiffAngle = gv->distortions[5] + k * 0.1f / 180.0f * M_PI;
                                    }
                                    else {
                                        gv->locDiffX = gv->distortions[6] + i * 0.01f;
                                        gv->locDiffY = gv->distortions[7] + j * 0.01f;
                                        gv->locDiffAngle = gv->distortions[8] + k * 0.1f / 180.0f * M_PI;
                                    }

                                    if (sqrt(gv->locDiffX * gv->locDiffX + gv->locDiffY * gv->locDiffY) <= action_space_dist && abs(gv->locDiffAngle) <= action_space_angle) {
                                        cd->recalculateCollider(&gv->distortedObstacles[n], gv->colliders[n + 12].offsets[0] + gv->locDiffX, gv->colliders[n + 12].offsets[1] + gv->locDiffY, gv->colliders[n + 12].offsets[2], gv->colliders[n + 12].angles[0] + gv->locDiffAngle, 0, 0);

                                        gv->locDiffTemp = cd->gjk(&gv->distortedObstacles[n], &gv->safetyCollidersForMTs[0]);
                                        if (gv->locDiffTemp > gv->locDiffMax) {
                                            gv->locDiffMax = gv->locDiffTemp;
                                            gv->iMax = i; gv->jMax = j, gv->angleMax = k;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (gv->locDiffMax < cd->epsilon3) {
                        gv->iMax = 0; gv->jMax = 0, gv->angleMax = 0;
                    }

                    gv->messageToSendML += hf->to_string_with_precision(float(gv->iMax), 4) + "," + hf->to_string_with_precision(float(gv->jMax), 4) + "," + hf->to_string_with_precision(float(gv->angleMax), 4) + ",";
                }
                else if (gv->distToEndColl[n] <= gv->distToStartColl[n] && gv->distToEndColl[n] <= 25.0f) {
                    gv->locDiffMax = -10.0e5;
                    gv->iMax = 0; gv->jMax = 0, gv->angleMax = 0;

                    for (int i = -1; i <= 1; i++) {
                        for (int j = -1; j <= 1; j++) {
                            for (int k = -1; k <= 1; k++) {
                                if ((i == 0 && j == 0 && k != 0) || (i == 0 && j != 0 && k == 0) || (i != 0 && j == 0 && k == 0) || (i == 0 && j == 0 && k == 0)) {
                                    if (n == 0) {
                                        gv->locDiffX = gv->distortions[0] + i * 0.01f;
                                        gv->locDiffY = gv->distortions[1] + j * 0.01f;
                                        gv->locDiffAngle = gv->distortions[2] + k * 0.1f / 180.0f * M_PI;
                                    }
                                    else if (n == 1) {
                                        gv->locDiffX = gv->distortions[3] + i * 0.01f;
                                        gv->locDiffY = gv->distortions[4] + j * 0.01f;
                                        gv->locDiffAngle = gv->distortions[5] + k * 0.1f / 180.0f * M_PI;
                                    }
                                    else {
                                        gv->locDiffX = gv->distortions[6] + i * 0.01f;
                                        gv->locDiffY = gv->distortions[7] + j * 0.01f;
                                        gv->locDiffAngle = gv->distortions[8] + k * 0.1f / 180.0f * M_PI;
                                    }

                                    if (sqrt(gv->locDiffX * gv->locDiffX + gv->locDiffY * gv->locDiffY) <= action_space_dist && abs(gv->locDiffAngle) <= action_space_angle) {
                                        cd->recalculateCollider(&gv->distortedObstacles[n], gv->colliders[n + 12].offsets[0] + gv->locDiffX, gv->colliders[n + 12].offsets[1] + gv->locDiffY, gv->colliders[n + 12].offsets[2], gv->colliders[n + 12].angles[0] + gv->locDiffAngle, 0, 0);

                                        gv->locDiffTemp = cd->gjk(&gv->distortedObstacles[n], &gv->safetyCollidersForMTs[1]);
                                        if (gv->locDiffTemp > gv->locDiffMax) {
                                            gv->locDiffMax = gv->locDiffTemp;
                                            gv->iMax = i; gv->jMax = j, gv->angleMax = k;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (gv->locDiffMax < cd->epsilon3) {
                        gv->iMax = 0; gv->jMax = 0, gv->angleMax = 0;
                    }

                    gv->messageToSendML += hf->to_string_with_precision(float(gv->iMax), 4) + "," + hf->to_string_with_precision(float(gv->jMax), 4) + "," + hf->to_string_with_precision(float(gv->angleMax), 4) + ",";
                }
                else {
                    gv->iMax = 0; gv->jMax = 0, gv->angleMax = 0;
                    gv->messageToSendML += "0,0,0,";
                }

                if (it == 550) {
                    if (n == 0) {
                        gv->messageToSendML += hf->to_string_with_precision(gv->reward1, 4) + ",1";
                    }
                    else if (n == 1) {
                        gv->messageToSendML += hf->to_string_with_precision(gv->reward2, 4) + ",1";
                    }
                    else {
                        gv->messageToSendML += hf->to_string_with_precision(gv->reward3, 4) + ",1";
                    }
//#if defined(WIN32)
                     sendMessageML(gv->messageToSendML);
                     receiveMessageML();
//#endif
                }
                else {
                    if (it != 0 && n == 0) gv->messageToSendML += hf->to_string_with_precision(gv->reward1, 4) + ",0";
                    if (it != 0 && n == 1) gv->messageToSendML += hf->to_string_with_precision(gv->reward2, 4) + ",0";
                    if (it != 0 && n == 2) gv->messageToSendML += hf->to_string_with_precision(gv->reward3, 4) + ",0";
//#if defined(WIN32)
                     sendMessageML(gv->messageToSendML);

                     gv->receivedAnswerML =  receiveMessageML();
                    gv->action = stoi( gv->receivedAnswerML);
//#endif
                    if (gv->action == 0) {
                        gv->iMin = 0;
                        gv->jMin = 0;
                        gv->angleMin = 0;
                    }
                    else if (gv->action == 1) {
                        gv->iMin = 1;
                        gv->jMin = 0;
                        gv->angleMin = 0;
                    }
                    else if (gv->action == 2) {
                        gv->iMin = -1;
                        gv->jMin = 0;
                        gv->angleMin = 0;
                    }
                    else if (gv->action == 3) {
                        gv->iMin = 0;
                        gv->jMin = 1;
                        gv->angleMin = 0;
                    }
                    else if (gv->action == 4) {
                        gv->iMin = 0;
                        gv->jMin = -1;
                        gv->angleMin = 0;
                    }
                    else if (gv->action == 5) {
                        gv->iMin = 0;
                        gv->jMin = 0;
                        gv->angleMin = 1;
                    }
                    else if (gv->action == 6) {
                        gv->iMin = 0;
                        gv->jMin = 0;
                        gv->angleMin = -1;
                    }

                    if (gv->iMax == gv->iMin && gv->jMax == gv->jMin && gv->angleMax == gv->angleMin) {
                        if (n == 0) gv->reward1 = 1.0f;
                        else if (n == 1) gv->reward2 = 1.0f;
                        else gv->reward3 = 1.0f;
                    }
                    else {
                        if (n == 0) gv->reward1 = -1.0f;
                        else if (n == 1) gv->reward2 = -1.0f;
                        else gv->reward3 = -1.0f;
                    }

                    bool changed = false;

                    if (n == 0 && gv->action != 0) {
                        gv->distortions[0] += gv->iMin * 0.01f;
                        gv->distortions[1] += gv->jMin * 0.01f;
                        gv->distortions[2] += gv->angleMin * 0.1f / 180.0f * M_PI;

                        if (sqrt(gv->distortions[0] * gv->distortions[0] + gv->distortions[1] * gv->distortions[1]) <= action_space_dist && abs(gv->distortions[2]) <= action_space_angle) {
                            cd->recalculateCollider(&gv->distortedObstacles[0], gv->colliders[12].offsets[0] + gv->distortions[0], gv->colliders[12].offsets[1] + gv->distortions[1], gv->colliders[12].offsets[2], gv->colliders[12].angles[0] + gv->distortions[2], 0, 0);
                            changed = true;
                        }
                        else {
                            gv->distortions[0] -= gv->iMin * 0.01f;
                            gv->distortions[1] -= gv->jMin * 0.01f;
                            gv->distortions[2] -= gv->angleMin * 0.1f / 180.0f * M_PI;
                        }
                    }

                    if (n == 1 && gv->action != 0) {
                        gv->distortions[3] += gv->iMin * 0.01f;
                        gv->distortions[4] += gv->jMin * 0.01f;
                        gv->distortions[5] += gv->angleMin * 0.1f / 180.0f * M_PI;

                        if (sqrt(gv->distortions[3] * gv->distortions[3] + gv->distortions[4] * gv->distortions[4]) <= action_space_dist && abs(gv->distortions[5]) <= action_space_angle) {
                            cd->recalculateCollider(&gv->distortedObstacles[1], gv->colliders[13].offsets[0] + gv->distortions[3], gv->colliders[13].offsets[1] + gv->distortions[4], gv->colliders[13].offsets[2], gv->colliders[13].angles[0] + gv->distortions[5], 0, 0);
                            changed = true;
                        }
                        else {
                            gv->distortions[3] -= gv->iMin * 0.01f;
                            gv->distortions[4] -= gv->jMin * 0.01f;
                            gv->distortions[5] -= gv->angleMin * 0.1f / 180.0f * M_PI;
                        }
                    }

                    if (n == 2 && gv->action != 0) {
                        gv->distortions[6] += gv->iMin * 0.01f;
                        gv->distortions[7] += gv->jMin * 0.01f;
                        gv->distortions[8] += gv->angleMin * 0.1f / 180.0f * M_PI;

                        if (sqrt(gv->distortions[6] * gv->distortions[6] + gv->distortions[7] * gv->distortions[7]) <= action_space_dist && abs(gv->distortions[8]) <= action_space_angle) {
                            cd->recalculateCollider(&gv->distortedObstacles[2], gv->colliders[14].offsets[0] + gv->distortions[6], gv->colliders[14].offsets[1] + gv->distortions[7], gv->colliders[14].offsets[2], gv->colliders[14].angles[0] + gv->distortions[8], 0, 0);
                            changed = true;
                        }
                        else {
                            gv->distortions[6] -= gv->iMin * 0.01f;
                            gv->distortions[7] -= gv->jMin * 0.01f;
                            gv->distortions[8] -= gv->angleMin * 0.1f / 180.0f * M_PI;
                        }
                    }

                    if (gv->writeDataIntoFiles) {
                        fstream file;
                        file.open("loc_actions_" + to_string(n) + "_" + to_string(gv->simCounter) + ".csv", std::ios_base::app);
                        if (file) {
                            if (changed) {
                                file << to_string(gv->action) + "\n";
                            }
                            else {
                                file << "0\n";
                            }
                        }
                        else {
                            cout << "Error: writing into file not possible!" << endl;
                        }
                        file.close();
                    }
                }
            }
            else {
                if (gv->writeDataIntoFiles && it < 550) {
                    fstream file;
                    file.open("loc_actions_" + to_string(n) + "_" + to_string(gv->simCounter) + ".csv", std::ios_base::app);
                    if (file) {
                        file << "0\n";
                    }
                    else {
                        cout << "Error: writing into file not possible!" << endl;
                    }
                    file.close();
                }
            }
        }
    }
}

void Adversaries::path_adversary(bool firstCall, bool lastCall, int transportedWorkpiece, bool prognosis) {
	CollisionDetection *cd=CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

    gv->posCurrent[0] = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Sin;
    gv->posCurrent[1] = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Cos;
    gv->posCurrent[2] = ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->len4 * gv->q123Cos;

    gv->posTarget[0] = (ik->len1 + ik->len2 * sin(gv->qValuesTarget[1] / 180.0f * M_PI) + ik->len3 * sin((gv->qValuesTarget[1] + gv->qValuesTarget[2]) / 180.0f * M_PI) + ik->len4 * sin((gv->qValuesTarget[1] + gv->qValuesTarget[2] + gv->qValuesTarget[3]) / 180.0f * M_PI)) * sin(gv->qValuesTarget[0] / 180.0f * M_PI);
    gv->posTarget[1] = (ik->len1 + ik->len2 * sin(gv->qValuesTarget[1] / 180.0f * M_PI) + ik->len3 * sin((gv->qValuesTarget[1] + gv->qValuesTarget[2]) / 180.0f * M_PI) + ik->len4 * sin((gv->qValuesTarget[1] + gv->qValuesTarget[2] + gv->qValuesTarget[3]) / 180.0f * M_PI)) * cos(gv->qValuesTarget[0] / 180.0f * M_PI);
    gv->posTarget[2] = ik->len0 + ik->len2 * cos(gv->qValuesTarget[1] / 180.0f * M_PI) + ik->len3 * cos((gv->qValuesTarget[1] + gv->qValuesTarget[2]) / 180.0f * M_PI) + ik->len4 * cos((gv->qValuesTarget[1] + gv->qValuesTarget[2] + gv->qValuesTarget[3]) / 180.0f * M_PI);

    if (pp->nodeIndex != 0) {
        for (int i = 0; i < 5; i++) {
           qValuesNext[i] = pp->nodeList[pp->nodeList[pp->nodeIndex].previous].nodeQVal[i];
        }

        gv->posNext[0] = (ik->len1 + ik->len2 * sin(qValuesNext[1] / 180.0f * M_PI) + ik->len3 * sin((qValuesNext[1] +qValuesNext[2]) / 180.0f * M_PI) + ik->len4 * sin((qValuesNext[1] +qValuesNext[2] +qValuesNext[3]) / 180.0f * M_PI)) * sin(qValuesNext[0] / 180.0f * M_PI);
        gv->posNext[1] = (ik->len1 + ik->len2 * sin(qValuesNext[1] / 180.0f * M_PI) + ik->len3 * sin((qValuesNext[1] +qValuesNext[2]) / 180.0f * M_PI) + ik->len4 * sin((qValuesNext[1] +qValuesNext[2] +qValuesNext[3]) / 180.0f * M_PI)) * cos(qValuesNext[0] / 180.0f * M_PI);
        gv->posNext[2] = ik->len0 + ik->len2 * cos(qValuesNext[1] / 180.0f * M_PI) + ik->len3 * cos((qValuesNext[1] +qValuesNext[2]) / 180.0f * M_PI) + ik->len4 * cos((qValuesNext[1] +qValuesNext[2] +qValuesNext[3]) / 180.0f * M_PI);

    }
    else {
       qValuesNext[4] = 0.0f;

        gv->posNext[0] = 0.0f;
        gv->posNext[1] = 0.0f;
        gv->posNext[2] = 0.0f;
    }

    gv->tempAdvAngle = gv->qValuesTarget[4];

    if (!prognosis) {
        gv->messageToSendML = "path,";
    }
    else {
        gv->messageToSendML = "path_p,";
    }

    if (firstCall) {
        gv->messageToSendML += "1,";
    }
    else {
        gv->messageToSendML += "0,";
    }

    gv->messageToSendML += hf->to_string_with_precision(gv->posCurrent[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posCurrent[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posCurrent[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->qValuesCurrent[4]), 4) + "," + hf->to_string_with_precision(cos(gv->qValuesCurrent[4]), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->posTarget[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posTarget[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posTarget[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->qValuesTarget[4]), 4) + "," + hf->to_string_with_precision(cos(gv->qValuesTarget[4]), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->posNext[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posNext[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posNext[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(qValuesNext[4]), 4) + "," + hf->to_string_with_precision(cos(qValuesNext[4]), 4) + ",";

    gv->messageToSendML += hf->to_string_with_precision(gv->colliders[8].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[8].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[8].offsets[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[8].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[8].angles[0]), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->colliders[9].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[9].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[9].offsets[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[9].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[9].angles[0]), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->colliders[10].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[10].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[10].offsets[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[10].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[10].angles[0]), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->colliders[11].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[11].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[11].offsets[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[11].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[11].angles[0]), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->colliders[12].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[12].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[12].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[12].angles[0]), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->colliders[13].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[13].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[13].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[13].angles[0]), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->colliders[14].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[14].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[14].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[14].angles[0]), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->colliders[15].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[15].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[15].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[15].angles[0]), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->colliders[16].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[16].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[16].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[16].angles[0]), 4) + ",";

    gv->messageToSendML += hf->to_string_with_precision(gv->adv_dev_x, 4) + "," + hf->to_string_with_precision(gv->adv_dev_y, 4) + "," + hf->to_string_with_precision(gv->adv_dev_z, 4) + ",";

    if (transportedWorkpiece == 0) {
        gv->messageToSendML += "1,0,0,0,";
    }
    else if (transportedWorkpiece == 1) {
        gv->messageToSendML += "0,1,0,0,";
    }
    else if (transportedWorkpiece == 2) {
        gv->messageToSendML += "0,0,1,0,";
    }
    else if (transportedWorkpiece == 3) {
        gv->messageToSendML += "0,0,0,1,";
    }
    gv->messageToSendML += hf->to_string_with_precision(sin(gv->grippingAngleDiff / 180.0f * M_PI), 4) + "," + hf->to_string_with_precision(cos(gv->grippingAngleDiff / 180.0f * M_PI), 4) + ",";
    gv->messageToSendML += hf->to_string_with_precision(gv->pathEndPos.x / 55.0f, 4) + "," + hf->to_string_with_precision(gv->pathEndPos.y / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->endObjectAngle), 4) + "," + hf->to_string_with_precision(cos(gv->endObjectAngle), 4) + ",";

    gv->collision_init = false;

    pp->maxJointDiff = -1.0f;
    for (int i = 0; i < 5; i++) {
        if (abs(gv->qValuesTarget[i] - gv->qValuesCurrent[i]) > pp->maxJointDiff) {
            pp->maxJointDiff = abs(gv->qValuesTarget[i] - gv->qValuesCurrent[i]);
        }
    }
    pp->sampleCount = (int)(floor(pp->maxJointDiff / 1.5f));

    gv->minAdvDist = 13.5f;

    for (int m = 0; m <= pp->sampleCount + 1; m++) {
        for (int n = 0; n < 5; n++) {
            pp->qValInter[n] = (gv->qValuesTarget[n] - gv->qValuesCurrent[n]) * float(m) / float(pp->sampleCount + 1) + gv->qValuesCurrent[n];
        }

        gv->q0Sin_pp = sin(pp->qValInter[0] / 180.0f * M_PI);
        gv->q0Cos_pp = cos(pp->qValInter[0] / 180.0f * M_PI);
        gv->q1Sin_pp = sin(pp->qValInter[1] / 180.0f * M_PI);
        gv->q1Cos_pp = cos(pp->qValInter[1] / 180.0f * M_PI);
        gv->q12Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
        gv->q12Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
        gv->q123Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);
        gv->q123Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);

        cd->recalculateCollider(&gv->adversaryCollider[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (90.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

        cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
        cd->rotationMatrix(gv->q0Cos_pp * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
        gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

        cd->recalculateColliderTransport(&gv->adversaryCollider[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);
        cd->recalculateColliderTransport(&gv->adversaryCollider[2], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

        if (gv->transportPhase == 1) {
            cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
            cd->recalculateColliderTransport(&gv->adversaryCollider[3], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
        }

        for (int n = 1; n < gv->collidersCount; n++) {
            if (n == 1 || n == 2 || (n >= 8 && n != gv->objectToTransport)) {
                if (cd->distApproximation(&gv->adversaryCollider[0], &gv->colliders[n]) <= gv->minAdvDist) {
                    gv->tempAdvDist = cd->gjk(&gv->adversaryCollider[0], &gv->colliders[n]);
                    if (gv->tempAdvDist < cd->epsilon3) {
                        gv->collision_init = true;
                        break;
                    }
                    else if (gv->tempAdvDist < gv->minAdvDist) {
                        if (m > 0) {
                            if (sqrt((gv->adv_dev_x - cd->v_direction[0]) * (gv->adv_dev_x - cd->v_direction[0]) + (gv->adv_dev_y - cd->v_direction[1]) * (gv->adv_dev_y - cd->v_direction[1]) + (gv->adv_dev_z - cd->v_direction[2]) * (gv->adv_dev_z - cd->v_direction[2])) * float(pp->sampleCount + 1) / float(m) <= 0.60f * 6.0f) {
                                if (sqrt(cd->v_direction[0] * cd->v_direction[0] +
                                		cd->v_direction[1] * cd->v_direction[1] +
										cd->v_direction[2] * cd->v_direction[2])
                                	* float(pp->sampleCount + 1) / float(m)
                                		<= 0.03f * 1.5f /  pp->simulationStepSize * m * 6.0f) {
                                    gv->minAdvDist = gv->tempAdvDist;

                                    gv->minDistVector[0] = -cd->v_direction[0];
                                    gv->minDistVector[1] = -cd->v_direction[1];
                                    gv->minDistVector[2] = -cd->v_direction[2];
                                }
                            }
                        }
                    }
                }

                if (cd->distApproximation(&gv->adversaryCollider[1], &gv->colliders[n]) <= gv->minAdvDist) {
                    gv->tempAdvDist = cd->gjk(&gv->adversaryCollider[1], &gv->colliders[n]);
                    if (gv->tempAdvDist < cd->epsilon3) {
                        gv->collision_init = true;
                        break;
                    }
                    else if (gv->tempAdvDist < gv->minAdvDist) {
                        if (m > 0) {
                            if (sqrt((gv->adv_dev_x - cd->v_direction[0]) * (gv->adv_dev_x - cd->v_direction[0]) + (gv->adv_dev_y - cd->v_direction[1]) * (gv->adv_dev_y - cd->v_direction[1]) + (gv->adv_dev_z - cd->v_direction[2]) * (gv->adv_dev_z - cd->v_direction[2])) * float(pp->sampleCount + 1) / float(m) <= 0.60f * 6.0f) {
                                if (sqrt(cd->v_direction[0] * cd->v_direction[0] + cd->v_direction[1] * cd->v_direction[1] + cd->v_direction[2] * cd->v_direction[2]) * float(pp->sampleCount + 1) / float(m) <= 0.03f * 1.5f / pp->simulationStepSize * m * 6.0f) {
                                    gv->minAdvDist = gv->tempAdvDist;

                                    gv->minDistVector[0] = -cd->v_direction[0];
                                    gv->minDistVector[1] = -cd->v_direction[1];
                                    gv->minDistVector[2] = -cd->v_direction[2];
                                }
                            }
                        }
                    }
                }

                if (cd->distApproximation(&gv->adversaryCollider[2], &gv->colliders[n]) <= gv->minAdvDist) {
                    gv->tempAdvDist = cd->gjk(&gv->adversaryCollider[2], &gv->colliders[n]);
                    if (gv->tempAdvDist < cd->epsilon3) {
                        gv->collision_init = true;
                        break;
                    }
                    else if (gv->tempAdvDist < gv->minAdvDist) {
                        if (m > 0) {
                            if (sqrt((gv->adv_dev_x - cd->v_direction[0]) * (gv->adv_dev_x - cd->v_direction[0]) + (gv->adv_dev_y - cd->v_direction[1]) * (gv->adv_dev_y - cd->v_direction[1]) + (gv->adv_dev_z - cd->v_direction[2]) * (gv->adv_dev_z - cd->v_direction[2])) * float(pp->sampleCount + 1) / float(m) <= 0.60f * 6.0f) {
                                if (sqrt(cd->v_direction[0] * cd->v_direction[0] + cd->v_direction[1] * cd->v_direction[1] + cd->v_direction[2] * cd->v_direction[2]) * float(pp->sampleCount + 1) / float(m) <= 0.03f * 1.5f / pp->simulationStepSize * m * 6.0f) {
                                    gv->minAdvDist = gv->tempAdvDist;

                                    gv->minDistVector[0] = -cd->v_direction[0];
                                    gv->minDistVector[1] = -cd->v_direction[1];
                                    gv->minDistVector[2] = -cd->v_direction[2];
                                }
                            }
                        }
                    }
                }

                if (gv->transportPhase == 1) {
                    if (cd->distApproximation(&gv->adversaryCollider[3], &gv->colliders[n]) <= gv->minAdvDist) {
                        gv->tempAdvDist = cd->gjk(&gv->adversaryCollider[3], &gv->colliders[n]);
                        if (gv->tempAdvDist < cd->epsilon3) {
                            gv->collision_init = true;
                            break;
                        }
                        else if (gv->tempAdvDist < gv->minAdvDist) {
                            if (m > 0) {
                                if (sqrt((gv->adv_dev_x - cd->v_direction[0]) * (gv->adv_dev_x - cd->v_direction[0]) + (gv->adv_dev_y - cd->v_direction[1]) * (gv->adv_dev_y - cd->v_direction[1]) + (gv->adv_dev_z - cd->v_direction[2]) * (gv->adv_dev_z - cd->v_direction[2])) * float(pp->sampleCount + 1) / float(m) <= 0.60f * 6.0f) {
                                    if (sqrt(cd->v_direction[0] * cd->v_direction[0] + cd->v_direction[1] * cd->v_direction[1] + cd->v_direction[2] * cd->v_direction[2]) * float(pp->sampleCount + 1) / float(m) <= 0.03f * 1.5f / pp->simulationStepSize * m * 6.0f) {
                                        gv->minAdvDist = gv->tempAdvDist;

                                        gv->minDistVector[0] = -cd->v_direction[0];
                                        gv->minDistVector[1] = -cd->v_direction[1];
                                        gv->minDistVector[2] = -cd->v_direction[2];
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if (gv->collision_init) break;
    }

    if (!gv->collision_init && pp->nodeIndex != 0) {
        pp->maxJointDiff = -1.0f;
        for (int i = 0; i < 5; i++) {
            if (abs(qValuesNext[i] - gv->qValuesTarget[i]) > pp->maxJointDiff) {
                pp->maxJointDiff = abs(qValuesNext[i] - gv->qValuesTarget[i]);
            }
        }
        pp->sampleCount = (int)(floor(pp->maxJointDiff / 1.5f));

        for (int m = 0; m <= pp->sampleCount + 1; m++) {
            for (int n = 0; n < 5; n++) {
                pp->qValInter[n] = (qValuesNext[n] - gv->qValuesTarget[n]) * float(m) / float(pp->sampleCount + 1) + gv->qValuesTarget[n];
            }

            gv->q0Sin_pp = sin(pp->qValInter[0] / 180.0f * M_PI);
            gv->q0Cos_pp = cos(pp->qValInter[0] / 180.0f * M_PI);
            gv->q1Sin_pp = sin(pp->qValInter[1] / 180.0f * M_PI);
            gv->q1Cos_pp = cos(pp->qValInter[1] / 180.0f * M_PI);
            gv->q12Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
            gv->q12Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
            gv->q123Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);
            gv->q123Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);

            cd->recalculateCollider(&gv->adversaryCollider[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (90.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

            cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
            cd->rotationMatrix(gv->q0Cos_pp * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
            gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

            cd->recalculateColliderTransport(&gv->adversaryCollider[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);
            cd->recalculateColliderTransport(&gv->adversaryCollider[2], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

            if (gv->transportPhase == 1) {
                cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                cd->recalculateColliderTransport(&gv->adversaryCollider[3], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
            }

            for (int n = 1; n < gv->collidersCount; n++) {
                if (n == 1 || n == 2 || (n >= 8 && n != gv->objectToTransport)) {
                    if (cd->distApproximation(&gv->adversaryCollider[0], &gv->colliders[n]) <= gv->minAdvDist) {
                        gv->tempAdvDist = cd->gjk(&gv->adversaryCollider[0], &gv->colliders[n]);
                        if (gv->tempAdvDist < cd->epsilon3) {
                            gv->collision_init = true;
                            break;
                        }
                        else if (gv->tempAdvDist < gv->minAdvDist) {
                            if (m < pp->sampleCount + 1) {
                                if (sqrt((gv->adv_dev_x - cd->v_direction[0]) * (gv->adv_dev_x - cd->v_direction[0]) + (gv->adv_dev_y - cd->v_direction[1]) * (gv->adv_dev_y - cd->v_direction[1]) + (gv->adv_dev_z - cd->v_direction[2]) * (gv->adv_dev_z - cd->v_direction[2])) * float(pp->sampleCount + 1) / float(pp->sampleCount + 1 - m) <= 0.60f * 6.0f) {
                                    if (sqrt(cd->v_direction[0] * cd->v_direction[0] + cd->v_direction[1] * cd->v_direction[1] + cd->v_direction[2] * cd->v_direction[2]) * float(pp->sampleCount + 1) / float(pp->sampleCount + 1 - m) <= 0.03f * 1.5f / pp->simulationStepSize * m * 6.0f) {
                                        gv->minAdvDist = gv->tempAdvDist;

                                        gv->minDistVector[0] = -cd->v_direction[0];
                                        gv->minDistVector[1] = -cd->v_direction[1];
                                        gv->minDistVector[2] = -cd->v_direction[2];
                                    }
                                }
                            }
                        }
                    }

                    if (cd->distApproximation(&gv->adversaryCollider[1], &gv->colliders[n]) <= gv->minAdvDist) {
                        gv->tempAdvDist = cd->gjk(&gv->adversaryCollider[1], &gv->colliders[n]);
                        if (gv->tempAdvDist < cd->epsilon3) {
                            gv->collision_init = true;
                            break;
                        }
                        else if (gv->tempAdvDist < gv->minAdvDist) {
                            if (m < pp->sampleCount + 1) {
                                if (sqrt((gv->adv_dev_x - cd->v_direction[0]) * (gv->adv_dev_x - cd->v_direction[0]) + (gv->adv_dev_y - cd->v_direction[1]) * (gv->adv_dev_y - cd->v_direction[1]) + (gv->adv_dev_z - cd->v_direction[2]) * (gv->adv_dev_z - cd->v_direction[2])) * float(pp->sampleCount + 1) / float(pp->sampleCount + 1 - m) <= 0.60f * 6.0f) {
                                    if (sqrt(cd->v_direction[0] * cd->v_direction[0] + cd->v_direction[1] * cd->v_direction[1] + cd->v_direction[2] * cd->v_direction[2]) * float(pp->sampleCount + 1) / float(pp->sampleCount + 1 - m) <= 0.03f * 1.5f / pp->simulationStepSize * m * 6.0f) {
                                        gv->minAdvDist = gv->tempAdvDist;

                                        gv->minDistVector[0] = -cd->v_direction[0];
                                        gv->minDistVector[1] = -cd->v_direction[1];
                                        gv->minDistVector[2] = -cd->v_direction[2];
                                    }
                                }
                            }
                        }
                    }

                    if (cd->distApproximation(&gv->adversaryCollider[2], &gv->colliders[n]) <= gv->minAdvDist) {
                        gv->tempAdvDist = cd->gjk(&gv->adversaryCollider[2], &gv->colliders[n]);
                        if (gv->tempAdvDist < cd->epsilon3) {
                            gv->collision_init = true;
                            break;
                        }
                        else if (gv->tempAdvDist < gv->minAdvDist) {
                            if (m < pp->sampleCount + 1) {
                                if (sqrt((gv->adv_dev_x - cd->v_direction[0]) * (gv->adv_dev_x - cd->v_direction[0]) + (gv->adv_dev_y - cd->v_direction[1]) * (gv->adv_dev_y - cd->v_direction[1]) + (gv->adv_dev_z - cd->v_direction[2]) * (gv->adv_dev_z - cd->v_direction[2])) * float(pp->sampleCount + 1) / float(pp->sampleCount + 1 - m) <= 0.60f * 6.0f) {
                                    if (sqrt(cd->v_direction[0] * cd->v_direction[0] + cd->v_direction[1] * cd->v_direction[1] + cd->v_direction[2] * cd->v_direction[2]) * float(pp->sampleCount + 1) / float(pp->sampleCount + 1 - m) <= 0.03f * 1.5f / pp->simulationStepSize * m * 6.0f) {
                                        gv->minAdvDist = gv->tempAdvDist;

                                        gv->minDistVector[0] = -cd->v_direction[0];
                                        gv->minDistVector[1] = -cd->v_direction[1];
                                        gv->minDistVector[2] = -cd->v_direction[2];
                                    }
                                }
                            }
                        }
                    }

                    if (gv->transportPhase == 1) {
                        if (cd->distApproximation(&gv->adversaryCollider[3], &gv->colliders[n]) <= gv->minAdvDist) {
                            gv->tempAdvDist = cd->gjk(&gv->adversaryCollider[3], &gv->colliders[n]);
                            if (gv->tempAdvDist < cd->epsilon3) {
                                gv->collision_init = true;
                                break;
                            }
                            else if (gv->tempAdvDist < gv->minAdvDist) {
                                if (m < pp->sampleCount + 1) {
                                    if (sqrt((gv->adv_dev_x - cd->v_direction[0]) * (gv->adv_dev_x - cd->v_direction[0]) + (gv->adv_dev_y - cd->v_direction[1]) * (gv->adv_dev_y - cd->v_direction[1]) + (gv->adv_dev_z - cd->v_direction[2]) * (gv->adv_dev_z - cd->v_direction[2])) * float(pp->sampleCount + 1) / float(pp->sampleCount + 1 - m) <= 0.60f * 6.0f) {
                                        if (sqrt(cd->v_direction[0] * cd->v_direction[0] + cd->v_direction[1] * cd->v_direction[1] + cd->v_direction[2] * cd->v_direction[2]) * float(pp->sampleCount + 1) / float(pp->sampleCount + 1 - m) <= 0.03f * 1.5f / pp->simulationStepSize * m * 6.0f) {
                                            gv->minAdvDist = gv->tempAdvDist;

                                            gv->minDistVector[0] = -cd->v_direction[0];
                                            gv->minDistVector[1] = -cd->v_direction[1];
                                            gv->minDistVector[2] = -cd->v_direction[2];
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (gv->collision_init) break;
        }
    }

    if (gv->minAdvDist < 13.5f - cd->epsilon4 && !gv->collision_init) {
        gv->advDirectionLength = sqrt(gv->minDistVector[0] * gv->minDistVector[0] + gv->minDistVector[1] * gv->minDistVector[1] + gv->minDistVector[2] * gv->minDistVector[2]);
        if (gv->advDirectionLength > cd->epsilon3) {
            gv->minDistVector[0] = gv->minDistVector[0] / gv->advDirectionLength;
            gv->minDistVector[1] = gv->minDistVector[1] / gv->advDirectionLength;
            gv->minDistVector[2] = gv->minDistVector[2] / gv->advDirectionLength;
        }

        gv->maxDot = -10.0e5;
        gv->iMax = 0; gv->jMax = 0; gv->jMax = 0;
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                for (int k = -1; k <= 1; k++) {
                    if ((i == 0 && j == 0) || (j == 0 && k == 0) || (i == 0 && k == 0)) {
                        if (float(i) * gv->minDistVector[0] + float(j) * gv->minDistVector[1] + float(k) * gv->minDistVector[2] > gv->maxDot) {
                            gv->maxDot = float(i) * gv->minDistVector[0] + float(j) * gv->minDistVector[1] + float(k) * gv->minDistVector[2];
                            gv->iMax = i; gv->jMax = j; gv->kMax = k;
                        }
                    }
                }
            }
        }

        gv->messageToSendML += hf->to_string_with_precision(float(gv->iMax), 4) + "," + hf->to_string_with_precision(float(gv->jMax), 4) + "," + hf->to_string_with_precision(float(gv->kMax), 4) + ",";
    }
    else {
        gv->iMax = 0; gv->jMax = 0; gv->jMax = 0;
        gv->messageToSendML += "0.0,0.0,0.0,";
    }

    if (lastCall) {
        gv->messageToSendML += hf->to_string_with_precision(gv->reward, 4) + ",1";
//#if defined(WIN32)
         sendMessageML(gv->messageToSendML);
         receiveMessageML();
//#endif
    }
    else {
        if (!firstCall) gv->messageToSendML += hf->to_string_with_precision(gv->reward, 4) + ",0";
//#if defined(WIN32)
         sendMessageML(gv->messageToSendML);

         gv->receivedAnswerML =  receiveMessageML();
        gv->action = stoi( gv->receivedAnswerML);
//#endif
        if (gv->action == 0) {
            gv->iMin = 0;
            gv->jMin = 0;
            gv->kMin = 0;
        }
        else if (gv->action == 1) {
            gv->iMin = 1;
            gv->jMin = 0;
            gv->kMin = 0;
        }
        else if (gv->action == 2) {
            gv->iMin = -1;
            gv->jMin = 0;
            gv->kMin = 0;
        }
        else if (gv->action == 3) {
            gv->iMin = 0;
            gv->jMin = 1;
            gv->kMin = 0;
        }
        else if (gv->action == 4) {
            gv->iMin = 0;
            gv->jMin = -1;
            gv->kMin = 0;
        }
        else if (gv->action == 5) {
            gv->iMin = 0;
            gv->jMin = 0;
            gv->kMin = 1;
        }
        else if (gv->action == 6) {
            gv->iMin = 0;
            gv->jMin = 0;
            gv->kMin = -1;
        }

        if (gv->iMax == gv->iMin && gv->jMax == gv->jMin && gv->kMax == gv->kMin) {
            gv->reward = 1.0f;
        }
        else {
            gv->reward = -1.0f;
        }

        bool changed = false;

        if (gv->action != 0 && pp->nodeIndex != 0) {
            if (sqrt((gv->adv_dev_x + gv->iMin * 0.03f) * (gv->adv_dev_x + gv->iMin * 0.03f) + (gv->adv_dev_y + gv->jMin * 0.03f) * (gv->adv_dev_y + gv->jMin * 0.03f) + (gv->adv_dev_z + gv->kMin * 0.03f) * (gv->adv_dev_z + gv->kMin * 0.03f)) <= 0.60f) {
                if (ik->doesThetaExist(gv->posTarget[0] + gv->iMin * 0.03f, gv->posTarget[1] + gv->jMin * 0.03f, gv->posTarget[2] + gv->kMin * 0.03f)) {
                    gv->adv_dev_x += gv->iMin * 0.03f;
                    gv->adv_dev_y += gv->jMin * 0.03f;
                    gv->adv_dev_z += gv->kMin * 0.03f;
                    ik->writeQValues(gv->qValuesTarget);

                    changed = true;

                    gv->qValuesTarget[4] = gv->tempAdvAngle;

                    for (int i = 0; i < 5; i++) {
                        gv->qValuesDiff[i] = gv->qValuesTarget[i] - gv->qValuesCurrent[i];
                    }

                    gv->maxValue = -1.0f;
                    gv->maxValueIndex = -1;
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
        }

        if (gv->writeDataIntoFiles) {
            fstream file;
            file.open("path_actions_" + to_string(gv->simCounter) + ".csv", std::ios_base::app);
            if (file) {
                if (changed) {
                    file << to_string(gv->action) + ";" + to_string(gv->timeCounter) + "\n";
                }
                else {
                    file << "0;" + to_string(gv->timeCounter) + "\n";
                }
            }
            else {
                cout << "Error: writing into file not possible!" << endl;
            }
            file.close();
        }
    }
}

void Adversaries::obs_preparation(int transportedWorkpiece) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

    gv->posCurrent[0] = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Sin;
    gv->posCurrent[1] = (ik->len1 + ik->len2 * gv->q1Sin + ik->len3 * gv->q12Sin + ik->len4 * gv->q123Sin) * gv->q0Cos;
    gv->posCurrent[2] = ik->len0 + ik->len2 * gv->q1Cos + ik->len3 * gv->q12Cos + ik->len4 * gv->q123Cos;

    gv->posTarget[0] = (ik->len1 + ik->len2 * sin(gv->qValuesTarget[1] / 180.0f * M_PI) + ik->len3 * sin((gv->qValuesTarget[1] + gv->qValuesTarget[2]) / 180.0f * M_PI) + ik->len4 * sin((gv->qValuesTarget[1] + gv->qValuesTarget[2] + gv->qValuesTarget[3]) / 180.0f * M_PI)) * sin(gv->qValuesTarget[0] / 180.0f * M_PI);
    gv->posTarget[1] = (ik->len1 + ik->len2 * sin(gv->qValuesTarget[1] / 180.0f * M_PI) + ik->len3 * sin((gv->qValuesTarget[1] + gv->qValuesTarget[2]) / 180.0f * M_PI) + ik->len4 * sin((gv->qValuesTarget[1] + gv->qValuesTarget[2] + gv->qValuesTarget[3]) / 180.0f * M_PI)) * cos(gv->qValuesTarget[0] / 180.0f * M_PI);
    gv->posTarget[2] = ik->len0 + ik->len2 * cos(gv->qValuesTarget[1] / 180.0f * M_PI) + ik->len3 * cos((gv->qValuesTarget[1] + gv->qValuesTarget[2]) / 180.0f * M_PI) + ik->len4 * cos((gv->qValuesTarget[1] + gv->qValuesTarget[2] + gv->qValuesTarget[3]) / 180.0f * M_PI);

    if (pp->nodeIndex != 0) {
        for (int i = 0; i < 5; i++) {
           qValuesNext[i] = pp->nodeList[pp->nodeList[pp->nodeIndex].previous].nodeQVal[i];
        }

        gv->posNext[0] = (ik->len1 + ik->len2 * sin(qValuesNext[1] / 180.0f * M_PI) + ik->len3 * sin((qValuesNext[1] +qValuesNext[2]) / 180.0f * M_PI) + ik->len4 * sin((qValuesNext[1] +qValuesNext[2] +qValuesNext[3]) / 180.0f * M_PI)) * sin(qValuesNext[0] / 180.0f * M_PI);
        gv->posNext[1] = (ik->len1 + ik->len2 * sin(qValuesNext[1] / 180.0f * M_PI) + ik->len3 * sin((qValuesNext[1] +qValuesNext[2]) / 180.0f * M_PI) + ik->len4 * sin((qValuesNext[1] +qValuesNext[2] +qValuesNext[3]) / 180.0f * M_PI)) * cos(qValuesNext[0] / 180.0f * M_PI);
        gv->posNext[2] = ik->len0 + ik->len2 * cos(qValuesNext[1] / 180.0f * M_PI) + ik->len3 * cos((qValuesNext[1] +qValuesNext[2]) / 180.0f * M_PI) + ik->len4 * cos((qValuesNext[1] +qValuesNext[2] +qValuesNext[3]) / 180.0f * M_PI);

    }
    else {
       qValuesNext[4] = 0.0f;

        gv->posNext[0] = 0.0f;
        gv->posNext[1] = 0.0f;
        gv->posNext[2] = 0.0f;
    }

    if (pp->nodeIndex != 0) {
        if (pp->nodeList[pp->nodeIndex].previous != 0) {
            for (int i = 0; i < 5; i++) {
                 qValuesFollowingNext[i] = pp->nodeList[pp->nodeList[pp->nodeList[pp->nodeIndex].previous].previous].nodeQVal[i];
            }

            gv->posFollowingNext[0] = (ik->len1 + ik->len2 * sin( qValuesFollowingNext[1] / 180.0f * M_PI) + ik->len3 * sin(( qValuesFollowingNext[1] +  qValuesFollowingNext[2]) / 180.0f * M_PI) + ik->len4 * sin(( qValuesFollowingNext[1] +  qValuesFollowingNext[2] +  qValuesFollowingNext[3]) / 180.0f * M_PI)) * sin( qValuesFollowingNext[0] / 180.0f * M_PI);
            gv->posFollowingNext[1] = (ik->len1 + ik->len2 * sin( qValuesFollowingNext[1] / 180.0f * M_PI) + ik->len3 * sin(( qValuesFollowingNext[1] +  qValuesFollowingNext[2]) / 180.0f * M_PI) + ik->len4 * sin(( qValuesFollowingNext[1] +  qValuesFollowingNext[2] +  qValuesFollowingNext[3]) / 180.0f * M_PI)) * cos( qValuesFollowingNext[0] / 180.0f * M_PI);
            gv->posFollowingNext[2] = ik->len0 + ik->len2 * cos( qValuesFollowingNext[1] / 180.0f * M_PI) + ik->len3 * cos(( qValuesFollowingNext[1] +  qValuesFollowingNext[2]) / 180.0f * M_PI) + ik->len4 * cos(( qValuesFollowingNext[1] +  qValuesFollowingNext[2] +  qValuesFollowingNext[3]) / 180.0f * M_PI);
        }
        else {
             qValuesFollowingNext[4] = 0.0f;

            gv->posFollowingNext[0] = 0.0f;
            gv->posFollowingNext[1] = 0.0f;
            gv->posFollowingNext[2] = 0.0f;
        }
    }
    else {
         qValuesFollowingNext[4] = 0.0f;

        gv->posFollowingNext[0] = 0.0f;
        gv->posFollowingNext[1] = 0.0f;
        gv->posFollowingNext[2] = 0.0f;
    }

    gv->coreMessageML = hf->to_string_with_precision(gv->posCurrent[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posCurrent[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posCurrent[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->qValuesCurrent[4]), 4) + "," + hf->to_string_with_precision(cos(gv->qValuesCurrent[4]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->posTarget[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posTarget[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posTarget[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->qValuesTarget[4]), 4) + "," + hf->to_string_with_precision(cos(gv->qValuesTarget[4]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->posNext[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posNext[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posNext[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(qValuesNext[4]), 4) + "," + hf->to_string_with_precision(cos(qValuesNext[4]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->posFollowingNext[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posFollowingNext[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->posFollowingNext[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin( qValuesFollowingNext[4]), 4) + "," + hf->to_string_with_precision(cos( qValuesFollowingNext[4]), 4) + ",";

    gv->coreMessageML += hf->to_string_with_precision(gv->colliders[8].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[8].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[8].offsets[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[8].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[8].angles[0]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->colliders[9].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[9].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[9].offsets[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[9].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[9].angles[0]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->colliders[10].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[10].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[10].offsets[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[10].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[10].angles[0]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->colliders[11].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[11].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[11].offsets[2] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[11].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[11].angles[0]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->colliders[12].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[12].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[12].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[12].angles[0]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->colliders[13].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[13].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[13].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[13].angles[0]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->colliders[14].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[14].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[14].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[14].angles[0]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->colliders[15].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[15].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[15].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[15].angles[0]), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->colliders[16].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[16].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[16].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[16].angles[0]), 4) + ",";

    if (transportedWorkpiece == 0) {
        gv->coreMessageML += "1,0,0,0,";
    }
    else if (transportedWorkpiece == 1) {
        gv->coreMessageML += "0,1,0,0,";
    }
    else if (transportedWorkpiece == 2) {
        gv->coreMessageML += "0,0,1,0,";
    }
    else if (transportedWorkpiece == 3) {
        gv->coreMessageML += "0,0,0,1,";
    }
    gv->coreMessageML += hf->to_string_with_precision(sin(gv->grippingAngleDiff / 180.0f * M_PI), 4) + "," + hf->to_string_with_precision(cos(gv->grippingAngleDiff / 180.0f * M_PI), 4) + ",";
    gv->coreMessageML += hf->to_string_with_precision(gv->pathEndPos.x / 55.0f, 4) + "," + hf->to_string_with_precision(gv->pathEndPos.y / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->endObjectAngle), 4) + "," + hf->to_string_with_precision(cos(gv->endObjectAngle), 4) + ",";

    for (int i = 0; i < 3; i++) {
        minInterIndices[i] = -1;
        minSegmentIndices[i] = -1;
        obstacleCollision[i] = false;
        obstacleDistsMin[i] = 50.0f;
    }

    pp->maxJointDiff = -1.0f;
    for (int i = 0; i < 5; i++) {
        if (abs(gv->qValuesTarget[i] - gv->qValuesCurrent[i]) > pp->maxJointDiff) {
            pp->maxJointDiff = abs(gv->qValuesTarget[i] - gv->qValuesCurrent[i]);
        }
    }
    pp->sampleCounts[0] = (int)(floor(pp->maxJointDiff / 1.5f));

    for (int m = 0; m < pp->sampleCounts[0] + 1; m++) {
        for (int n = 0; n < 5; n++) {
            pp->qValInter[n] = (gv->qValuesTarget[n] - gv->qValuesCurrent[n]) * float(m) / float(pp->sampleCounts[0] + 1) + gv->qValuesCurrent[n];
        }

        gv->q0Sin_pp = sin(pp->qValInter[0] / 180.0f * M_PI);
        gv->q0Cos_pp = cos(pp->qValInter[0] / 180.0f * M_PI);
        gv->q1Sin_pp = sin(pp->qValInter[1] / 180.0f * M_PI);
        gv->q1Cos_pp = cos(pp->qValInter[1] / 180.0f * M_PI);
        gv->q12Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
        gv->q12Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
        gv->q123Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);
        gv->q123Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);

        cd->recalculateCollider(&gv->adversaryCollider[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (90.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

        cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
        cd->rotationMatrix(gv->q0Cos_pp * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
        gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

        cd->recalculateColliderTransport(&gv->adversaryCollider[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);
        cd->recalculateColliderTransport(&gv->adversaryCollider[2], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

        if (gv->transportPhase == 1) {
            cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
            cd->recalculateColliderTransport(&gv->adversaryCollider[3], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
        }

        for (int i = 0; i < 3; i++) {
            if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[0].limits[5] - 1.5f && !obstacleCollision[i]) {
                if (cd->distApproximation(&gv->adversaryCollider[0], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                    obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[0], &gv->colliders[i + 12]);
                    if (obstacleDistsTemp[i] < cd->epsilon3) {
                        obstacleCollision[i] = true;
                    }
                    else if (obstacleDistsTemp[i] / 0.15f < m * 1.5f / pp->simulationStepSize * 1.5f) {
                        if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                            obstacleDistsMin[i] = obstacleDistsTemp[i];
                            minInterIndices[i] = m;
                            minSegmentIndices[i] = 0;
                        }
                    }
                }
            }

            if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[1].limits[5] - 1.5f && !obstacleCollision[i]) {
                if (cd->distApproximation(&gv->adversaryCollider[1], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                    obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[1], &gv->colliders[i + 12]);
                    if (obstacleDistsTemp[i] < cd->epsilon3) {
                        obstacleCollision[i] = true;
                    }
                    else if (obstacleDistsTemp[i] / 0.15f < m * 1.5f / pp->simulationStepSize * 1.5f) {
                        if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                            obstacleDistsMin[i] = obstacleDistsTemp[i];
                            minInterIndices[i] = m;
                            minSegmentIndices[i] = 0;
                        }
                    }
                }
            }

            if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[2].limits[5] - 1.5f && !obstacleCollision[i]) {
                if (cd->distApproximation(&gv->adversaryCollider[2], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                    obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[2], &gv->colliders[i + 12]);
                    if (obstacleDistsTemp[i] < cd->epsilon3) {
                        obstacleCollision[i] = true;
                    }
                    else if (obstacleDistsTemp[i] / 0.15f < m * 1.5f / pp->simulationStepSize * 1.5f) {
                        if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                            obstacleDistsMin[i] = obstacleDistsTemp[i];
                            minInterIndices[i] = m;
                            minSegmentIndices[i] = 0;
                        }
                    }
                }
            }

            if (gv->transportPhase == 1) {
                if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[3].limits[5] - 1.5f && !obstacleCollision[i]) {
                    if (cd->distApproximation(&gv->adversaryCollider[3], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                        obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[3], &gv->colliders[i + 12]);
                        if (obstacleDistsTemp[i] < cd->epsilon3) {
                            obstacleCollision[i] = true;
                        }
                        else if (obstacleDistsTemp[i] / 0.15f < m * 1.5f / pp->simulationStepSize * 1.5f) {
                            if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                                obstacleDistsMin[i] = obstacleDistsTemp[i];
                                minInterIndices[i] = m;
                                minSegmentIndices[i] = 0;
                            }
                        }
                    }
                }
            }
        }
    }

    if (pp->nodeIndex != 0) {
        pp->maxJointDiff = -1.0f;
        for (int i = 0; i < 5; i++) {
            if (abs(qValuesNext[i] - gv->qValuesTarget[i]) > pp->maxJointDiff) {
                pp->maxJointDiff = abs(qValuesNext[i] - gv->qValuesTarget[i]);
            }
        }
        pp->sampleCounts[1] = (int)(floor(pp->maxJointDiff / 1.5f));

        for (int m = 0; m < pp->sampleCounts[1] + 1; m++) {
            for (int n = 0; n < 5; n++) {
                pp->qValInter[n] = (qValuesNext[n] - gv->qValuesTarget[n]) * float(m) / float(pp->sampleCounts[1] + 1) + gv->qValuesTarget[n];
            }

            gv->q0Sin_pp = sin(pp->qValInter[0] / 180.0f * M_PI);
            gv->q0Cos_pp = cos(pp->qValInter[0] / 180.0f * M_PI);
            gv->q1Sin_pp = sin(pp->qValInter[1] / 180.0f * M_PI);
            gv->q1Cos_pp = cos(pp->qValInter[1] / 180.0f * M_PI);
            gv->q12Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
            gv->q12Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
            gv->q123Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);
            gv->q123Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);

            cd->recalculateCollider(&gv->adversaryCollider[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (90.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

            cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
            cd->rotationMatrix(gv->q0Cos_pp * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
            gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

            cd->recalculateColliderTransport(&gv->adversaryCollider[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);
            cd->recalculateColliderTransport(&gv->adversaryCollider[2], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

            if (gv->transportPhase == 1) {
                cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                cd->recalculateColliderTransport(&gv->adversaryCollider[3], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
            }

            for (int i = 0; i < 3; i++) {
                if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[0].limits[5] - 1.5f && !obstacleCollision[i]) {
                    if (cd->distApproximation(&gv->adversaryCollider[0], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                        obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[0], &gv->colliders[i + 12]);
                        if (obstacleDistsTemp[i] < cd->epsilon3) {
                            obstacleCollision[i] = true;
                        }
                        else if (obstacleDistsTemp[i] / 0.15f < (m + pp->sampleCounts[0]) * 1.5f / pp->simulationStepSize * 1.5f) {
                            if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                                obstacleDistsMin[i] = obstacleDistsTemp[i];
                                minInterIndices[i] = m;
                                minSegmentIndices[i] = 1;
                            }
                        }
                    }
                }

                if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[1].limits[5] - 1.5f && !obstacleCollision[i]) {
                    if (cd->distApproximation(&gv->adversaryCollider[1], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                        obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[1], &gv->colliders[i + 12]);
                        if (obstacleDistsTemp[i] < cd->epsilon3) {
                            obstacleCollision[i] = true;
                        }
                        else if (obstacleDistsTemp[i] / 0.15f < (m + pp->sampleCounts[0]) * 1.5f / pp->simulationStepSize * 1.5f) {
                            if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                                obstacleDistsMin[i] = obstacleDistsTemp[i];
                                minInterIndices[i] = m;
                                minSegmentIndices[i] = 1;
                            }
                        }
                    }
                }

                if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[2].limits[5] - 1.5f && !obstacleCollision[i]) {
                    if (cd->distApproximation(&gv->adversaryCollider[2], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                        obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[2], &gv->colliders[i + 12]);
                        if (obstacleDistsTemp[i] < cd->epsilon3) {
                            obstacleCollision[i] = true;
                        }
                        else if (obstacleDistsTemp[i] / 0.15f < (m + pp->sampleCounts[0]) * 1.5f / pp->simulationStepSize * 1.5f) {
                            if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                                obstacleDistsMin[i] = obstacleDistsTemp[i];
                                minInterIndices[i] = m;
                                minSegmentIndices[i] = 1;
                            }
                        }
                    }
                }

                if (gv->transportPhase == 1) {
                    if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[3].limits[5] - 1.5f && !obstacleCollision[i]) {
                        if (cd->distApproximation(&gv->adversaryCollider[3], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                            obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[3], &gv->colliders[i + 12]);
                            if (obstacleDistsTemp[i] < cd->epsilon3) {
                                obstacleCollision[i] = true;
                            }
                            else if (obstacleDistsTemp[i] / 0.15f < (m + pp->sampleCounts[0]) * 1.5f / pp->simulationStepSize * 1.5f) {
                                if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                                    obstacleDistsMin[i] = obstacleDistsTemp[i];
                                    minInterIndices[i] = m;
                                    minSegmentIndices[i] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (pp->nodeIndex != 0) {
        if (pp->nodeList[pp->nodeIndex].previous != 0) {
            pp->maxJointDiff = -1.0f;
            for (int i = 0; i < 5; i++) {
                if (abs( qValuesFollowingNext[i] -qValuesNext[i]) > pp->maxJointDiff) {
                    pp->maxJointDiff = abs( qValuesFollowingNext[i] -qValuesNext[i]);
                }
            }
            pp->sampleCounts[2] = (int)(floor(pp->maxJointDiff / 1.5f));

            for (int m = 0; m < pp->sampleCounts[2] + 1; m++) {
                for (int n = 0; n < 5; n++) {
                    pp->qValInter[n] = ( qValuesFollowingNext[n] -qValuesNext[n]) * float(m) / float(pp->sampleCounts[2] + 1) +qValuesNext[n];
                }

                gv->q0Sin_pp = sin(pp->qValInter[0] / 180.0f * M_PI);
                gv->q0Cos_pp = cos(pp->qValInter[0] / 180.0f * M_PI);
                gv->q1Sin_pp = sin(pp->qValInter[1] / 180.0f * M_PI);
                gv->q1Cos_pp = cos(pp->qValInter[1] / 180.0f * M_PI);
                gv->q12Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
                gv->q12Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
                gv->q123Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);
                gv->q123Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);

                cd->recalculateCollider(&gv->adversaryCollider[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (90.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

                cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                cd->rotationMatrix(gv->q0Cos_pp * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
                gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

                cd->recalculateColliderTransport(&gv->adversaryCollider[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);
                cd->recalculateColliderTransport(&gv->adversaryCollider[2], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

                if (gv->transportPhase == 1) {
                    cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
                    cd->recalculateColliderTransport(&gv->adversaryCollider[3], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
                }

                for (int i = 0; i < 3; i++) {
                    if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[0].limits[5] - 1.5f && !obstacleCollision[i]) {
                        if (cd->distApproximation(&gv->adversaryCollider[0], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                            obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[0], &gv->colliders[i + 12]);
                            if (obstacleDistsTemp[i] < cd->epsilon3) {
                                obstacleCollision[i] = true;
                            }
                            else if (obstacleDistsTemp[i] / 0.15f < (m + pp->sampleCounts[0] + pp->sampleCounts[1]) * 1.5f / pp->simulationStepSize * 1.5f) {
                                if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                                    obstacleDistsMin[i] = obstacleDistsTemp[i];
                                    minInterIndices[i] = m;
                                    minSegmentIndices[i] = 2;
                                }
                            }
                        }
                    }

                    if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[1].limits[5] - 1.5f && !obstacleCollision[i]) {
                        if (cd->distApproximation(&gv->adversaryCollider[1], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                            obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[1], &gv->colliders[i + 12]);
                            if (obstacleDistsTemp[i] < cd->epsilon3) {
                                obstacleCollision[i] = true;
                            }
                            else if (obstacleDistsTemp[i] / 0.15f < (m + pp->sampleCounts[0] + pp->sampleCounts[1]) * 1.5f / pp->simulationStepSize * 1.5f) {
                                if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                                    obstacleDistsMin[i] = obstacleDistsTemp[i];
                                    minInterIndices[i] = m;
                                    minSegmentIndices[i] = 2;
                                }
                            }
                        }
                    }

                    if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[2].limits[5] - 1.5f && !obstacleCollision[i]) {
                        if (cd->distApproximation(&gv->adversaryCollider[2], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                            obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[2], &gv->colliders[i + 12]);
                            if (obstacleDistsTemp[i] < cd->epsilon3) {
                                obstacleCollision[i] = true;
                            }
                            else if (obstacleDistsTemp[i] / 0.15f < (m + pp->sampleCounts[0] + pp->sampleCounts[1]) * 1.5f / pp->simulationStepSize * 1.5f) {
                                if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                                    obstacleDistsMin[i] = obstacleDistsTemp[i];
                                    minInterIndices[i] = m;
                                    minSegmentIndices[i] = 2;
                                }
                            }
                        }
                    }

                    if (gv->transportPhase == 1) {
                        if (gv->colliders[i + 12].limits[4] > gv->adversaryCollider[3].limits[5] - 1.5f && !obstacleCollision[i]) {
                            if (cd->distApproximation(&gv->adversaryCollider[3], &gv->colliders[i + 12]) <= obstacleDistsMin[i]) {
                                obstacleDistsTemp[i] = cd->gjk(&gv->adversaryCollider[3], &gv->colliders[i + 12]);
                                if (obstacleDistsTemp[i] < cd->epsilon3) {
                                    obstacleCollision[i] = true;
                                }
                                else if (obstacleDistsTemp[i] / 0.15f < (m + pp->sampleCounts[0] + pp->sampleCounts[1]) * 1.5f / pp->simulationStepSize * 1.5f) {
                                    if (obstacleDistsTemp[i] < obstacleDistsMin[i]) {
                                        obstacleDistsMin[i] = obstacleDistsTemp[i];
                                        minInterIndices[i] = m;
                                        minSegmentIndices[i] = 2;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void Adversaries::obs_adversary(bool firstCall, bool lastCall, int obsIndex, bool prognosis) {
	HelperFunctions *hf = HelperFunctions::get_instance();
	CollisionDetection *cd = CollisionDetection::get_instance();
	InverseKinematic *ik = InverseKinematic::get_instance();

    if (!prognosis) {
        if (obsIndex == 12) {
            gv->messageToSendML = "obs1,";
        }
        else if (obsIndex == 13) {
            gv->messageToSendML = "obs2,";
        }
        else {
            gv->messageToSendML = "obs3,";
        }
    }
    else {
        if (obsIndex == 12) {
            gv->messageToSendML = "obs1_p,";
        }
        else if (obsIndex == 13) {
            gv->messageToSendML = "obs2_p,";
        }
        else {
            gv->messageToSendML = "obs3_p,";
        }
    }

    if (firstCall) {
        gv->messageToSendML += "1,";
    }
    else {
        gv->messageToSendML += "0,";
    }

    gv->messageToSendML += gv->coreMessageML;

    if (obsIndex == 12) {
        gv->messageToSendML += "1,0,0,";
    }
    else if (obsIndex == 13) {
        gv->messageToSendML += "0,1,0,";
    }
    else {
        gv->messageToSendML += "0,0,1,";
    }

    if (minSegmentIndices[obsIndex - 12] != -1 && !obstacleCollision[obsIndex - 12]) {
        if (minSegmentIndices[obsIndex - 12] == 0) {
            for (int n = 0; n < 5; n++) {
                pp->qValInter[n] = (gv->qValuesTarget[n] - gv->qValuesCurrent[n]) * float(minInterIndices[obsIndex - 12]) / float(pp->sampleCounts[0] + 1) + gv->qValuesCurrent[n];
            }
        }
        else if (minSegmentIndices[obsIndex - 12] == 1) {
            for (int n = 0; n < 5; n++) {
                pp->qValInter[n] = (qValuesNext[n] - gv->qValuesTarget[n]) * float(minInterIndices[obsIndex - 12]) / float(pp->sampleCounts[1] + 1) + gv->qValuesTarget[n];
            }
        }
        else {
            for (int n = 0; n < 5; n++) {
                pp->qValInter[n] = ( qValuesFollowingNext[n] -qValuesNext[n]) * float(minInterIndices[obsIndex - 12]) / float(pp->sampleCounts[2] + 1) +qValuesNext[n];
            }
        }

        gv->q0Sin_pp = sin(pp->qValInter[0] / 180.0f * M_PI);
        gv->q0Cos_pp = cos(pp->qValInter[0] / 180.0f * M_PI);
        gv->q1Sin_pp = sin(pp->qValInter[1] / 180.0f * M_PI);
        gv->q1Sin_pp = sin(pp->qValInter[1] / 180.0f * M_PI);
        gv->q1Cos_pp = cos(pp->qValInter[1] / 180.0f * M_PI);
        gv->q12Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
        gv->q12Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2]) / 180.0f * M_PI);
        gv->q123Sin_pp = sin((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);
        gv->q123Cos_pp = cos((pp->qValInter[1] + pp->qValInter[2] + pp->qValInter[3]) / 180.0f * M_PI);

        cd->recalculateCollider(&gv->adversaryCollider[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->lastSegmentLen + ik->gripperBaseLen / 2.0f) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (90.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

        cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
        cd->rotationMatrix(gv->q0Cos_pp * (gv->grippingWidth / 2.0f + 0.45f), -gv->q0Sin_pp * (gv->grippingWidth / 2.0f + 0.45f), 0, 2);
        gv->grippingDiffVector[0] = cd->xVal_cd; gv->grippingDiffVector[1] = cd->yVal_cd; gv->grippingDiffVector[2] = cd->zVal_cd;

        cd->recalculateColliderTransport(&gv->adversaryCollider[1], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp + gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp + gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp + gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);
        cd->recalculateColliderTransport(&gv->adversaryCollider[2], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Sin_pp - gv->grippingDiffVector[0], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Sin_pp) * gv->q0Cos_pp - gv->grippingDiffVector[1], ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + 2.25f + gv->gripperInstallationOffset) * gv->q123Cos_pp - gv->grippingDiffVector[2], (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, pp->qValInter[4] / 180.0f * M_PI);

        if (gv->transportPhase == 1) {
            cd->updateMatricesTransport((-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI, gv->q123Sin_pp * gv->q0Sin_pp, gv->q123Sin_pp * gv->q0Cos_pp, gv->q123Cos_pp);
            cd->recalculateColliderTransport(&gv->adversaryCollider[3], (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Sin_pp, (ik->len1 + ik->len2 * gv->q1Sin_pp + ik->len3 * gv->q12Sin_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Sin_pp) * gv->q0Cos_pp, ik->len0 + ik->len2 * gv->q1Cos_pp + ik->len3 * gv->q12Cos_pp + (ik->len4 + gv->objectHeight + gv->objectGripDepth) * gv->q123Cos_pp, (-pp->qValInter[0] + 90.0f) / 180.0f * M_PI, (180.0f - pp->qValInter[1] - pp->qValInter[2] - pp->qValInter[3]) / 180.0f * M_PI, (pp->qValInter[4] + gv->grippingAngleDiff) / 180.0f * M_PI);
        }

        oldXPos = gv->colliders[obsIndex].offsets[0];
        oldYPos = gv->colliders[obsIndex].offsets[1];
        oldAngleVal = gv->colliders[obsIndex].angles[0];

        gv->obsDiffMin = 10e5;
        gv->iMax = 0; gv->jMax = 0; gv->angleMax = 0;

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                for (int k = -1; k <= 1; k++) {
                    if ((i == 0 && j == 0 && k != 0) || (i == 0 && j != 0 && k == 0) || (i != 0 && j == 0 && k == 0) || (i == 0 && j == 0 && k == 0)) {
                        cd->recalculateCollider(&gv->colliders[obsIndex], oldXPos + i * 0.15f, oldYPos + j * 0.15f, gv->colliders[obsIndex].offsets[2], oldAngleVal + k * 3.0f / 180.0f * M_PI, 0.0f, 0.0f);

                        gv->collision_init = false;
                        for (int l = 1; l < gv->collidersCount; l++) {
                            if ((l == 1 || l >= 5) && l != obsIndex) {
                                if (cd->checkForCollision(&gv->colliders[l], &gv->colliders[obsIndex])) {
                                    gv->collision_init = true;
                                    break;
                                }
                            }
                        }

                        if (!gv->collision_init && cd->checkForCollision(&gv->grippers[0], &gv->colliders[obsIndex])) {
                            gv->collision_init = true;
                        }

                        if (!gv->collision_init && cd->checkForCollision(&gv->grippers[1], &gv->colliders[obsIndex])) {
                            gv->collision_init = true;
                        }

                        if (!(gv->collision_init || oldXPos + 0.15f * gv->iMax > 55.0f || oldXPos + 0.15f * gv->iMax < -55.0f || oldYPos + 0.15f * gv->jMax < -12.5f || oldYPos + 0.15f * gv->jMax > 55.0f)) {
                            gv->obsDiffTemp = cd->gjk(&gv->adversaryCollider[0], &gv->colliders[obsIndex]);
                            if (gv->obsDiffTemp < gv->obsDiffMin) {
                                gv->obsDiffMin = gv->obsDiffTemp;
                                gv->iMax = i; gv->jMax = j; gv->angleMax = k;
                            }

                            gv->obsDiffTemp = cd->gjk(&gv->adversaryCollider[1], &gv->colliders[obsIndex]);
                            if (gv->obsDiffTemp < gv->obsDiffMin) {
                                gv->obsDiffMin = gv->obsDiffTemp;
                                gv->iMax = i; gv->jMax = j; gv->angleMax = k;
                            }

                            gv->obsDiffTemp = cd->gjk(&gv->adversaryCollider[2], &gv->colliders[obsIndex]);
                            if (gv->obsDiffTemp < gv->obsDiffMin) {
                                gv->obsDiffMin = gv->obsDiffTemp;
                                gv->iMax = i; gv->jMax = j; gv->angleMax = k;
                            }

                            if (gv->transportPhase == 1) {
                                gv->obsDiffTemp = cd->gjk(&gv->adversaryCollider[3], &gv->colliders[obsIndex]);
                                if (gv->obsDiffTemp < gv->obsDiffMin) {
                                    gv->obsDiffMin = gv->obsDiffTemp;
                                    gv->iMax = i; gv->jMax = j; gv->angleMax = k;
                                }
                            }
                        }
                    }
                }
            }
        }

        cd->recalculateCollider(&gv->colliders[obsIndex], oldXPos, oldYPos, gv->colliders[obsIndex].offsets[2], oldAngleVal, 0.0f, 0.0f, true);
        gv->messageToSendML += hf->to_string_with_precision(float(gv->iMax), 4) + "," + hf->to_string_with_precision(float(gv->jMax), 4) + "," + hf->to_string_with_precision(float(gv->angleMax), 4) + ",";
    }
    else {
        gv->iMax = 0; gv->jMax = 0; gv->angleMax = 0;
        gv->messageToSendML += "0.0,0.0,0.0,";
    }

    if (lastCall) {
        if (obsIndex == 12) gv->messageToSendML += hf->to_string_with_precision(gv->reward1, 4) + ",1";
        else if (obsIndex == 13) gv->messageToSendML += hf->to_string_with_precision(gv->reward2, 4) + ",1";
        else if (obsIndex == 14) gv->messageToSendML += hf->to_string_with_precision(gv->reward3, 4) + ",1";
//#if defined(WIN32)
         sendMessageML(gv->messageToSendML);
         receiveMessageML();
//#endif
    }
    else {
        if (!firstCall && obsIndex == 12) gv->messageToSendML += hf->to_string_with_precision(gv->reward1, 4) + ",0";
        if (!firstCall && obsIndex == 13) gv->messageToSendML += hf->to_string_with_precision(gv->reward2, 4) + ",0";
        if (!firstCall && obsIndex == 14) gv->messageToSendML += hf->to_string_with_precision(gv->reward3, 4) + ",0";
//#if defined(WIN32)
         sendMessageML(gv->messageToSendML);

         gv->receivedAnswerML =  receiveMessageML();
        gv->action = stoi( gv->receivedAnswerML);
//#endif
        if (gv->action == 0) {
            gv->iMin = 0;
            gv->jMin = 0;
            gv->angleMin = 0;
        }
        else if (gv->action == 1) {
            gv->iMin = 1;
            gv->jMin = 0;
            gv->angleMin = 0;
        }
        else if (gv->action == 2) {
            gv->iMin = -1;
            gv->jMin = 0;
            gv->angleMin = 0;
        }
        else if (gv->action == 3) {
            gv->iMin = 0;
            gv->jMin = 1;
            gv->angleMin = 0;
        }
        else if (gv->action == 4) {
            gv->iMin = 0;
            gv->jMin = -1;
            gv->angleMin = 0;
        }
        else if (gv->action == 5) {
            gv->iMin = 0;
            gv->jMin = 0;
            gv->angleMin = 1;
        }
        else if (gv->action == 6) {
            gv->iMin = 0;
            gv->jMin = 0;
            gv->angleMin = -1;
        }

        if (obsIndex == 12) {
            if (gv->iMax == gv->iMin && gv->jMax == gv->jMin && gv->angleMax == gv->angleMin) {
                gv->reward1 = 1.0f;
            }
            else {
                gv->reward1 = -1.0f;
            }
        }
        else if (obsIndex == 13) {
            if (gv->iMax == gv->iMin && gv->jMax == gv->jMin && gv->angleMax == gv->angleMin) {
                gv->reward2 = 1.0f;
            }
            else {
                gv->reward2 = -1.0f;
            }
        }
        else if (obsIndex == 14) {
            if (gv->iMax == gv->iMin && gv->jMax == gv->jMin && gv->angleMax == gv->angleMin) {
                gv->reward3 = 1.0f;
            }
            else {
                gv->reward3 = -1.0f;
            }
        }

        bool changed = false;

        if (gv->action != 0 && abs(gv->currentDetectedObsPosX[obsIndex - 12]) < cd->epsilon3 && abs(gv->currentDetectedObsPosY[obsIndex - 12]) < cd->epsilon3) {
            changed = true;

            oldXPos = gv->colliders[obsIndex].offsets[0];
            oldYPos = gv->colliders[obsIndex].offsets[1];
            oldAngleVal = gv->colliders[obsIndex].angles[0];
            cd->recalculateCollider(&gv->colliders[obsIndex], oldXPos + 0.15f * gv->iMin, oldYPos + 0.15f * gv->jMin, gv->colliders[obsIndex].offsets[2], oldAngleVal + gv->angleMin * 3.0f / 180.0f * M_PI, 0, 0, true);

            gv->collision_init = false;
            for (int l = 1; l < gv->collidersCount; l++) {
                if ((l == 1 || l >= 5) && l != obsIndex) {
                    if (cd->checkForCollision(&gv->colliders[l], &gv->colliders[obsIndex])) {
                        gv->collision_init = true;
                        break;
                    }
                }
            }

            if (!gv->collision_init && cd->checkForCollision(&gv->grippers[0], &gv->colliders[obsIndex])) {
                gv->collision_init = true;
            }

            if (!gv->collision_init && cd->checkForCollision(&gv->grippers[1], &gv->colliders[obsIndex])) {
                gv->collision_init = true;
            }

            if (gv->collision_init || oldXPos + 0.15f * gv->iMin > 55.0f || oldXPos + 0.15f * gv->iMin < -55.0f || oldYPos + 0.15f * gv->jMin < -12.5f || oldYPos + 0.15f * gv->jMin > 55.0f) {
                cd->recalculateCollider(&gv->colliders[obsIndex], oldXPos, oldYPos, gv->colliders[obsIndex].offsets[2], oldAngleVal, 0, 0, true);
                changed = false;
            }

            if (obsIndex == 12) cd->recalculateCollider(&gv->distortedObstacles[0], gv->colliders[12].offsets[0] + gv->distortions[0], gv->colliders[12].offsets[1] + gv->distortions[1], gv->colliders[12].offsets[2], gv->colliders[12].angles[0] + gv->distortions[2], 0, 0);
            else if (obsIndex == 13) cd->recalculateCollider(&gv->distortedObstacles[1], gv->colliders[13].offsets[0] + gv->distortions[3], gv->colliders[13].offsets[1] + gv->distortions[4], gv->colliders[13].offsets[2], gv->colliders[13].angles[0] + gv->distortions[5], 0, 0);
            else cd->recalculateCollider(&gv->distortedObstacles[2], gv->colliders[14].offsets[0] + gv->distortions[6], gv->colliders[14].offsets[1] + gv->distortions[7], gv->colliders[14].offsets[2], gv->colliders[14].angles[0] + gv->distortions[8], 0, 0);

            gv->tempCounter = gv->totalPlatePoints;
            for (int i = 0; i < obsIndex; i++) {
                gv->tempCounter += gv->colliders[i].facesTimes3;
            }
            hf->calcEdgesForConvexHull(gv->object_edges, &gv->colliders[obsIndex], gv->tempCounter);

            gv->tempCounter = gv->colliders[8].patternCount * 4 + gv->colliders[9].patternCount * 4 + gv->colliders[10].patternCount * 4 + gv->colliders[11].patternCount * 4;
            if (obsIndex == 12) {
                hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[12], gv->tempCounter);
            }
            else if (obsIndex == 13) {
                hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[13], gv->tempCounter + gv->colliders[12].patternCount * 4);
            }
            else {
                hf->calcEdgesForPatterns(gv->object_pattern_edges, &gv->colliders[14], gv->tempCounter + gv->colliders[12].patternCount * 4 + gv->colliders[13].patternCount * 4);
            }
        }

        if (gv->writeDataIntoFiles) {
            fstream file;
            file.open("obs_actions_" + to_string(obsIndex - 12) + "_" + to_string(gv->simCounter) + ".csv", std::ios_base::app);
            if (file) {
                if (changed) {
                    file << to_string(gv->action) + ";" + to_string(gv->timeCounter) + "\n";
                }
                else {
                    file << "0;" + to_string(gv->timeCounter) + "\n";
                }
            }
            else {
                cout << "Error: writing into file not possible!" << endl;
            }
            file.close();
        }
    }
}
