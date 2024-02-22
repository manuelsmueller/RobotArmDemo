/*
 * Protagonists.cpp
 *
 *  Created on: 05.12.2022
 *      Author: manuel
 */

#include "Protagonists.h"

Protagonists::Protagonists() {
	// TODO Auto-generated constructor stub
	gv=GlobalVariables::get_instance();
	hf=HelperFunctions::get_instance();
	mt=MonitoringToolModels::get_instance();

}

Protagonists::~Protagonists() {
	// TODO Auto-generated destructor stub
}

/*
 * singelton pattern
 */
Protagonists* Protagonists::prot = 0;
Protagonists* Protagonists::get_instance(){
	static bool isInit=false;

//	InverseKinematic *ik;
	if(!isInit){
		prot  = new Protagonists();
		isInit=true;
	}
	return prot;
}




void Protagonists::mt_preparation(int mtIndex,
		default_random_engine generator1,normal_distribution<float> distribution1,
		default_random_engine generator2, normal_distribution<float>  distribution2,
		default_random_engine generator3, normal_distribution<float> distribution3)
{
	CollisionDetection *cd = CollisionDetection::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();
	FrequentOps *fo = FrequentOps::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

	gv->maxViewVal = -10.0e5;
    int counterMTrepos = 0;

    const int maxRuns1 = 1000;
    const int maxRuns2 = 10000;

    while (counterMTrepos < maxRuns1) {
        gv->tempViewVal = 0.0f;
        gv->tempMTPos[0] = ((float)rand() / RAND_MAX) * 90.0f - 45.0f;
        gv->tempMTPos[1] = ((float)rand() / RAND_MAX) * 57.5f - 12.5f;
        gv->tempMTPos[2] = 2 * M_PI * ((float)rand() / RAND_MAX);

        if (mtIndex == 1) {
            if (counterMTrepos == 0) {
                gv->tempMTPos[0] = gv->oldMTPos[0];
                gv->tempMTPos[1] = gv->oldMTPos[1];
                gv->tempMTPos[2] = gv->oldMTPos[2];

                gv->collision_init = false;
            }
            else {
                cd->recalculateCollider(&gv->colliders[15], gv->tempMTPos[0], gv->tempMTPos[1], gv->objectOffsets[4], gv->tempMTPos[2], 0, 0, false, true, gv->camVectorsMT1);

                gv->collision_init = false;
                if (cd->checkForCollision(&gv->colliders[15], &gv->colliders[1])) {
                    gv->collision_init = true;
                }
                if (!gv->collision_init && cd->checkForCollision(&gv->colliders[15], &gv->colliders[16])) {
                    gv->collision_init = true;
                }
                if (!gv->collision_init && cd->checkForCollision(&gv->colliders[15], &gv->safetyCollidersForMTs[0])) {
                    gv->collision_init = true;
                }
                if (!gv->collision_init && cd->checkForCollision(&gv->colliders[15], &gv->safetyCollidersForMTs[1])) {
                    gv->collision_init = true;
                }
                if (!gv->collision_init) {
                    for (int i = 8; i <= 11; i++) {
                        if (i != gv->objectToTransport) {
                            if (cd->checkForCollision(&gv->colliders[15], &gv->colliders[i])) {
                                gv->collision_init = true;
                                break;
                            }
                        }
                    }
                }
                if (!gv->collision_init && (abs(gv->currentDetectedObsPosX[0]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[0]) > cd->epsilon3)) {
                    if (cd->checkForCollision(&gv->colliders[15], &gv->distortedObstacles[0])) {
                        gv->collision_init = true;
                    }
                }
                if (!gv->collision_init && (abs(gv->currentDetectedObsPosX[1]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[1]) > cd->epsilon3)) {
                    if (cd->checkForCollision(&gv->colliders[15], &gv->distortedObstacles[1])) {
                        gv->collision_init = true;
                    }
                }
                if (!gv->collision_init && (abs(gv->currentDetectedObsPosX[2]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[2]) > cd->epsilon3)) {
                    if (cd->checkForCollision(&gv->colliders[15], &gv->distortedObstacles[2])) {
                        gv->collision_init = true;
                    }
                }
            }
        }
        else {
            cd->recalculateCollider(&gv->colliders[16], gv->tempMTPos[0], gv->tempMTPos[1], gv->objectOffsets[4], gv->tempMTPos[2], 0, 0, false, true, gv->camVectorsMT2);

            gv->collision_init = false;
            if (cd->checkForCollision(&gv->colliders[16], &gv->colliders[1])) {
                gv->collision_init = true;
            }
            if (!gv->collision_init && cd->checkForCollision(&gv->colliders[16], &gv->colliders[15])) {
                gv->collision_init = true;
            }
            if (!gv->collision_init && cd->checkForCollision(&gv->colliders[16], &gv->safetyCollidersForMTs[0])) {
                gv->collision_init = true;
            }
            if (!gv->collision_init && cd->checkForCollision(&gv->colliders[16], &gv->safetyCollidersForMTs[1])) {
                gv->collision_init = true;
            }
            if (!gv->collision_init) {
                for (int i = 8; i <= 11; i++) {
                    if (i != gv->objectToTransport) {
                        if (cd->checkForCollision(&gv->colliders[16], &gv->colliders[i])) {
                            gv->collision_init = true;
                            break;
                        }
                    }
                }
            }
            if (!gv->collision_init && (abs(gv->currentDetectedObsPosX[0]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[0]) > cd->epsilon3)) {
                if (cd->checkForCollision(&gv->colliders[16], &gv->distortedObstacles[0])) {
                    gv->collision_init = true;
                }
            }
            if (!gv->collision_init && (abs(gv->currentDetectedObsPosX[1]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[1]) > cd->epsilon3)) {
                if (cd->checkForCollision(&gv->colliders[16], &gv->distortedObstacles[1])) {
                    gv->collision_init = true;
                }
            }
            if (!gv->collision_init && (abs(gv->currentDetectedObsPosX[2]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[2]) > cd->epsilon3)) {
                if (cd->checkForCollision(&gv->colliders[16], &gv->distortedObstacles[2])) {
                    gv->collision_init = true;
                }
            }
        }

        if (!gv->collision_init) {
            for (int j = 0; j < 30; j++) {

            	int maxRuns=maxRuns2; // establish a limit of tries!

                do {
                    gv->randomObs = (rand() % 3);
                    fo->getRandomObstaclePosition(generator1, distribution1, generator2, distribution2, generator3, distribution3);

                    if (gv->randomObs == 0) {
                        cd->recalculateCollider(&gv->colliders[12], fo->randomObstalcePosition[0], fo->randomObstalcePosition[1], 4.0f, 2 * M_PI * ((float)rand() / RAND_MAX), 0, 0, true);

                        if (abs(gv->currentDetectedObsPosX[1]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[1]) > cd->epsilon3) {
                            if (((float)rand() / RAND_MAX) > 0.5f) {
                                cd->recalculateCollider(&gv->colliders[13], gv->currentDetectedObsPosX[1], gv->currentDetectedObsPosY[1], 3.0f, gv->currentDetectedObsPosA[1], 0, 0, true);
                            }
                            else {
                                cd->recalculateCollider(&gv->colliders[13], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                            }
                        }
                        else {
                            cd->recalculateCollider(&gv->colliders[13], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                        }

                        if (abs(gv->currentDetectedObsPosX[2]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[2]) > cd->epsilon3) {
                            if (((float)rand() / RAND_MAX) > 0.5f) {
                                cd->recalculateCollider(&gv->colliders[14], gv->currentDetectedObsPosX[2], gv->currentDetectedObsPosY[2], 6.0f, gv->currentDetectedObsPosA[2], 0, 0, true);
                            }
                            else {
                                cd->recalculateCollider(&gv->colliders[14], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                            }
                        }
                        else {
                            cd->recalculateCollider(&gv->colliders[14], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                        }
                    }
                    else if (gv->randomObs == 1) {
                        if (abs(gv->currentDetectedObsPosX[0]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[0]) > cd->epsilon3) {
                            if (((float)rand() / RAND_MAX) > 0.5f) {
                                cd->recalculateCollider(&gv->colliders[12], gv->currentDetectedObsPosX[0], gv->currentDetectedObsPosY[0], 4.0f, gv->currentDetectedObsPosA[0], 0, 0, true);
                            }
                            else {
                                cd->recalculateCollider(&gv->colliders[12], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                            }
                        }
                        else {
                            cd->recalculateCollider(&gv->colliders[12], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                        }

                        cd->recalculateCollider(&gv->colliders[13], fo->randomObstalcePosition[0], fo->randomObstalcePosition[1], 3.0f, 2 * M_PI * ((float)rand() / RAND_MAX), 0, 0, true);

                        if (abs(gv->currentDetectedObsPosX[2]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[2]) > cd->epsilon3) {
                            if (((float)rand() / RAND_MAX) > 0.5f) {
                                cd->recalculateCollider(&gv->colliders[14], gv->currentDetectedObsPosX[2], gv->currentDetectedObsPosY[2], 6.0f, gv->currentDetectedObsPosA[2], 0, 0, true);
                            }
                            else {
                                cd->recalculateCollider(&gv->colliders[14], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                            }
                        }
                        else {
                            cd->recalculateCollider(&gv->colliders[14], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                        }
                    }
                    else {
                        if (abs(gv->currentDetectedObsPosX[0]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[0]) > cd->epsilon3) {
                            if (((float)rand() / RAND_MAX) > 0.5f) {
                                cd->recalculateCollider(&gv->colliders[12], gv->currentDetectedObsPosX[0], gv->currentDetectedObsPosY[0], 4.0f, gv->currentDetectedObsPosA[0], 0, 0, true);
                            }
                            else {
                                cd->recalculateCollider(&gv->colliders[12], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                            }
                        }
                        else {
                            cd->recalculateCollider(&gv->colliders[12], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                        }

                        if (abs(gv->currentDetectedObsPosX[1]) > cd->epsilon3 || abs(gv->currentDetectedObsPosY[1]) > cd->epsilon3) {
                            if (((float)rand() / RAND_MAX) > 0.5f) {
                                cd->recalculateCollider(&gv->colliders[13], gv->currentDetectedObsPosX[1], gv->currentDetectedObsPosY[1], 3.0f, gv->currentDetectedObsPosA[1], 0, 0, true);
                            }
                            else {
                                cd->recalculateCollider(&gv->colliders[13], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                            }
                        }
                        else {
                            cd->recalculateCollider(&gv->colliders[13], 0.0f, 0.0f, 10.0e5, 0, 0, 0, true);
                        }

                        cd->recalculateCollider(&gv->colliders[14], fo->randomObstalcePosition[0], fo->randomObstalcePosition[1], 6.0f, 2 * M_PI * ((float)rand() / RAND_MAX), 0, 0, true);
                    }

                    gv->collision_init = false;
                    if (cd->checkForCollision(&gv->colliders[gv->randomObs + 12], &gv->colliders[1])) {
                        gv->collision_init = true;
                    }
                    if (!gv->collision_init && cd->checkForCollision(&gv->colliders[gv->randomObs + 12], &gv->colliders[15])) {
                        gv->collision_init = true;
                    }
                    if (!gv->collision_init && cd->checkForCollision(&gv->colliders[gv->randomObs + 12], &gv->colliders[16])) {
                        gv->collision_init = true;
                    }
                    if (!gv->collision_init) {
                        for (int i = 8; i <= 14; i++) {
                            if (i != gv->objectToTransport && i != gv->randomObs + 12) {
                                if (cd->checkForCollision(&gv->colliders[gv->randomObs + 12], &gv->colliders[i])) {
                                    gv->collision_init = true;
                                    break;
                                }
                            }
                        }
                    }

                    if(maxRuns<=0){
                    	break;
                    }

                    maxRuns--;
                } while (gv->collision_init);

                if(maxRuns<=0){
                	std::cout<<"E: optimmization did not succeed within maxRuns!"<<std::endl;
                	// cout << "debug: prot. " << __LINE__ <<endl;
                }

                if (mtIndex == 1) {
                    if (mt->isObjectVisible(gv->randomObs + 12, 1) != -1) {
                        gv->vectorLenDir = sqrt(gv->camVectorsMT1[3] * gv->camVectorsMT1[3] + gv->camVectorsMT1[4] * gv->camVectorsMT1[4]);
                        gv->vectorLenObj = sqrt((gv->colliders[gv->randomObs + 12].offsets[0] - gv->camVectorsMT1[0]) * (gv->colliders[gv->randomObs + 12].offsets[0] - gv->camVectorsMT1[0]) + (gv->colliders[gv->randomObs + 12].offsets[1] - gv->camVectorsMT1[1]) * (gv->colliders[gv->randomObs + 12].offsets[1] - gv->camVectorsMT1[1]));
                        gv->tempViewVal += ((gv->colliders[gv->randomObs + 12].offsets[0] - gv->camVectorsMT1[0]) / gv->vectorLenObj * gv->camVectorsMT1[3] / gv->vectorLenDir + (gv->colliders[gv->randomObs + 12].offsets[1] - gv->camVectorsMT1[1]) / gv->vectorLenObj * gv->camVectorsMT1[4] / gv->vectorLenDir);
                    }
                }
                else {
                    if (mt->isObjectVisible(gv->randomObs + 12, 1) != -1) {
                        gv->vectorLenDir = sqrt(gv->camVectorsMT1[3] * gv->camVectorsMT1[3] + gv->camVectorsMT1[4] * gv->camVectorsMT1[4]);
                        gv->vectorLenObj = sqrt((gv->colliders[gv->randomObs + 12].offsets[0] - gv->camVectorsMT1[0]) * (gv->colliders[gv->randomObs + 12].offsets[0] - gv->camVectorsMT1[0]) + (gv->colliders[gv->randomObs + 12].offsets[1] - gv->camVectorsMT1[1]) * (gv->colliders[gv->randomObs + 12].offsets[1] - gv->camVectorsMT1[1]));
                        gv->tempViewVal += ((gv->colliders[gv->randomObs + 12].offsets[0] - gv->camVectorsMT1[0]) / gv->vectorLenObj * gv->camVectorsMT1[3] / gv->vectorLenDir + (gv->colliders[gv->randomObs + 12].offsets[1] - gv->camVectorsMT1[1]) / gv->vectorLenObj * gv->camVectorsMT1[4] / gv->vectorLenDir);
                    }
                    if (mt->isObjectVisible(gv->randomObs + 12, 2) != -1) {
                        gv->vectorLenDir = sqrt(gv->camVectorsMT2[3] * gv->camVectorsMT2[3] + gv->camVectorsMT2[4] * gv->camVectorsMT2[4]);
                        gv->vectorLenObj = sqrt((gv->colliders[gv->randomObs + 12].offsets[0] - gv->camVectorsMT2[0]) * (gv->colliders[gv->randomObs + 12].offsets[0] - gv->camVectorsMT2[0]) + (gv->colliders[gv->randomObs + 12].offsets[1] - gv->camVectorsMT2[1]) * (gv->colliders[gv->randomObs + 12].offsets[1] - gv->camVectorsMT2[1]));
                        gv->tempViewVal += ((gv->colliders[gv->randomObs + 12].offsets[0] - gv->camVectorsMT2[0]) / gv->vectorLenObj * gv->camVectorsMT2[3] / gv->vectorLenDir + (gv->colliders[gv->randomObs + 12].offsets[1] - gv->camVectorsMT2[1]) / gv->vectorLenObj * gv->camVectorsMT2[4] / gv->vectorLenDir);
                    }
                }
            }

            if (mtIndex == 1 && counterMTrepos == 0) gv->oldViewVal = gv->tempViewVal;

            if (gv->tempViewVal > gv->maxViewVal) {
            	gv->maxViewVal = gv->tempViewVal;
                gv->newMTPos[0] = gv->tempMTPos[0]; gv->newMTPos[1] = gv->tempMTPos[1]; gv->newMTPos[2] = gv->tempMTPos[2];
            }
//            counterMTrepos++;
        }
        counterMTrepos++;
    }
}

void Protagonists::mt_protagonist(int mtIndex, int transportedWorkpiece, bool prognosis) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	GlobalVariables *gv=GlobalVariables::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

    gv->tempMTPos[0] = gv->oldMTPos[0];
    gv->tempMTPos[1] = gv->oldMTPos[1];
    gv->tempMTPos[2] = gv->oldMTPos[2];

    for (int it = 0; it <= 1000; it++) {
        if (!prognosis) {
            if (mtIndex == 1) {
                gv->messageToSendML = "mt1,";
            }
            else {
                gv->messageToSendML = "mt2,";
            }
        }
        else {
            if (mtIndex == 1) {
                gv->messageToSendML = "mt1_p,";
            }
            else {
                gv->messageToSendML = "mt2_p,";
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

        if (gv->currentDetectedObsPosX[0] > cd->epsilon3 || gv->currentDetectedObsPosY[0] > cd->epsilon3) {
            gv->messageToSendML += hf->to_string_with_precision(gv->distortedObstacles[0].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->distortedObstacles[0].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->distortedObstacles[0].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->distortedObstacles[0].angles[0]), 4) + ",";
        }
        else {
            gv->messageToSendML += "0,0,0,0,";
        }

        if (gv->currentDetectedObsPosX[1] > cd->epsilon3 || gv->currentDetectedObsPosY[1] > cd->epsilon3) {
            gv->messageToSendML += hf->to_string_with_precision(gv->distortedObstacles[1].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->distortedObstacles[1].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->distortedObstacles[1].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->distortedObstacles[1].angles[0]), 4) + ",";
        }
        else {
            gv->messageToSendML += "0,0,0,0,";
        }

        if (gv->currentDetectedObsPosX[2] > cd->epsilon3 || gv->currentDetectedObsPosY[2] > cd->epsilon3) {
            gv->messageToSendML += hf->to_string_with_precision(gv->distortedObstacles[2].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->distortedObstacles[2].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->distortedObstacles[2].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->distortedObstacles[2].angles[0]), 4) + ",";
        }
        else {
            gv->messageToSendML += "0,0,0,0,";
        }

        gv->messageToSendML += hf->to_string_with_precision(gv->colliders[15].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[15].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[15].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[15].angles[0]), 4) + ",";
        gv->messageToSendML += hf->to_string_with_precision(gv->colliders[16].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[16].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[16].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[16].angles[0]), 4) + ",";

        gv->messageToSendML += hf->to_string_with_precision(gv->tempMTPos[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->tempMTPos[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->tempMTPos[2]), 4) + "," + hf->to_string_with_precision(cos(gv->tempMTPos[2]), 4) + ",";

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

        if (mtIndex == 1) {
            gv->messageToSendML += "1,0,";
        }
        else {
            gv->messageToSendML += "0,1,";
        }

        gv->distVectorMT[0] = gv->newMTPos[0] - gv->tempMTPos[0];
        gv->distVectorMT[1] = gv->newMTPos[1] - gv->tempMTPos[1];
        gv->distVectorMTlen = sqrt(gv->distVectorMT[0] * gv->distVectorMT[0] + gv->distVectorMT[1] * gv->distVectorMT[1]);
        gv->angleDiffMT = (gv->newMTPos[2] - gv->tempMTPos[2]) / M_PI * 180.0f;

        if (abs(gv->angleDiffMT) / 1.5f < gv->distVectorMTlen / 0.5f) {
            if (gv->distVectorMTlen > 0.5f / 2.0f + cd->epsilon2) {
                gv->distVectorMT[0] /= gv->distVectorMTlen;
                gv->distVectorMT[1] /= gv->distVectorMTlen;

                gv->maxDot = -10.0e5;
                gv->iMax = 0; gv->jMax = 0; gv->angleMax = 0;

                for (int i = -1; i <= 1; i++) {
                    for (int j = -1; j <= 1; j++) {
                        if ((i != 0 || j != 0) && i * j == 0) {
                            if (gv->distVectorMT[0] * float(i) + gv->distVectorMT[1] * float(j) > gv->maxDot) {
                                gv->maxDot = gv->distVectorMT[0] * float(i) + gv->distVectorMT[1] * float(j);
                                gv->iMax = i; gv->jMax = j;
                            }
                        }
                    }
                }

                gv->messageToSendML += hf->to_string_with_precision(float(gv->iMax), 4) + "," + hf->to_string_with_precision(float(gv->jMax), 4) + ",0,";
            }
            else {
                gv->iMax = 0; gv->jMax = 0; gv->angleMax = 0;
                gv->messageToSendML += "0,0,0,";
            }
        }
        else {
            if (abs(gv->angleDiffMT) > 1.5f / 2.0f + cd->epsilon2) {
                gv->iMax = 0; gv->jMax = 0; gv->angleMax = 0;
                if (gv->angleDiffMT > 0) {
                    gv->angleMax = 1;
                }
                else if (gv->angleDiffMT < 0) {
                    gv->angleMax = -1;
                }
                gv->messageToSendML += "0,0," + hf->to_string_with_precision(float(gv->angleMax), 4) + ",";
            }
            else {
                gv->iMax = 0; gv->jMax = 0; gv->angleMax = 0;
                gv->messageToSendML += "0,0,0,";
            }
        }

        if (it == 1000) {
            gv->messageToSendML += hf->to_string_with_precision(gv->reward, 4) + ",1";

        }
        else {
            if (it != 0) gv->messageToSendML += hf->to_string_with_precision(gv->reward, 4) + ",0";

            gv->action = stoi(gv->receivedAnswerML);
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
                gv->reward = 1.0f;
            }
            else {
                gv->reward = -1.0f;
            }

            bool changed = false;

            if (gv->action != 0) {
                changed = true;
                gv->tempMTPos[0] += float(gv->iMin) * 0.5f;
                gv->tempMTPos[1] += float(gv->jMin) * 0.5f;
                gv->tempMTPos[2] += float(gv->angleMin) * 1.5f / 180.0f * M_PI;

                if (gv->tempMTPos[0] > 45.0f) {
                    gv->tempMTPos[0] = 45.0f;
                    changed = false;
                }
                if (gv->tempMTPos[0] < -45.0f) {
                    gv->tempMTPos[0] = -45.0f;
                    changed = false;
                }

                if (gv->tempMTPos[1] > 45.0f) {
                    gv->tempMTPos[1] = 45.0f;
                    changed = false;
                }
                if (gv->tempMTPos[1] < -12.5f) {
                    gv->tempMTPos[1] = -12.5f;
                    changed = false;
                }

                if (gv->tempMTPos[2] > 2 * M_PI) {
                    gv->tempMTPos[2] = 2 * M_PI;
                    changed = false;
                }
                if (gv->tempMTPos[2] < 0.0f) {
                    gv->tempMTPos[2] = 0.0f;
                    changed = false;
                }
            }

            if (gv->writeDataIntoFiles) {
                fstream file;
                file.open("mt_actions_" + to_string(mtIndex) + "_" + to_string(gv->simCounter) + ".csv", std::ios_base::app);
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
}

void Protagonists::safety_area_protagonist(int transportedWorkpiece, bool prognosis) {
	CollisionDetection *cd = CollisionDetection::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();

    gv->objectDimensionTemp.x = 0.5f;
    gv->objectDimensionTemp.y = 0.5f;
    gv->objectDimensionTemp.z = 0.5f;

    for (int it = 0; it <= 500; it++) {
        if (!prognosis) {
            gv->messageToSendML = "safety,";
        }
        else {
            gv->messageToSendML = "safety_p,";
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

        if (gv->currentDetectedObsPosX[0] > cd->epsilon3 || gv->currentDetectedObsPosY[0] > cd->epsilon3) {
            gv->messageToSendML += hf->to_string_with_precision(gv->distortedObstacles[0].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->distortedObstacles[0].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->distortedObstacles[0].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->distortedObstacles[0].angles[0]), 4) + ",";
        }
        else {
            gv->messageToSendML += "0,0,0,0,";
        }

        if (gv->currentDetectedObsPosX[1] > cd->epsilon3 || gv->currentDetectedObsPosY[1] > cd->epsilon3) {
            gv->messageToSendML += hf->to_string_with_precision(gv->distortedObstacles[1].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->distortedObstacles[1].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->distortedObstacles[1].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->distortedObstacles[1].angles[0]), 4) + ",";
        }
        else {
            gv->messageToSendML += "0,0,0,0,";
        }

        if (gv->currentDetectedObsPosX[2] > cd->epsilon3 || gv->currentDetectedObsPosY[2] > cd->epsilon3) {
            gv->messageToSendML += hf->to_string_with_precision(gv->distortedObstacles[2].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->distortedObstacles[2].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->distortedObstacles[2].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->distortedObstacles[2].angles[0]), 4) + ",";
        }
        else {
            gv->messageToSendML += "0,0,0,0,";
        }

        gv->messageToSendML += hf->to_string_with_precision(gv->colliders[15].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[15].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[15].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[15].angles[0]), 4) + ",";
        gv->messageToSendML += hf->to_string_with_precision(gv->colliders[16].offsets[0] / 55.0f, 4) + "," + hf->to_string_with_precision(gv->colliders[16].offsets[1] / 55.0f, 4) + "," + hf->to_string_with_precision(sin(gv->colliders[16].angles[0]), 4) + "," + hf->to_string_with_precision(cos(gv->colliders[16].angles[0]), 4) + ",";

        gv->messageToSendML += hf->to_string_with_precision(gv->objectDimension.x / 10.0f, 4) + "," + hf->to_string_with_precision(gv->objectDimension.y / 10.0f, 4) + "," + hf->to_string_with_precision(gv->objectDimension.z / 10.0f, 4) + ",";
        gv->messageToSendML += hf->to_string_with_precision(gv->objectDimensionTemp.x / 10.0f, 4) + "," + hf->to_string_with_precision(gv->objectDimensionTemp.y / 10.0f, 4) + "," + hf->to_string_with_precision(gv->objectDimensionTemp.z / 10.0f, 4) + ",";

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

        gv->diffVectorSafety[0] = gv->objectDimension.x - gv->objectDimensionTemp.x;
        gv->diffVectorSafety[1] = gv->objectDimension.y - gv->objectDimensionTemp.y;
        gv->diffVectorSafety[2] = gv->objectDimension.z - gv->objectDimensionTemp.z;

        if (abs(gv->diffVectorSafety[0]) < 0.01f + cd->epsilon2 && abs(gv->diffVectorSafety[1]) < 0.01f + cd->epsilon2 && abs(gv->diffVectorSafety[2]) < 0.01f + cd->epsilon2) {
            gv->iMax = 0; gv->jMax = 0, gv->kMax = 0;
            gv->messageToSendML += "0,0,0,";
        }
        else if (abs(gv->diffVectorSafety[0]) >= abs(gv->diffVectorSafety[1]) && abs(gv->diffVectorSafety[0]) >= abs(gv->diffVectorSafety[2])) {
            gv->jMax = 0, gv->kMax = 0;
            if (gv->diffVectorSafety[0] >= 0.02f) {
                gv->iMax = 1;
                gv->messageToSendML += "1,0,0,";
            }
            else if (gv->diffVectorSafety[0] <= -0.02f) {
                gv->iMax = -1;
                gv->messageToSendML += "-1,0,0,";
            }
            else {
                gv->iMax = 0;
                gv->messageToSendML += "0,0,0,";
            }
        }
        else if (abs(gv->diffVectorSafety[1]) >= abs(gv->diffVectorSafety[0]) && abs(gv->diffVectorSafety[1]) >= abs(gv->diffVectorSafety[2])) {
            gv->iMax = 0, gv->kMax = 0;
            if (gv->diffVectorSafety[1] >= 0.02f) {
                gv->jMax = 1;
                gv->messageToSendML += "0,1,0,";
            }
            else if (gv->diffVectorSafety[1] <= -0.02f) {
                gv->jMax = -1;
                gv->messageToSendML += "0,-1,0,";
            }
            else {
                gv->jMax = 0;
                gv->messageToSendML += "0,0,0,";
            }
        }
        else if (abs(gv->diffVectorSafety[2]) >= abs(gv->diffVectorSafety[0]) && abs(gv->diffVectorSafety[2]) >= abs(gv->diffVectorSafety[1])) {
            gv->iMax = 0, gv->jMax = 0;
            if (gv->diffVectorSafety[2] >= 0.02f) {
                gv->kMax = 1;
                gv->messageToSendML += "0,0,1,";
            }
            else if (gv->diffVectorSafety[2] <= -0.02f) {
                gv->kMax = -1;
                gv->messageToSendML += "0,0,-1,";
            }
            else {
                gv->kMax = 0;
                gv->messageToSendML += "0,0,0,";
            }
        }
        else {
            cout << "Error: invalid safety protagonist action!" << endl;

            gv->iMax = 0; gv->jMax = 0, gv->kMax = 0;
            gv->messageToSendML += "0,0,0,";
        }

        if (it == 500) {
            gv->messageToSendML += hf->to_string_with_precision(gv->reward, 4) + ",1";

        }
        else {
            if (it != 0) gv->messageToSendML += hf->to_string_with_precision(gv->reward, 4) + ",0";

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

            if (gv->action != 0) {
                changed = true;

                gv->objectDimensionTemp.x += float(gv->iMin) * 0.02f;
                gv->objectDimensionTemp.y += float(gv->jMin) * 0.02f;
                gv->objectDimensionTemp.z += float(gv->kMin) * 0.02f;

                if (gv->objectDimensionTemp.x > 7.5f) {
                    gv->objectDimensionTemp.x = 7.5f;
                    changed = false;
                }
                if (gv->objectDimensionTemp.x < 0.25f) {
                    gv->objectDimensionTemp.x = 0.25f;
                    changed = false;
                }

                if (gv->objectDimensionTemp.y > 7.5f){
                    gv->objectDimensionTemp.y = 7.5f;
                    changed = false;
                }
                if (gv->objectDimensionTemp.y < 0.25f) {
                    gv->objectDimensionTemp.y = 0.25f;
                    changed = false;
                }

                if (gv->objectDimensionTemp.z > 7.5f) {
                    gv->objectDimensionTemp.z = 7.5f;
                    changed = false;
                }
                if (gv->objectDimensionTemp.z < 0.25f) {
                    gv->objectDimensionTemp.z = 0.25f;
                    changed = false;
                }
            }

            if (gv->writeDataIntoFiles) {
                fstream file;
                file.open("safety_actions_" + to_string(gv->simCounter) + ".csv", std::ios_base::app);
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
}
