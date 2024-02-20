/*
 * Protagonists.h
 *
 *  Created on: 05.12.2022
 *      Author: manuel
 */

#ifndef PROTAGONISTS_H_
#define PROTAGONISTS_H_

#include "MonitoringToolModels.h"
#include "CollisionDetection.h"
#include "GlobalVariables.h"
#include "FrequentOps.h"
#include "HelperFunctions.h"


class Protagonists {

private:
	GlobalVariables *gv;
	HelperFunctions *hf;
	MonitoringToolModels *mt;
	static Protagonists* prot;
	Protagonists();
public:
	virtual ~Protagonists();
	static Protagonists* get_instance();

	void mt_preparation(int mtIndex, default_random_engine generator1, normal_distribution<float> distribution1, default_random_engine generator2, normal_distribution<float>  distribution2, default_random_engine generator3, normal_distribution<float> distribution3) ;
	void mt_protagonist(int mtIndex, int transportedWorkpiece, bool prognosis);// {
	void safety_area_protagonist(int transportedWorkpiece, bool prognosis);// {



};

#endif /* PROTAGONISTS_H_ */
