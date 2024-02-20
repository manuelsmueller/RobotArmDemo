/*
 * RealRobotExecution.h
 *
 *  Created on: 22.01.2023
 *      Author: manuel
 */

#ifndef REALROBOTEXECUTION_H_
#define REALROBOTEXECUTION_H_

#include "GlobalVariables.h"
#include "MonitoringToolModels.h"
#include "FrequentOps.h"
#include "HelperFunctions.h"
#include "Protagonists.h"

#include "ObjectPositionInterface.h"


#include <iostream>

class RealRobotExecution {
private:
	RealRobotExecution();
	static RealRobotExecution* rre;
public:
	virtual ~RealRobotExecution();
	static RealRobotExecution* get_instance();

	void sim_control_ask_real_execution();
	void sim_control_physical_mode();

	void sim_control_transportPhase1(bool &repaint,
				default_random_engine generator1,
				normal_distribution<float> distribution1,
				default_random_engine generator2,
				normal_distribution<float> distribution2,
				default_random_engine generator3,
				normal_distribution<float> distribution3);

	void sim_control_transportPhas2(
			bool &repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	bool real_execution_phase0(bool repaint);

	bool position_monitoring_tools1(bool repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	bool execute_real_phase2(bool repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	bool real_exec_phase3(bool repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	bool real_exec_phase4(bool repaint,
				default_random_engine generator1,
				normal_distribution<float> distribution1,
				default_random_engine generator2,
				normal_distribution<float> distribution2,
				default_random_engine generator3,
				normal_distribution<float> distribution3);

	bool real_exec_phase5(bool repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	bool real_exec_phase6(bool repaint);

	bool real_exec_phase7(bool repaint,
				default_random_engine generator1,
				normal_distribution<float> distribution1,
				default_random_engine generator2,
				normal_distribution<float> distribution2,
				default_random_engine generator3,
				normal_distribution<float> distribution3);
};

#endif /* REALROBOTEXECUTION_H_ */
