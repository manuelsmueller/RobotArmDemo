/*
 * SimulationControl.h
 *
 *  Created on: 18.12.2022
 *      Author: manuel
 */

#ifndef SIMULATIONCONTROL_H_
#define SIMULATIONCONTROL_H_

#include "InverseKinematic.h"
#include "GlobalVariables.h"
#include "PathPlanner.h"
#include "ConvHull3d.h"
#include "FrequentOps.h"
#include "Protagonists.h"
#include "Adversaries.h"
#include "RealRobotExecution.h"

#include "MixedFunctions.h"

#include "ObjectPositionInterface.h"

class SimulationControl {

private:
	static SimulationControl* sc;

	SimulationControl();
	void validate_obstacle_positions();

	void Position_Obstacles_from_MTs(FrequentOps *fo, CollisionDetection *cd, GlobalVariables *gv);
	void Position_Obstacle_from_Ext(unsigned int index, bool boOverride, float x1,float y1,float phi1 );// old and deprecated
	void Position_Obstacle_from_Ext(bool boOverride, ObjectPositionInterface obj);
	void Position_Obstacles_Randomly(ObjectPositionInterface *obj,bool boOverride);

	//deprecated old version. Better use functions above.
	void Position_Obstacles_Old(FrequentOps *fo, CollisionDetection *cd, GlobalVariables *gv);

public:
	static SimulationControl* get_instance();

	virtual ~SimulationControl();
	void train_agents(bool &repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);
	bool sim_control_keyup(SDL_Event event, bool repaint_old, bool boWaitForSpace);

	void sim_control_mouse_ctrl(SDL_Event event, SDL_Window* window,
			bool &repaint,
			bool &forwards,
			bool &backwards,
			bool &left,
			bool &right,
			bool &up,
			bool &down,
			bool &rotationDown,
			bool &rotationUp);

	void sim_control_joystick(SDL_Event event,
			bool &repaint,
			bool &forwards,
			bool &backwards,
			bool &left,
			bool &right,
			bool &up,
			bool &down,
			bool &rotationDown,
			bool &rotationUp);

	bool sim_control_close();

	void sim_control_take_action_to_user_input(
			bool &repaint,
			bool &forwards,
			bool &backwards,
			bool &left,
			bool &right,
			bool &up,
			bool &down,
			bool &rotationDown,
			bool &rotationUp
	);

	void sim_control_wait_for_completed_init(bool boWaitForUserInput);
	void sim_control_do_something2();

	bool do_something5(bool repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	bool do_something6(bool repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	void sim_control_enter_seed(
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);
	void sim_control_reorganize_after_init(
			bool &repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);
	void sim_control_add_obj_during_initialization(
			bool &repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	void sim_control_select_agents();

	void sim_control_print_prepare_models();

	void sim_control_apply_changes_from_init();

	void sim_control_virtual_mode1();

	bool sim_control_do_something3();
	bool sim_control_position_obstacles(ObjectPositionInterface *obj, bool write);

	void sim_control_apply_agents_actions();
	void sim_control_recalculate_colliders();

	void sim_control_calc_edges();

	void sim_control_do_something4(
			bool &repaint,
			default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	void sim_control_sync_sim(default_random_engine generator1,
			normal_distribution<float> distribution1,
			default_random_engine generator2,
			normal_distribution<float> distribution2,
			default_random_engine generator3,
			normal_distribution<float> distribution3);

	void sim_control_clean_objects(bool &repaint);
	void sim_control_simple_add_obj_on_init(bool &repaint);

	void transport_phase1();
};

#endif /* SIMULATIONCONTROL_H_ */
