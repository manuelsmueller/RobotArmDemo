/* File to automate the process of simulation in Simulator*/

#include "AutomateSimState.h"

AutoSim::AutoSim() {

}

AutoSim::~AutoSim() {

}

/*
 * singelton pattern
 */
AutoSim* AutoSim::autoss = 0;
AutoSim* AutoSim::get_instance(){
	static bool isInit=false;


	if(!isInit){
		autoss  = new AutoSim();
		isInit=true;
	}
	return autoss;
}

// Automate the initilaization process that is done manually by pressing 'i'
void AutoSim::automateInit()
{
    GlobalVariables *gv=GlobalVariables::get_instance();
    if (gv->atStandby) {
			gv->asynMode = 1;
			gv->initConfirmation = true;
			gv->simConfirmation = false;
			gv->manualConfirmation = false;

			gv->initSpacePressed = false;
			cout << "Info: changing operation mode to initialization" << endl;
		}
		else cout << "Warning: move the real robot back to the standby position first before switching to init. mode!" << endl;
}

// Automate the process of placing workpieces and MTs in workspace that is done manually by pressing spacebar
void AutoSim::automateSetWorkpiece()
{
    GlobalVariables *gv=GlobalVariables::get_instance();
    if (gv->synMode == 1) gv->initSpacePressed = true;
}

// Automate the process of placing obstacles in workspace that is done manually by pressing 's'
void AutoSim::automateSimulation()
{
    GlobalVariables *gv=GlobalVariables::get_instance();
    if (gv->atStandby) {
    if (!gv->initProcessed) cout << "Warning: initialization was not processed yet. Do this first!" << endl;
    else {
        gv->asynMode = 2;
        gv->initConfirmation = false;
        gv->simConfirmation = true;
        gv->manualConfirmation = false;
        cout << "Info: changing operation mode to simulation" << endl;
    }
}
else cout << "Warning: move the real robot back to the standby position first before switching to simulation mode!" << endl;
}

// Automate the process of selecting target and starting transportation
bool AutoSim::automateTransportation()
{
	GlobalVariables *gv=GlobalVariables::get_instance();
	HelperFunctions *hf = HelperFunctions::get_instance();
	hf->selectionMarkerModeSwitch();
	if (gv->synMode == 2) 
	{
		gv->selectedWorkpieceIndex = WORKPIECEINDEX;
		//Load the target location for workpiece
		gv->selectedTargetPosition[0] = TARGETX;
		gv->selectedTargetPosition[1] = TARGETY;
		gv->selectedTargetPosition[2] = TARGETZ;
		gv->selectedTargetPosition[3] = TARGETROT;
		gv->selectionMarkerMode = false;
		gv->simTargetSelection = false;
		gv->simTransport = true;

		return true;
	}

	return false;

}