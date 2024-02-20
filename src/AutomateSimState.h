#ifndef AUTOMATESIMSTATE_H_
#define AUTOMATESIMSTATE_H_

#include "GlobalVariables.h"
#include "HelperFunctions.h"

// choose the work piece ID to be transported by the robot
#define WORKPIECEINDEX 3
// choose the target destination's x coordinate to which the selected work piece is to transported by the robot
#define TARGETX -18.5// T1:7.5,T2:14.75, T3:19.5, T4:16.25, T5: -18.5
// choose the target destination's y coordinate to which the selected work piece is to transported by the robot
#define TARGETY 21.75//T1:30, T2:23, T3:12, T4: 14.5, T5: 21.75
// choose the target destination's z coordinate to which the selected work piece is to transported by the robot
#define TARGETZ 0
// choose the target yaw the selected work piece should be placed at the destination
#define TARGETROT 0

class AutoSim
{
private:

    AutoSim();
    static AutoSim *autoss;
public:
virtual ~AutoSim();
static AutoSim* get_instance();
void automateInit();
void automateSetWorkpiece();
void automateSimulation();
bool automateTransportation();

};




#endif /* AUTOMATESIMSTATE_H_*/