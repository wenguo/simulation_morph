/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#include <stdlib.h>
#include <unistd.h>
#include "stage_robot.hh"

using namespace Morph;

extern "C" int Init( Model* mod )
{
    StageRobot *robot = new StageRobot;
    robot->Init(mod);

    return 0; //ok
}
