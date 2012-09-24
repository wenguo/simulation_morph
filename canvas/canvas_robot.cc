/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#include "canvas_robot.hh"

CanvasRobot::CanvasRobot():Robot()
{
    if(name)
        free(name);

    name = strdup("CanvasRobot");

}

CanvasRobot::~CanvasRobot()
{

}

void CanvasRobot::UpdateSensors()
{
    printf("Update Sensors %d\n", timestamp);
}
void CanvasRobot::UpdateCommunication()
{
    printf("Update Communication %d\n", timestamp);
}
void CanvasRobot::UpdateActuators()
{
    printf("Update Actuators %d\n", timestamp);
}

void CanvasRobot::Exploring()
{
    std::cout<<mytree<<std::endl;
}
void CanvasRobot::Resting()
{
}
void CanvasRobot::Seeding()
{
}
void CanvasRobot::Foraging()
{
}
void CanvasRobot::Assembly()
{
}
void CanvasRobot::LocateEnergy()
{
}
void CanvasRobot::LocateBeacon()
{
}
void CanvasRobot::Alignment()
{
}
void CanvasRobot::Docking()
{
}
void CanvasRobot::InOrganism()
{
}
void CanvasRobot::Disassembly()
{
}
void CanvasRobot::Recruitment()
{
}

