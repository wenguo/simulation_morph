/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef CANVAS_ROBOT_HH
#define CANVAS_ROBOT_HH
#include "robot.hh"

class CanvasRobot:public Robot
{
    public:
        CanvasRobot();
        ~CanvasRobot();

    protected:
        virtual void UpdateSensors();
        virtual void UpdateCommunication();
        virtual void UpdateActuators();

        virtual void Exploring();
        virtual void Resting();
        virtual void Seeding();
        virtual void Foraging();
        virtual void Assembly();
        virtual void LocateEnergy();
        virtual void LocateBeacon();
        virtual void Alignment();
        virtual void Docking();
        virtual void InOrganism();
        virtual void Disassembly();
        virtual void Recruitment();

};

#endif
