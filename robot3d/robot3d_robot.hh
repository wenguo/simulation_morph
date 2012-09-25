/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef ROBOT3D_ROBOT_HH
#define ROBOT3D_ROBOT_HH

#include <srCore/hal.h>
#include <srCore/agent.h>
#include "robot.hh"

namespace Morph{

class Robot3dRobot:public Robot, public HAL
{
    public:
        Robot3dRobot(srCore::Agent& agent, std::string properties="");
        ~Robot3dRobot();

        void onTickMessage();
        void task(){};

        void Init(void *);
        
        void SetLED(uint8_t side, bool on);
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
        virtual void Recover();
        virtual void Docking();
        virtual void InOrganism();
        virtual void Disassembly();
        virtual void Undocking();
        virtual void Reshaping();
        virtual void Recruitment();
        virtual void MacroLocomotion();

        virtual void LoadParameters();
        virtual void SetStatus(const IPC::status_t& status);
        virtual void SetGlobalPose(const IPC::position_t&);
        virtual void SetCameraPose(const IPC::camera_pose_t& cam_pose);

        virtual bool Connect(robot_side side);

        virtual void BroadcastIRMessage(int channel, char type, const uint8_t data){}; //broadcast message via wireless 
        virtual void BroadcastIRMessage(int channel, const uint8_t data){}; //broadcast message via wireless 
        virtual void BroadcastIRMessage(int channel, char type, int size, const uint8_t *data){};

    private:
        srCore::Agent& _agent;
        uint32_t real_timestamp;

        double _size;
        uint8_t dock_required; //docking command from stage need to be processed
        uint8_t undock_required; //undocking command from stage need to be processed

        bool newCameraPose;
        osg::Vec3f camPos;
        osg::Vec3f camOri;
        
	float phaseShift;					//of the controller
	float period;						//of the periodic behaviour in milliseconds
	float precision;					//of the control
	float upperLimit, lowerLimit;				//range of the hinge

        int macrolocomotion_count;

};

}
#endif
