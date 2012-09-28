/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#include <osg/Vec3d>
#include <sys/time.h>
#include "robot3d_robot.hh"

namespace Morph{

#define ROBOT3D_ROBOTKIT_SIZE 2.2 //robot size (x,y) in robot3d, don't care about z
#define STAGE_ROBOTKIT_SIZE 0.09
const double cf = ROBOT3D_ROBOTKIT_SIZE / STAGE_ROBOTKIT_SIZE;

const char robot3d_side[SIDE_COUNT]={'B', 'A', 'D', 'C'};

Robot3dRobot::Robot3dRobot(srCore::Agent& agent, std::string properties):
    Robot(),
    HAL(agent),
    _agent(agent),
    _size(ROBOT3D_ROBOTKIT_SIZE),
    dock_required(0),
    undock_required(0),
    macrolocomotion_count(0),
    newCameraPose(false),
    camPos(osg::Vec3f(-40,40,10)),
    camOri(osg::Vec3f(0,0,0)),
    period(1000),
    precision(100),
    upperLimit(1.4),
    lowerLimit(-1.4)
{
    phaseShift = atof(properties.c_str());

    if(name)
        free(name);

    name = strdup(getRobotId().c_str());

    //get robot id, TODO: be careful if robot is not name as rX
    sscanf(name,"r%d", &id);
    printf("name: %s id: %d\n", name, id);

    //disable dynamics by default
    _agent.getRobot()->EnableDynamics(false);

    Init(name);
}

Robot3dRobot::~Robot3dRobot()
{

}

void Robot3dRobot::Init(void *ptr)
{
    LoadParameters();

    Robot::Init(ptr);
    sm->AddRobot(this);

}

void Robot3dRobot::LoadParameters()
{
    if(!sm->LoadParameters("robot3d_option.cfg"))
        exit();

    para = &sm->para;    
}

//The method is called after t = 4.48*timeScale / FPS [s]
void Robot3dRobot::onTickMessage()
{
    timeval t;
    gettimeofday(&t,NULL);
    uint32_t current_time = (uint32_t)(t.tv_sec * 1000 + t.tv_usec/1000 );
    int32_t realtime_interval = current_time - real_timestamp;
    int32_t simulation_interval = getTime() - timestamp;
    real_timestamp = current_time;
    timestamp = getTime();
    //printf("%d %s Ticking message %d %d -- ratio: %f\n", timestamp, name, simulation_interval, realtime_interval, realtime_interval*1.0/simulation_interval);

    Update();
}

void Robot3dRobot::SetLED(uint8_t side, bool on)
{
}

void Robot3dRobot::UpdateSensors()
{
}
void Robot3dRobot::UpdateCommunication()
{
}
void Robot3dRobot::UpdateActuators()
{
    //set new position
    if(newPose)
    {
        osg::Vec3d pos = _agent.getRobotPosition();
        osg::Vec3d rot = _agent.getRobot()->getRotation();
        newPose = false;
        pos[0] = _x;
        pos[1] = _y;
        rot[0] = _pa * 180 / 3.1415926 + 90;
        _agent.setRobotTranslation(pos);
        _agent.getRobot()->setRotation(rot);
    }

    //checking connector status
    for(int i=0;i<SIDE_COUNT;i++)
    {
        if(isConnected(robot3d_side[i]))
        {
            //set flag
            docked[i] = true;
            //clear dockign request
            dock_required &= ~(1<<i);
        }
    }

    //need to process docking command?
    if(dock_required)
    {
        for(int i=0;i<SIDE_COUNT;i++)
        {
            if(dock_required & (1<<i))
            {
                printf("%d -- %s is trying to connect via connector No.%c\n", timestamp, name, robot3d_side[i]);
                connect(robot3d_side[i]);
            }
        }
    }

    //need to process un-docking command?
    if(undock_required)
    {
        for(int i=0;i<SIDE_COUNT;i++)
        {
            if(undock_required & (1<<i))
                disconnect(robot3d_side[i]);
        }
    }


    //checking camera pos
    if(newCameraPose)
    {
        newCameraPose = false;
        dtCore::Transform xform;
        osg::Vec3 up = osg::Vec3(0,0,1);
        xform.Set(camPos, camOri, up);
        _agent.GetGameManager()->GetApplication().GetCamera()->SetTransform(xform);
    }

}

void Robot3dRobot::SetStatus(const IPC::status_t& status)
{
    //set pose only when robots are not connected, to avoid robots crashing
    bool flag = false;
    for(int i=0;i<SIDE_COUNT;i++)
    {
        if(docked[i])
        {
            flag=true;
            break;
        }
    }
    if(!flag)
        SetGlobalPose(status.pos);

    //set state
    if(status.state < STATE_COUNT)
        current_state = (fsm_state_t)status.state;
}


void Robot3dRobot::SetCameraPose(const IPC::camera_pose_t& cam_pose)
{
    if(!newCameraPose)
    {
        newCameraPose = true;
        camPos[0] = cam_pose.x * cf / 1000.0;
        camPos[1] = cam_pose.y * cf / 1000.0;
        camPos[2] = cam_pose.z * cf / 1000.0;
        camOri[2] = cam_pose.yaw * cf / 1000.0;
        camOri[1] = cam_pose.pitch * cf / 1000.0;
        camOri[0] = 0;
    }
}

void Robot3dRobot::SetGlobalPose(const IPC::position_t& pos)
{
    if(!newPose)
    {
        newPose = true;
        _x = pos.x * cf / 1000.0;
        _y = pos.y * cf / 1000.0;
        _pa = pos.pa / 1000.0;
    }
}

bool Robot3dRobot::Connect(robot_side side)
{
    dock_required |= 1 << side;
}

void Robot3dRobot::Exploring()
{
}
void Robot3dRobot::Resting()
{
}
void Robot3dRobot::Seeding()
{
}
void Robot3dRobot::Foraging()
{
}
void Robot3dRobot::Assembly()
{
}
void Robot3dRobot::LocateEnergy()
{
}
void Robot3dRobot::LocateBeacon()
{
}
void Robot3dRobot::Alignment()
{
}
void Robot3dRobot::Recover()
{
}
void Robot3dRobot::Docking()
{
}
void Robot3dRobot::InOrganism()
{
    _agent.getRobot()->setSelectionState(true);
    _agent.getRobot()->renderCollisionGeometry(true);
}
void Robot3dRobot::Disassembly()
{
}

void Robot3dRobot::Undocking()
{
}

void Robot3dRobot::Reshaping()
{
}

void Robot3dRobot::Recruiting()
{
}

void Robot3dRobot::MacroLocomotion()
{
    return;
    //added a short delay to make sure the last robot are connected
    //as stage and robot3d are running in differnt timescale
    macrolocomotion_count++;
    if(macrolocomotion_count < 5)
    {
        _agent.getRobot()->EnableDynamics(true);
        return;
    }
    //sinusMove coped from tutorial
 //   _agent.getRobot()->PrintConnectorStates();
    double time = getTime();
    float phase = (int(( (float)(time)/period + phaseShift) * precision) % std::max(1,int(precision))) / precision;
    float range = fabs(upperLimit) + fabs(lowerLimit);
    float center = (upperLimit + lowerLimit) / 2;
    moveActuator(0, sin(2*M_PI*phase) * range/2 + center);

    //send position back to stage
    osg::Vec3d pos = _agent.getRobotPosition();
    osg::Vec3d rot = _agent.getRobot()->getRotation();
    pos[0] = pos[0] / cf;
    pos[1] = pos[1] / cf;
    rot[0] = (rot[0] - 90 )* 3.1415926 / 180.0;
    IPCSendStatus(pos[0], pos[1], rot[0]);
}
}
