/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#include "robot.hh"
#include "support.hh"

namespace Morph{

const char* state_names[STATE_COUNT]={
    "Exploring",
    "Resting",
    "Seeding",
    "Foraging",
    "Assembly",
    "LocateEnergy",
    "LocateBeacon",
    "Alignment",
    "Recover",
    "Docking",
    "Disassembly",
    "InOrganism",
    "Undocking",
    "Reshaping",
    "Recruitment",
    "MacroLocomotion"
};


Robot::Robot():
    beacon_signals_detected(0),
    recruitment_signals_detected(0),
    msg_undocked_received(0),
    msg_reshaping_received(0),
    robot_in_range_replied(0),
    expelling_signals_detected(0),
    bumped(0),
    type(ROBOT_KIT),
    active(true),
    fp_fsm(NULL),
    seeding_count(0),
    foraging_blind_count(0),
    logtofile(true),
    docking_done_syn(false),
    assembly_info_checked(false),
    num_robots_inorganism(1),
    timestamp(0),
    organism_found(false),
    powersource_found(false),
    shape_completed(false),
    seed(false),
    organism_formed(false),
    og(NULL),
    bus_id(0),
    com_bus(NULL),
    leftwheel(0),
    rightwheel(0),
    newPose(false)
{
    name = strdup("Robot");
    timestamp = 0;

    sm = SimulationManager::getInstance();
    
    

    RegisterBehaviour(&Robot::Exploring, EXPLORING);
    RegisterBehaviour(&Robot::Resting, RESTING);
    RegisterBehaviour(&Robot::Seeding, SEEDING);
    RegisterBehaviour(&Robot::Foraging, FORAGING);
    RegisterBehaviour(&Robot::Assembly, ASSEMBLY);
    RegisterBehaviour(&Robot::LocateEnergy,LOCATEENERGY);
    RegisterBehaviour(&Robot::LocateBeacon, LOCATEBEACON);
    RegisterBehaviour(&Robot::Alignment, ALIGNMENT);
    RegisterBehaviour(&Robot::Recover, RECOVER);
    RegisterBehaviour(&Robot::Docking, DOCKING);
    RegisterBehaviour(&Robot::InOrganism, INORGANISM);
    RegisterBehaviour(&Robot::Disassembly, DISASSEMBLY);
    RegisterBehaviour(&Robot::Undocking, UNDOCKING);
    RegisterBehaviour(&Robot::Reshaping, RESHAPING);
    RegisterBehaviour(&Robot::Recruitment, RECRUITMENT);
    RegisterBehaviour(&Robot::MacroLocomotion, MACROLOCOMOTION);

    current_state = EXPLORING;
    last_state = EXPLORING;


    for (int i = 0; i < NUM_IRS; i++)
    {
        proximity[i] = 0;
        beacon[i] = 0;
    }

    for (int i = 0; i < NUM_DOCKS; i++)
    {
        comm_status[i] = 0;
        docked[i] = false;
        docking_done[i] = false;
        recruitment_signal_interval_count[i]=DEFAULT_RECRUITMENT_COUNT;
        recruitment_count[i]=0;
        neighbour_message[i]=NULL;
        neighbours[i]=NULL;
    }

    //clear just in case 
    mytree.Clear();

}

Robot::~Robot()
{

}

void Robot::Init(void *rname)
{
    //format log file name;
    std::string time_string;
    ::time_t time_now = time(NULL);
    struct tm * timeinfo;
    timeinfo = localtime (&time_now);
    time_string = datetime_to_string(*timeinfo, "%Y%m%d%H%M");
    if(logtofile)
    {
        std::ostringstream oss;
        oss << "./log/"<< time_string << "/";
        mkdir(oss.str().c_str(), 0777);
        oss << (char *)rname << ".log";
        logFile.open(oss.str().c_str());
    }    

    if(!sm->logFile.is_open() &&
            !sm->logstateFile.is_open())
    {
        int exp_no = 0;
        std::fstream tempfile;
        tempfile.open("./log/exp_no");
        if(tempfile.is_open())
        {
            tempfile>>exp_no;
            exp_no++;
            tempfile.seekg(std::ios::beg);
            tempfile<<exp_no<<std::endl;
            tempfile.close();
        }
        std::ostringstream oss;
        oss << "./log/exp_" << exp_no << ".log";
        sm->logFile.open(oss.str().c_str());
        std::ostringstream oss1;
        oss1 << "./log/exp_" << exp_no << ".state";
        sm->logstateFile.open(oss1.str().c_str());
    }

}

int Robot::RegisterBehaviour(robot_callback_t fnp, fsm_state_t state)
{
    behaviours[state] = fnp;
}

void Robot::Update()
{
    //update Sensors
    UpdateSensors();
    UpdateCommunication();

    //behaviour
    //    printf("%s %s %d\n",name, state_names[current_state], timestamp);
    behaviours[current_state](this);

    //update actuators
    UpdateActuators();

    sm->logState(timestamp, current_state);


    if(active && logtofile)
        Log();

}

void Robot::UpdateSensors()
{
    memset(proximity, 0, sizeof(int32_t)*NUM_IRS);
    memset(beacon, 0, sizeof(int32_t)*NUM_IRS);
}
void Robot::UpdateCommunication()
{
}
void Robot::UpdateActuators()
{
}

void Robot::Exploring()
{
}
void Robot::Resting()
{
}
void Robot::Seeding()
{
}
void Robot::Foraging()
{
}
void Robot::Assembly()
{
}
void Robot::LocateEnergy()
{
}
void Robot::LocateBeacon()
{
}
void Robot::Alignment()
{
}
void Robot::Recover()
{
}
void Robot::Docking()
{
}
void Robot::InOrganism()
{
}
void Robot::Disassembly()
{
}

void Robot::Undocking()
{
}

void Robot::Reshaping()
{
}

void Robot::Recruitment()
{
}

void Robot::MacroLocomotion()
{
}

void Robot::SetOgSequence(const OrganismSequence& og_seq)
{
    mytree = og_seq;
}

const OrganismSequence& Robot::GetOgSequence()
{
    return mytree;
}

bool Robot::isNeighboured(Robot * r)
{
    for (int i = RIGHT; i <= LEFT; i++)
    {
        if(neighbours[i] && neighbours[i] == r)
            return true;
    }

    return false;
}

void Robot::BroadcastMessage(Message msg)
{
    if(!com_bus)
        return;

    std::vector<Robot*> *robotList  = &(com_bus->CommunicationNodeList);
    for (int i=0; i<robotList->size(); i++)
    {
        //if((*robotList)[i] !=this) //no need send message to myself
        (*robotList)[i]->blackboard.push_back(new Message(msg));
        std::cout<<(*robotList)[i]->name<<std::endl;
    }

//    std::cout<<timestamp<<" : "<<msg<<std::endl;
}

void Robot::SendBranchTree(int channel, const OrganismSequence& seq)
{
    if(!neighbours[channel] || seq.Size()<=0)
        return;

    OrganismSequence::Symbol connection_info = seq.getSymbol(0);

    if(channel != (int) connection_info.side1)
    {
        std::cout<<"ERROR!!! sending branch tree ( "<<seq<<" ) in wrong channel" << channel<<std::endl;
        return;
    }

    if(seq.Size() > 0)
    {
        uint8_t *data = new uint8_t[seq.Size()];
        for(int i=0; i < seq.Size();i++)
            data[i] =seq.Encoded_Seq()[i].data;
        
        neighbours[channel]->neighbour_message[connection_info.side2] = new Message(name, neighbours[channel]->name, MSG_TYPE_GROWING, data, seq.Size(), timestamp);
        std::cout<<timestamp<<": "<<name<<" send branch:"<<seq<<std::endl;
        //std::cout<<*neighbours[channel]->neighbour_message[connection_info.side2]<<std::endl;
    }
    else
    {
        printf("ERROR!!!!, empty branches, shouldn't be here\n");
    }

}

void Robot::SendMessage(int channel, Message * msg)
{
    //NULL point
    if(!msg)
        return;

    //no robot attached on that channel
    if(!neighbours[channel])
    {
        delete msg;
        return;
    }

    for(int i=0; i<SIDE_COUNT; i++)
    {
        if(neighbours[channel]->neighbours[i] == this)
        {
            //release the old message if appliable
            if(neighbours[channel]->neighbour_message[i])
                delete neighbours[channel]->neighbour_message[i];

            neighbours[channel]->neighbour_message[i]=msg;
//            std::cout<<timestamp<<" : "<<*msg<<std::endl;
            break;
        }
    }
}


bool Robot::IPCSendStatus(double x, double y, double pa)
{
    if(!sm)
        return false;

    IPC::status_t status;
    status.id = id;
    status.state = current_state;
    //    state.state = state.state << 16 | last_state;
    status.pos.x = x * 1000;
    status.pos.y = y * 1000;
    status.pos.pa = pa * 1000;

    return sm->IPCSendStatus(status);
}

bool Robot::IPCSendCommand(uint8_t cmd_type, uint8_t * data, uint8_t len)
{
    if(!sm)
        return false;

    IPC::command_t cmd;
    cmd.id = id;
    cmd.cmd_type = cmd_type;
    memcpy(cmd.data, data, len);

    return sm->IPCSendCommand(cmd);
}

bool Robot::IPCSendDockingCommand(int side, bool flag)
{
    uint8_t data[5];
    data[0]=side;
    data[1]=0;
    data[2]=0;
    data[3]=0;
    data[4]=0;
    if(flag)
        return IPCSendCommand(IPC::CMD_DOCKING, data, 5);
    else
        return IPCSendCommand(IPC::CMD_UNDOCKING, data, 5);
}

bool Robot::IPCSendOrganismInfo()
{
    if(!sm || mytree.Size()<=0 )
        return false;

    IPC::organism_info_t og_info;
    og_info.id = id;
    og_info.size = mytree.Size();
    og_info.data = new uint8_t[og_info.size];
    for(int i=0; i < mytree.Size();i++)
        og_info.data[i] =mytree.Encoded_Seq()[i].data;
    bool ret = sm->IPCSendOrganismInfo(og_info);
    delete []og_info.data;

    return ret;
}

bool Robot::IPCSendCameraPose(double x, double y, double z, double yaw, double pitch)
{
    if(!sm)
        return false;

    IPC::camera_pose_t cam_pose;
    cam_pose.id = id;
    cam_pose.x = x * 1000;
    cam_pose.y = y * 1000;
    cam_pose.z = z * 1000;
    cam_pose.yaw = yaw * 1000;
    cam_pose.pitch = pitch * 1000;
    return sm->IPCSendCameraPose(cam_pose);

}


bool Robot::SetOrganismInfo(const IPC::organism_info_t& og_info)
{
    rt_status ret = mytree.reBuild(og_info.data, og_info.size);
    std::cout<<"Received organism info from Stage:"<<mytree<<std::endl;
    return ret.status < RT_ERROR;
}

std::string Robot::ClockString()
{
    const uint32_t msec_per_hour   = 36000U; //100msec
    const uint32_t msec_per_minute = 600U;
    const uint32_t msec_per_second = 10U;

    uint32_t hours   = timestamp / msec_per_hour;
    uint32_t minutes = (timestamp % msec_per_hour) / msec_per_minute;
    uint32_t seconds = (timestamp % msec_per_minute) / msec_per_second;
    uint32_t msec    = (timestamp % msec_per_second) ;

    std::string str;
    char buf[256];

    if ( hours > 0 )
    {
        snprintf( buf, 255, "%uh:", hours );
        str += buf;
    }

    snprintf( buf, 255, " %um %02us %03umsec", minutes, seconds, 100* msec);
    str += buf;

    return str;
}



//below defined functions for debugging
void Robot::PrintProximitySensor()
{
    std::cout << timestamp << ": robot " << name << "-IR --[";
    for (int i = 0; i < NUM_IRS; i++)
        std::cout << proximity[i]<<" ";
    std::cout << "]" << std::endl;
}

void Robot::PrintStatus()
{
    std::cout << timestamp << ": robot" << name << " in state " << state_names[current_state]
        << " [" << state_names[last_state] << "] recover count: " << recover_count
        << " speed (" << leftwheel << " , " << rightwheel << " )"  << std::endl;
}

void Robot::Log()
{
    if (logFile.is_open())
    {
        logFile << timestamp <<" ("<<ClockString() << "):" << name << " - " << state_names[current_state] <<" <-- "<< state_names[last_state] <<"\t ";
        for(int i=0; i< SIDE_COUNT; i++)
        {
            logFile<<"("<<side_names[i];
            if(docked[i] && neighbours[i])
                logFile<<") "<<setw(5)<<neighbours[i]->name<<" | ";
            else
                logFile<<") false | ";
        }

        logFile << std::endl;
     /* 
        for(int i=0; i< SIDE_COUNT; i++)
        {
            if(neighbour_message[i])
                logFile<<"\t--"<<side_names[i]<<" - "<<*neighbour_message[i]<<std::endl;;
        }
*/
    }

}

}
