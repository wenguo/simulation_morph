/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <sstream>
#include "simulation_manager.hh"
#include "robot.hh"

namespace Morph
{

void SimulationManager::logState(long ts, int state)
{
    //create new record if not exist
    if(ts > latest_ts)
    {
        state_record *record = new state_record(STATE_COUNT); 
        record->ts = ts;
        record->num_robots_in_state[state]++;
        s_record.push_front(record);
        latest_ts = ts;
    }
    //otherwise find the record and push it in
    else
    {
        std::list<state_record*>::iterator it;
        for(it=s_record.begin();it!=s_record.end();it++)
        {
            if((*(it))->ts == ts)
            {
                (*(it))->num_robots_in_state[state]++;
                break;
            }
        }
    }
}

//TODO: update latest_ts if this is called on the fly instead of end of the simulation
void SimulationManager::dumpStateRecord()
{
    if(logstateFile.is_open())
    {
        state_record * record;
        std::list<state_record*> tmp_s_record;
        while(!s_record.empty())
        {
            std::stringstream oss;
            record = s_record.back();
            oss<<record->ts;
            int sum=0;
            for(int i=0;i<STATE_COUNT;i++)
            {
                oss<<"\t"<<record->num_robots_in_state[i];
                sum += record->num_robots_in_state[i];
            }

            s_record.pop_back();

            if(sum == para.num_used_robots)
            {
                delete record;
                logstateFile<<oss.str()<<"\t"<<sum<<std::endl;
            }
            else
            {
                tmp_s_record.push_front(record);
                std::cout<<"----- incomplete record -----" << std::endl;
            }
        }

        while(!tmp_s_record.empty())
        {
            std::cout<<"----- push back the incomplete record into list s_record -----" << std::endl;
            s_record.push_front(tmp_s_record.back());
            tmp_s_record.pop_back();
        }
    }
}

bool SimulationManager::LoadParameters(const char * filename)
{
    static bool loaded = false;

    if(loaded)
        return true;

    loaded = true;
    optionfile = new Morph::Worldfile();
    if(optionfile->Load(filename)==-1)
    {
        std::cout<<"can not find option.cfg, please check the path"<<std::endl;
        return false;
    }

    int pos_index=0;

    for( int entity = 1; entity < optionfile->GetEntityCount(); ++entity )
    {
        const char *typestr = (char*)optionfile->GetEntityType(entity);      	  

        // don't load window entries here
        if( strcmp( typestr, "Seeding" ) == 0 )
        {
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "motion_PD_para" ) )	
            {
                para.Kp = atof( optionfile->GetPropertyValue( prop, 0 ));
                para.Kd = atof( optionfile->GetPropertyValue( prop, 1 ));
                para.Err = atoi( optionfile->GetPropertyValue( prop, 2 ));
            }
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "waiting_time" ) )
            {
            }        

        }
        else if( strcmp( typestr, "Global" ) == 0 )
        {        
            para.num_used_robots = optionfile->ReadInt(entity, "num_robots", 2);
            para.logtofile = optionfile->ReadInt(entity, "logtofile", 1);
            para.og_index = optionfile->ReadInt(entity, "og_index", 0);
            para.E_hungry = optionfile->ReadFloat(entity, "E_hungry", 0.8); //remaining energy ratio
            para.E_critical = optionfile->ReadFloat(entity, "E_critical", 0.2);
            para.E1 = optionfile->ReadFloat(entity, "E1", 0.7);
            para.V1 = optionfile->ReadFloat(entity, "V1", 0.1);//blob size ratio 
            para.V2 = optionfile->ReadFloat(entity, "V2", 0.8);
            para.S1 = optionfile->ReadFloat(entity, "S1", 1.2);
            para.S2 = optionfile->ReadFloat(entity, "S2", 0.0);
            para.K1 = optionfile->ReadFloat(entity, "K1", 0.01);
            para.K2 = optionfile->ReadFloat(entity, "K2", 0.3);
            para.K3 = optionfile->ReadFloat(entity, "K3", 7.0);
            para.T_s = optionfile->ReadInt(entity, "T_s", 600);

            para.K_r = optionfile->ReadFloat(entity, "K_r", 1);
            para.k = optionfile->ReadFloat(entity, "k", 0.3);
            para.S3 = optionfile->ReadFloat(entity, "S3", 2700.0);
        }
        else if( strcmp( typestr, "InitPose" ) == 0 )
        {
            //initial position of robots
            vect2 center, size;
            int num;
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "center" ) )
            {
                center.x =  atof( optionfile->GetPropertyValue( prop, 0 ));
                center.y =  atof( optionfile->GetPropertyValue( prop, 1 ));
            }
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "size" ) )
            {
                size.x =  atof( optionfile->GetPropertyValue( prop, 0 ));
                size.y =  atof( optionfile->GetPropertyValue( prop, 1 ));
            }
            num =   optionfile->ReadInt(entity, "num", 0 );
            init_pos_matrix(center, size, para.pos_data + pos_index, num);
            pos_index +=num;
            printf("init pose %d -- %d\n", pos_index, num);
        }
        else if( strcmp( typestr, "colour") == 0)
        {
            for(int i=0;i<STATE_COUNT;i++)
            {
                //    const string& str = optionfile->ReadString(entity, status_t[i].name, status_t[i].color);
                //    status_t[i].color = strdup(str.c_str());
            }
        }
        else if( strcmp( typestr, "IPC") == 0)
        {
            const string& str = optionfile->ReadString(entity, "host", "localhost");
            para.host = strdup(str.c_str());
            para.port = optionfile->ReadInt(entity, "port", 10000);
            para.server = optionfile->ReadInt(entity, "server", 1);
        }
        else
            printf("loading something else %d\n", entity);
    }

    ipc.SetCallback(ProcessMessage, this);
    ipc.Start(para.host, para.port, para.server);

    for(int i=0;i<26;i++)
        ogList[i]=new Organism;

    dars2012a0(ogList[0]);
    dars2012a1(ogList[1]);
    dars2012a2(ogList[2]);
    dars2012a3(ogList[3]);
    dars2012a4(ogList[4]);
    dars2012a5(ogList[5]);
    dars2012a6(ogList[6]);
    dars2012a7(ogList[7]);
    dars2012a8(ogList[8]);
    dars2012a9(ogList[9]);
    dars2012a10(ogList[10]);
    dars2012b0(ogList[11]);
    dars2012b1(ogList[12]);
    dars2012b2(ogList[13]);
    dars2012b3(ogList[14]);
    dars2012b4(ogList[15]);
    dars2012b5(ogList[16]);
    dars2012b6(ogList[17]);
    dars2012b7(ogList[18]);
    dars2012b8(ogList[19]);
    dars2012b9(ogList[20]);
    dars2012b10(ogList[21]);

    snake(ogList[22]);
    humanlike(ogList[23]);
    MEchapter_shape1(ogList[24]);
    MEchapter_shape2(ogList[25]);

    return true;
}

void SimulationManager::ProcessMessage(const LolMessage * msg, void * ptr)
{
    if(!ptr)
        return;

    SimulationManager* sm = (SimulationManager*)ptr;

    switch (msg->command)
    {
        case IPC::STATUS_INFO:
            IPC::status_t state;
            memcpy(&state, msg->data, sizeof(IPC::status_t));
            if(sm->robots_by_id[state.id])
                ((Robot*)sm->robots_by_id[state.id])->SetStatus(state);
            break;
        case IPC::COMMAND:
            IPC::command_t cmd;
            memcpy(&cmd, msg->data, sizeof(IPC::command_t));
            if(sm->robots_by_id[cmd.id])
            {
                if(cmd.cmd_type == IPC::CMD_DOCKING)
                    ((Robot*)sm->robots_by_id[cmd.id])->Connect((robot_side)cmd.data[0]);
            }
            break;
        case IPC::CAMERA_POSE:
            IPC::camera_pose_t cam_pose;
            memcpy(&cam_pose, msg->data, sizeof(IPC::camera_pose_t));
            if(sm->robots_by_id[cam_pose.id])
                ((Robot*)sm->robots_by_id[cam_pose.id])->SetCameraPose(cam_pose);
            break;
        case IPC::ORGANISM_INFO:
            IPC::organism_info_t og_info;
            memcpy(&og_info, msg->data, 8);//copy the header
            og_info.data = new uint8_t[og_info.size]; //allocate the correct memory
            memcpy(og_info.data, msg->data + 8, og_info.size); //copy the data
            if(sm->robots_by_id[og_info.id])
                ((Robot*)sm->robots_by_id[og_info.id])->SetOrganismInfo(og_info);
            delete []og_info.data;
        default:
            break;
    }
}

bool SimulationManager::IPCSendStatus(const IPC::status_t& state)
{
    return ipc.SendData(IPC::STATUS_INFO, (uint8_t*)&state, sizeof(IPC::status_t));
}

bool SimulationManager::IPCSendCommand(const IPC::command_t &cmd)
{    
    return ipc.SendData(IPC::COMMAND, (uint8_t*)&cmd, sizeof(IPC::command_t));
}

bool SimulationManager::IPCSendCameraPose(const IPC::camera_pose_t&cam_pose)
{    
    return ipc.SendData(IPC::CAMERA_POSE, (uint8_t*)&cam_pose, sizeof(IPC::camera_pose_t));
}

bool SimulationManager::IPCSendOrganismInfo(const IPC::organism_info_t &og)
{
    uint8_t* data = new uint8_t[og.size + 8];
    memcpy(data, (uint8_t*)&og, 8);
    memcpy(data+8, og.data, og.size);
    bool ret = ipc.SendData(IPC::ORGANISM_INFO, data, og.size + 8); 
    delete data;
    return ret;
}


void SimulationManager::AddRobot(void* robot)
{
    if(robot)
        robots_by_id[((Robot*)robot)->id]=robot;
}

}//end namespace
