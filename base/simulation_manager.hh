/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef SIMULATON_MANAGER_HH
#define SIMULATON_MANAGER_HH
#include <iostream>
#include <fstream>
#include <list>
#include <map>
#include <string.h>
#include "support.hh"
#include "utils/worldfile.hh"
#include "utils/lolmsg.h"
#include "ipc_interface.hh"
#include "utils/ipc.hh"
#include "og/organism_sample.hh"


namespace Morph{

class Parameter
{
    public:
        Parameter():server(true), host(NULL){};
        ~Parameter(){};

        double Kp;
        double Kd;
        double Err;

        int num_used_robots;
        bool logtofile;
        int og_index;

        int V_max;
        double E_max;
        double E_hungry;
        double E_critical;
        double E1;
        double V1;
        double V2;

        double K1;
        double K2;
        double K3;
        double S1;
        double S2;

        //function 3
        double K_r;
        double k;
        double S3;

        vect2 pos_data[100];

        int server;
        char *host;
        int port;


        //timer threshold
        int recruiting_time;
        int seeding_time;

};

class state_record
{
    public: 
        state_record(int n)
        {
            ts = 0;
            num_robots_in_state = new int[n];
            memset(num_robots_in_state, 0, n*sizeof(int));
        }
        ~state_record()
        {
            if(num_robots_in_state!=NULL)
                delete num_robots_in_state;
        }
        long ts;
        int *num_robots_in_state;
};


class SimulationManager
{
    SimulationManager():
        worldgui(NULL),
        docking_trial(0),
        expelled_trial(0),
        successed_trial(0),
        failure_trial(0),
        robot_index(0),
        oldest_ts(0),
        latest_ts(0){}
    public:
    inline static SimulationManager * getInstance()
    {
        static SimulationManager *the_instance = NULL;

        static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
        pthread_mutex_lock(&mutex);

        if(the_instance ==NULL)
        {
            the_instance = new SimulationManager;
        }
        pthread_mutex_unlock(&mutex);
        return the_instance;
    }
    
    inline static int getBusIndex()
    {   
        static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
        static int bus_index=0;
        pthread_mutex_lock(&mutex);
            bus_index++;
        pthread_mutex_unlock(&mutex);
        return bus_index;


    }
    void AddRobot(void *robot);

    void logState(long ts, int state);
    void dumpStateRecord();
    bool LoadParameters(const char * filename="option.cfg");
    static void ProcessMessage(const LolMessage * msg, void *);
    bool IPCSendStatus(const IPC::status_t& state);
    bool IPCSendCommand(const IPC::command_t& cmd);
    bool IPCSendCameraPose(const IPC::camera_pose_t& cam_pose);
    bool IPCSendOrganismInfo(const IPC::organism_info_t& og_info);

    void * worldgui;
    Morph::Worldfile * optionfile;
    Parameter para;
    std::vector<void*> busList;

    int docking_trial;
    int failure_trial;
    int expelled_trial;
    int successed_trial;
    int robot_index; //used to indicate the index that robot is added into the simulation
    //int *num_robots_in_state;
    std::ofstream logFile;
    std::ofstream logstateFile;

    std::map<int, void*> robots_by_id;

    Organism * ogList[24];

    private:
    IPC::IPC ipc;
    std::list<state_record*> s_record;
    long oldest_ts; //oldest timestamp in list s_record
    long latest_ts; //lastest timestamp in list s_record

};

}

#endif
