/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef ROBOT_HH
#define ROBOT_HH
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <locale>
#include <stdexcept>
#include <sys/stat.h>
#include "og/organism.hh"
#include "og/organism_sample.hh"
#include "simulation_manager.hh"
#include "communication.hh"
#include "ipc_interface.hh"
namespace Morph
{

#define NUM_LEDS                       4        //number of blinkenlight
#define NUM_IRCOMMS                    4        //number of IR Communication channels
#define NUM_DOCKS                      4
#define NUM_LIGHTS                     8        //number of lightdetector
#define NUM_IRS                        8        //number of ir proximity sensor

#define RECRUITMENT_SIGNAL_INTERVAL     10
#define DEFAULT_RECRUITMENT_COUNT       150
#define DEFAULT_DOCKING_COUNT           20
#define DEFAULT_UNDOCKING_COUNT         30
#define DEFAULT_ORGANISM_COUNT          250 //simulates the duration the robots will be in Organism mode
#define DEFAULT_ASSEMBLY_COUNT          1200 //120 seconds -- 2 minutes
#define DEFAULT_RECRUITMENT_WAIT_COUNT  2400 //4 minutes
#define DEFAULT_FORAGING_BLIND_COUNT  100 //10 seconds period when robot doesn't repond to the wall or barrier objects encontered
#define MSG_ASSEMBLY_INFO_REQ           'a'
#define MSG_INRANGE                     'b'
#define MSG_DOCKINGDONE                 'c'
#define MSG_EXPELLING                   'd'
#define MSG_UNDOCKED                    'u'
#define MSG_ASSEMBLYSTARTED             's'
#define MSG_TYPE_GROWING 'G'
#define MSG_TYPE_DISASSEMBLY 'U'
#define MSG_TYPE_BROADCAST 'B'
#define MSG_TYPE_UNDOCKED 'X'

#define MSG_ORGANISM_FORMED  "Organism Formed"
#define MSG_DISASSEMBLY      "Disassembly"
#define MSG_RESHAPING        "Reshaping"


    enum fsm_state_t {EXPLORING = 0, RESTING, SEEDING, FORAGING, ASSEMBLY, LOCATEENERGY, LOCATEBEACON, ALIGNMENT, RECOVER, DOCKING, DISASSEMBLY, UNDOCKING, RESHAPING, INORGANISM, RECRUITMENT, MACROLOCOMOTION, STATE_COUNT};
    extern const char* state_names[STATE_COUNT];
    enum robot_mode_t {SWARM, ORGANISM};
    enum ir_pos_t {FR=0, RF, RB, BR, BL, LB, LF, FL}; //F -- FRONT, R -- RIGHT, B-- BACK, L--LEFT
    enum irmsg_type_t {IR_MSG_TYPE_SIMPLE=1, IR_MSG_TYPE_RECRUITING, IR_MSG_TYPE_OBJECTTYPE, IR_MSG_TYPE_ASSEMBLY_INFO, IR_MSG_TYPE_COMPLEX};

    //class Parameter;
    //class SimulationManager;
    class Robot
    {

        typedef void(*robot_callback_t)(Robot * robot);

        public:
        Robot();
        ~Robot();
        void Update();

        void Init(void *);

        void SetOgSequence(const OrganismSequence & og_seq);
        const OrganismSequence& GetOgSequence();

        virtual void SetStatus(const IPC::status_t& status)=0;
        virtual void SetCameraPose(const IPC::camera_pose_t& cam_pose)=0;
        bool SetOrganismInfo(const IPC::organism_info_t& og_info);

        virtual bool Connect(robot_side side)=0;
        void PrintProximitySensor();
        void PrintStatus();
        void Log();


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

        virtual void LoadParameters()=0;

        bool IPCSendStatus(double x, double y, double pa);
        bool IPCSendCommand(uint8_t cmd, uint8_t * data, uint8_t len);
        bool IPCSendDockingCommand(int side, bool flag);
        bool IPCSendCameraPose(double x, double y, double z, double yaw, double pitch);
        bool IPCSendOrganismInfo();

        std::string ClockString();

        void BroadcastMessage(Message); //broadcast message via wired communication bus
        void SendMessage(int i, Message*);
        void SendBranchTree(int i, const OrganismSequence& seq);
        bool isNeighboured(Robot *);

        virtual void BroadcastIRMessage(int channel, char type, const uint8_t data) = 0; //broadcast message via wireless 
        virtual void BroadcastIRMessage(int channel, char type, int size, const uint8_t *data) = 0;

        private:
        static void Exploring(Robot * robot) {robot->Exploring();}
        static void Resting(Robot * robot) {robot->Resting();}
        static void Seeding(Robot * robot) {robot->Seeding();}
        static void Foraging(Robot * robot){robot->Foraging();}
        static void Assembly(Robot * robot){robot->Assembly();}
        static void LocateEnergy(Robot * robot){robot->LocateEnergy();}
        static void LocateBeacon(Robot * robot){robot->LocateBeacon();}
        static void Alignment(Robot * robot){robot->Alignment();}
        static void Recover(Robot * robot){robot->Recover();} //used in stage
        static void Docking(Robot * robot){robot->Docking();}
        static void InOrganism(Robot * robot){robot->InOrganism();}
        static void Disassembly(Robot * robot){robot->Disassembly();}
        static void Undocking(Robot * robot){robot->Undocking();}
        static void Reshaping(Robot * robot){robot->Reshaping();}
        static void Recruitment(Robot * robot){robot->Recruitment();}
        static void MacroLocomotion(Robot * robot){robot->MacroLocomotion();}
        int RegisterBehaviour(robot_callback_t fnp, fsm_state_t state);
        robot_callback_t behaviours[STATE_COUNT];

        public:

        uint32_t timestamp;
        uint32_t id;
        robot_type type;
        char *name;
        bool active;

        fsm_state_t current_state;
        fsm_state_t last_state;
        robot_mode_t current_mode;

        //sensor for status variables
        int32_t proximity[NUM_IRS];
        int32_t beacon[NUM_LIGHTS];
        float comm_status[NUM_IRCOMMS];

        uint8_t bumped;
        bool docked[NUM_DOCKS];
        bool docking_done[NUM_DOCKS];   

        bool organism_found;
        bool powersource_found;
        bool organism_formed;

        int32_t recover_count;
        int32_t recruitment_signal_interval_count[NUM_DOCKS];
        int32_t recruitment_count[NUM_DOCKS];
        int32_t docking_count;
        int32_t inorganism_count;
        int32_t undocking_count; //step for undocking, i.e. open connectors and wait a few steps
        int32_t assembly_count;  //how long the robot detected last recruitment signals
        int32_t seeding_count;
        int32_t foraging_blind_count;

        uint8_t beacon_signals_detected;
        uint8_t recruitment_signals_detected;
        uint8_t expelling_signals_detected;
        uint8_t robot_in_range_replied;
        uint8_t msg_undocked_received; //undocking message
        uint8_t msg_reshaping_received;

        bool shape_completed;
        unsigned char newrobot_attached; //indicating new robot joined the organism, used to propagate the message to the whole organism
        bool docking_done_syn; // recived docking done synchronisation signals (new id, or subtree, or stop growing instruction) from the robot in organism, which is in recruitment state previously
        bool assembly_info_checked;

        int32_t leftwheel;
        int32_t rightwheel;

        int32_t num_robots_inorganism;


        //organism related
        OrganismSequence mytree;
        std::vector<OrganismSequence> mybranches;
        Organism * og;
        OrganismSequence::Symbol assembly_info; //information for which types of robot and which side is required by recruiting robots
        bool seed;
        Robot * neighbours[NUM_DOCKS];
        Message * neighbour_message[NUM_DOCKS]; //for communication between two connected robot
        std::vector<Message *> blackboard;//for communication with the organism, accessed from CommunicationBus instance
        int bus_id;
        CommunicationBus * com_bus;

        Parameter *para;

        FILE * fp_fsm;
        std::ofstream logFile;
        bool logtofile;

        SimulationManager * sm;

        bool newPose;
        double _x;
        double _y;
        double _pa;



    };

}
#endif

