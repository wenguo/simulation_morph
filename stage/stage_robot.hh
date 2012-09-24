/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef STAGE_ROBOT_HH
#define STAGE_ROBOT_HH
#include <stage.hh>
#include "robot.hh"
#include "history.hh"

using namespace Stg;
namespace Morph{

#define MIN_BLOB_PERCENT 0.95

#define WHEEL_DIA                      0.04
#define WHEEL_SEP                      0.052


enum blob_channel_t {BLOB_POWERSOCKET=0, BLOB_RESERVED1, BLOB_RESERVED2, BLOB_CHANNEL_COUNT};
enum object_type_t {OBJECT_UNKNOWN1=0, OBJECT_UNKNOWN2, OBJECT_ROBOT, OBJECT_POWERSOCKET, OBJECT_WALL, OBJECT_BARRIER, OBJECT_COUNT};
enum og_type{S1=0,S2,S3,S4};//organism type


#define DEFAULT_RECOVER_COUNT           10

class StageRobot:public Robot, public Visualizer
{
    public:
        StageRobot();
        ~StageRobot();

        void Init(void *);

    protected:
        virtual void UpdateSensors();
        virtual void UpdateCommunication();
        virtual void UpdateActuators();
        
        virtual void BroadcastIRMessage(int channel, char type, const uint8_t data); //broadcast message via wireless 
        virtual void BroadcastIRMessage(int channel, const uint8_t data); //broadcast message via wireless 
        virtual void BroadcastIRMessage(int channel, char type, int size, const uint8_t *data);

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
        virtual void Recruitment();
        virtual void MacroLocomotion();

        virtual void LoadParameters();
        virtual void SetStatus(const IPC::status_t& status);
        virtual void SetGlobalPose(const IPC::position_t&);
        virtual void SetCameraPose(const IPC::camera_pose_t& cam_pose){};
        virtual bool Connect(robot_side side){};
        
        void Follow();
        void Avoidance();

    private:
        ModelPosition* pos;
        ModelRanger* ir;
        ModelBlobfinder *blob;
        ModelBlinkenlight *leds[NUM_LEDS];
        ModelLightdetector *lights[NUM_LIGHTS];
        ModelIRCommunication *ircomms[NUM_IRCOMMS];
        ModelConnector *docking_units[NUM_DOCKS];
        PowerPack *powerpack;
        static int IRUpdate( Model* , StageRobot *);
        static int BlobfinderUpdate(Model* , StageRobot*);
        static int PositionUpdate( Model* , StageRobot* );
        static int BlinkenlightUpdate(Model *, StageRobot * );
        static int LightdetectorUpdate(Model *, StageRobot * );
        static int IRCommUpdate(Model* , StageRobot *);
        static int ConnectorUpdate(Model*, StageRobot*);

        void EnableLightDetector(bool flag);
        void EnableProximitySensor(bool flag);
        void EnableBlobfinder(bool flag);
        void EnableConnector(int id, bool flag);
        void PrintStatus();

        unsigned int GetChannelByColor(Color color);

        void SetSpeed(int lspeed, int rspeed);

        double SeedingProbability(double Es, double Vb);
        og_type organism_type(double Vb, object_type_t Ot[NUM_IRS], object_type_t Y[NUM_DOCKS], double V1, double V2);

        //for visualisation
        virtual void Visualize( Model* mod, Camera* cam );
        bool draw_organism;
        bool draw_state;
        bool draw_wheel;
        bool selected;

        int recover_count;

        object_type_t objects_encountered[NUM_IRS];
        object_type_t objects_encountered_by_neighbour[NUM_DOCKS];
        Hist powersource_detected_hist;
        Hist powersource_blob_hist;
        bool proximity_enabled;
        bool lightdetector_enabled;
        bool blobfinder_enabled;

        int16_t blob_offset[BLOB_CHANNEL_COUNT];  //for some strange reason, it has to be defined as int6_t type instead of int type (on Mac), the order is in epuck.inc, blobfinder
        int16_t deriv_blob_offset[BLOB_CHANNEL_COUNT];
        int16_t blob_size[BLOB_CHANNEL_COUNT];
        uint32_t blob_size_threshold;

        double S_o;
        double S_l;
        double S_r;
        double S_s;
        int seeding_shared_info_received[NUM_DOCKS];
        double Vb;  //ratio of vision blob
        double Es;  //energy
        object_type_t Ot;
        object_type_t Y;

        bool pos_initialised;

};

}
#endif
