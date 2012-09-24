/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#include "stage_robot.hh"
#include "sensor_data.hh"
#include "function.hh"

//TODO:
//1. process all messages in UpdateCommunication -- neighbour_message, blackboard
#define PREDEFINED_SHAPES

namespace Morph{

const int weightleft[8] = { 2, 4, 8, 10, 10, -8, -4, -2};
const int weightright[8] = { -2, -4, -8, -10, -10, -8, 4, 2};
const int avoid_weightleft[8] = { -8, -3, -1, 2, 2, 1, 3, 6};
const int avoid_weightright[8] = {6, 3, 1, 2, 2, -1, -3, -8};
const int recover_weightleft[8] = { -4, -1, 2, 2, 2, 2, -1, -2};
const int recover_weightright[8] = { -2, -1, 2, 2, 2, 2, -1, -3};
const int ircomm_weightleft[4] = {20, 60, -30, -60};
const int ircomm_weightright[4] = {20, -60, 30, 60};

StageRobot::StageRobot():
    Robot(),
    Visualizer( "shape", "vis_graph" ),
    recover_count(DEFAULT_RECOVER_COUNT),
    blobfinder_enabled(false),
    proximity_enabled(false),
    lightdetector_enabled(false),
    selected(false),
    draw_state(false),
    draw_organism(true),
    draw_wheel(true),
    pos_initialised(false),
    S_o(0),
    S_l(0),
    S_r(0),
    S_s(0)
{
    if(name)
        free(name);


    for (int i = 0; i < NUM_IRS; i++)
        objects_encountered[i] = OBJECT_UNKNOWN1;

    for (int i = 0; i < NUM_DOCKS; i++)
    {
        objects_encountered_by_neighbour[i] = OBJECT_UNKNOWN1;
        seeding_shared_info_received[i] = 0;
    }

}

StageRobot::~StageRobot()
{

}

void StageRobot::LoadParameters()
{
    if(!sm->LoadParameters())
        exit(-1);

    para = &sm->para;    
    logtofile = para->logtofile;
}

void StageRobot::Init(void * mod)
{

    LoadParameters();

    id = sm->robot_index++;
    std::ostringstream rname;
    rname<<"r"<<id;

    if(id > para->num_used_robots - 1) //didn't do anything
    {
        foraging_blind_count = DEFAULT_FORAGING_BLIND_COUNT;
        current_state = FORAGING;
        last_state = FORAGING;
        active=false;
        return;
    }

    //prepare log files etc
    Robot::Init((void*)rname.str().c_str());
    
    sm->AddRobot(this);

    pos = (ModelPosition*) mod;
    pos->AddCallback(Model::CB_UPDATE,(model_callback_t)PositionUpdate, this);
    pos->Subscribe();
    pos->AddVisualizer(this, true);

    //set names
    pos->SetToken(rname.str());
    name = strdup(pos->Token());

    //manually setting robot type so far
    type = ROBOT_KIT;//(robot_type)IRandom(ROBOT_KIT, ROBOT_SCOUT);
    pos->SetColor(type == ROBOT_KIT ? Color("yellow"): Color("green"));



    ir = (ModelRanger *) pos->GetUnusedModelOfType("ranger");
    if (ir)
    {
        ir->AddCallback( Model::CB_UPDATE,(model_callback_t)IRUpdate, this);
        ir->Subscribe();
        EnableProximitySensor(true);
    }

    blob =(ModelBlobfinder *) pos->GetUnusedModelOfType("blobfinder");
    if(blob)
    {
        blob->AddCallback( Model::CB_UPDATE, (model_callback_t)BlobfinderUpdate,this);
        blob->Subscribe();
        blob_size_threshold = MIN_BLOB_PERCENT * blob->scan_width;
        EnableBlobfinder(true);
        para->V_max = blob->scan_width;

        powersource_detected_hist.Resize(20);
        powersource_blob_hist.Resize(8);
    }

    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = (ModelBlinkenlight*) pos->GetUnusedModelOfType("blinkenlight");
        if (leds[i])
        {
            leds[i]->AddCallback( Model::CB_UPDATE,(model_callback_t)BlinkenlightUpdate, this);
            leds[i]->Subscribe(); // starts the ranger finder
        }
    }

    for (int i = 0; i < NUM_LIGHTS; i++)
    {
        lights[i] = (ModelLightdetector*) pos->GetUnusedModelOfType("lightdetector");
        if (lights[i])
        {
            lights[i]->AddCallback( Model::CB_UPDATE,(model_callback_t)LightdetectorUpdate, this);
            lights[i]->Subscribe(); // starts the ranger finder
        }
    }
    EnableLightDetector(false);

    for (int i = 0; i < NUM_IRCOMMS; i++)
    {
        ircomms[i] = (ModelIRCommunication*) pos->GetUnusedModelOfType("ircomm");
        if (ircomms[i])
        {
            ircomms[i]->AddCallback( Model::CB_UPDATE,(model_callback_t) IRCommUpdate, this);
            ircomms[i]->Subscribe(); // starts the ranger finder
        }
    }

    for (int i = 0; i < NUM_DOCKS; i++)
    {
        docking_units[i] = (ModelConnector*) pos->GetUnusedModelOfType("connector");
        if (docking_units[i])
        {
            docking_units[i]->AddCallback( Model::CB_UPDATE,(model_callback_t) ConnectorUpdate, this);
            docking_units[i]->Subscribe(); // starts connector model
            docking_units[i]->SetParentRobot((void*)this);
        }

        recruitment_count[i] = 0;
        seeding_shared_info_received[i]=0;
    }

    // set world gui
    if( !sm->worldgui)
    {
        sm->worldgui = pos->GetWorldGui();
    }

    powerpack = pos->FindPowerPack();
    if(powerpack)
    {
        para->E_max = powerpack->GetCapacity();
        double energy = IRandom(para->E_max * para->E_hungry, para->E_max);
        powerpack->SetStored(energy);
        //printf("%s -- set energy to be: %.0f\n", name, energy);
    }
    else
        para->E_max = 99999999;


}

void StageRobot::UpdateSensors()
{

}
void StageRobot::UpdateCommunication()
{

}
void StageRobot::UpdateActuators()
{

}
void StageRobot::EnableLightDetector(bool flag)
{
    for (int i = 0; i < NUM_LIGHTS; i++)
        lights[i] ->SetActive(flag);
    lightdetector_enabled = flag;
}

void StageRobot::EnableProximitySensor(bool flag)
{
    ir->SetActive(flag);
    proximity_enabled = flag;
}
void StageRobot::EnableBlobfinder(bool flag)
{
    blob->SetActive(flag);
    blobfinder_enabled = flag;
}

void StageRobot::EnableConnector(int id, bool flag)
{
    docking_units[id]->SetActive(flag);
    docking_units[id]->SetConnectorReturn(flag);
}

void StageRobot::Exploring()
{
    Avoidance();

    if(powerpack && powerpack->ProportionRemaining() < para->E_critical)
    {
        current_state = RESTING;
        last_state = EXPLORING;

    }
    else if(powerpack && powerpack->ProportionRemaining() < para->E_hungry)
    {
        current_state = FORAGING;
        last_state = EXPLORING;
    }
}

void StageRobot::Resting()
{   
    leftwheel = 0;
    rightwheel = 0;

    return;
}

void StageRobot::Seeding()
{
    double offset = proximity[FR] - proximity[FL];
    static double proximity_offset = 0;
    static double proximity_offset_deriv = 0;
    double temp = proximity_offset;
    proximity_offset = proximity[FR] - proximity[FL];
    proximity_offset_deriv = proximity_offset - temp;

    seeding_count++;

    if(seeding_shared_info_received[LEFT]>0)
        S_l = Score_neighbour(seeding_count);
    if(seeding_shared_info_received[RIGHT]>0)
        S_r = Score_neighbour(seeding_count);

    //facing towards the object detected
    if(fabs(proximity_offset) > para->Err)
    {
        leftwheel = proximity_offset * para->Kp + proximity_offset_deriv * para->Kd;
        rightwheel = - proximity_offset * para->Kp - proximity_offset_deriv * para->Kd;
    }
    else
    {
        leftwheel = 0;
        rightwheel = 0;
    }

    //broadcast messages 
    if(timestamp % 30 == IRandom(10, 20))
    {
        uint8_t objtype[2];
        objtype[0] = objects_encountered[FL];
        objtype[1] = objects_encountered[FR];
        BroadcastIRMessage(LEFT, IR_MSG_TYPE_OBJECTTYPE, 2, objtype );
        BroadcastIRMessage(RIGHT, IR_MSG_TYPE_OBJECTTYPE, 2, objtype);
    }

    double f1 = para->K1 * S_o * seeding_count + para->K2 * (S_r + S_l) - para->K3 * S_s;

    //printf("%d -- %s (f1:%.1f\tSo:%.1f\tSr:%.1f\tSl:%.1f\tSs:%.1f)\n", timestamp, name, f1, S_o, S_r, S_l, S_s);
#define TEMP_MOVIE
#ifndef  TEMP_MOVIE
    if( f1 > para->S1)
#endif
    {
        mytree.Clear();
#ifdef PREDEFINED_SHAPES
        //select predefined organism
        og = sm->ogList[sm->para.og_index];

        draw_organism = true;
        og->GraphToSequence(mytree);
#else
        //random generate organism
        do
        {
            if(og)
                delete og;
            mytree.Clear();

            OrganismSequence::RandomInit(mytree, 5);
            og = new Organism;
            Organism::SequenceToGraph(mytree, *og);
        }
        while(!og->Valid());
#endif
        std::cout<<*og<<std::endl;

        num_robots_inorganism = 1;

        printf("%d -- %s success, select organism (%f, %#x-%#x, %#x-%#x)\n", timestamp, name, 
                Vb, objects_encountered[FL], objects_encountered[FR], objects_encountered_by_neighbour[FL],
                objects_encountered_by_neighbour[FR]);

        for(int i=0;i<SIDE_COUNT;i++)
        {
            recruitment_count[i] = 0;
            recruitment_signal_interval_count[i] = DEFAULT_RECRUITMENT_COUNT;
        }

        current_state = RECRUITMENT;
        last_state = SEEDING;

        seed = true;


        //prepare branches sequence
        rt_status ret=OrganismSequence::fillBranches(mytree, mybranches);
        if(ret.status >= RT_ERROR)
        {
            std::cout<<ClockString()<<" : "<<name<<" : ERROR in filling branches !!!!!!!!!!!!!!!!!!!!"<<std::endl;
            pos->GetWorld()->Stop();
        }

        std::vector<OrganismSequence>::iterator it;
        for(it = mybranches.begin() ; it != mybranches.end(); it++)
        {
            //check the first symbol that indicates the parent and child side of the connection
            uint8_t branch_side = it->getSymbol(0).side1;
            docking_units[branch_side]->SetActive(true);
            docking_units[branch_side]->SetConnectorReturn(true);
            docking_units[branch_side]->CommandClose();
            std::cout<<name<<" branch "<<*it<<std::endl;
            leds[branch_side]->EnableLight(true);
        }

        //set some flags
        organism_found = true;
        draw_organism = true;

        //turn off some sensors to speed up simulation, or save energy in real life
        EnableLightDetector(false);
        EnableProximitySensor(false);
        EnableBlobfinder(false);

        //be the first node added into the communication bus
        //create a new communication bus
        bus_id = time(NULL);
        com_bus = new CommunicationBus(bus_id, mytree.Edges()+1);
        sm->busList.push_back(com_bus);
        com_bus->addCommunicationNode(this);
        com_bus->printNodeList();
    }
#ifndef TEMP_MOVIE 
    else if( f1 < para->S2 || seeding_count > para->T_s)
    {
        printf("%d -- %s failed\n", timestamp, name);
        current_state = FORAGING;
        last_state = SEEDING;
        foraging_blind_count = DEFAULT_FORAGING_BLIND_COUNT;
        EnableBlobfinder(false);
    }
#endif
    return;

}

//actions to recover from stall in simulation due to the cubic shape
void StageRobot::Recover()
{
    if (recover_count >= 0)
    {
        recover_count--;

        /*leftwheel = 0;
          rightwheel = 0;

          for (int i = 0; i < NUM_IRS; i++)
          {
          leftwheel += recover_weightleft[i] * (proximity[i] >> 1);
          rightwheel += recover_weightright[i] * (proximity[i] >> 1);
          }*/

        leftwheel = 0;
        rightwheel = 0;

        for (int i = 0; i < NUM_IRS; i++)
        {
            leftwheel += avoid_weightleft[i] * (proximity[i]);
            rightwheel += avoid_weightright[i] * (proximity[i]);
        }

    }
    else
    {
        current_state = FORAGING;

        foraging_blind_count = DEFAULT_FORAGING_BLIND_COUNT;
        EnableBlobfinder(false);

        pos->SetObstacleReturn(1);
    }
}



void StageRobot::Foraging()
{
    foraging_blind_count--;

    if(foraging_blind_count==-1)
        EnableBlobfinder(true);


    //to state Resting
    /*
    if(powerpack && powerpack->ProportionRemaining() < para->E_critical)
    {
        current_state = RESTING;
        last_state = FORAGING;
        return;
    }*/

    if(powersource_found)
    {
        current_state = LOCATEENERGY;
        last_state = FORAGING;
    }
    else if((objects_encountered[FL] == OBJECT_BARRIER  || objects_encountered[FR] == OBJECT_BARRIER))
    {
        if(bumped & 0x81)
        {
            Vb = powersource_blob_hist.Avg() / (para->V_max * 1.0);// blob_size[BLOB_POWERSOCKET]/(para->V_max * 1.0);
            Es = powerpack->ProportionRemaining();

            for(int i=0; i< SIDE_COUNT;i++)
            {
                objects_encountered_by_neighbour[i] = OBJECT_UNKNOWN1;
                seeding_shared_info_received[i]=0;
            }
            S_o = SeedingProbability(Es,Vb);
            S_l = 0;
            S_r = 0;
            S_s = 0;
            seeding_count = 0;
            current_state = SEEDING;
            last_state = FORAGING;
        }
        else
            ;//keep going
    }
    else if (organism_found)
    {
        current_state = ASSEMBLY;
        last_state = FORAGING;
    }
    else
    {
        Avoidance();
    }
  
    return;
}

void StageRobot::Assembly()
{
    //reset organism_found status
    if(assembly_count--<=0)
    {
        organism_found = false;
        EnableBlobfinder(true);

        current_state = FORAGING;
        last_state = ASSEMBLY;
    }
    //recruitment message recived, then locatebeacon
    else if (comm_status[0] > 0.8 || comm_status[1] > 0.8 || comm_status[2] > 0.8 || comm_status[3] > 0.8)
    {
        current_state = LOCATEBEACON;
        last_state = FORAGING;
        EnableLightDetector(true);
        EnableBlobfinder(false);
    }
    else
        Avoidance();

    return;
}

void StageRobot::LocateEnergy()
{
    leftwheel = 200;
    rightwheel = 200;

    //using a very simple PD controller moving towards power source
    leftwheel  -= blob_offset[0]*4 + deriv_blob_offset[0] * 3;
    rightwheel += blob_offset[0]*4 + deriv_blob_offset[0] * 3;

    //lost power source?
    if(!powersource_found)
    {
        current_state = FORAGING;
        last_state = LOCATEENERGY;
    }
    //sensor 0 and 7 triggered?
    else if(bumped & 0x81)
    {
        if(objects_encountered[FL] == OBJECT_BARRIER  || objects_encountered[FR] == OBJECT_BARRIER
                ||objects_encountered[FL] == OBJECT_WALL  || objects_encountered[FR] == OBJECT_WALL)
        {
            Vb = powersource_blob_hist.Avg() / (para->V_max * 1.0);// blob_size[BLOB_POWERSOCKET]/(para->V_max * 1.0);
            Es = powerpack->ProportionRemaining();

            for(int i=0; i<SIDE_COUNT;i++)
            {
                objects_encountered_by_neighbour[i] = OBJECT_UNKNOWN1;
                seeding_shared_info_received[i]=0;
            }

            S_o = SeedingProbability(Vb, Es);
            S_l = 0;
            S_r = 0;
            S_s = 0;
            seeding_count = 0;
            current_state = SEEDING;
            last_state = LOCATEENERGY;
        }
        else
        {
            Avoidance();//keep going
        }
    }
}

void StageRobot::LocateBeacon()
{
    leftwheel = 100;
    rightwheel = 100;

    //calculate new speed
    for (int i = 0; i < NUM_IRCOMMS; i++)
    {
        leftwheel += ircomm_weightleft[i] * comm_status[i];
        rightwheel += ircomm_weightright[i] * comm_status[i];
    }

    if (bumped)
    {
        current_state = FORAGING;
        last_state = LOCATEBEACON;
        EnableLightDetector(false);
        return;
    }

    if (beacon_signals_detected & 0x81)
    {
        //printf("%d: %s detect beacon signals, send message\n", pos->GetWorld()->GetUpdateCount(),pos->Token());
        current_state = ALIGNMENT;
        last_state = LOCATEBEACON;

        BroadcastIRMessage(FRONT, MSG_INRANGE);
        docking_units[FRONT]->SetActive(true);  //enalbe connector beam and contact update
        docking_units[FRONT]->CommandOpen();
        docking_units[FRONT]->SetConnectorReturn(true);

        return;
    }

    if (comm_status[0] < 0.08 && comm_status[1] < 0.08 && comm_status[2] < 0.08 && comm_status[3] < 0.08)
    {
        current_state = FORAGING;
        last_state = LOCATEBEACON;
        EnableLightDetector(false);
        return;
    }
}

void StageRobot::Alignment()
{
    leftwheel = 150;
    rightwheel = 150;

    //sometimes, it may lost to recive any beacon signals due to error of the simulation model.
    //TODO: fix the model
    if (beacon_signals_detected == 0)
    {
        current_state = FORAGING;
        last_state = ALIGNMENT;
        //EnableConnector(FRONT, false);
        docking_units[FRONT]->SetConnectorReturn(false);
        EnableLightDetector(false);
        docking_units[FRONT]->SetActive(true);
        docking_units[FRONT]->CommandOpen();

        return;
    }


    //calculate new speed
    for (int i = 0; i < NUM_LIGHTS; i++)
    {
        leftwheel += weightleft[i] * beacon[i];
        rightwheel += weightright[i] * beacon[i];
    }

    //expelling signals detected, reverse to minimise interference
    if ((expelling_signals_detected &0xd))
    {
        expelling_signals_detected = 0;
        if(bumped)
        {
            current_state = FORAGING;
            last_state = ALIGNMENT;
            docking_units[FRONT]->SetActive(true);
            docking_units[FRONT]->CommandOpen();
            docking_units[FRONT]->SetConnectorReturn(false);
            EnableLightDetector(false);

            return;
        }
    }

    ModelConnector::config_t cfg = docking_units[FRONT]->GetConfig();
    //two connectors are close enough?
    //request assembly_info for double checking, this is essential espeically in scenario of multiple organisms
    if (cfg.beam[0] != NULL && cfg.beam[1] != NULL)
    {
        if(!assembly_info_checked)
            BroadcastIRMessage(FRONT, MSG_ASSEMBLY_INFO_REQ);
        else if(assembly_info.type2 == type) // am I the right robot?
        {
            //reset
            assembly_info_checked = false;

            current_state = DOCKING;
            last_state = ALIGNMENT;

            docking_count = DEFAULT_DOCKING_COUNT;


            //disable front connector
            docking_units[FRONT]->SetActive(false);  //enalbe connector beam and contact update
            docking_units[FRONT]->CommandOpen();
            docking_units[FRONT]->SetConnectorReturn(false);
            //enable corresponding connector
            docking_units[assembly_info.side2]->SetActive(true);  //enalbe connector beam and contact update
            docking_units[assembly_info.side2]->SetConnectorReturn(true);
            docking_units[assembly_info.side2]->CommandClose();

            Pose pose = pos->GetGlobalPose();
            pose.a += dtor(assembly_info.side2 * 90);
            //magic turn if docking side is not FRONT
            pos->SetGlobalPose(pose);
        }
        else
        {
            printf("%d : %s wrong type, go back to foraging\n", timestamp, name);
            current_state = FORAGING;
            last_state = ALIGNMENT;
            docking_units[FRONT]->SetActive(true);
            docking_units[FRONT]->CommandOpen();
            docking_units[FRONT]->SetConnectorReturn(false);
            EnableLightDetector(false);
        }

        leftwheel = 0;
        rightwheel = 0;
    }
    else
        assembly_info_checked = false; // in case it receives info not requested by itself

}

//step 1, check if assembly side is the robot side, if no, undocking
//step 2, turn robot to the right position
//step 3, close the docking, set the right commbus etc
void StageRobot::Docking()
{
    leftwheel = 0;
    rightwheel = 0;

    //perform some docking closure, here simply wait for a while
    if (docking_count-- == 0)
    {
        //TODO: using communication bus ?
        if (docking_units[assembly_info.side2]->GetAttachedRobot()!=NULL)
        {

            std::cout<<timestamp<<" : "<<name<<" attached to "<<((Robot*)docking_units[assembly_info.side2]->GetAttachedRobot())->name<<std::endl;
            neighbours[assembly_info.side2] = (Robot*)docking_units[assembly_info.side2]->GetAttachedRobot();
            BroadcastIRMessage(assembly_info.side2, MSG_DOCKINGDONE);

            //Add me into the Communication bus so others will know I am in the organism
            //CommunicationBus::getInstance().addCommunicationNode(this);
            bus_id = neighbours[assembly_info.side2]->bus_id;
            com_bus = neighbours[assembly_info.side2]->com_bus;

            if(com_bus)
            {
                com_bus->addCommunicationNode(this);
                com_bus->printNodeList();
            }

            current_state = INORGANISM;
            last_state = DOCKING;
            docked[assembly_info.side2] = true;
            current_mode = ORGANISM;


            EnableLightDetector(false);
            EnableProximitySensor(false);
        }
        else
        {
            std::cout<<timestamp<<" : "<<name<<" can not find the attached robots, simulation errors, need to be improved\n back to state Foraging"<<std::endl;
            current_state = FORAGING;
            last_state = DOCKING;
        }
    }

}

void StageRobot::InOrganism()
{
    //be static
    leftwheel = 0;
    rightwheel = 0;

    num_robots_inorganism = com_bus->Size();

    if (seed)
    {
        //send out warning message
        for (int i = 0; i < SIDE_COUNT; i++)
        {
            if(!docked[i]  && timestamp % 10 ==i)
                BroadcastIRMessage(i, MSG_ASSEMBLYSTARTED);
        }

        if(!organism_formed)
        {
            //find the robots from the CommunicationBus Node List, starting from the latest insert one
            std::vector<Robot*> *robotList  = &(com_bus->CommunicationNodeList);
            if(robotList->size() == mytree.Edges()+1)
            {
                organism_formed = true;
                inorganism_count = DEFAULT_ORGANISM_COUNT;
            }
        }
        else
        {
            inorganism_count--;

            if(inorganism_count==DEFAULT_ORGANISM_COUNT - 10)
            {
               //((WorldGui*)sm->worldgui)->Screenshot();
               BroadcastMessage(Message(name, "ALL", MSG_TYPE_BROADCAST, MSG_ORGANISM_FORMED,timestamp));

                current_state = MACROLOCOMOTION;
                last_state = INORGANISM;

               // BroadcastMessage(Message(name, "ALL", MSG_TYPE_BROADCAST, MSG_RESHAPING,timestamp));
                //BroadcastMessage(Message(name, "ALL", MSG_TYPE_BROADCAST, MSG_DISASSEMBLY,timestamp));
                //quit
                //pos->GetWorld()->Quit();
                SimulationManager::getInstance()->dumpStateRecord();
            }
        }
    }

    //if received new subtrees from the robot in organism, then transit to state INORGANISM,
    //this should happen after it send a docking message to the recruitment robot
    for(int i=0;i<SIDE_COUNT;i++)
    {
        if (neighbour_message[i] && neighbour_message[i]->timestamp < timestamp)
        {
            if (neighbour_message[i]->type == MSG_TYPE_GROWING)
            {
                docking_done_syn = true;
                //first checking received new tree, is it match with assembly_info?
                if(neighbour_message[i]->data[0] & 0xF0 != assembly_info.data & 0xF0)
                {
                    printf("%d -- %s ERROR !!!!, docking to the wrong robot\n",timestamp, name);
                    std::cout<<"expecting: "<<assembly_info<<", but received: "
                        <<OrganismSequence::Symbol(neighbour_message[i]->data[0])<<std::endl;
                    pos->GetWorld()->Stop();
                }
                else
                {
                    //fill mytree, removing the parental info
                    //std::cout<<*neighbour_message[assembly_info.side2]<<std::endl;
                    rt_status ret = mytree.reBuild(neighbour_message[i]->data + 1, neighbour_message[i]->size - 2);

                    //fill branches sequence
                    if(ret.status < RT_ERROR)
                        ret=OrganismSequence::fillBranches(mytree, mybranches);

                    if(ret.status >= RT_ERROR)
                    {
                        std::cout<<ClockString()<<" : "<<name<<" : ERROR in filling branches !!!!!!!!!!!!!!!!!!!!"<<std::endl;
                        pos->GetWorld()->Stop();
                    }

                    std::cout<<ClockString()<<": "<<name<<" receive new subtree: "<<mytree<<std::endl;

                    //check if  need to recruit new robots
                    std::vector<OrganismSequence>::iterator it = mybranches.begin();
                    bool recruiting_required = false;
                    while(it!=mybranches.end())
                    {
                        //check the first symbol that indicates the parent and child side of the connection
                        uint8_t branch_side = it->getSymbol(0).side1;
                        docking_units[branch_side]->SetActive(true);
                        docking_units[branch_side]->SetConnectorReturn(true);
                        docking_units[branch_side]->CommandClose();
                        std::cout<<name<<" branch "<<*it<<std::endl;
                        leds[branch_side]->EnableLight(true);

                        recruitment_count[branch_side] = 0;
                        recruitment_signal_interval_count[branch_side] = DEFAULT_RECRUITMENT_COUNT;

                        ++it;

                        recruiting_required = true;
                    }

                    if(recruiting_required)
                    {
                        current_state = RECRUITMENT;
                        last_state = INORGANISM;

                    }

                }
            }
            else if(neighbour_message[i]->type == MSG_TYPE_UNDOCKED)
            {
                ;//to be implemented
            }
        
            //clear the messages from memory
            delete neighbour_message[i];
            neighbour_message[i] = NULL;
        }

    }

    //sending docking done message in case the recruiting robot doesn't repond
    if(last_state == DOCKING && !docking_done_syn)
    {
        if(timestamp % 10 == IRandom(0,9))
        {
            BroadcastIRMessage(assembly_info.side2, MSG_DOCKINGDONE);
            printf("%d -- %s: sending docking done message to recruiting robot as it doesn't repond\n", timestamp, name);
        }
    }

    //send branch info in case the newly joined robot doesn't receive it
    for(int i=0; i<SIDE_COUNT; i++)
    {
        if(docking_done[i] && docked[i])
        {
            docking_done[i]=false;
            OrganismSequence seq;
            rt_status ret = mytree.getBranch(seq, (robot_side)i);
            if(ret.status == RT_OK)
                SendBranchTree(i, seq);
            else
                printf("Error in getting branch\n");

           // pos->GetWorld()->Stop();
        }
    }

    //check if there is something broadcast message, should have only one message, otherwise, errors
    while(!blackboard.empty())
    {
        Message * message = blackboard.back(); //it is a bit dangerous, since other process can push_back its blackboard
        if(message && message->timestamp < timestamp)
        {
            switch(message->type)
            {
                case MSG_TYPE_BROADCAST:
                    std::cout<<timestamp<<" : "<<name<<" received "<<(char*)message->data<<std::endl;
                    if (strcmp((char*)message->data,MSG_ORGANISM_FORMED) == 0)
                    {
                        printf("  -- Organism formed\n");
                        organism_formed = true;

                        current_state = MACROLOCOMOTION;
                        last_state = INORGANISM;
                    }
                    else if (strcmp((char*)message->data,MSG_DISASSEMBLY) == 0)
                    {
                        current_state = DISASSEMBLY;
                        last_state = INORGANISM;
                        current_mode = SWARM;
                        organism_found = false;
                        organism_formed = false;
                        num_robots_inorganism = 1;

                        //disable all connector_return
                        for(int i=0; i< SIDE_COUNT; i++)
                        {
                            docking_units[i]->SetConnectorReturn(false);
                        }
                    }
                    /*
                    else if(strcmp((char*)message->data, MSG_RESHAPING) == 0)
                    {
                        current_state = RESHAPING;
                        last_state = INORGANISM;
                        organism_formed = false;
                    }*/
                    break;
                default:
                    break;
            }
            delete message;
            blackboard.pop_back();
        }
        else
            break;
    }


}

void StageRobot::Disassembly()
{
}

void StageRobot::Recruitment()
{
    //disable motor
    leftwheel = 0;
    rightwheel = 0;
                        

    //broadcast assembly started message if it is a seed robots
    if(seed)
    {
        for(int i=0;i<SIDE_COUNT;i++)
            if(timestamp % RECRUITMENT_SIGNAL_INTERVAL== 4 + 2 * i)
                BroadcastIRMessage(i, MSG_ASSEMBLYSTARTED);
    }

    //no messages are expecting, simply removeall
    for(int i=0;i<SIDE_COUNT;i++)
    {
        if (neighbour_message[i] && neighbour_message[i]->timestamp < timestamp)
        {
            delete neighbour_message[i];
            neighbour_message[i]=NULL;
        }
    }
       
    std::vector<OrganismSequence>::iterator it1 = mybranches.begin();
    while(it1 !=mybranches.end())
    {
        int index = it1->getSymbol(0).side1;

        //new robot docked?
        if(docking_done[index])
        {
            recruitment_count[index] = 0;
            docking_done[index] = false;
            docked[index] = true;
            leds[index]->EnableLight(false);
            docking_units[index]->SetActive(true);
            docking_units[index]->SetConnectorReturn(true);
            robot_in_range_replied &= ~(1<<index);

            if (docking_units[index]->GetAttachedRobot()!=NULL)
            {
                neighbours[index] = (Robot*)docking_units[index]->GetAttachedRobot();
            }
            else
            {
                //find the robots from the CommunicationBus Node List, starting from the latest inserted one
                //TODO: not an efficient way, using a List instead of vectors for storing?
                std::vector<Robot*>::iterator it = com_bus->CommunicationNodeList.begin();
                while(it != com_bus->CommunicationNodeList.end())
                {
                    if ((*it)->neighbours[FRONT] && ((*it)->neighbours[FRONT]->name == name) && !isNeighboured(*it))
                    {
                        neighbours[index] =  *it;
                        break;
                    }
                    it++;
                }
            }

            std::cout<<timestamp<<" : "<<name<<" send branch"<<std::endl;
            SendBranchTree(index,*it1);

            //send to robot3d
            IPCSendDockingCommand(index, true);

            //remove finished branch
            std::cout<<timestamp<<" : "<<name<<" erase branch"<<*it1;
            it1 = mybranches.erase(it1);
            std::cout<<"... done"<<std::endl;

        }
        else
        {
            //reset flags
            if (recruitment_signal_interval_count[index]-- <= 0)
                robot_in_range_replied &= ~(1<<index);

            recruitment_count[index]++;
            //no robot in range replied?
            if ((timestamp % RECRUITMENT_SIGNAL_INTERVAL == index) && ((~robot_in_range_replied) & (1<<index)))
            {
                BroadcastIRMessage(index, IR_MSG_TYPE_RECRUITING, it1->getSymbol(0).data);
                recruitment_signal_interval_count[index] = DEFAULT_RECRUITMENT_COUNT;
            }

            ++it1;
        }
    }

    //recruitment done?
    if(mybranches.empty())
    {
        current_state = INORGANISM;
        last_state = RECRUITMENT;
        memset(docking_done, 0, NUM_DOCKS);
        robot_in_range_replied = 0;

        //send organism info to robot3d
        IPCSendOrganismInfo();
    }
}

void StageRobot::MacroLocomotion()
{
    leftwheel = 0;
    rightwheel = 0;

    if(timestamp % 20 < 10)
        pos->SetColor(Color("red"));
    else
        pos->SetColor(Color("LightGray"));


   // pos->GetWorld()->Quit();


}
void StageRobot::SetSpeed(int lspeed, int rspeed)
{
    if (lspeed < -1000)
        lspeed = -1000;
    if (lspeed > 1000)
        lspeed = 1000;
    if (rspeed < -1000)
        rspeed = -1000;
    if (rspeed > 1000)
        rspeed = 1000;

    double x, a;

    x = (lspeed + rspeed) * M_PI * WHEEL_DIA / (2 * 1000.0);
    a = (rspeed - lspeed) * M_PI * WHEEL_DIA * 2 / (WHEEL_SEP * 1000.0);

    pos->SetSpeed(x, 0, a);
}

void StageRobot::Avoidance()
{

    leftwheel = 200;
    rightwheel = 200;

    for (int i = 0; i < NUM_IRS; i++)
    {
        leftwheel += avoid_weightleft[i] * (proximity[i]);
        rightwheel += avoid_weightright[i] * (proximity[i]);
    }

}

void StageRobot::Follow()
{
    leftwheel = 200;
    rightwheel = 200;

    //calculate new speed
    for (int i = 0; i < NUM_LIGHTS; i++)
    {
        leftwheel += weightleft[i] * beacon[i];
        rightwheel += weightright[i] * beacon[i];
    }

    if (proximity[3] > 0 || proximity[4] > 0)
    {
        leds[2]->EnableLight(true);
    }
    else
        leds[2]->EnableLight(false);

    if (ircomms[0]->msg.size() > 0)
    {
        std::vector<stg_msg_t>::iterator it;
        for (it = ircomms[0]->msg.begin(); it != ircomms[0]->msg.end(); it++)
        {

            stg_msg_t m = *it;
        }
    }

}


int StageRobot::IRUpdate( Model* mod, StageRobot *robot)
{
    if (robot->proximity_enabled)
    {
        //put measure sensor data to robot
        const std::vector<ModelRanger::Sensor>& ir = ((ModelRanger*)mod)->GetSensors();
        int index =0;

        //reset
        for (int i = 0; i < NUM_IRS; i++)
        {
            robot->proximity[i]=0;
            robot->objects_encountered[i]=OBJECT_UNKNOWN1;
        }

        if(ir.size()>0)
        {
            robot->bumped = 0;
            for (int i = 0; i < NUM_IRS; i++)
            {
                object_type_t object = OBJECT_UNKNOWN1;
                float ir_min = 9999;
                for(int j=0;j<ir[i].ranges.size();j++)
                {
                    if(ir[i].ranges[j]<ir_min)
                    {
                        ir_min = ir[i].ranges[j];
                        object =(object_type_t) ir[i].intensities[j];
                    }
                }
                index = ir_min * 1000;
                if(index >=MAX_PROXIMITY_RANGE)
                    robot->proximity[i] =  simple_normal_deviate(proximity_data[116][0],proximity_data[116][1]);
                else
                    robot->proximity[i] = simple_normal_deviate(proximity_data[index][0],proximity_data[index][1]);

                if(robot->foraging_blind_count <0 )
                    robot->objects_encountered[i] = object;

                if (robot->proximity[i] > 10)
                    robot->bumped |= 1 << i;
            }
        }
    }
    return 0;
}
int StageRobot::BlobfinderUpdate(Model* mod, StageRobot* robot)
{
    robot->powersource_found = false;
    if(robot->blobfinder_enabled)
    {
        unsigned int count=0;
        unsigned int index=0;
        uint16_t temp=0;
        ModelBlobfinder::Blob * b = robot->blob->GetBlobs(&count);
        //reset
        for(int i=0;i<BLOB_CHANNEL_COUNT;i++)
        {
            robot->blob_offset[i]=0;
            robot->deriv_blob_offset[i]=0;
            robot->blob_size[i]=0;
        }

        for(unsigned int i=0; i<count; i++)
        {
            index = robot->GetChannelByColor(b[i].color);
            temp = robot->blob_offset[index];
            robot->blob_offset[index] = (robot->blob->scan_width - b[i].left-b[i].right)>>1;
            robot->deriv_blob_offset[index] = robot->blob_offset[index] - temp;

            robot->blob_size[index]= b[i].right-b[i].left;

// printf("%s found food: %d (%d %d %d %d) %.2f --%d\n", robot->name, i, b[i].left,b[i].right, b[i].top, b[i].bottom, b[i].range, robot->blob->scan_width);
            //std::cout<<robot->GetChannelByColor(b[i].color)<<" "<<robot->blob_offset[robot->GetChannelByColor(b[i].color)]<<std::endl;
            //printf("scan_width: %d\toffset: %d\n",robot->blob->scan_width,offset);
        }

        double blob_ratio = robot->blob_size[BLOB_POWERSOCKET] * 1.0 / robot->blob->scan_width;
        if(blob_ratio >= robot->para->V1) 
        {
            robot->powersource_detected_hist.Push(1);
        }
        else
        {
            robot->powersource_detected_hist.Push(0);
        }

        robot->powersource_blob_hist.Push(robot->blob_size[BLOB_POWERSOCKET]);

        if(robot->foraging_blind_count< 0 && robot->powersource_detected_hist.Sum()>12)
            robot->powersource_found = true;
    }
    else
    {
        //reset
        for(int i=0;i<BLOB_CHANNEL_COUNT;i++)
        {
            robot->blob_offset[i]=0;
            robot->deriv_blob_offset[i]=0;
            robot->blob_size[i]=0;
        }
    }
    return 0;
}

int StageRobot::PositionUpdate( Model* mod, StageRobot* robot)
{
    if(robot->pos->GetWorldGui()->ModelSelected(mod))
        robot->selected = true;
    else
        robot->selected = false;



    if(!robot->pos_initialised)
    {
        robot->pos_initialised = true;
        int a = IRandom(0,6);


        vect2* pos_data = robot->sm->para.pos_data;

        robot->pos->SetGlobalPose(Pose(pos_data[robot->id].x,pos_data[robot->id].y,0,dtor(a*45)));
        std::cout<<robot->id<<" -- "<<robot->name<<" adjusts pose to ["<<pos_data[robot->id].x<<", "
            <<pos_data[robot->id].y<<", "<<a*45<<"\%]"<<std::endl; 
        robot->blob_size[0]=0;
        robot->powersource_found = true;


    }

    if(robot->pos->Stalled())
    {
        switch (robot->current_state)
        {
            case EXPLORING:
            case FORAGING:
            case ASSEMBLY:
            case LOCATEENERGY:
            case LOCATEBEACON:
            case ALIGNMENT:
                robot->last_state = robot->current_state;
                robot->current_state = RECOVER;
                robot->recover_count = DEFAULT_RECOVER_COUNT;
                robot->pos->SetObstacleReturn(0);
                robot->pos->SetStall(false);
                break;
            default:
                break;
        }
    }

    robot->Update();
    robot->SetSpeed(robot->leftwheel, robot->rightwheel);
   // robot->sm->logState(robot->timestamp, robot->current_state);
    robot->timestamp = robot->pos->GetWorld()->GetUpdateCount();


    //received position from Robot3D
    if(robot->newPose)
    {
        robot->newPose = false;
        robot->pos->SetGlobalPose(Pose(robot->_x, robot->_y, 0, robot->_pa));
    }
    //send position and state info to Robot3D
    Pose pose=robot->pos->GetGlobalPose();
    robot->IPCSendStatus(pose.x, pose.y, pose.a );

    return 0;
}
int StageRobot::BlinkenlightUpdate(Model* mod, StageRobot *robot)
{
    return 0;
}
int StageRobot::LightdetectorUpdate(Model *mod, StageRobot *robot)
{
    if (robot->lightdetector_enabled)
    {
        ModelLightdetector *mod_ld = (ModelLightdetector *)mod;
        uint32_t ll_count = mod_ld->lights_count;

        //put measured sensor data to the robot
        int index;
        if(mod_ld->lights.size()>0)
            FOR_EACH(it, mod_ld->lights)
            {
                index = (int)(it->range * 1000);
                if (index >= MAX_LIGHTDETECTOR_RANGE || index < 0)
                    it->intensity = 0;
                else
                    it->intensity = (int) ( lightdetector_data[index][1] * cos(it->incidence_angle)
                            * light_relative_radiant_intensity[(int)rtod(fabs(it->emiter_angle))][1]);
            }

        //TODO: how about the case two leds are detected?
        //reset the bits in beacon_signals_detected
        robot->beacon_signals_detected &= ~(1 << mod->GetIndex());
        if (ll_count > 0)
        {
            std::vector<stg_lightdetector_t>::iterator it;
            for ( it = mod_ld->lights.begin(); it != mod_ld->lights.end(); it++ )
            {
                robot->beacon[mod->GetIndex()] = it->intensity;
            }
            robot->beacon_signals_detected |= 1 << mod->GetIndex();
        }
        else
            robot->beacon[mod->GetIndex()] = 0;
    }

    return 0;
}
int StageRobot::IRCommUpdate(Model* mod, StageRobot *robot)
{
    unsigned char j = mod->GetIndex();
    robot->comm_status[j] *= 0.8;

    std::vector<stg_msg_t>::iterator it;
    for (it = robot->ircomms[j]->msg.begin(); it != robot->ircomms[j]->msg.end(); it++)
    {
        if (it->ts < robot->pos->GetWorld()->GetUpdateCount())
        {
            switch (it->msg[1])
            {
                case IR_MSG_TYPE_SIMPLE:
                    switch (it->msg[2])
                    {
                        case MSG_DOCKINGDONE:
                            {
                                if(robot->current_state == RECRUITMENT || robot->current_state == INORGANISM)
                                    robot->docking_done[j] = true;
                            }
                            break;
                        case MSG_INRANGE:
                            {
                                if(robot->current_state == RECRUITMENT )
                                    robot->robot_in_range_replied |= 1 << j;
                            }
                            break;
                        case MSG_EXPELLING:
                            {
                                if(robot->current_state == ALIGNMENT)
                                    robot->expelling_signals_detected |= 1 << j;
                            }
                            break;
                        case MSG_UNDOCKED:
                            robot->msg_undocked_received |= 1<<j;
                            break;
                        case MSG_ASSEMBLYSTARTED:
                            robot->S_s = 1;
                            break;
                        case MSG_ASSEMBLY_INFO_REQ:
                            {
                                OrganismSequence seq;
                                rt_status ret = robot->mytree.getBranch(seq, (robot_side)j);
                                if(ret.status == RT_OK)
                                    robot->BroadcastIRMessage(j, IR_MSG_TYPE_ASSEMBLY_INFO, seq.getSymbol(0).data);
                                else
                                    printf("Error in getting branch, no Assembly info be sent\n");
                            }
                            break;
                        default:
                            break; 

                    }
                    break;
                case IR_MSG_TYPE_RECRUITING:
                    {
                        if(robot->current_state!=INORGANISM)
                        {
                            if (!robot->organism_found)
                            {
                                robot->organism_found = true;
                                robot->assembly_count = DEFAULT_ASSEMBLY_COUNT;
                                robot->EnableBlobfinder(false);
                            }

                            //only repond when type is correct
                            if(OrganismSequence::Symbol(it->msg[2]).type2 == robot->type)
                                robot->comm_status[j] = 1;
                            else
                                robot->comm_status[j] = 0;
                        }
                    }
                    break;
                case IR_MSG_TYPE_ASSEMBLY_INFO:
                    {
                        robot->assembly_info_checked = true;
                        robot->assembly_info = OrganismSequence::Symbol(it->msg[2]);
                    }
                    break;
                case IR_MSG_TYPE_OBJECTTYPE:
                    robot->objects_encountered_by_neighbour[j] = (object_type_t)it->msg[2];
                    robot->seeding_shared_info_received[j] +=1;
                    break;
                case IR_MSG_TYPE_COMPLEX:
                    break;
                default:
                    break;
            }

        }
    }
    return 0;
}
int StageRobot::ConnectorUpdate(Model* mod, StageRobot*)
{
    return 0;
}

unsigned int StageRobot::GetChannelByColor(Color color)
{
    const std::vector<Color>& cl=blob->GetColors();
    unsigned int i;
    for (i=0; i < cl.size(); i++)
    {
        if (color == cl[i])
            break;
    }
    return i;
}

void StageRobot::BroadcastIRMessage(int channel, char type, int size, const uint8_t *data)
{
    if(!ircomms[channel])
        return;

    char * buf = new char[size+3];
    buf[0]=size + 2;
    buf[1]=type;
    //memcpy(buf+2, data, size);
    for(int i=0;i<size;i++)
        buf[2+i]=data[i];
    buf[size+2]='\0';

    ircomms[channel]->Transmit(buf);
}

void StageRobot::BroadcastIRMessage(int channel, char type, const uint8_t data) 
{
    BroadcastIRMessage(channel, type, 1, &data);
}

void StageRobot::BroadcastIRMessage(int channel, const uint8_t data)
{
    BroadcastIRMessage(channel, IR_MSG_TYPE_SIMPLE, data);
}

void StageRobot::Visualize( Model* mod, Camera* cam )
{


    // it is not the best place to create the build list, but this works
    OrganismNode::Geom::BuildList();
    //send camera postion to robot3d for better view
    //only need one robot to send
    if(id ==0)
        IPCSendCameraPose(cam->x(), cam->y(), cam->z(), cam->yaw(), cam->pitch());

    if(draw_organism)
    {
        float yaw, pitch;
        pitch = - cam->pitch();
        yaw = - cam->yaw();
        float robotAngle = rtod( pos->GetGlobalPose().a);

        glPushMatrix();

        Pose pose = pos->GetGlobalPose();
        Gl::pose_inverse_shift( pose );

        if(og)
            og->Draw(pose.x, pose.y, 0.0, robotAngle, true);
        glPopMatrix();
    }

    if(draw_state || selected)
    {
        glPushMatrix();

        glPointSize(2.0f);
        glLineWidth(1.0f);

        Gl::pose_inverse_shift( mod->GetGlobalPose() );

        int old_font=fl_font();
        int old_size=fl_size();
        gl_font(FL_HELVETICA_BOLD,24);

        glPointSize(2.0f);
        glColor4f(1.0f,0.0f,0.0f,1.0f);
        Gl::pose_shift( ((ModelPosition*)mod)->est_pose );
        Gl::draw_string( 0, 0.02, 0.06, state_names[current_state]);
        gl_font(old_font,old_size);

        glPopMatrix();
    }

}

void StageRobot::SetStatus(const IPC::status_t& status)
{
    Pose p = pos->GetGlobalPose();
    printf("%d : %s -- Received position from Robot3D (%.2f %.2f %.2f) vs. orig (%.2f %.2f %.2f)\n", timestamp, name,
            status.pos.x/1000.0, status.pos.y/1000.0, status.pos.pa/1000.0, p.x, p.y, p.a);
    SetGlobalPose(status.pos);
}

void StageRobot::SetGlobalPose(const IPC::position_t& p)
{
    if(!newPose)
    {
        newPose = true;
        _x = p.x / 1000.0;
        _y = p.y / 1000.0;
        _pa = p.pa / 1000.0;
    }

}

//print out all status variable
void StageRobot::PrintStatus()
{
    std::cout<<"***--- "<<ClockString()<<" ---*** "<<name<<" *** status"<<std::endl;
    std::cout<<"\t speed: "<<leftwheel<<"  "<<rightwheel<<std::endl;
    std::cout<<"\t current state : "<<state_names[current_state]<<std::endl;
    std::cout<<"\t last state : "<<state_names[last_state]<<std::endl;
    std::cout<<"\t seed : "<<(seed ? "true" : "false")<<std::endl;
    std::cout<<"\t organism found : "<<(organism_found ? "true" : "false")<<std::endl;
    std::cout<<"\t organism formed : "<<(organism_formed ? "true" : "false")<<std::endl;
    std::cout<<"\t proximity enabled : "<<(proximity_enabled ? "true" : "false")<<std::endl;
    std::cout<<"\t lightdetector enabled : "<<(lightdetector_enabled ? "true" : "false")<<std::endl;
    std::cout<<"\t blobfinder enabled : "<<(blobfinder_enabled ? "true" : "false")<<std::endl;
    std::cout<<"\t connectors enabled : \t\t";
    for(int i=FRONT; i<=LEFT; i++)
        std::cout<<"("<<side_names[i]<<(docking_units[i]->GetConfig().active ? ") true  | ":") false | ");
    std::cout<<std::endl;
    std::cout<<"\t robot_in_range_replied :"<<convBase(robot_in_range_replied,2)<< "\t";
    for(int i=FRONT; i<=LEFT; i++)
        std::cout<<"("<<side_names[i]<< (robot_in_range_replied & (1<<i) ? ") true  | ":") false | ");
    std::cout<<std::endl;
    std::cout<<"\t expelling_signals_detected :"<<convBase(expelling_signals_detected,2)<< "\t";
    for(int i=FRONT; i<=LEFT; i++)
        std::cout<<"("<<side_names[i]<< (expelling_signals_detected & (1<<i) ? ") true  | ":") false | ");
    std::cout<<std::endl;
    std::cout<<"\t docked : \t\t\t";
    for(int i=FRONT; i<=LEFT; i++)
    {
        std::cout<<"("<<side_names[i];
        if(docked[i] && neighbours[i])
            std::cout<<") "<<setw(5)<<neighbours[i]->name<<" | ";
        else
            std::cout<<") false | ";
    }
    std::cout<<std::endl;
    std::cout<<"\t recruitment_signal_interval_count : \t\t";
    for(int i=FRONT; i<=LEFT; i++)
        std::cout<<"("<<side_names[i]<< ") "<<setw(5)<<recruitment_signal_interval_count[i]<<" | ";
    std::cout<<std::endl;

    if(current_state == INORGANISM || current_state == RECRUITMENT)
    {
        if(mytree.Size()>0)
            std::cout<<mytree<<std::endl;
        else
            std::cout<<"empty tree, leaf node?"<<std::endl;
    }


    //std::cout<<"\t new robot attached : "<<(newrobot_attached ? "true" : "false")<<std::endl;
    //std::cout<<"\t organism index : "<<og_index<<std::endl;
    //std::cout<<"\t number of robots in organism : "<<num_robots_inorganism<<std::endl;
    //std::cout<<"\t my id in organism : "<<id_inorganism<<std::endl;
    //std::cout<<"\t size : "<<testOG[og_index]->sortedRecruitmentNodeList.size()<<std::endl;
    //if(num_robots_inorganism <testOG[og_index]->sortedNodeList.size())
    //    std::cout<<"\t recruitment id in organism: "<<testOG[og_index]->sortedRecruitmentNodeList[num_robots_inorganism - 1]->id<<std::endl;
    //std::cout<<"\t obstacle_return: "<<pos->vis.obstacle_return<<std::endl;
    //PrintProximitySensor();
    //PrintState();

}
//Es -- stored energy ratio , Vb -- detected blob size ratio
double StageRobot::SeedingProbability(double Es, double Vb)
{
    double ret = Vb * Vb;

    if(Es - para->E1 > 1e-6)
        ret += (1 - Vb)*(1 - Es)/(1 - para->E1);
    else
        ret += (1 - Vb)*(Es - para->E_critical)/(para->E1 - para->E_critical);

    return ret;
}

og_type StageRobot::organism_type(double Vb, object_type_t Ot[NUM_IRS], object_type_t Y[NUM_DOCKS], double V1, double V2)
{
    og_type ret;
    if(Ot[FL] == OBJECT_BARRIER || Ot[FR] ==OBJECT_BARRIER)
    {
        if(Vb < V1)
            ret = S1;
        else
            ret = S4;
    }
    else if(Ot[FL] == OBJECT_WALL || Ot[FR] == OBJECT_WALL)
    {
        if(Y[LEFT] == OBJECT_WALL || Y[RIGHT]== OBJECT_WALL)
            ret = S3;
        else
            ret = S2;
    }
    return ret;
}


}
