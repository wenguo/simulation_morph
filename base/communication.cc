/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#include "communication.hh"
#include "simulation_manager.hh"
#include "robot.hh"

namespace Morph{

    void CommunicationBus::addCommunicationNode(Robot * node)
    {
        CommunicationNodeList.push_back(node);

        //log the first node added to the organism
        if(SimulationManager::getInstance()->logFile.is_open())
        {
            SimulationManager::getInstance()->logFile<<((Robot*)node)->timestamp<<"\t"<<this->bus_id<<
                "\t"<<this->Size()<<"\t"<<this->expected_size<<std::endl;
        }

    }


    void CommunicationBus::removeCommunicationNode(Robot * node)
    {
        std::vector<Robot*>::iterator it = CommunicationNodeList.begin();
        while (it != CommunicationNodeList.end())
        {
            if (node == *it)
                break;

            it++;
        }

        CommunicationNodeList.erase(it);
    }

    bool CommunicationBus::inNodeList(Robot *node)
    {
        bool ret = false;
        std::vector<Robot*>::iterator it = CommunicationNodeList.begin();
        while (it != CommunicationNodeList.end())
        {
            if (node == *it)
            {
                ret = true;
                break;
            }

            it++;
        }

        return ret;

    }

    void CommunicationBus::printNodeList()
    {
        std::vector<Robot*>::iterator it = CommunicationNodeList.begin();
        std::cout<<"bus "<<bus_id<<" :";
        while (it != CommunicationNodeList.end())
        {
            std::cout<< ((Robot*)(*it))->name <<"\t";
            it++;
        }
        std::cout<<std::endl;

    }


}
