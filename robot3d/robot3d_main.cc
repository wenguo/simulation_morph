/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#include <stdlib.h>
#include <unistd.h>
#include <srCore/agentPluginEntryPointBase.h>
#include <srCore/export.h>
#include "robot3d_robot.hh"

using namespace Morph;

class MorphAgentPlugin:public srCore::AgentPluginEntryPointBase
{

    public:
        MorphAgentPlugin()
        {
            started = false;
            setInstanceName("Example Agent");
        }
        ~MorphAgentPlugin(){};
        void startPlugin(srCore::Agent& agent, std::string parameter)
        {
            if (started) return;
            started = true;

            // 	LOG_INFO("Start an agent: " + name + " controlling robot " + robotName);

            Robot3dRobot* a = new Robot3dRobot(agent, parameter);
            //	agent->init(comp->GetConfigObject().getPlugins()[i].properties);
        }

    private:
        bool started;

};


/// This entry point is necessary to be loaded by the Morph simulator
extern "C" ROBOT_EXPORT MorphAgentPlugin* CreatePluginEntryPoint() {

    std::cout << "create plugin entry point " <<std::endl;
    return new MorphAgentPlugin();
}

/// This entry point is necessary to be deallocated by the Morph simulator
extern "C" ROBOT_EXPORT void DestroyPluginEntryPoint() {
    // destroying a singleton?
}
