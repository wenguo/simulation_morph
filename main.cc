#include <stdlib.h>
#include <unistd.h>
#include "canvas_robot.hh"
#include "stage_robot.hh"
#include "robot3d_robot.hh"

int main(int argc, char ** argv)
{
    Robot3dRobot robot;
    OrganismSequence og_seq;
    OrganismSequence::RandomInit(og_seq, 6);
    robot.SetOgSequence(og_seq);

    while(1)
    {
        robot.Update();
        usleep(1000000);
    }
    return 1;
}
