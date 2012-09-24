/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#include "organism_sample.hh"

void testOrganism1(Organism * og)
{
    OrganismNode * node[12];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_SCOUT), BACK, FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), FRONT, LEFT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT, FRONT);
    node[3] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK, LEFT);
    node[4] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), FRONT, RIGHT);
    node[5] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), RIGHT, LEFT);
    node[6] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), LEFT, RIGHT);
    node[7] = og->Insert(node[5], new OrganismNode(ROBOT_AW), RIGHT, FRONT);
    node[8] = og->Insert(node[6], new OrganismNode(ROBOT_AW), LEFT, FRONT);
    node[9] = og->Insert(node[0], new OrganismNode(ROBOT_AW), BACK, FRONT);
    node[10] = og->Insert(node[2], new OrganismNode(ROBOT_AW), BACK, FRONT);
}

//a crossing with 7 robots
void testOrganism2(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), LEFT);
    node[6] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), RIGHT);
}

//a snake with 5 robots
void testOrganism3(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
}

//a Z with 8 robots
void testOrganism4(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), LEFT);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
}

/*
 * Example shape for morphgenesis paper
 */
void MEchapter_shape1(Organism * og)
{
    Organism *test = new Organism;
    OrganismNode *node0, *node1;

    node0 = og->Insert(NULL,new OrganismNode(ROBOT_KIT));
    node0 = og->Insert(node0,new OrganismNode(ROBOT_KIT),BACK);
    node1 = og->Insert(node0,new OrganismNode(ROBOT_KIT),BACK);
    node0 = og->Insert(node1,new OrganismNode(ROBOT_KIT),BACK);
    node0 = og->Insert(node0,new OrganismNode(ROBOT_KIT),BACK);
    node0 = og->Insert(node1,new OrganismNode(ROBOT_KIT),RIGHT);

    node0 = og->Insert(node0,new OrganismNode(ROBOT_KIT),BACK);
    node1 = og->Insert(node0,new OrganismNode(ROBOT_KIT),BACK);
    node0 = og->Insert(node1,new OrganismNode(ROBOT_KIT),RIGHT);
    node0 = og->Insert(node0,new OrganismNode(ROBOT_KIT),BACK);
    node0 = og->Insert(node1,new OrganismNode(ROBOT_KIT),LEFT);
    node0 = og->Insert(node0,new OrganismNode(ROBOT_KIT),BACK);

}


/*
 * Example shape for morphgenesis paper
 */
void MEchapter_shape2(Organism * og)
{

    OrganismNode *node0, *node1, *node2;
    node0 = og->Insert(NULL,new OrganismNode(ROBOT_KIT));
    node1 = og->Insert(node0,new OrganismNode(ROBOT_KIT),BACK);
    node1 = og->Insert(node1,new OrganismNode(ROBOT_KIT),BACK);
    node2 = og->Insert(node1,new OrganismNode(ROBOT_KIT),BACK);

    node1 = og->Insert(node0,new OrganismNode(ROBOT_KIT),LEFT);
    node1 = og->Insert(node1,new OrganismNode(ROBOT_KIT),BACK);

    node1 = og->Insert(node0,new OrganismNode(ROBOT_KIT),RIGHT);
    node1 = og->Insert(node1,new OrganismNode(ROBOT_KIT),BACK);

    node1 = og->Insert(node2,new OrganismNode(ROBOT_KIT),LEFT);
    node1 = og->Insert(node1,new OrganismNode(ROBOT_KIT),BACK);

    node1 = og->Insert(node2,new OrganismNode(ROBOT_KIT),RIGHT);
    node1 = og->Insert(node1,new OrganismNode(ROBOT_KIT),BACK);
}

void dars2012a0(Organism * og)
{
    OrganismNode * node[11];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[8] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
    node[9] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
}
void dars2012a1(Organism * og)
{
    OrganismNode * node[11];
    node[1] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), FRONT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[8] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
    node[9] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
}
void dars2012a2(Organism * og)
{
    OrganismNode * node[11];
    node[2] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), FRONT);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[8] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
    node[9] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
}
void dars2012a3(Organism * og)
{
    OrganismNode * node[11];
    node[3] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[8] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
    node[9] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
}
void dars2012a4(Organism * og)
{
    OrganismNode * node[11];
    node[4] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), FRONT);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[8] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
    node[9] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
}
void dars2012a5(Organism * og)
{
    OrganismNode * node[11];
    node[5] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[4] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), FRONT);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[8] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
    node[9] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
}
void dars2012a6(Organism * og)
{
    OrganismNode * node[11];
    node[6] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[5] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), FRONT);
    node[4] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[8] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
    node[9] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
}
void dars2012a7(Organism * og)
{
    OrganismNode * node[11];
    node[7] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), FRONT);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
    node[8] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
}
void dars2012a8(Organism * og)
{
    OrganismNode * node[11];
    node[8] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), FRONT);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[7] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
}
void dars2012a9(Organism * og)
{
    OrganismNode * node[11];
    node[9] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[5] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), FRONT);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[8] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
}
void dars2012a10(Organism * og)
{
    OrganismNode * node[11];
    node[10] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[5] = og->Insert(node[10], new OrganismNode(ROBOT_KIT), FRONT);
    node[4] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[9] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT);
    node[8] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT);
}

void snake(Organism * og)
{
    OrganismNode * node[11];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), FRONT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}

void humanlike(Organism *og)
{
    OrganismNode * node[33];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), RIGHT);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), LEFT);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), LEFT);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
    node[11] = og->Insert(node[10], new OrganismNode(ROBOT_KIT), BACK);
    node[12] = og->Insert(node[11], new OrganismNode(ROBOT_KIT), RIGHT);
    node[13] = og->Insert(node[12], new OrganismNode(ROBOT_KIT), BACK);
    node[14] = og->Insert(node[13], new OrganismNode(ROBOT_KIT), BACK);
    node[15] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), FRONT);
    node[16] = og->Insert(node[15], new OrganismNode(ROBOT_KIT), BACK);
    node[17] = og->Insert(node[16], new OrganismNode(ROBOT_KIT), BACK);
    node[18] = og->Insert(node[17], new OrganismNode(ROBOT_KIT), BACK);
    node[19] = og->Insert(node[18], new OrganismNode(ROBOT_KIT), BACK);
    node[20] = og->Insert(node[19], new OrganismNode(ROBOT_KIT), BACK);
    node[21] = og->Insert(node[20], new OrganismNode(ROBOT_KIT), RIGHT);
    node[22] = og->Insert(node[20], new OrganismNode(ROBOT_KIT), LEFT);
    node[23] = og->Insert(node[16], new OrganismNode(ROBOT_KIT), RIGHT);
    node[24] = og->Insert(node[23], new OrganismNode(ROBOT_KIT), BACK);
    node[25] = og->Insert(node[24], new OrganismNode(ROBOT_KIT), BACK);
    node[26] = og->Insert(node[25], new OrganismNode(ROBOT_KIT), BACK);
    node[27] = og->Insert(node[26], new OrganismNode(ROBOT_KIT), BACK);
    node[28] = og->Insert(node[16], new OrganismNode(ROBOT_KIT), LEFT);
    node[29] = og->Insert(node[28], new OrganismNode(ROBOT_KIT), BACK);
    node[30] = og->Insert(node[29], new OrganismNode(ROBOT_KIT), BACK);
    node[31] = og->Insert(node[30], new OrganismNode(ROBOT_KIT), BACK);
    node[32] = og->Insert(node[31], new OrganismNode(ROBOT_KIT), BACK);

}

void dars2012b0(Organism * og)
{
    OrganismNode * node[11];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}
void dars2012b1(Organism * og)
{
    OrganismNode * node[11];
    node[1] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), FRONT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}

void dars2012b2(Organism * og)
{
    OrganismNode * node[11];
    node[2] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), FRONT);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}

void dars2012b3(Organism * og)
{
    OrganismNode * node[11];
    node[3] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}

void dars2012b4(Organism * og)
{
    OrganismNode * node[11];
    node[4] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), FRONT);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}

void dars2012b5(Organism * og)
{
    OrganismNode * node[11];
    node[5] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[4] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), FRONT);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}

void dars2012b6(Organism * og)
{
    OrganismNode * node[11];
    node[6] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[5] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), FRONT);
    node[4] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}

void dars2012b7(Organism * og)
{
    OrganismNode * node[11];
    node[7] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[6] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), FRONT);
    node[5] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}

void dars2012b8(Organism * og)
{
    OrganismNode * node[11];
    node[8] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[7] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), FRONT);
    node[6] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}

void dars2012b9(Organism * og)
{
    OrganismNode * node[11];
    node[9] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[8] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), FRONT);
    node[7] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[1] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[0] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}

void dars2012b10(Organism * og)
{
    OrganismNode * node[11];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), FRONT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
    node[8] = og->Insert(node[7], new OrganismNode(ROBOT_KIT), BACK);
    node[9] = og->Insert(node[8], new OrganismNode(ROBOT_KIT), BACK);
    node[10] = og->Insert(node[9], new OrganismNode(ROBOT_KIT), BACK);
}



