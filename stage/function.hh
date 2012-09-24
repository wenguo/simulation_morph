/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef FUNCTION_HH
#define FUNCTION_HH
#include <math.h>

inline double Score_neighbour(int t, double k=0.01)
{
    return (1-exp(-k*(600-t)))/(1+ exp(-k*(600-t)));
}

inline double f3(int n, int t, double K_r = 1, double k = 0.3) 
{ 
    return K_r * 1/(1+exp(-k*n)) * t;
    //return K_r * (1-exp(-k*n))/(1+exp(-k*n)) * t;
}

#endif
