/*
 * support.hh
 *
 *  Created on: 29 Apr 2010
 *      Author: wliu
 */

#ifndef SUPPORT_HH_
#define SUPPORT_HH_

#include <string>

using namespace std;

class vect2
{
    public:
        vect2():x(0),y(0){};
        vect2(float _x, float _y):x(_x),y(_y){};
        ~vect2(){};
    float x;
    float y;
};


double simple_normal_deviate( double mean, double stddev );
string datetime_to_string(const tm& time, const char* format);
string convBase(unsigned char v, long base);
int next_comb(int *comb, int k, int n);
bool  init_pos_matrix(vect2 center, vect2 size, vect2 * pos_data, int num);

#endif /* SUPPORT_HH_ */
