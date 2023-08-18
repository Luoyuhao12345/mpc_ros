#include "car_obj.h"
#include "util.h"


CarObj::CarObj()
{
    x = y = yaw = 0;
    cur_vel = straight_vel =0;
    arc_vel = steer = 0;
}

int arc = 60;
float CarObj::get_aim_vel()
{
    if(abs(rad2deg(steer))<arc) return straight_vel;
    else return arc_vel;
}

float CarObj::get_k()
{
    if(abs(rad2deg(steer))<arc) return 1;
    else return 1;
}