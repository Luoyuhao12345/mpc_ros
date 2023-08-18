#ifndef CAR_OBJ_H
#define CAR_OBJ_H


class CarObj
{
public:
    CarObj();
    float get_aim_vel();
    float get_k();
public:
    float x;
    float y;
    float yaw;
    float cur_vel;
    float straight_vel;
    float arc_vel;
    float steer;
};

#endif