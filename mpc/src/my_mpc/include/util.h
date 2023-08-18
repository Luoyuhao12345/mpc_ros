#ifndef __UTIL_H_
#define __UTIL_H_

#include "way_points.h"

#define pi 3.14159

int find_match_point(my_lqr::way_points ref_path, float car_x, float car_y);
float yaw2yaw(float yaw);
float rad2deg(float x);

#endif