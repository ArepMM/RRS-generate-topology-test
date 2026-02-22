#ifndef TRAJECTORY_STRUCT_H
#define TRAJECTORY_STRUCT_H

#include    <vector>
#include    <string>
#include    "vec3.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct point_t
{
    dvec3 point = {0.0, 0.0, 0.0};
    double railway_coord = 0.0;
    double trajectory_coord = 0.0;
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct trajectory_t
{
    std::string name = "";
    std::vector<point_t> points = {};
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct map_object_position_t
{
    std::string obj_name = "";
    dvec3       position = dvec3(0.0, 0.0, 0.0);
    dvec3       attitude = dvec3(0.0, 0.0, 0.0);
    std::string obj_info = "";
};

#endif // TRAJECTORY_STRUCT_H
