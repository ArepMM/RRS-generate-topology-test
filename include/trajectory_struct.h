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
struct switch_t
{
    std::string name = "";
    std::string name_bwd_plus = "";
    std::string name_bwd_minus = "";
    std::string name_fwd_plus = "";
    std::string name_fwd_minus = "";
    bool reversed_bwd_plus = false;
    bool reversed_bwd_minus = false;
    bool reversed_fwd_plus = false;
    bool reversed_fwd_minus = false;
    bool bwd_to_minus = false;
    bool fwd_to_minus = false;
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
