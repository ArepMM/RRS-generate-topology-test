#ifndef TRAJECTORY_STRUCT_H
#define TRAJECTORY_STRUCT_H

#include    <vector>
#include    <string>
#include    "vec3.h"

constexpr double LEN = 100.0;
constexpr double SIDE_BIAS = 7.5;

/// Моделируем траекторию отклонения пятью отрезками
/// с четырями промежуточными точками по полиномиальному сплайну
/// пятой степени 6*x^5 - 15*x^4 + 10*x^3
constexpr size_t NUM_BIAS_POINTS = 4;
constexpr double COORD_COEFF[NUM_BIAS_POINTS] = {0.2, 0.4, 0.6, 0.8};
constexpr double BIAS_COEFF[NUM_BIAS_POINTS] =
{
    COORD_COEFF[0] * COORD_COEFF[0] * COORD_COEFF[0] * (6.0 * COORD_COEFF[0] * COORD_COEFF[0] - 15.0 * COORD_COEFF[0] + 10.0),
    COORD_COEFF[1] * COORD_COEFF[1] * COORD_COEFF[1] * (6.0 * COORD_COEFF[1] * COORD_COEFF[1] - 15.0 * COORD_COEFF[1] + 10.0),
    COORD_COEFF[2] * COORD_COEFF[2] * COORD_COEFF[2] * (6.0 * COORD_COEFF[2] * COORD_COEFF[2] - 15.0 * COORD_COEFF[2] + 10.0),
    COORD_COEFF[3] * COORD_COEFF[3] * COORD_COEFF[3] * (6.0 * COORD_COEFF[3] * COORD_COEFF[3] - 15.0 * COORD_COEFF[3] + 10.0)
};
constexpr double COORD[NUM_BIAS_POINTS] =
{
    LEN * COORD_COEFF[0],
    LEN * COORD_COEFF[1],
    LEN * COORD_COEFF[2],
    LEN * COORD_COEFF[3]
};
constexpr double BIAS[NUM_BIAS_POINTS] =
{
    SIDE_BIAS * BIAS_COEFF[0],
    SIDE_BIAS * BIAS_COEFF[1],
    SIDE_BIAS * BIAS_COEFF[2],
    SIDE_BIAS * BIAS_COEFF[3]
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct point_t
{
    dvec3 point = {0.0, 0.0, 0.0};
    double railway_coord = 0.0;
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct trajectory_t
{
    std::string name = "";
    std::vector<point_t> points = {};
    bool reversed;
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
