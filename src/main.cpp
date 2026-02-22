#include <filesystem>
#include <fstream>
#include <iostream>

#include "trajectory_struct.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    {
        std::string command_line = "";
        for (int i = 0; i < argc; ++i)
        {
            command_line += " ";
            command_line += argv[i];
        }
        std::cout << command_line << std::endl;
    }

    std::filesystem::path cur_dir = std::filesystem::current_path();
    std::filesystem::path routes_dir = cur_dir / "routes";
    std::filesystem::path test_route_dir = routes_dir / "topology-test";
    std::filesystem::path topology_dir = test_route_dir / "topology";
    std::filesystem::path trajectories_dir = topology_dir / "trajectories";
    std::filesystem::path map_dir = topology_dir / "map";
    std::filesystem::create_directories(trajectories_dir);
    std::filesystem::create_directories(map_dir);

    dvec3 begin = {0.0, 0.0, 0.0};
    dvec3 shift = {0.0, 100.0, 0.0};
    dvec3 attitude = {0.0, 0.0, 0.0};

    map_object_position_t obj;
    obj.obj_name = "1track";
    obj.position = begin;
    obj.attitude = attitude;
    obj.obj_info = "";

    char DELIMITER_SYMBOL = ',';
    char END_LINE_SYMBOL = ';';
    char NEW_LINE_SYMBOL = '\n';

    std::filesystem::path map_file = map_dir / "route1.map";
    std::cout << map_file << std::endl;
    std::ofstream map_file_stream(map_file, std::ios::out);
    map_file_stream << std::fixed << std::setprecision(6);

    map_file_stream << obj.obj_name.c_str()
        << DELIMITER_SYMBOL << obj.position.x
        << DELIMITER_SYMBOL << obj.position.y
        << DELIMITER_SYMBOL << obj.position.z
        << DELIMITER_SYMBOL << obj.attitude.x
        << DELIMITER_SYMBOL << obj.attitude.y
        << DELIMITER_SYMBOL << obj.attitude.z
        << END_LINE_SYMBOL << NEW_LINE_SYMBOL
        << obj.obj_info.c_str() << NEW_LINE_SYMBOL;

    point_t p1, p2;
    p1.point = begin;
    p1.railway_coord = 0;
    p1.trajectory_coord = 0;
    p2.point = p1.point + shift;
    p2.railway_coord = p1.railway_coord + length(shift);
    p2.trajectory_coord = p1.trajectory_coord + length(shift);

    trajectory_t traj;
    traj.name = "test1.traj";
    traj.points = {p1, p2};

    std::filesystem::path traj_file = trajectories_dir / traj.name;
    std::cout << traj_file << std::endl;
    std::ofstream traj_file_stream(traj_file, std::ios::out);
    traj_file_stream << std::fixed << std::setprecision(6);

    DELIMITER_SYMBOL = '\t';
    for (auto point = traj.points.begin(); point != traj.points.end(); ++point)
    {
        traj_file_stream        << point->point.x
            << DELIMITER_SYMBOL << point->point.y
            << DELIMITER_SYMBOL << point->point.z
            << DELIMITER_SYMBOL << static_cast<int>(round(point->railway_coord))
            << DELIMITER_SYMBOL << point->trajectory_coord
            << NEW_LINE_SYMBOL;
    }

    int tmp;
    std::cin >> tmp;

    return tmp;
}

