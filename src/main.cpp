#include <filesystem>
#include <fstream>
#include <iostream>

#include "trajectory_struct.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void copy_files(std::filesystem::path& from_dir, std::filesystem::path& to_dir)
{
    for (const auto& file_it : std::filesystem::directory_iterator{from_dir})
    {
        std::filesystem::path from = file_it;
        std::filesystem::path to = to_dir / from.filename();
        std::cout << "Copy file: [" << from << "]  ->  [" << to << "]" << std::endl;

        std::error_code ec;
        std::filesystem::remove(to);
        std::filesystem::copy(from, to, ec);
        if (ec)
        {
            std::cout << "[" << ec.value() << "] " << ec.category().name() << ": " << ec.message() << std::endl;
        }
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void write_ref(std::ofstream& ref_file_stream, const char* model_name)
{
    constexpr char DELIMITER_SYMBOL = '\t';
    constexpr char NEW_LINE_SYMBOL = '\n';

    ref_file_stream << model_name << DELIMITER_SYMBOL
        << "/models/" << model_name << ".gltf" << NEW_LINE_SYMBOL;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void write_map(std::ofstream& map_file_stream, const map_object_position_t& obj)
{
    constexpr char DELIMITER_SYMBOL = ',';
    constexpr char END_LINE_SYMBOL = ';';
    constexpr char NEW_LINE_SYMBOL = '\n';

    map_file_stream << obj.obj_name.c_str()
                    << DELIMITER_SYMBOL << obj.position.x
                    << DELIMITER_SYMBOL << obj.position.y
                    << DELIMITER_SYMBOL << obj.position.z
                    << DELIMITER_SYMBOL << obj.attitude.x
                    << DELIMITER_SYMBOL << obj.attitude.y
                    << DELIMITER_SYMBOL << obj.attitude.z
                    << END_LINE_SYMBOL << NEW_LINE_SYMBOL
                    << obj.obj_info.c_str() << NEW_LINE_SYMBOL;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void write_traj(std::ofstream& traj_file_stream, const trajectory_t& traj)
{
    constexpr char DELIMITER_SYMBOL = '\t';
    constexpr char NEW_LINE_SYMBOL = '\n';

    for (auto point = traj.points.begin(); point != traj.points.end(); ++point)
    {
        traj_file_stream                     << point->point.x
                         << DELIMITER_SYMBOL << point->point.y
                         << DELIMITER_SYMBOL << point->point.z
                         << DELIMITER_SYMBOL << static_cast<int>(round(point->railway_coord))
                         << DELIMITER_SYMBOL << point->trajectory_coord
                         << NEW_LINE_SYMBOL;
    }
}

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

    // Структура папок маршрута
    std::filesystem::path cur_dir = std::filesystem::current_path();
    std::filesystem::path routes_dir = cur_dir / "routes";
    std::filesystem::path test_route_dir = routes_dir / "topology-test";
    std::filesystem::path topology_dir = test_route_dir / "topology";
    std::filesystem::path trajectories_dir = topology_dir / "trajectories";
    std::filesystem::path map_dir = topology_dir / "map";
    std::filesystem::path models_dir = test_route_dir / "models";
    std::filesystem::create_directories(trajectories_dir);
    std::filesystem::create_directories(map_dir);
    std::filesystem::create_directories(models_dir);

    // Модели путей
    std::filesystem::path repository_data = cur_dir.parent_path() / "RRS-generate-topology-test" / "data";
    copy_files(repository_data, models_dir);

    // Файл objects.ref со ссылками на модели путей
    std::filesystem::path ref_file = test_route_dir / "objects.ref";
    std::cout << ref_file << std::endl;
    std::ofstream ref_file_stream(ref_file, std::ios::out);
    write_ref(ref_file_stream, "1track");
    write_ref(ref_file_stream, "1track1+2");
    write_ref(ref_file_stream, "1track2+1");
    write_ref(ref_file_stream, "2track");
    ref_file_stream.close();

    // Файл route1.map с положениями объектов
    std::filesystem::path map_file = map_dir / "route1.map";
    std::cout << map_file << std::endl;
    std::ofstream map_file_stream(map_file, std::ios::out);
    map_file_stream << std::fixed << std::setprecision(6);

    // Файл test1.traj с траекторией для топологии путей
    trajectory_t traj;
    traj.name = "test1.traj";
    std::filesystem::path traj_file = trajectories_dir / traj.name;
    std::cout << traj_file << std::endl;
    std::ofstream traj_file_stream(traj_file, std::ios::out);
    traj_file_stream << std::fixed << std::setprecision(6);

    dvec3 begin = {0.0, 0.0, 0.0};
    dvec3 route_shift = {0.0, 100.0, 0.0};
    dvec3 models_shift = {0.0, 0.0, -0.3114};
    dvec3 attitude = {0.0, 0.0, 0.0};
    double len = length(route_shift);
    double railway_coord = 0.0;
    double trajectory_coord = 0.0;

    point_t point;
    point.point = begin;
    point.railway_coord = 0;
    point.trajectory_coord = 0;
    traj.points = {point};

    for (size_t i = 0; i < 4; ++i)
    {
        // Модель однопутной траектории
        map_object_position_t obj;
        obj.obj_name = "1track";
        obj.position = begin + models_shift;
        obj.attitude = attitude;
        obj.obj_info = "";

        write_map(map_file_stream, obj);

        begin = begin + route_shift;
        railway_coord = railway_coord + len;
        trajectory_coord = trajectory_coord + len;

        // Однопутная траектория
        point.point = begin;
        point.railway_coord = railway_coord;
        point.trajectory_coord = trajectory_coord;
        traj.points.push_back(point);
    }

    write_traj(traj_file_stream, traj);
    traj_file_stream.close();

    map_file_stream.close();
    return 0;
}

