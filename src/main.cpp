#include <filesystem>
#include <fstream>
#include <iostream>

#include "tinyxml2.h"
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
void write_switch(tinyxml2::XMLPrinter& topology_file_printer, const switch_t& sw)
{
    constexpr const char* SWITCH_NODE = "Switch";
    constexpr const char* NAME_NODE = "Name";
    constexpr const char* BWD_PLUS = "bwdPlusTraj";
    constexpr const char* BWD_MINUS = "bwdMinusTraj";
    constexpr const char* FWD_PLUS = "fwdPlusTraj";
    constexpr const char* FWD_MINUS = "fwdMinusTraj";
    constexpr const char* BWD_PLUS_R = "bwdPlusTrajReversed";
    constexpr const char* BWD_MINUS_R = "bwdMinusTrajReversed";
    constexpr const char* FWD_PLUS_R = "fwdPlusTrajReversed";
    constexpr const char* FWD_MINUS_R = "fwdMinusTrajReversed";
    constexpr const char* BWD_STATE = "state_bwd";
    constexpr const char* FWD_STATE = "state_fwd";

    topology_file_printer.OpenElement(SWITCH_NODE);
    if (!sw.name.empty())
    {
        topology_file_printer.OpenElement(NAME_NODE);
        topology_file_printer.PushText(sw.name.c_str());
        topology_file_printer.CloseElement();
    }
    if (!sw.name_bwd_plus.empty())
    {
        if (sw.reversed_bwd_plus)
        {
            topology_file_printer.OpenElement(BWD_PLUS_R);
            topology_file_printer.PushText(sw.name_bwd_plus.c_str());
            topology_file_printer.CloseElement();
        }
        else
        {
            topology_file_printer.OpenElement(BWD_PLUS);
            topology_file_printer.PushText(sw.name_bwd_plus.c_str());
            topology_file_printer.CloseElement();
        }
    }
    if (!sw.name_bwd_minus.empty())
    {
        if (sw.reversed_bwd_minus)
        {
            topology_file_printer.OpenElement(BWD_MINUS_R);
            topology_file_printer.PushText(sw.name_bwd_minus.c_str());
            topology_file_printer.CloseElement();
        }
        else
        {
            topology_file_printer.OpenElement(BWD_MINUS);
            topology_file_printer.PushText(sw.name_bwd_minus.c_str());
            topology_file_printer.CloseElement();
        }
    }
    if (!sw.name_fwd_plus.empty())
    {
        if (sw.reversed_fwd_plus)
        {
            topology_file_printer.OpenElement(FWD_PLUS_R);
            topology_file_printer.PushText(sw.name_fwd_plus.c_str());
            topology_file_printer.CloseElement();
        }
        else
        {
            topology_file_printer.OpenElement(FWD_PLUS);
            topology_file_printer.PushText(sw.name_fwd_plus.c_str());
            topology_file_printer.CloseElement();
        }
    }
    if (!sw.name_fwd_minus.empty())
    {
        if (sw.reversed_fwd_minus)
        {
            topology_file_printer.OpenElement(FWD_MINUS_R);
            topology_file_printer.PushText(sw.name_fwd_minus.c_str());
            topology_file_printer.CloseElement();
        }
        else
        {
            topology_file_printer.OpenElement(FWD_MINUS);
            topology_file_printer.PushText(sw.name_fwd_minus.c_str());
            topology_file_printer.CloseElement();
        }
    }
    if (sw.bwd_to_minus)
    {
        topology_file_printer.OpenElement(BWD_STATE);
        topology_file_printer.PushText("-1");
        topology_file_printer.CloseElement();
    }
    if (sw.fwd_to_minus)
    {
        topology_file_printer.OpenElement(FWD_STATE);
        topology_file_printer.PushText("-1");
        topology_file_printer.CloseElement();
    }
    topology_file_printer.CloseElement();
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

    // Файл topology.xml с соединениями траекторий
    std::filesystem::path topology_file = topology_dir / "topology.xml";
    std::FILE* topology_std_file = std::fopen(topology_file.string().c_str(), "w");
    tinyxml2::XMLPrinter topology_file_printer = tinyxml2::XMLPrinter(topology_std_file);
    topology_file_printer.PushHeader(true, true);
    topology_file_printer.OpenElement("Config");

    const dvec3 route_shift = {0.0, 100.0, 0.0};
    const dvec3 models_shift = {0.0, 0.0, -0.3114};
    const dvec3 attitude = {0.0, 0.0, 0.0};
    dvec3 begin = {0.0, 0.0, 0.0};
    double len = length(route_shift);
    double railway_coord = 0.0;
    double trajectory_coord = 0.0;
    std::string prev_traj_name = "";
    const std::string traj_name_prefix = "test_";
    const std::string traj_extension = ".traj";

    for (size_t j = 0; j < 4; ++j)
    {
        // Файл test_XXX.traj с траекторией для топологии путей
        trajectory_t traj;
        traj.name = traj_name_prefix + std::to_string(j);
        std::filesystem::path traj_file = trajectories_dir / (traj.name + traj_extension);
        std::cout << traj_file << std::endl;
        std::ofstream traj_file_stream(traj_file, std::ios::out);
        traj_file_stream << std::fixed << std::setprecision(6);

        point_t point;
        point.point = begin;
        point.railway_coord = railway_coord;
        point.trajectory_coord = trajectory_coord;
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

        if (j)
        {
            switch_t sw;
            sw.name = std::to_string(j);
            while(sw.name.size() < 4)
            {
                sw.name = "0" + sw.name;
            }
            sw.name_bwd_plus = prev_traj_name;
            sw.name_fwd_plus = traj.name;
            write_switch(topology_file_printer, sw);
        }
        prev_traj_name = traj.name;
    }

    topology_file_printer.CloseElement();
    std::fclose(topology_std_file);

    map_file_stream.close();
    return 0;
}

