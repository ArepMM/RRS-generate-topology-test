#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>

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

    dvec3 prev_point = traj.points.front().point;
    double trajectory_coord = 0.0;
    for (auto point = traj.points.begin(); point != traj.points.end(); ++point)
    {
        trajectory_coord += length(point->point - prev_point);
        prev_point = point->point;

        traj_file_stream                     << point->point.x
                         << DELIMITER_SYMBOL << point->point.y
                         << DELIMITER_SYMBOL << point->point.z
                         << DELIMITER_SYMBOL << static_cast<int>(round(point->railway_coord))
                         << DELIMITER_SYMBOL << trajectory_coord
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
struct next_trajectory_param_t
{
    bool change_num_track = false;
    bool reverse_main_track = false;
    bool reverse_side_track = false;
    bool reverse_switch = false;

    next_trajectory_param_t()
    {
        init();
    }

    void update()
    {
        int tmp;
        tmp = distribution_change_num_track(generator);
        std::cout << "changenum " << tmp;
        change_num_track = (tmp == 0);
        tmp = distribution_reverse_main_track(generator);
        std::cout << " reversemain " << tmp;
        reverse_main_track = (tmp == 0);
        tmp = distribution_reverse_side_track(generator);
        std::cout << " reverseside " << tmp;
        reverse_side_track = (tmp == 0);
        tmp = distribution_reverse_switch(generator);
        std::cout << " reverseswitch " << tmp << std::endl;
        reverse_switch = (tmp == 0);
    }

private:

    std::minstd_rand generator;
    std::uniform_int_distribution<> distribution_change_num_track;
    std::uniform_int_distribution<> distribution_reverse_main_track;
    std::uniform_int_distribution<> distribution_reverse_side_track;
    std::uniform_int_distribution<> distribution_reverse_switch;

    void init()
    {
        generator = std::minstd_rand();
        distribution_change_num_track = std::uniform_int_distribution<>(0, 6);
        distribution_reverse_main_track = std::uniform_int_distribution<>(0, 3);
        distribution_reverse_side_track = std::uniform_int_distribution<>(0, 3);
        distribution_reverse_switch = std::uniform_int_distribution<>(0, 8);
    }
};

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

    size_t switches = 30;
    if (argc > 1)
    {
        std::string parameter = argv[1];
        int switches_from_args = std::stoi(parameter);
        if (switches_from_args > 0)
        {
            switches = std::min(switches_from_args, 65535);
        }
    }
    if (!(switches % 2))
    {
        ++switches;
    }

    // Структура папок маршрута
    std::filesystem::path cur_dir = std::filesystem::current_path();
    std::filesystem::path routes_dir = cur_dir / "routes";
    std::filesystem::path test_route_dir = routes_dir / "topology-test";
    std::filesystem::path topology_dir = test_route_dir / "topology";
    std::filesystem::path trajectories_dir = topology_dir / "trajectories";
    std::filesystem::path map_dir = topology_dir / "map";
    std::filesystem::path models_dir = test_route_dir / "models";

    // Очищаем предыдущую версию маршрута, создаём папки под новую
    std::filesystem::remove_all(topology_dir);
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

    const dvec3 route_shift = {0.0, LEN, 0.0};
    const dvec3 models_shift = {0.0, 0.0, -0.3114};
    const dvec3 attitude = {0.0, 0.0, 0.0};
    dvec3 begin = {0.0, 0.0, 0.0};
    double railway_coord = 0.0;
    std::string prev_main_traj_name = "";
    std::string prev_side_traj_name = "";
    const std::string traj_main_name_prefix = "test_main_";
    const std::string traj_side_name_prefix = "test_side_";
    const std::string traj_extension = ".traj";
    bool is_2_tracks = false;

    next_trajectory_param_t ntp = next_trajectory_param_t();

    for (size_t j = 0; j < switches; ++j)
    {
        std::string idx_name = std::to_string(j);
        while(idx_name.size() < 5)
        {
            idx_name = "0" + idx_name;
        }

        size_t num_track = 1;
        do
        {
            ntp.update();
            ++num_track;
        }
        while((!(ntp.change_num_track)) && (num_track < 10));

        // Модель стрелки или однопутного участка
        map_object_position_t obj;
        obj.obj_name = (is_2_tracks) ? "1track1+2" : "1track";
        obj.position = begin + models_shift;
        obj.attitude = attitude;
        obj.obj_info = "";
        write_map(map_file_stream, obj);

        // Первые точки траекторий
        point_t point;
        point.point = begin;
        point.railway_coord = railway_coord;

        trajectory_t traj_main;
        traj_main.reversed = ntp.reverse_main_track;
        traj_main.points = {point};

        trajectory_t traj_side;
        traj_side.reversed = ntp.reverse_side_track;
        traj_side.points = {point};
        for (size_t i = 0; i < NUM_BIAS_POINTS; ++i)
        {
            point.point.x = begin.x + BIAS[i];
            point.point.y = begin.y + COORD[i];
            point.point.z = begin.z;
            point.railway_coord = railway_coord + COORD[i];
            traj_side.points.push_back(point);
        }

        for (size_t i = 0; i < (num_track - 1); ++i)
        {
            if (i > 0)
            {
                // Модель двухпутного или однопутного участка
                obj.obj_name = (is_2_tracks) ? "2track" : "1track";
                obj.position = begin + models_shift;
                obj.attitude = attitude;
                obj.obj_info = "";
                write_map(map_file_stream, obj);
            }

            begin = begin + route_shift;
            railway_coord = railway_coord + LEN;

            // Промежуточные точки траекторий
            point.point = begin;
            point.railway_coord = railway_coord;
            traj_main.points.push_back(point);

            point.point.x = point.point.x + SIDE_BIAS;
            traj_side.points.push_back(point);
        }

        // Модель стрелки или однопутного участка
        obj.obj_name = (is_2_tracks) ? "1track2+1" : "1track";
        obj.position = begin + models_shift;
        obj.attitude = attitude;
        obj.obj_info = "";
        write_map(map_file_stream, obj);

        for (size_t i = 0; i < NUM_BIAS_POINTS; ++i)
        {
            point_t bias_point;
            point.point.x = begin.x + BIAS[NUM_BIAS_POINTS - i - 1];
            point.point.y = begin.y + COORD[i];
            point.point.z = begin.z;
            point.railway_coord = railway_coord + COORD[i];
            traj_side.points.push_back(point);
        }

        begin = begin + route_shift;
        railway_coord = railway_coord + LEN;
        point.point = begin;
        point.railway_coord = railway_coord;
        traj_main.points.push_back(point);
        traj_side.points.push_back(point);

        // Стрелка в топологии
        switch_t sw;
        sw.name = idx_name;
        sw.name_bwd_plus = prev_main_traj_name;
        sw.name_bwd_minus = prev_side_traj_name;

        {
            // Файл test_main_XXX.traj с траекторией для топологии путей
            traj_main.name = traj_main_name_prefix + idx_name;
            sw.name_fwd_plus = traj_main.name;

            std::filesystem::path traj_file = trajectories_dir / (traj_main.name + traj_extension);
            std::cout << traj_file << std::endl;
            std::ofstream traj_file_stream(traj_file, std::ios::out);
            traj_file_stream << std::fixed << std::setprecision(6);

            write_traj(traj_file_stream, traj_main);
            traj_file_stream.close();
        }

        if (is_2_tracks)
        {
            // Файл test_side_XXX.traj с траекторией для топологии путей
            traj_side.name = traj_side_name_prefix + idx_name;
            sw.name_fwd_minus = traj_side.name;

            std::filesystem::path traj_file = trajectories_dir / (traj_side.name + traj_extension);
            std::cout << traj_file << std::endl;
            std::ofstream traj_file_stream(traj_file, std::ios::out);
            traj_file_stream << std::fixed << std::setprecision(6);

            write_traj(traj_file_stream, traj_side);
            traj_file_stream.close();
        }
        else
        {
            sw.name_fwd_minus = "";
        }

        if (j)
        {
            write_switch(topology_file_printer, sw);
        }
        prev_main_traj_name = sw.name_fwd_plus;
        prev_side_traj_name = sw.name_fwd_minus;
        is_2_tracks = !is_2_tracks;
    }

    topology_file_printer.CloseElement();
    std::fclose(topology_std_file);

    map_file_stream.close();
    return 0;
}

