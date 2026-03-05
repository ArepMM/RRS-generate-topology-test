#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <algorithm>
#include <ctime>

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
void write_switch(tinyxml2::XMLPrinter& topology_file_printer, const switch_t& sw,
                  const char* signal_bwd, const char* signal_fwd)
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

    constexpr const char* SIG_LITER_BWD = "SignalLiterBwd";
    constexpr const char* SIG_MODEL_BWD = "SignalModelBwd";
    constexpr const char* SIG_POS_BWD = "RelPosVectorBwd";

    constexpr const char* SIG_LITER_FWD = "SignalLiterFwd";
    constexpr const char* SIG_MODEL_FWD = "SignalModelFwd";
    constexpr const char* SIG_POS_FWD = "RelPosVectorFwd";

    constexpr const char* SIG_MODEL_VALUE = "ab_rout";
    constexpr const char* SIG_POS_VALUE = "2.5 -2.75 0.0";

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
    if (signal_bwd)
    {
        topology_file_printer.OpenElement(SIG_LITER_BWD);
        topology_file_printer.PushText(signal_bwd);
        topology_file_printer.CloseElement();
        topology_file_printer.OpenElement(SIG_MODEL_BWD);
        topology_file_printer.PushText(SIG_MODEL_VALUE);
        topology_file_printer.CloseElement();
        topology_file_printer.OpenElement(SIG_POS_BWD);
        topology_file_printer.PushText(SIG_POS_VALUE);
        topology_file_printer.CloseElement();
    }
    if (signal_fwd)
    {
        topology_file_printer.OpenElement(SIG_LITER_FWD);
        topology_file_printer.PushText(signal_fwd);
        topology_file_printer.CloseElement();
        topology_file_printer.OpenElement(SIG_MODEL_FWD);
        topology_file_printer.PushText(SIG_MODEL_VALUE);
        topology_file_printer.CloseElement();
        topology_file_printer.OpenElement(SIG_POS_FWD);
        topology_file_printer.PushText(SIG_POS_VALUE);
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
    bool signal_fwd = false;
    bool signal_bwd = false;
    int speed_limit = 100;

    next_trajectory_param_t(int seed)
    {
        init(seed);
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
        std::cout << " reverseswitch " << tmp;
        reverse_switch = (tmp == 0);
        tmp = distribution_signal_fwd(generator);
        std::cout << " signalfwd " << tmp;
        signal_fwd = (tmp == 0);
        tmp = distribution_signal_bwd(generator);
        std::cout << " signalbwd " << tmp;
        signal_bwd = (tmp == 0);
        speed_limit = 20 * distribution_speed_limit(generator);
        std::cout << " speedlimit " << speed_limit << std::endl;
    }

private:

    std::minstd_rand generator;
    std::uniform_int_distribution<> distribution_change_num_track;
    std::uniform_int_distribution<> distribution_reverse_main_track;
    std::uniform_int_distribution<> distribution_reverse_side_track;
    std::uniform_int_distribution<> distribution_reverse_switch;
    std::uniform_int_distribution<> distribution_signal_fwd;
    std::uniform_int_distribution<> distribution_signal_bwd;
    std::uniform_int_distribution<> distribution_speed_limit;

    void init(int seed)
    {
        generator = std::minstd_rand();
        generator.seed(seed);
        distribution_change_num_track = std::uniform_int_distribution<>(0, 6);
        distribution_reverse_main_track = std::uniform_int_distribution<>(0, 3);
        distribution_reverse_side_track = std::uniform_int_distribution<>(0, 3);
        distribution_reverse_switch = std::uniform_int_distribution<>(0, 8);
        distribution_signal_fwd = std::uniform_int_distribution<>(0, 1);
        distribution_signal_bwd = std::uniform_int_distribution<>(0, 1);
        distribution_speed_limit = std::uniform_int_distribution<>(4, 6);//80,100,120
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

    size_t switches = 50;
    int seed = 0;
    if (argc > 1)
    {
        std::string parameter = argv[1];
        int seed_from_args = std::stoi(parameter);
        if (seed_from_args > 0)
        {
            seed = seed_from_args;
        }
    }
    if (seed)
    {
        std::cout << "Seed = " << seed << std::endl;
    }
    else
    {
        const std::time_t system_timestamp = std::time(nullptr);
        seed = system_timestamp;
        std::cout << "Seed: timestamp = " << seed << std::endl;
    }

    // Структура папок маршрута
    std::filesystem::path cur_dir = std::filesystem::current_path();
    std::filesystem::path routes_dir = cur_dir / "routes";
    std::filesystem::path test_route_dir = routes_dir / "topology-test";
    std::filesystem::path topology_dir = test_route_dir / "topology";
    std::filesystem::path trajectories_dir = topology_dir / "trajectories";
    std::filesystem::path map_dir = topology_dir / "map";
    std::filesystem::path models_dir = test_route_dir / "models";
    std::filesystem::path speedmap_dir = topology_dir / "trajectory-speedmap";
    std::filesystem::path ALSN_dir = topology_dir / "trajectory-ALSN";

    // Очищаем предыдущую версию маршрута, создаём папки под новую
    std::filesystem::remove_all(topology_dir);
    std::filesystem::create_directories(trajectories_dir);
    std::filesystem::create_directories(map_dir);
    std::filesystem::create_directories(models_dir);
    std::filesystem::create_directories(speedmap_dir);
    std::filesystem::create_directories(ALSN_dir);

    // Описание
    {
        std::filesystem::path description_file = test_route_dir / "description.xml";
        std::cout << description_file << std::endl;
        std::FILE* description_std_file = std::fopen(description_file.string().c_str(), "w");
        tinyxml2::XMLPrinter description_file_printer = tinyxml2::XMLPrinter(description_std_file);
        description_file_printer.PushHeader(true, true);
        description_file_printer.OpenElement("Config");
        description_file_printer.OpenElement("Route");

        description_file_printer.OpenElement("Title");
        description_file_printer.PushText("Topology test route");
        description_file_printer.CloseElement();
        description_file_printer.OpenElement("Description");
        description_file_printer.PushText("Topology test route");
        description_file_printer.CloseElement();

        description_file_printer.CloseElement();
        description_file_printer.CloseElement();
        std::fclose(description_std_file);
    }

    // Пути к моделям светофоров
    {
        std::filesystem::path models_config_file = topology_dir / "models-config.xml";
        std::cout << models_config_file << std::endl;
        std::FILE* models_config_std_file = std::fopen(models_config_file.string().c_str(), "w");
        tinyxml2::XMLPrinter models_config_file_printer = tinyxml2::XMLPrinter(models_config_std_file);
        models_config_file_printer.PushHeader(true, true);
        models_config_file_printer.OpenElement("Config");
        models_config_file_printer.OpenElement("Models");

        models_config_file_printer.OpenElement("SignalModelsDir");
        models_config_file_printer.PushText("default-objects");
        models_config_file_printer.CloseElement();
        models_config_file_printer.OpenElement("SignalAnimationsDir");
        models_config_file_printer.PushText("default-objects");
        models_config_file_printer.CloseElement();

        models_config_file_printer.CloseElement();
        models_config_file_printer.CloseElement();
        std::fclose(models_config_std_file);
    }

    // Модели путей
    {
        std::filesystem::path repository_data = cur_dir.parent_path() / "RRS-generate-topology-test" / "data";
        copy_files(repository_data, models_dir);
    }

    // Файл objects.ref со ссылками на модели путей
    {
        std::filesystem::path ref_file = test_route_dir / "objects.ref";
        std::cout << ref_file << std::endl;
        std::ofstream ref_file_stream(ref_file, std::ios::out);
        write_ref(ref_file_stream, "1track");
        write_ref(ref_file_stream, "1track1+2");
        write_ref(ref_file_stream, "1track2+1");
        write_ref(ref_file_stream, "2track");
        ref_file_stream.close();
    }

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

    // Файлы speedmap с ограничениями скорости
    std::filesystem::path speedmap_main_file = speedmap_dir / "speedmap_main.xml";
    std::FILE* speedmap_main_std_file = std::fopen(speedmap_main_file.string().c_str(), "w");
    tinyxml2::XMLPrinter speedmap_main_file_printer = tinyxml2::XMLPrinter(speedmap_main_std_file);
    speedmap_main_file_printer.PushHeader(true, true);
    speedmap_main_file_printer.OpenElement("Config");

    std::filesystem::path speedmap_side_file = speedmap_dir / "speedmap_side.xml";
    std::FILE* speedmap_side_std_file = std::fopen(speedmap_side_file.string().c_str(), "w");
    tinyxml2::XMLPrinter speedmap_side_file_printer = tinyxml2::XMLPrinter(speedmap_side_std_file);
    speedmap_side_file_printer.PushHeader(true, true);
    speedmap_side_file_printer.OpenElement("Config");

    // Файлы ALSN с частотами передачи кода АЛСН в рельсовую цепь
    std::filesystem::path ALSN_main_file = ALSN_dir / "ALSN_main.xml";
    std::FILE* ALSN_main_std_file = std::fopen(ALSN_main_file.string().c_str(), "w");
    tinyxml2::XMLPrinter ALSN_main_file_printer = tinyxml2::XMLPrinter(ALSN_main_std_file);
    ALSN_main_file_printer.PushHeader(true, true);
    ALSN_main_file_printer.OpenElement("Config");

    std::filesystem::path ALSN_side_file = ALSN_dir / "ALSN_side.xml";
    std::FILE* ALSN_side_std_file = std::fopen(ALSN_side_file.string().c_str(), "w");
    tinyxml2::XMLPrinter ALSN_side_file_printer = tinyxml2::XMLPrinter(ALSN_side_std_file);
    ALSN_side_file_printer.PushHeader(true, true);
    ALSN_side_file_printer.OpenElement("Config");


    const dvec3 route_shift = {0.0, LEN, 0.0};
    const dvec3 models_shift = {0.0, 0.0, -0.3114};
    const dvec3 attitude = {0.0, 0.0, 0.0};
    dvec3 begin = {0.0, 0.0, 0.0};
    double railway_coord = 0.0;
    const std::string traj_main_name_prefix = "test_main_";
    const std::string traj_side_name_prefix = "test_side_";
    const std::string traj_extension = ".traj";
    std::string prev_main_traj_name = "";
    std::string prev_side_traj_name = "";
    bool prev_main_reversed = false;
    bool prev_side_reversed = false;
    bool is_2_tracks = false;
    std::vector<int> speed_limit = {100};
    std::vector<double> coord_speed_limit = {railway_coord};

    next_trajectory_param_t ntp = next_trajectory_param_t(seed);

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
            if (ntp.speed_limit != speed_limit.back())
            {
                speed_limit.push_back(ntp.speed_limit);
                coord_speed_limit.push_back(railway_coord + LEN * num_track);
            }
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
        // Точки траектории съезда на боковой путь
        for (size_t i = 0; i < NUM_BIAS_POINTS; ++i)
        {
            point.point.x = begin.x + BIAS[i];
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

        point.point.x = point.point.x + SIDE_BIAS;
        traj_side.points.push_back(point);

        for (size_t i = 1; i < (num_track - 1); ++i)
        {
            // Модель двухпутного или однопутного участка
            obj.obj_name = (is_2_tracks) ? "2track" : "1track";
            obj.position = begin + models_shift;
            obj.attitude = attitude;
            obj.obj_info = "";
            write_map(map_file_stream, obj);

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

        begin = begin + route_shift;
        railway_coord = railway_coord + LEN;

        // Точки траектории съезда с бокового пути
        for (size_t i = NUM_BIAS_POINTS; i > 0; --i)
        {
            point_t bias_point;
            point.point.x = begin.x + BIAS[i - 1];
            point.point.y = begin.y - COORD[i - 1];
            point.point.z = begin.z;
            point.railway_coord = railway_coord - COORD[i - 1];
            traj_side.points.push_back(point);
        }

        point.point = begin;
        point.railway_coord = railway_coord;
        traj_main.points.push_back(point);
        traj_side.points.push_back(point);

        // Стрелка в топологии
        switch_t sw;
        sw.name = idx_name;
        if (ntp.reverse_switch)
        {
            sw.name_fwd_plus = prev_main_traj_name;
            sw.name_fwd_minus = prev_side_traj_name;
            sw.reversed_fwd_plus = !prev_main_reversed;
            sw.reversed_fwd_minus = !prev_side_reversed;
        }
        else
        {
            sw.name_bwd_plus = prev_main_traj_name;
            sw.name_bwd_minus = prev_side_traj_name;
            sw.reversed_bwd_plus = prev_main_reversed;
            sw.reversed_bwd_minus = prev_side_reversed;
        }

        {
            // Файл test_main_XXX.traj с траекторией для топологии путей
            traj_main.name = traj_main_name_prefix + idx_name;
            if (ntp.reverse_switch)
            {
                sw.name_bwd_plus = traj_main.name;
                sw.reversed_bwd_plus = !traj_main.reversed;
            }
            else
            {
                sw.name_fwd_plus = traj_main.name;
                sw.reversed_fwd_plus = traj_main.reversed;
            }
            if (traj_main.reversed)
            {
                std::reverse(traj_main.points.begin(), traj_main.points.end());
            }

            speedmap_main_file_printer.OpenElement("Trajectory");
            speedmap_main_file_printer.OpenElement("Name");
            speedmap_main_file_printer.PushText(traj_main.name.c_str());
            speedmap_main_file_printer.CloseElement();
            speedmap_main_file_printer.CloseElement();

            ALSN_main_file_printer.OpenElement("Trajectory");
            ALSN_main_file_printer.OpenElement("Name");
            ALSN_main_file_printer.PushText(traj_main.name.c_str());
            ALSN_main_file_printer.CloseElement();
            ALSN_main_file_printer.CloseElement();

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
            if (ntp.reverse_switch)
            {
                sw.name_bwd_minus = traj_side.name;
                sw.reversed_bwd_minus = !traj_side.reversed;
            }
            else
            {
                sw.name_fwd_minus = traj_side.name;
                sw.reversed_fwd_minus = traj_side.reversed;
            }
            if (traj_side.reversed)
            {
                std::reverse(traj_side.points.begin(), traj_side.points.end());
            }

            speedmap_side_file_printer.OpenElement("Trajectory");
            speedmap_side_file_printer.OpenElement("Name");
            speedmap_side_file_printer.PushText(traj_side.name.c_str());
            speedmap_side_file_printer.CloseElement();
            speedmap_side_file_printer.CloseElement();

            ALSN_side_file_printer.OpenElement("Trajectory");
            ALSN_side_file_printer.OpenElement("Name");
            ALSN_side_file_printer.PushText(traj_side.name.c_str());
            ALSN_side_file_printer.CloseElement();
            ALSN_side_file_printer.CloseElement();

            std::filesystem::path traj_file = trajectories_dir / (traj_side.name + traj_extension);
            std::cout << traj_file << std::endl;
            std::ofstream traj_file_stream(traj_file, std::ios::out);
            traj_file_stream << std::fixed << std::setprecision(6);

            write_traj(traj_file_stream, traj_side);
            traj_file_stream.close();
        }
        else
        {
            if (ntp.reverse_switch)
            {
                sw.name_bwd_minus = "";
                sw.reversed_bwd_minus = true;
            }
            else
            {
                sw.name_fwd_minus = "";
                sw.reversed_fwd_minus = false;
            }
        }

        if (j)
        {
            const char* liter_bwd = ntp.signal_bwd ? idx_name.c_str() : nullptr;
            const char* liter_fwd = ntp.signal_fwd ? idx_name.c_str() : nullptr;
            write_switch(topology_file_printer, sw, liter_bwd, liter_fwd);
        }
        prev_main_traj_name = traj_main.name;
        prev_side_traj_name = traj_side.name;
        prev_main_reversed = traj_main.reversed;
        prev_side_reversed = traj_side.reversed;
        is_2_tracks = !is_2_tracks;
    }

    coord_speed_limit.push_back(railway_coord);
    for (size_t i = 0; i < speed_limit.size(); ++i)
    {
        std::string limit = std::to_string(speed_limit[i])
                    + " " + std::to_string(coord_speed_limit[i])
                    + " " + std::to_string(coord_speed_limit[i+1]);
        speedmap_main_file_printer.OpenElement("SpeedMap");
        speedmap_main_file_printer.OpenElement("SpeedLimit");
        speedmap_main_file_printer.PushText(limit.c_str());
        speedmap_main_file_printer.CloseElement();
        speedmap_main_file_printer.CloseElement();
    }

    speedmap_main_file_printer.CloseElement();
    std::fclose(speedmap_main_std_file);

    std::string limit = std::to_string(40)
                + " " + std::to_string(coord_speed_limit.front())
                + " " + std::to_string(coord_speed_limit.back());
    speedmap_side_file_printer.OpenElement("SpeedMap");
    speedmap_side_file_printer.OpenElement("SpeedLimit");
    speedmap_side_file_printer.PushText(limit.c_str());
    speedmap_side_file_printer.CloseElement();
    speedmap_side_file_printer.CloseElement();

    speedmap_side_file_printer.CloseElement();
    std::fclose(speedmap_side_std_file);

    ALSN_main_file_printer.OpenElement("ALSN");
    ALSN_main_file_printer.OpenElement("Frequency");
    ALSN_main_file_printer.PushText("50");
    ALSN_main_file_printer.CloseElement();
    ALSN_main_file_printer.CloseElement();

    ALSN_main_file_printer.CloseElement();
    std::fclose(ALSN_main_std_file);

    ALSN_side_file_printer.OpenElement("ALSN");
    ALSN_side_file_printer.OpenElement("Frequency");
    ALSN_side_file_printer.PushText("0");
    ALSN_side_file_printer.CloseElement();
    ALSN_side_file_printer.CloseElement();

    ALSN_side_file_printer.CloseElement();
    std::fclose(ALSN_side_std_file);



    topology_file_printer.CloseElement();
    std::fclose(topology_std_file);

    map_file_stream.close();
    return 0;
}

