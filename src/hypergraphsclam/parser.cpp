#include <string>
#include <sstream>
#include <array>
#include <unistd.h>
#include <boost/filesystem/operations.hpp>

#include <GrabData.hpp>
#include <StampedGPSPose.hpp>
#include <StringHelper.hpp>

void
prepare_all_directories()
{
    // remove the directory contents recursively, and then removes the directory.
    boost::filesystem::remove_all("/dados/tmp");
    boost::filesystem::create_directory("/dados/tmp");
    boost::filesystem::create_directory("/dados/tmp/sick");
    boost::filesystem::create_directory("/dados/tmp/velodyne");
    boost::filesystem::create_directory("/dados/tmp/lgm");
    boost::filesystem::create_directory("/dados/tmp/lgm/sick");
    boost::filesystem::create_directory("/dados/tmp/lgm/velodyne");
    boost::filesystem::create_directory("/dados/tmp/images");
    boost::filesystem::create_directory("/dados/tmp/loops");
    boost::filesystem::create_directory("/dados/tmp/loops/sick");
    boost::filesystem::create_directory("/dados/tmp/loops/velodyne");
    std::cout << "Necessary directories created." << std::endl;
}

std::vector<std::array<std::string, 3>> 
log_list_parser(const std::string &log_list_filename)
{
    std::string carmen_home(getenv("CARMEN_HOME"));
    std::string carmen_data_folder(carmen_home + "/data/");
    std::string config_folder(carmen_data_folder + "hypergraphsclam/config/");
    std::string carmen_src_folder(carmen_home + "/src/");

    std::vector<std::array<std::string, 3>> log_list;

    std::ifstream file { log_list_filename, std::ifstream::in };

    if (file.is_open())
    {
        std::stringstream current_line;

        while (hyper::StringHelper::ReadLine(file, current_line) > 0)
        {
            std::string log;
            std::string parser_config_file;
            std::string carmen_ini;

            current_line >> log;
            current_line >> parser_config_file;
            current_line >> carmen_ini;

            if (log.empty())
                std::cerr << "Invalid line of " << log_list_filename << " could not get log file name!" << std::endl;
            else if (parser_config_file.empty())
                std::cerr << "Invalid line of " << log_list_filename << " could not get parser_config_file name!" << std::endl;
            else if (carmen_ini.empty())
                std::cerr << "Invalid line of " << log_list_filename << " could not get carmen_ini file name!" << std::endl;
            else
                log_list.emplace_back(std::array<std::string, 3> { log, config_folder + parser_config_file, carmen_src_folder + carmen_ini });
        }
    }
    else
        std::cerr << "Unable to open the log list file: " << log_list_filename << "\n";

    return (log_list);
}


bool
parse_logs(std::vector<hyper::GrabData> &gds, std::vector<std::array<std::string, 3>> &logs)
{
	std::size_t gid { 1 };
    std::size_t last_id { 6 };

    for (auto &input_files : logs)
    {
        gds.emplace_back(hyper::GrabData { gid++ });

        hyper::GrabData &gd { gds.back() };

        gd.Configure(input_files[1], input_files[2]);

        gd.LoadLoopClosureBuffer("_sync.txt");

        last_id = gd.ParseLogFile(input_files[0], last_id);

        if (0 < last_id)
        {
            gd.BuildHyperGraph();
        }
        else
        {
            std::cerr << "Could not parse the log file: " << input_files[0] << std::endl;
            return false;
        }
    }
    return true;
}


void
build_loop_closures(std::vector<hyper::GrabData> &gds, double external_loop_min_distance, double external_loop_required_distance)
{
	for (unsigned i = 0; i < gds.size(); ++i)
		for (unsigned j = i + 1; j < gds.size(); ++j)
			gds[i].BuildExternalLoopClosures(gds[j], external_loop_min_distance, external_loop_required_distance);
}

void
save_separated_graph(hyper::GrabData &gd, unsigned b)
{
    std::stringstream ss;
    std::string base;
    ss << b;
    ss >> base;
    base += "_";

    auto output_file { base + "sync.txt" };

    std::ofstream sync { output_file, std::ofstream::out };
    
    if (!sync.is_open())
    {
        throw std::invalid_argument("Could not open the separated sync.txt output file!");
    }

    gd.SaveHyperGraph(sync, false);

    gd.SaveEstimates(base);
}

void
save_hyper_graphs(std::vector<hyper::GrabData> &gds)
{
    std::ofstream sync("sync.txt");

    if (!sync.is_open())
    {
        std::cerr << "Could not open the sync.txt output file!" << std::endl;
        return;
    }

    unsigned b = 1;

    sync << "LOGS_QUANTITY " << gds.size() << std::endl;

    for (hyper::GrabData &gd : gds)
    {
        // save the hyper graph in a global file
        gd.SaveHyperGraph(sync, true);
        
        save_separated_graph(gd, b++);
    }

    for (hyper::GrabData &gd : gds)
    {
        gd.SaveExternalLidarLoopEdges(sync);
    }

    sync.close();
}


int
main (int argc, char **argv)
{
    if (2 > argc)
    {
        std::cout << "Usage: ./parser <log_list_file_path> [external_loop_min_distance] [external_loop_required_distance]" << std::endl;
        return -1;
    }

    prepare_all_directories();

    double external_loop_min_distance = 1.0f;
	double external_loop_required_distance = 5.0f;

    if (2 < argc)
    {
        std::string elmd(argv[2]);
        std::stringstream ss;
        ss << elmd;
        ss >> external_loop_min_distance;
    }

    if (3 < argc)
    {
        std::string eld(argv[3]);
        std::stringstream ss;
        ss << eld;
        ss >> external_loop_required_distance;
    }

    std::vector<hyper::GrabData> gds(0);
    
    auto logs { log_list_parser(argv[1]) };

    if (parse_logs(gds, logs))
    {
        build_loop_closures(gds, external_loop_min_distance, external_loop_required_distance);
        save_hyper_graphs(gds);
    }

    for (hyper::GrabData &gd : gds)
        gd.Clear();

    std::cout << "Done!\n";

    return 0;
}
