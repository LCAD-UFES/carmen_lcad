#include <string>
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
    std::cout << "Necessary directories created." << std::endl;
}

std::vector<std::array<std::string, 3>> log_list_parser(std::string log_list_filename)
{
    std::vector<std::array<std::string, 3>> log_list;

    std::ifstream file(log_list_filename);

    if (file.is_open())
    {
        std::stringstream current_line;

        while(-1 != hyper::StringHelper::ReadLine(file, current_line))
        {
            std::string log;
            std::string parser_config_file;
            std::string carmen_ini;

            current_line >> log;
            current_line >> parser_config_file;
            current_line >> carmen_ini;

            if (log.empty() || parser_config_file.empty() || carmen_ini.empty())
            {
                std::cerr << "Invalid line! Take a look in your input file!" << std::endl;
            }
            else
            {
                log_list.emplace_back(std::array<std::string, 3> { log, parser_config_file, carmen_ini });
            }
        }
    }
    else
    {
        std::cerr << "Unable to open the log list file: " << log_list_filename << "\n";
    }

    return log_list;
}


bool
parse_logs(std::vector<hyper::GrabData> &gds, std::vector<std::array<std::string, 3>> &logs)
{
    unsigned last_id = 6;

    for (std::array<std::string, 3> &input_files : logs)
    {
        gds.emplace_back(hyper::GrabData());

        hyper::GrabData &gd = gds.back();

        gd.Configure(input_files[1], input_files[2]);

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
build_loop_closures(std::vector<hyper::GrabData> &gds)
{
	// TODO 1 -> 2 -> 3 ... -> n then 2 -> 3 -> ... n then 3 -> 4 -> ... -> n
    std::vector<hyper::GrabData>::iterator end(gds.end());
    std::vector<hyper::GrabData>::iterator curr(gds.begin());
    std::vector<hyper::GrabData>::iterator prev(curr++);

    while (end != curr)
    {
        curr->BuildExternalLoopClosures(*prev);
        prev = curr++;
    }
}

void
save_separated_graph(hyper::GrabData &gd, unsigned b)
{
    std::stringstream ss;
    std::string base;
    ss << b;
    ss >> base;
    base += "_";

    std::ofstream sync(base + "sync.txt");;
    if (!sync.is_open())
    {
        std::cerr << "Could not open the separated sync.txt output file!" << std::endl;
        return;
    }

    gd.SaveHyperGraph(sync);

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

    for (hyper::GrabData &gd : gds)
    {
        // save the hyper graph in a global file
        gd.SaveHyperGraph(sync);

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
        std::cout << "Usage: ./parser <log_list_file_path>" << std::endl;
        return -1;
    }

    prepare_all_directories();

    std::vector<hyper::GrabData> gds(0);
    std::vector<std::array<std::string, 3>> logs(log_list_parser(argv[1]));

    if (parse_logs(gds, logs))
    {
        build_loop_closures(gds);
        save_hyper_graphs(gds);
    }

    for (hyper::GrabData &gd : gds)
        gd.Clear();

    std::cout << "Done!\n";

    return 0;
}
