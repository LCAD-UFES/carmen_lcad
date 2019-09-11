#include <string>
#include <unistd.h>
#include <GrabData.hpp>
#include <StampedGPSPose.hpp>
#include <boost/filesystem/operations.hpp>
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

std::vector<std::string> log_list_parser(std::string log_list_filename)
{
	std::vector<std::string> log_list();
	
	std::ifstream file(log_list_filename);

    if (file.is_open())
	{
		std::string current_line;
		
		while(-1 != hyper::StringHelper::ReadLine(file, current_line))
		{
			log_list.push_back(current_line);
		}
	}
    else
	{
        std::cerr << "Unable to open the input file: " << log_list_filename << "\n";
    }

    return log_list;
}

int 
main (int argc, char **argv) 
{
    if (2 > argc) 
    {
        std::cout << "Usage: ./parser <log_list_filepath> [parser_config_filepath] [carmen_ini_filepath]" << std::endl;
        return -1;
    }

    prepare_all_directories();
	
    std::string carmen_home(getenv("CARMEN_HOME"));

	std::string config_filename = 2 < argc ? std::string(argv[3]) : carmen_home + "/src/hypergraphsclam/config/parser_config.txt";
    std::string carmen_ini = 3 < argc ? std::string(argv[4]) : carmen_home + "/src/carmen-ford-escape.ini";

	std::vector<hyper::GrabData> gds(0);
	std::vector<std::string> logs = log_list_parser(argv[1]);

	for (std::string &input_file : logs)
	{
		gds.emplace_back(hyper::GrabData());
		
		hyper::GrabData &gd = gds.back();
		
		gd.Configure(config_filename, carmen_ini);
		
		if (gd.ParseLogFile(input_file)
		{
			gd.BuildHyperGraph();
		}
		
	}
	
	
	
    // create a grab data object
    hyper::GrabData gd;

    // configure it
    gd.Configure(config_filename, carmen_ini);

    // try to process the log file
    if (gd.ParseLogFile(input_file)) 
    {
        // build the hyper graph
        gd.BuildHyperGraph();

        // save the hyper graph
        gd.SaveHyperGraph(output_file);

        // save extra info
        gd.SaveEstimates();
    } 
    else 
    {
        std::cout << "Error! Could not parse the log file!\n";
    }

    // close everything
    gd.Clear();
    std::cout << "Done!\n";

    return 0;
}
