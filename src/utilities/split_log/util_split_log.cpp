#include <iostream>
#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <unistd.h>
#include <sys/stat.h>


using namespace std;


std::streampos
fileSize(string filePath)
{
    std::streampos fsize = 0;
    std::ifstream file(filePath.c_str(), std::ios::binary);

    fsize = file.tellg();
    file.seekg(0, std::ios::end);
    fsize = file.tellg() - fsize;
    file.close();

    return fsize;
}


int
main(int argc, char **argv)
{
	unsigned int i;

	int partition_number;
	int partition_max_size;

	long partition_size;

	string line, header;
	ifstream myfile;
	ofstream output_file;

	char *log_name;
	char *output_log_name;
	char output_complete_name[1024];
	char timestamp_as_string[64];

	double starting_timestamp;
	double message_timestamp;

	vector<string> strs;

	if (argc < 4)
	{
		cout << "Use " << argv[0] << " <log name> <partition size in MB> <name of the output log>" << endl;
		exit(-1);
	}

	log_name = argv[1];
	partition_max_size = atoi(argv[2]);
	output_log_name = argv[3];

	myfile.open(log_name, ios::in);

	if (myfile.is_open())
	{
		// read the header
		// OBS: note that, in the end of the loop, we already read a line that doesn't belong to the header.
		while (getline(myfile, line))
		{
			if (line.size() < 4)
				continue;

			if (line[0] == '#' || line.substr(0, 5).compare("PARAM") == 0)
				header += (line + "\n");
			else
				break;
		}

		partition_number = 0;
		partition_size = 0;

		sprintf(output_complete_name, "%s.part%d.txt", output_log_name, partition_number);
		output_file.open(output_complete_name);

		if (!output_file.is_open())
		{
			cout << "Error: Couldn't open the file '" << output_complete_name << "'" << endl;
			exit(-1);
		}

		output_file << header << "\n";

		boost::split(strs, line, boost::is_any_of(" "));
		starting_timestamp = atof(strs[strs.size() - 1].c_str());
		sprintf(timestamp_as_string, "%lf", 0.0);

		for (i = 0; i < strs.size() - 1; i++)
			output_file << strs[i] << " ";

		output_file << timestamp_as_string << "\n";
		partition_size += (line.size());

		while (getline(myfile, line))
		{
			if (line.size() < 4)
				continue;

			if ((partition_size / (1024 * 1024)) > partition_max_size)
			{
				output_file.close();

				partition_size = 0;
				partition_number++;

				sprintf(output_complete_name, "%s.part%d.txt", output_log_name, partition_number);
				output_file.open(output_complete_name);

				if (!output_file.is_open())
				{
					cout << "Error: Couldn't open the file '" << output_complete_name << "'" << endl;
					exit(-1);
				}

				output_file << header << "\n";

				boost::split(strs, line, boost::is_any_of(" "));
				starting_timestamp = atof(strs[strs.size() - 1].c_str());
			}

			boost::split(strs, line, boost::is_any_of(" "));
			message_timestamp = atof(strs[strs.size() - 1].c_str());

			if (strs[0].substr(0, 29).compare("VELODYNE_PARTIAL_SCAN_IN_FILE") == 0 ||
				strs[0].substr(0, 35).compare("BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE") == 0)
			{
				string basedir;
				vector<string> parts_of_the_name;

				boost::split(parts_of_the_name, strs[1], boost::is_any_of("/"));

				if (strs[0].substr(0, 29).compare("VELODYNE_PARTIAL_SCAN_IN_FILE") == 0)
					basedir = string(output_complete_name) + "_velodyne";
				else
					basedir = string(output_complete_name) + "_bumblebee";

				string subdir1 = parts_of_the_name[parts_of_the_name.size() - 3];
				string subdir2 = parts_of_the_name[parts_of_the_name.size() - 2];
				string filename = parts_of_the_name[parts_of_the_name.size() - 1];

				mkdir(basedir.c_str(), ACCESSPERMS); // if the directory exists, mkdir returns an error silently
				mkdir((basedir + "/" + subdir1).c_str(), ACCESSPERMS); // if the directory exists, mkdir returns an error silently
				mkdir((basedir + "/" + subdir1 + "/" + subdir2).c_str(), ACCESSPERMS); // if the directory exists, mkdir returns an error silently

				string new_filename = basedir + "/" + subdir1 + "/" + subdir2 + "/" + filename;

				// check if succeded
				system(("cp " + strs[1] + " " + new_filename).c_str());

				strs[1] = new_filename;
				partition_size += fileSize(new_filename);
			}

			for (i = 0; i < strs.size() - 1; i++)
				output_file << strs[i] << " ";

			sprintf(timestamp_as_string, "%lf", message_timestamp - starting_timestamp);
			output_file << timestamp_as_string << "\n";

			partition_size += (line.size());
		}

		myfile.close();
	}
	else
		cout << "Error: Unable to open the file '" << log_name << "'" << endl;

	return 0;
}


