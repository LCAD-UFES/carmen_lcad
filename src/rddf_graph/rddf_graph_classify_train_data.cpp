#include "rddf_graph_utils.h"

using namespace std;
bool g_ipc_required = false;
char *g_database_filename;
bool first_classify;

typedef struct{
	string filename;
	string pattern;
} t_pattern;

typedef struct{
	int pattern;
	int file_index;
} l_class;


static void
define_messages()
{
}


static void
register_handlers()
{
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		if (g_ipc_required)
			carmen_ipc_disconnect();
		exit(printf("rddf_graph_operations_on_graph_main: disconnected.\n"));
	}
}


void
create_image_3_3(cv::Mat *image_3_3, string pattern)
{
	int middle_x = ceil(image_3_3->rows/2);
	int middle_y = ceil(image_3_3->cols/2);
	int cont = 0;

	for (int x = middle_x - 1; x <= middle_x + 1; x++)
	{
		for (int y = middle_y - 1; y <= middle_y + 1; y++)
		{

			if(pattern[cont] == '1')
			{
				cv::Vec3b pixel_color (0, 0, 0);
				paint_image(x, y, pixel_color, image_3_3);
			}
			cont++;
		}
	}
}


int
convert_binary_to_decimal (int num)
{
	int decimal_val = 0, base = 1, rem;

	 while (num > 0)
	 {
		 rem = num % 10;
		 decimal_val = decimal_val + rem * base;
		 num = num / 10 ;
		 base = base * 2;
	 }

	 return (decimal_val);
}


void
classify_train_data(FILE *f_last_classified, vector < vector<t_pattern> > database_filenames, l_class last_classified)
{
	cv::namedWindow("z->valid | x->invalid | esc->exit", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("image 3x3", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("image 3x3", 15*20, 0);
	cv::Mat image_15_15;
	cv::Mat image_15_15_scaled;
	cv::Mat image_3_3;
	cv::Mat image_3_3_scaled;
	cv::Size size(15 * 20, 15 * 20);
	int k;
	l_class actual;

	FILE *f_database_classified = fopen("train_data/database_classified.txt", "a+");

	for (unsigned int i = last_classified.file_index; i<database_filenames.size();i++)
	{
		for (unsigned int j = last_classified.pattern; i<database_filenames[i].size();j++)
		{
			if (database_filenames[i].size() != 0)
			{
				actual.file_index = i;
				actual.pattern = j;
				image_15_15 = cv::Mat(15, 15, CV_8UC3, cv::Scalar(255, 255, 255));
				image_3_3 = cv::Mat(15, 15, CV_8UC3, cv::Scalar(255, 255, 255));
				image_15_15 = cv::imread(database_filenames[i][j].filename);
				if (!image_15_15.data)
				{
					std::cout << "Image " << database_filenames[i][j].filename <<  " not loaded"<<endl;
					exit(1);
				}
				create_image_3_3(&image_3_3, database_filenames[i][j].pattern);
				//cv::resize(image_15_15, image_15_15_scaled, size, 0, 0, cv::INTER_NEAREST);
				cv::resize(image_15_15, image_15_15_scaled, size);
				cv::resize(image_3_3, image_3_3_scaled, size);
				cv::imshow("z->valid | x->invalid | esc->exit", image_15_15_scaled);
				cv::imshow("image 3x3", image_3_3_scaled);

				k = (char)cv::waitKey();
				if (k == 122) //z - valido!!!
				{
					fprintf(f_database_classified, "%s %s\n", database_filenames[i][j].filename.c_str(), database_filenames[i][j].pattern.c_str());
				}
				if (k == 120) //x - invalido!!!!!!
				{
					string new_filename;
					new_filename = database_filenames[i][j].filename.substr(0, database_filenames[i][j].filename.size()-6) + "0.jpg";
					fprintf(f_database_classified, "%s %s\n", new_filename.c_str(), database_filenames[i][j].pattern.c_str());
					string command = "mv " + database_filenames[i][j].filename + " " + new_filename;
					system(command.c_str());
				}
				if (k == 27) //sair
				{
					fprintf(f_last_classified, "%d %d\n", actual.file_index, actual.pattern);
					fclose (f_database_classified);
					fclose (f_last_classified);
					exit(1);
				}
			}
		}

	}

}


void
fill_database_matrix(FILE *f_database_filenames, vector < vector<t_pattern> > &database_filenames)
{
	int id, pattern_binary, pattern_decimal;
	//string str_filename;
	char filename[40];
	char pattern[9];
	int cont = 0;
	t_pattern p;


	while(fscanf(f_database_filenames, "%d %s %s\n", &id, filename, pattern) != EOF)
	{
		p.filename = filename;
		p.pattern = pattern;
		pattern_binary = atoi(p.pattern.c_str());
		pattern_decimal = convert_binary_to_decimal(pattern_binary);
		database_filenames[pattern_decimal].push_back(p);
		//cout<<id<<" "<<filename<<" "<<pattern_binary<<" "<<pattern_decimal<<endl;
		//getchar();
	}

	for (unsigned int i = 0; i<database_filenames.size();i++)
	{
		if (database_filenames[i].size() != 0)
		{
			cont++;
			//cout<<i<<" ";
			//cout<<database_filenames[i][0]<<endl;
			//cout<<database_filenames[i].size()<<endl;
		}
		//cout<<i<<" "<<database_filenames[i].size()<<endl;getchar();
	}
	cout<<"Database has "<<cont<<" different patterns"<<endl;
}


static void
read_parameters(int argc, char **argv)
{
	const char usage[] = "<database_dir>/<database_file>.txt -f(for first classification) or -c(continue classifying)";
	if (argc < 3){
		printf("Incorrect Input!.\nUsage:\n%s %s\n", argv[0], usage);
		exit(1);
	}
	else
	{
		g_database_filename = argv[1];
		if(strcmp(argv[2], "-f") == 0)
			first_classify = true;
		else
			first_classify = false;
	}
}


int
main(int argc, char **argv)
{
	read_parameters(argc,argv);

	if (g_ipc_required)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		define_messages();
	}
	//signal(SIGINT, shutdown_module);

	FILE *f_database_filenames;
	FILE *f_last_classified;
	vector < vector<t_pattern> > database_filenames(512);
	l_class last_classified;

	f_database_filenames = fopen (g_database_filename,"r");
	if(f_database_filenames == NULL)
	{
		printf("Graph file could not be read!\n");
		exit(1);
	}
	fill_database_matrix(f_database_filenames, database_filenames);
	fclose (f_database_filenames);

	if(first_classify == true)
	{
		f_last_classified = fopen("train_data/last_classified.txt","w");
		last_classified.file_index = 0;
		last_classified.pattern = 0;
	}
	else
	{
		f_last_classified = fopen ("train_data/last_classified.txt","r+");
		fscanf(f_last_classified, "%d %d\n", &last_classified.file_index, &last_classified.pattern);
	}


	classify_train_data (f_last_classified, database_filenames, last_classified);

	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}

}
