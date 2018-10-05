#include "rddf_graph_utils.h"

using namespace std;
bool g_ipc_required = false;
char *g_database_filename;


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
fill_database_matrix(FILE *f, vector < vector<string> > &database_filenames)
{
	int id, pattern_binary, pattern_decimal;
	//string str_filename;
	char filename[30];
	int cont = 0;


	while(fscanf(f, "%d %s %d\n", &id, filename, &pattern_binary) != EOF)
	{
		string str_filename (filename);
		pattern_decimal = convert_binary_to_decimal(pattern_binary);
		database_filenames[pattern_decimal].push_back(str_filename);
		//cout<<id<<" "<<filename<<" "<<pattern_binary<<" "<<pattern_decimal<<endl;
		//getchar();
	}

	for (unsigned int i = 0; i<database_filenames.size();i++)
	{
		if (database_filenames[i].size() != 0)
		{
			cont++;
			cout<<i<<" ";
			//cout<<database_filenames[i][0]<<endl;
			cout<<database_filenames[i].size()<<endl;
		}
		//cout<<i<<" "<<database_filenames[i].size()<<endl;getchar();
	}
	cout<<cont<<endl;
}


static void
read_parameters(int argc, char **argv)
{
	const char usage[] = "<database_dir>/<database_file>.txt";
	if (argc < 2){
		printf("Incorrect Input!.\nUsage:\n%s %s\n", argv[0], usage);
		exit(1);
	}
	else
	{
		g_database_filename = argv[1];
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

	FILE *f;
	vector < vector<string> > database_filenames(512);
	vector<string> filename_array;

	f = fopen (g_database_filename,"r");
	if(f == NULL)
	{
		printf("Graph file could not be read!\n");
		exit(1);
	}

	fill_database_matrix(f, database_filenames);

	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
	fclose (f);
}
