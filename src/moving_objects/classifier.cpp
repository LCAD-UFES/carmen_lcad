/////////////////////////////////////////////////////////////
//ESTE CODIGO TEM A FUNCAO PARA PASSAR PARA O EDUARDO INSERIR NO MODULO DELE
//A FUNÇAO JA ESTA PRONTA, RECEBE COM PARAMETRO UMA NUVEM E RETORNA UM PAIR<INT,FLOAT>
/////////////////////////////////////////////////////////

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include <vector>
#include <utility> 
#include <string>
#include <string.h>
#include <dirent.h>

typedef std::pair<std::string, std::vector<float> > vfh_model;

/** \brief Loads an n-D histogram file as a VFH signature
  * \param path the input file name
  * \param vfh the resultant VFH model
  */
bool
loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
  int vfh_idx;
  // Load the file as a PCD
  try
  {
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type; unsigned int idx;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

    vfh_idx = pcl::getFieldIndex (cloud, "vfh");
    if (vfh_idx == -1)
      return (false);
    if ((int)cloud.width * cloud.height != 1)
      return (false);
  }
  catch (pcl::InvalidConversionException e)
  {
    return (false);
  }

  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  pcl::io::loadPCDFile (path.string (), point);
  vfh.second.resize (308);

  std::vector <pcl::PCLPointField> fields;
  getFieldIndex (point, "vfh", fields);
  	printf("vfh_idx = %d\n",fields[vfh_idx].count);
  for (size_t i = 0; i < fields[vfh_idx].count; ++i)
  {
    vfh.second[i] = point.points[0].histogram[i];
  }
  vfh.first = path.string ();
  return (true);
}


/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
inline void
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  float * f = new float[model.second.size ()];
  flann::Matrix<float> p = flann::Matrix<float>(f, 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  int *i = new int[k];
  float *j = new float[k];
  indices = flann::Matrix<int>(i, 1, k);
  distances = flann::Matrix<float>(j, 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool
loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}

char **split(char frase[], char separador)
{
    int i, j, k, contsep = 0;

     for(i=0,contsep=0;i<strlen(frase);i++)
       if(frase[i] == separador)
          contsep++;

    char  aux[contsep][200];
    char **result = (char**)malloc((contsep+1)*sizeof(char*));
   
    if(contsep)
    {
    	
        for(i=0; i<=contsep; i++ )
          *(result + i) = (char*)malloc(200*sizeof(char));
        
        for(i=0,k=0,j=0; i < strlen(frase); i++)
           if(frase[i] != separador)
           {
              aux[k][j] = frase[i];
              j++;
           }
           else
           {
              aux[k][j] = 0;
              k++;
              j=0;
           }
        aux[k][j] = 0;
        for(i=0;i<=contsep;i++)
        	strcpy(result[i], aux[i]);
        return result;
    }
    else
        printf("Nenhum Separador Encontrado");
}

bool pairCompare(const std::pair<std::string,double>& firstElem, const std::pair<std::string, double>& secondElem) {
  return firstElem.second > secondElem.second;

}

std::pair<int,float> return_classe_distancia(char* objeto, float distancia)
{
	std::pair<int,float> pair;
	if(strcmp(objeto,"4wd")==0)
	{
		pair = std::make_pair(1,distancia);
	}
	else if(strcmp(objeto,"bicycle")==0)
	{
		pair = std::make_pair(2,distancia);
	}
	else if(strcmp(objeto,"car")==0)
	{
		pair = std::make_pair(3,distancia);
	}
	else if(strcmp(objeto,"pedestrian")==0)
	{
		pair = std::make_pair(4,distancia);
	}
	else if(strcmp(objeto,"pole")==0)
	{
		pair = std::make_pair(5,distancia);
	}
	else if(strcmp(objeto,"scooter")==0)
	{
		pair = std::make_pair(6,distancia);
	}
	else if(strcmp(objeto,"traffic_lights")==0)
	{
		pair = std::make_pair(7,distancia);
	}
	else if(strcmp(objeto,"trailer")==0)
	{
		pair = std::make_pair(8,distancia);
	}
	else if(strcmp(objeto,"tree")==0)
	{
		pair = std::make_pair(9,distancia);
	}
	else if(strcmp(objeto,"trunk")==0)
	{
		pair = std::make_pair(10,distancia);
	}
	else if(strcmp(objeto,"ute")==0)
	{
		pair = std::make_pair(11,distancia);
	}
	else if(strcmp(objeto,"vegetation")==0)
	{
		pair = std::make_pair(12,distancia);
	}
	else if(strcmp(objeto,"bench")==0)
	{
		pair = std::make_pair(13,distancia);
	}
	else if(strcmp(objeto,"building")==0)
	{
		pair = std::make_pair(14,distancia);
	}
	else if(strcmp(objeto,"pillar")==0)
	{
		pair = std::make_pair(15,distancia);
	}
	else if(strcmp(objeto,"post")==0)
	{
		pair = std::make_pair(16,distancia);
	}
	else if(strcmp(objeto,"ticket_machine")==0)
	{
		pair = std::make_pair(17,distancia);
	}
	else if(strcmp(objeto,"traffic_sign")==0)
	{
		pair = std::make_pair(18,distancia);
	}
	else if(strcmp(objeto,"trash")==0)
	{
		pair = std::make_pair(19,distancia);
	}
	else if(strcmp(objeto,"truck")==0)
	{
		pair = std::make_pair(20,distancia);
	}
	else if(strcmp(objeto,"umbrella")==0)
	{
		pair = std::make_pair(21,distancia);
	}
	else if(strcmp(objeto,"van")==0)
	{
		pair = std::make_pair(22,distancia);
	}
	else
	{
		pair = std::make_pair(23,distancia);	
	}
	
	return pair;
}

std::pair<int,float> classifier_point_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	int k=1,j;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	
	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (1.00);
	
	// Compute the features
	ne.compute (*cloud_normals);
	
	// Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloud);
	vfh.setInputNormals (cloud_normals);
	
	// Output datasets
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

	// Compute the features
	vfh.compute (*vfhs);
	vfh_model histogram;
	histogram.second.resize (308);	
	
	for (size_t i = 0; i < 308; ++i)
	{
		histogram.second[i] = vfhs->points[0].histogram[i];
	}
	histogram.first = "objeto";
	
	std::string kdtree_idx_file_name = "kdtree.idx";
	std::string training_data_h5_file_name = "training_data.h5";
	std::string training_data_list_file_name = "training_data.list";

	std::vector<vfh_model> models;
	flann::Matrix<int> k_indices;
	flann::Matrix<float> k_distances;
	flann::Matrix<float> data;
	if (!boost::filesystem::exists ("training_data.h5") || !boost::filesystem::exists ("training_data.list"))
	{
				
		pcl::console::print_error ("Could not find training data models files %s and %s!\n", 
		training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
	}
	else
	{
		loadFileList (models, training_data_list_file_name);		
		flann::load_from_file (data, training_data_h5_file_name, "training_data");
	}
	if (!boost::filesystem::exists (kdtree_idx_file_name))
	{
		pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
	}
	else
	{
		flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("kdtree.idx"));
		index.buildIndex ();
		nearestKSearch (index, histogram, k, k_indices, k_distances);
	}



	char** object = split((char*)models.at (k_indices[0][0]).first.c_str (),'/');

	char **object1 = split(object[5],'.');
	std::pair<int,float> classifier = return_classe_distancia (object1[0],k_distances[0][0]); 
//	printf("Object = %s\n",object1[0]);

		
	for (j=0;j<5;j++)
	{
		delete[] object[j];
	}
	for (j=0;j<4;j++)
	{
		delete[] object1[j];
	}
	
	delete[] k_indices.ptr();
	delete[] k_distances.ptr();
	delete[] data.ptr();	
	
	return classifier;

}

