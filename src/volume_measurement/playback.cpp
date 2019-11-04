/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <fcntl.h>
#include <carmen/carmen.h>
#include <carmen/playback_interface.h>

//includes da kdtree
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
//includes do icp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
//includes do gicp
#include <pcl/registration/gicp.h>
#include <eigen3/Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
//include centroid pcl
#include <pcl/common/centroid.h>
//outros includes
#include <iostream>
#include <fstream>
#include <string>

#include <pcl/io/obj_io.h>
#include <pcl/common/common.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>

#include <time.h>

#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/PCLPointCloud2.h>

#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

//#include <vtk-5.8/vtkVersion.h>
#include <vtk-5.8/vtkPolyData.h>
#include <vtk-5.8/vtkDataObject.h>
#include <vtk-5.8/vtkCleanPolyData.h>

#include <vtk-5.8/vtkPolyDataConnectivityFilter.h>
#include <vtk-5.8/vtkMassProperties.h>

#include <vtk-5.8/vtkSmartPointer.h>
#include <vtk-5.8/vtkSphereSource.h>
#include <vtk-5.8/vtkConnectivityFilter.h>
#include <vtk-5.8/vtkPolyDataConnectivityFilter.h>
#include <vtk-5.8/vtkPolyDataMapper.h>
#include <vtk-5.8/vtkActor.h>
#include <vtk-5.8/vtkProperty.h>
#include <vtk-5.8/vtkRenderer.h>
#include <vtk-5.8/vtkRenderWindow.h>
#include <vtk-5.8/vtkRenderWindowInteractor.h>
#include <vtk-5.8/vtkAppendPolyData.h>

#include <vtk-5.8/vtkPointData.h>
#include <vtk-5.8/vtkConeSource.h>

#include <vtk-5.8/vtkTriangleFilter.h>
#include <vtk-5.8/vtkFillHolesFilter.h>
#include <vtk-5.8/vtkPolyDataNormals.h>
#include <vtk-5.8/vtkFeatureEdges.h>

using namespace Eigen;
using namespace pcl;

#define        MAX_LINE_LENGTH           (5*4000000)
#define		   MAX_SIGNED_INT			 (0x7FFFFFFF)

carmen_FILE *logfile = NULL;
carmen_logfile_index_p logfile_index = NULL;

carmen_FILE *point_cloud_xyz_file = NULL;

carmen_base_ackerman_odometry_message odometry_ackerman;
carmen_robot_ackerman_velocity_message velocity_ackerman;

carmen_visual_odometry_pose6d_message visual_odometry;
carmen_simulator_ackerman_truepos_message truepos_ackerman;
carmen_robot_ackerman_laser_message laser_ackerman1, laser_ackerman2, laser_ackerman3, laser_ackerman4, laser_ackerman5;
carmen_laser_laser_message rawlaser1, rawlaser2, rawlaser3, rawlaser4, rawlaser5;
carmen_laser_ldmrs_message laser_ldmrs;
carmen_laser_ldmrs_new_message laser_ldmrs_new;
carmen_laser_ldmrs_objects_message laser_ldmrs_objects;
carmen_laser_ldmrs_objects_data_message laser_ldmrs_objects_data;

carmen_imu_message imu;
carmen_gps_gpgga_message gpsgga;
carmen_gps_gphdt_message gpshdt;
carmen_gps_gprmc_message gpsrmc;

carmen_kinect_depth_message raw_depth_kinect_0, raw_depth_kinect_1;
carmen_kinect_video_message raw_video_kinect_0, raw_video_kinect_1;

carmen_velodyne_variable_scan_message velodyne_variable_scan, velodyne_variable_scan1, velodyne_variable_scan2, velodyne_variable_scan3;
carmen_velodyne_partial_scan_message velodyne_partial_scan;
carmen_velodyne_gps_message velodyne_gps;

carmen_xsens_global_euler_message xsens_euler;
carmen_xsens_global_quat_message xsens_quat;
carmen_xsens_global_matrix_message xsens_matrix;
carmen_xsens_mtig_message xsens_mtig;
carmen_bumblebee_basic_stereoimage_message bumblebee_basic_stereoimage1, bumblebee_basic_stereoimage2, bumblebee_basic_stereoimage3, bumblebee_basic_stereoimage4, bumblebee_basic_stereoimage5, bumblebee_basic_stereoimage6, bumblebee_basic_stereoimage7, bumblebee_basic_stereoimage8, bumblebee_basic_stereoimage9;

carmen_web_cam_message web_cam_message;

carmen_base_ackerman_motion_command_message ackerman_motion_message;
carmen_ultrasonic_sonar_sensor_message ultrasonic_message;

carmen_ford_escape_status_message ford_escape_status;

carmen_localize_ackerman_globalpos_message globalpos;

double playback_starttime = 0.0;
double last_logfile_time = 0.0;
double playback_speed = 1.0;

int offset = 0;
int autostart = 0;
int paused = 1;
int fast = 0;
int advance_frame = 0;
int rewind_frame = 0;
int basic_messages = 0;

int g_publish_odometry = 1;

double timestamp_last_message_published = 0;

double playback_timestamp;
double playback_timestamp_v1 = 0.0;
double playback_timestamp_v1_ant = 0.0;
double playback_timestamp_v2 = 0.0;
double taxaDePosicao;

int current_position = 0;

int final_position = MAX_SIGNED_INT;
int stop_position = MAX_SIGNED_INT;

int save_velodyne_xyz = 0;

//SELECAO DA TAREFA A SER REALIZADA

int task = 3; // 0-> executa o log sem efetuar nenhuma operacao
			  // 1-> executa calibracao
			  // 2-> executa processo de salvar dados das retas - inicia quando uma linha de uma nuvem já foi preenchida, e depois passa a nao ser mais preenchida por qualquer valor
			  // 3-> executa processo de medicao de volume - inicia quando o caminhao encosta a frente na ultima linha
			  // 4-> cria malha a partir da nuvem de pontos do caminhao vazio gerado nos passos anteriores

bool geraNuvemGICP = true; //cria nuvem de pontos para o icp a partir do log de caminhao vazio da task 3

//BLOCOS DE REGIAO DE INTERESSE E CALIBRACAO

PointCloud<PointXYZRGB>::Ptr MatrixCube1(new PointCloud<PointXYZRGB>);							//nuvens de pontos dos cubos de regiao de interesse
PointCloud<PointXYZRGB>::Ptr MatrixCube2(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr MatrixCube2_transformed(new PointCloud<PointXYZRGB>);

float cx1 = 0, cy1 = 0, cz1 = 0, rx1 = 0, ry1 = 0, rz1 = 0, tx1 = 0, ty1 = 0, tz1 = 0;			//dados da transformacao do posicionamento dos cubos (comprimento, rotacao e translacao) para o posicionamento manual
float cx2 = 0, cy2 = 0, cz2 = 0, rx2 = 0, ry2 = 0, rz2 = 0, tx2 = 0, ty2 = 0, tz2 = 0;			//esses valores nao sao utilizados no posicionamento inicial executado pela selecao de pontos

Matrix<double, 4, 4> transformationCubeV1;														//matrizes de transformacao de posicionamento dos cubos
Matrix<double, 4, 4> transformationCubeV2;

float c_rx = 0, c_ry = 0, c_rz = 0;																//calibracao
float c_tx = 0, c_ty = 0, c_tz = 0;

Matrix<double, 4, 4> manual_transformation;
Matrix<double, 4, 4> gicp_transformation;
Matrix<double, 4, 4> final_transformation;														//matriz de transformacao para alinhar a nuvem do velodyne 2 com o velodyne 1

PointCloud<PointXYZRGB>::Ptr cloud_pointPickingV1 (new PointCloud<PointXYZRGB>);				//nuvens de pontos utilizados para calibracao manual utilizando a selecao de 3 pontos de cada nuvem
PointCloud<PointXYZRGB>::Ptr cloud_pointPickingV2 (new PointCloud<PointXYZRGB>);

//KD-TREE PARA ELIMINACAO DO AMBIENTE

PointXYZRGB searchPoint;
bool flagSaveEnvironment = false;
int numberOfMessagesV1 = 2, numberOfMessagesV2 = 2;

PointCloud<PointXYZRGB>::Ptr cloud_environmentRemovalV1 (new PointCloud<PointXYZRGB>);			//nuvens de pontos para serem utilizadas na kdtree, para limpar as nuvens seguintes
PointCloud<PointXYZRGB>::Ptr cloud_environmentRemovalV2 (new PointCloud<PointXYZRGB>);

KdTreeFLANN<PointXYZRGB> kdtree_environmentV1;
KdTreeFLANN<PointXYZRGB> kdtree_environmentV2;
std::vector<int> pointIdxRadiusSearch;
std::vector<float> pointRadiusSquaredDistance;

float radius = 0.20; 																			//raio de busca dos pontos próximos da kdtree (em metros)
bool showPoints = true; 																		//exibe ou não os pontos removidos pela kdtree

//NUVENS DE PONTOS IMPORTANTES

PointCloud<PointXYZRGB>::Ptr cloud_v1_world (new PointCloud<PointXYZRGB>); 						//nuvem de pontos do mundo do v1
PointCloud<PointXYZRGB>::Ptr cloud_v2_world (new PointCloud<PointXYZRGB>); 						//nuvem de pontos do mundo do v2
PointCloud<PointXYZRGB>::Ptr cloud_v2_world_transformed (new PointCloud<PointXYZRGB>); 			//nuvem com pontos do mundo do v2 ajustados para o sistema de coordenadas do v1
PointCloud<PointXYZRGB>::Ptr cloud_v1_truck (new PointCloud<PointXYZRGB>); 						//nuvem de pontos do mundo do v1
PointCloud<PointXYZRGB>::Ptr cloud_v2_truck (new PointCloud<PointXYZRGB>); 						//nuvem de pontos do mundo do v2
PointCloud<PointXYZRGB>::Ptr cloud_v2_truck_transformed (new PointCloud<PointXYZRGB>); 			//nuvem com pontos do mundo do v2 ajustados para o sistema de coordenadas do v1
PointCloud<PointXYZRGB>::Ptr cloud_v2_truck_corrected (new PointCloud<PointXYZRGB>); 			//nuvem com pontos do mundo do v2 ajustados para o sistema de coordenadas do v1 e corrigidos devido a movimentacao do caminhao e rotacao do velodyne
PointCloud<PointXYZRGB>::Ptr output (new PointCloud<PointXYZRGB>); 								//nao é exibida, é apenas o retorno do gicp (que nao esta aplicando a transformacao calculada)cloud_mountage_v1v2_downsampled

float middleAngle = 180.0;																		//angulo "central"
float rangeAngle = 45.0;																		//angulo de abertura para esquerda e direita a partir do angulo central, para busca dos pontos das extremidades
PointCloud<PointXYZRGB>::Ptr cloud_v2_border_left(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_v2_border_right(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_v2_border_left_accumulated(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_v2_border_right_accumulated(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_v2_border_left_accumulated_final(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_v2_border_right_accumulated_final(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_v1_truck_single_line (new PointCloud<PointXYZRGB>);

PointCloud<PointXYZRGB>::Ptr cloud_v2_border_left_transformed(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_v2_border_right_transformed(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_v2_border_left_accumulated2(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_v2_border_right_accumulated2(new PointCloud<PointXYZRGB>);

PointCloud<PointXYZRGB>::Ptr cloud_lateral_borders(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_lateral_points_to_gicp(new PointCloud<PointXYZRGB>);


//VARIÁVEIS DE CONTROLE DO CODIGO

int vetLasersV2[16];																			//vetor com quantidade de pontos em cada feixe de laser de V2
int sizeToEndProcess = 100;																		//tamanho que a nuvem de pontos de V2 deve ter para finalizar o processo de task 2 ou 3
int sizeShotProcess2 = 50;
int sizeToStartProcess3 = 1000;

bool process_start = false;
bool process_end = false;

//VARIÁVEIS DAS RETAS LATERAIS E CENTRAL

float point_x1, point_y1, point_z1, point_x2, point_y2, point_z2, point_x3, point_y3, point_z3;	//variaveis das retas esquerda, direita e central
float vector_x1, vector_y1, vector_z1, vector_x2, vector_y2, vector_z2, vector_x3, vector_y3, vector_z3;

PointCloud<PointXYZRGB>::Ptr cloud_line_points(new PointCloud<PointXYZRGB>);					//nuvem para visualizacao dos pontos das retas laterais e central estimadas

//PROJECAO DE PONTOS NA RETA CENTRAL

int projectionMethod = 2;
PointCloud<PointXYZRGB>::Ptr cloud_line_v1_projection (new PointCloud<PointXYZRGB>); 			//nuvem de pontos de projecoes do v1 sobre a reta central
PointCloud<PointXYZRGB>::Ptr cloud_mostDensePoints_v1_projection (new PointCloud<PointXYZRGB>); //nuvem de pontos dos pontos mais densos das projecoes de V1
PointCloud<PointXYZRGB>::Ptr cloud_onlyMostDensePoint_v1 (new PointCloud<PointXYZRGB>); 		//ponto mais denso da projecao de v1
PointCloud<PointXYZRGB>::Ptr cloud_mostDensePoints_v1_projection_correction_v2 (new PointCloud<PointXYZRGB>); //nuvem de pontos densos de v1 corrigidos para v2 de acordo com o timestamp das mensagens

//NUVENS DA MONTAGEM DO CAMINHAO

PointCloud<PointXYZRGB>::Ptr cloud_mountage_v1 (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_mountage_v2 (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_mountage_v1v2 (new PointCloud<PointXYZRGB>);

PointCloud<PointXYZRGB>::Ptr cloud_mountage_v1_transformed (new PointCloud<PointXYZRGB>);	//nuvens da montagem transformadas para posicao do modelo 3D
PointCloud<PointXYZRGB>::Ptr cloud_mountage_v2_transformed (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_mountage_v1v2_transformed (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_mountage_v2_internal (new PointCloud<PointXYZRGB>);

//ALINHAMENTO DAS NUVENS E MODELOS

double V1[3], V2[3], V3[3], O1[3]; //SISTEMA DE COORDENADAS DA CACAMBA
double V4[3], V5[3], V6[3], O2[3]; //SISTEMA DE COORDENADAS DA MONTAGEM

PointCloud<PointXYZRGB>::Ptr cloud_emptyBucketLaser (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_emptyBucketLaserGICP (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_emptyBucketModel (new PointCloud<PointXYZRGB>);

PointCloud<PointXYZRGB>::Ptr cloud_bucketCorners (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_bucketAxis (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_mountageAxis (new PointCloud<PointXYZRGB>);

Matrix<double, 4, 4> gicp_transformation_bucketLaser_to_bucketModel;
Matrix<double, 4, 4> transformation_bucketLaser_to_bucketMountage;
Matrix<double, 4, 4> gicp_transformation_bucketMountage_to_bucketLaserPreviousGICP;

PointCloud<PointXYZRGB>::Ptr cloud_modelPoints (new PointCloud<PointXYZRGB>);			//nuvem de pontos do modelo 3d para remocao de pontos proximos
PointCloud<PointXYZRGB>::Ptr cloud_floating_points (new PointCloud<PointXYZRGB>); 		//nuvem de pontos flutuantes para montagem da base da cacamba

//MESH DA CACAMBA

pcl::PolygonMesh meshBucket;
pcl::PointCloud<pcl::PointNormal>::Ptr meshPoligonsNormalsCacamba(new pcl::PointCloud<pcl::PointNormal>);
pcl::PolygonMesh meshBucketSimplified;
pcl::PointCloud<pcl::PointNormal>::Ptr meshPoligonsNormalsCacambaSimplified(new pcl::PointCloud<pcl::PointNormal>);
//malha de volume reconstruido
pcl::PolygonMesh meshReconstruction;
pcl::PolygonMesh::Ptr meshReconstructionPointer(&meshReconstruction);
pcl::PolygonMesh meshReconstructionFinal;
//pcl::PolygonMesh::Ptr meshReconstructionPointerFinal(&meshReconstructionFinal);

pcl::PolygonMesh meshTeste;
pcl::PolygonMesh meshTesteFinal;

//JANELAS DE VISUALIZACAO

visualization::PCLVisualizer viewer2, viewer1;

//OUTROS

int contPoints = 0;
PointXYZRGB ponto;
PointCloud<PointXYZRGB>::Ptr pointToTransform(new PointCloud<PointXYZRGB>); 				//nuvem com ponto unico para transformar
PointCloud<PointXYZRGB>::Ptr cloud_testPoisson (new PointCloud<PointXYZRGB>); 				//nuvem de pontos do carro na posicao inicial
int velodyne_number;

clock_t t;
float tempoTotal = 0.0;
float tempoMontagem = 0.0;
FILE *fileResults;
//FILE *fileScenario;

//PLANO DO CHAO ESTIMADO
pcl::ModelCoefficients::Ptr planeCoefficients (new pcl::ModelCoefficients);

void configMatrixCube_size();
void configMatrixCube_position();
void configureManualCalibration(bool saveFile);
void calibration_gicp();

void drawCloudsCalibration(bool waitSpin);
bool waitSpin = false;

void configureCubePosition(bool saveFile);

void print_playback_status(void)
{
	char str[100];

	if(paused)
		sprintf(str, "PAUSED ");
	else
		sprintf(str, "PLAYING");
	fprintf(stderr, "\rSTATUS:    %s   TIME:    %f    Current message: %d                ",
			str, playback_timestamp, current_position);
}

void playback_command_handler(carmen_playback_command_message *command)
{
	switch(command->cmd) {
	case CARMEN_PLAYBACK_COMMAND_PLAY:
		offset = 0;
		if(paused) {
			playback_starttime = 0.0;
			paused = 0;
			//      fprintf(stderr, " PLAY ");
			print_playback_status();
		}
		break;
	case CARMEN_PLAYBACK_COMMAND_STOP:
		offset = 0;
		if(!paused) {
			paused = 1;
			//      fprintf(stderr, " STOP ");
			print_playback_status();
		}
		break;
	case CARMEN_PLAYBACK_COMMAND_RESET:
		offset = 0;
		if(!paused)
			paused = 1;
		current_position = 0;
		playback_starttime = 0.0;
		//    fprintf(stderr, "\nRESET ");
		playback_timestamp = 0;
		print_playback_status();
		break;
	case CARMEN_PLAYBACK_COMMAND_FORWARD:
		offset = command->arg;
		advance_frame = 1;
		break;
	case CARMEN_PLAYBACK_COMMAND_REWIND:
		offset = -1 * command->arg;
		rewind_frame = 1;
		break;
	case CARMEN_PLAYBACK_COMMAND_FWD_SINGLE:
		offset = 0;
		advance_frame = 1;
		break;
	case CARMEN_PLAYBACK_COMMAND_RWD_SINGLE:
		offset = 0;
		rewind_frame = 1;
		break;
	case CARMEN_PLAYBACK_COMMAND_SET_SPEED:
		break;
	case CARMEN_PLAYBACK_COMMAND_SET_INITIAL_TIME:
		offset = 0;
		if(!paused)
			paused = 1;
		current_position = command->arg;
		//printf("\nspeed = %f, playback_speed = %f\n", command->speed, playback_speed);
		print_playback_status();
		break;
	case CARMEN_PLAYBACK_COMMAND_SET_FINAL_TIME:
		if(!paused)
			paused = 1;
		if(command->arg <= 0)
			final_position = MAX_SIGNED_INT;
		else
			final_position = command->arg;
		//printf("\nspeed = %f, playback_speed = %f\n", command->speed, playback_speed);
		//print_playback_status();
		break;
		//---------------------------------------------------------------------------------------------
	case CARMEN_PLAYBACK_COMMAND_SET_NUMBER_OF_MESSAGES_V1:
		if(!paused)
			paused = 1;
		numberOfMessagesV1 = command->arg;
		break;
	case CARMEN_PLAYBACK_COMMAND_SET_NUMBER_OF_MESSAGES_V2:
		if(!paused)
			paused = 1;
		numberOfMessagesV2 = command->arg;
		break;
	case CARMEN_PLAYBACK_CX1:
		cx1 = command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_CY1:
		cy1 = command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_CZ1:
		cz1 = command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_RX1:
		rx1 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_RY1:
		ry1 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_RZ1:
		rz1 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_TX1:
		tx1 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_TY1:
		ty1 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_TZ1:
		tz1 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_CX2:
		cx2 = command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_CY2:
		cy2 = command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_CZ2:
		cz2 = command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_RX2:
		rx2 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_RY2:
		ry2 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_RZ2:
		rz2 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_TX2:
		tx2 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_TY2:
		ty2 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_TZ2:
		tz2 += command->arg;
		configureCubePosition(true);
		break;
	case CARMEN_PLAYBACK_COMMAND_SHOW_POINTS:
		showPoints = !showPoints;
		break;
	case CARMEN_PLAYBACK_COMMAND_CALIBRATION:
		if(!paused)
			paused = 1;
		calibration_gicp();
		break;
	case CARMEN_PLAYBACK_C_RX:
		c_rx += command->arg;
		configureManualCalibration(true);
		break;
	case CARMEN_PLAYBACK_C_RY:
		c_ry += command->arg;
		configureManualCalibration(true);
		break;
	case CARMEN_PLAYBACK_C_RZ:
		c_rz += command->arg;
		configureManualCalibration(true);
		break;
	case CARMEN_PLAYBACK_C_TX:
		c_tx += command->arg;
		configureManualCalibration(true);
		break;
	case CARMEN_PLAYBACK_C_TY:
		c_ty += command->arg;
		configureManualCalibration(true);
		break;
	case CARMEN_PLAYBACK_C_TZ:
		c_tz += command->arg;
		configureManualCalibration(true);
		break;
	}
	if(fabs(command->speed - playback_speed) > 0.001) {
		playback_starttime = 0.0;
		playback_speed = command->speed;
		print_playback_status();
	}
}

void define_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_BASE_ACKERMAN_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_ROBOT_ACKERMAN_VELOCITY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME);

	err = IPC_defineMsg(CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_ULTRASONIC_SONAR_SENSOR_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_ULTRASONIC_SONAR_SENSOR_NAME);

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_ROBOT_ACKERMAN_FRONTLASER_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_REARLASER_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_ROBOT_ACKERMAN_REARLASER_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_REARLASER_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LASER3_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LASER_LASER3_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER3_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LASER4_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LASER_LASER4_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER4_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LASER5_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LASER_LASER5_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER5_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	err = IPC_defineMsg(CARMEN_GPS_GPGGA_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_GPS_GPGGA_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_GPS_GPGGA_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_GPS_GPRMC_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_GPS_GPRMC_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_GPS_GPRMC_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_KINECT_DEPTH_MSG_0_NAME, IPC_VARIABLE_LENGTH, CARMEN_KINECT_DEPTH_MSG_0_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_KINECT_DEPTH_MSG_0_NAME);

	err = IPC_defineMsg(CARMEN_KINECT_VIDEO_MSG_0_NAME, IPC_VARIABLE_LENGTH, CARMEN_KINECT_VIDEO_MSG_0_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_KINECT_VIDEO_MSG_0_NAME);

	err = IPC_defineMsg(CARMEN_KINECT_DEPTH_MSG_1_NAME, IPC_VARIABLE_LENGTH, CARMEN_KINECT_DEPTH_MSG_1_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_KINECT_DEPTH_MSG_1_NAME);

	err = IPC_defineMsg(CARMEN_KINECT_VIDEO_MSG_1_NAME, IPC_VARIABLE_LENGTH, CARMEN_KINECT_VIDEO_MSG_1_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_KINECT_VIDEO_MSG_1_NAME);

	err = IPC_defineMsg(CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME);

	err = IPC_defineMsg("carmen_stereo_velodyne_scan_message8", IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", "carmen_stereo_velodyne_scan_message8");

	err = IPC_defineMsg(CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE1_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE1_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE1_NAME);

	err = IPC_defineMsg(CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE2_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE2_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE2_NAME);

	err = IPC_defineMsg(CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE3_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE3_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE3_NAME);

	err = IPC_defineMsg(CARMEN_VELODYNE_GPS_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_GPS_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_GPS_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_MATRIX_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_XSENS_GLOBAL_MATRIX_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_XSENS_GLOBAL_MATRIX_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_EULER_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_XSENS_GLOBAL_EULER_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_XSENS_GLOBAL_EULER_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_XSENS_GLOBAL_QUAT_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_XSENS_GLOBAL_QUAT_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_MTIG_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_MTIG_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_MTIG_NAME);

	int camera;
	for (camera = 1; camera <= 9; camera++)
		carmen_bumblebee_basic_define_messages(camera);

	err = IPC_defineMsg(CARMEN_WEB_CAM_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_WEB_CAM_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_WEB_CAM_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);

	err = IPC_defineMsg(CARMEN_FORD_ESCAPE_STATUS_NAME, IPC_VARIABLE_LENGTH, CARMEN_FORD_ESCAPE_STATUS_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_FORD_ESCAPE_STATUS_NAME);

	carmen_subscribe_message(CARMEN_PLAYBACK_COMMAND_NAME,
			CARMEN_PLAYBACK_COMMAND_FMT,
			NULL, sizeof(carmen_playback_command_message),
			(carmen_handler_t)playback_command_handler,
			CARMEN_SUBSCRIBE_LATEST);
}

// ts is in logfile time
void wait_for_timestamp(double playback_timestamp)
{
	double current_time; // in logfile time
	struct timeval tv;

	//printf("playback_timestamp = %lf, playback_starttime = %lf, carmen_get_time = %lf\n", playback_timestamp, playback_starttime, carmen_get_time());
	// playback_starttime is offset between file-start and playback-start
	if(playback_starttime == 0.0)
		playback_starttime = (carmen_get_time() - playback_timestamp / playback_speed);
	current_time = (carmen_get_time() - playback_starttime) * playback_speed;
	if(!fast && !paused && playback_timestamp > current_time) {
		double towait = (playback_timestamp - current_time) / playback_speed;
		tv.tv_sec = (int)floor(towait);
		tv.tv_usec = (towait - tv.tv_sec) * 1e6;
		select(0, NULL, NULL, NULL, &tv);
	}
}

typedef char *(*converter_func)(char *, void *);

typedef struct {
	char *logger_message_name;
	char *ipc_message_name;
	converter_func conv_func;
	void *message_data;
	int interpreted;
} logger_callback_t;

static logger_callback_t logger_callbacks[] =
	{
		{"LASER_LDMRS", CARMEN_LASER_LDMRS_NAME, (converter_func) carmen_string_to_laser_ldmrs_message, &laser_ldmrs, 0},
		{"LASER_LDMRS_NEW", CARMEN_LASER_LDMRS_NEW_NAME, (converter_func) carmen_string_to_laser_ldmrs_new_message, &laser_ldmrs_new, 0},
		{"LASER_LDMRS_OBJECTS", CARMEN_LASER_LDMRS_OBJECTS_NAME, (converter_func) carmen_string_to_laser_ldmrs_objects_message, &laser_ldmrs_objects, 0},
		{"LASER_LDMRS_OBJECTS_DATA", CARMEN_LASER_LDMRS_OBJECTS_DATA_NAME, (converter_func) carmen_string_to_laser_ldmrs_objects_data_message, &laser_ldmrs_objects_data, 0},
		{"RAWLASER1", CARMEN_LASER_FRONTLASER_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser1, 0},
		{"RAWLASER2", CARMEN_LASER_REARLASER_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser2, 0},
		{"RAWLASER3", CARMEN_LASER_LASER3_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser3, 0},
		{"RAWLASER4", CARMEN_LASER_LASER4_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser4, 0},
		{"RAWLASER5", CARMEN_LASER_LASER5_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser5, 0},
		{"ROBOTLASER_ACK1", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman1, 0},
		{"ROBOTLASER_ACK2", CARMEN_ROBOT_ACKERMAN_REARLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman2, 0},
		{"ROBOTLASER_ACK3", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman3, 0},
		{"ROBOTLASER_ACK4", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman4, 0},
		{"ROBOTLASER_ACK5", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman5, 0},
		
		{"ODOM_ACK", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, (converter_func) carmen_string_to_base_ackerman_odometry_message, &odometry_ackerman, 0},
		{"ROBOTVELOCITY_ACK", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, (converter_func) carmen_string_to_robot_ackerman_velocity_message, &velocity_ackerman, 0},
		
		{"VISUAL_ODOMETRY", CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME, (converter_func) carmen_string_to_visual_odometry_message, &visual_odometry, 0},
		{"TRUEPOS_ACK", CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, (converter_func) carmen_string_to_simulator_ackerman_truepos_message, &truepos_ackerman, 0},
		{"IMU", CARMEN_IMU_MESSAGE_NAME, (converter_func) carmen_string_to_imu_message, &imu, 0},
		{"NMEAGGA", CARMEN_GPS_GPGGA_MESSAGE_NAME, (converter_func) carmen_string_to_gps_gpgga_message, &gpsgga, 0},
		{"NMEAHDT", CARMEN_GPS_GPHDT_MESSAGE_NAME, (converter_func) carmen_string_to_gps_gphdt_message, &gpshdt, 0},
		{"NMEARMC", CARMEN_GPS_GPRMC_MESSAGE_NAME, (converter_func) carmen_string_to_gps_gprmc_message, &gpsrmc, 0},
		{"RAW_KINECT_DEPTH0", CARMEN_KINECT_DEPTH_MSG_0_NAME, (converter_func) carmen_string_to_kinect_depth_message, &raw_depth_kinect_0, 0},
		{"RAW_KINECT_DEPTH1", CARMEN_KINECT_DEPTH_MSG_1_NAME, (converter_func) carmen_string_to_kinect_depth_message, &raw_depth_kinect_1, 0},
		{"RAW_KINECT_VIDEO0", CARMEN_KINECT_VIDEO_MSG_0_NAME, (converter_func) carmen_string_to_kinect_video_message, &raw_video_kinect_0, 0},
		{"RAW_KINECT_VIDEO1", CARMEN_KINECT_VIDEO_MSG_1_NAME, (converter_func) carmen_string_to_kinect_video_message, &raw_video_kinect_1, 0},
		{"VELODYNE_PARTIAL_SCAN", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, (converter_func) carmen_string_to_velodyne_partial_scan_message, &velodyne_partial_scan, 0},
		{"VELODYNE_PARTIAL_SCAN_IN_FILE", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, (converter_func) carmen_string_and_file_to_velodyne_partial_scan_message, &velodyne_partial_scan, 0},
		{"VARIABLE_VELODYNE_SCAN", "carmen_stereo_velodyne_scan_message8", (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan, 0},
		{"VELODYNE_GPS", CARMEN_VELODYNE_GPS_MESSAGE_NAME, (converter_func) carmen_string_to_velodyne_gps_message, &velodyne_gps, 0},
		{"XSENS_EULER", CARMEN_XSENS_GLOBAL_EULER_NAME, (converter_func) carmen_string_to_xsens_euler_message, &xsens_euler, 0},
		{"XSENS_QUAT", CARMEN_XSENS_GLOBAL_QUAT_NAME, (converter_func) carmen_string_to_xsens_quat_message, &xsens_quat, 0},
		{"XSENS_MATRIX", CARMEN_XSENS_GLOBAL_MATRIX_NAME, (converter_func) carmen_string_to_xsens_matrix_message, &xsens_matrix, 0},
		{"XSENS_MTIG", CARMEN_XSENS_MTIG_NAME, (converter_func) carmen_string_to_xsens_mtig_message, &xsens_mtig, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE1", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE1_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage1, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE2", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage2, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE3", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE3_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage3, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE4", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE4_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage4, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE5", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE5_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage5, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE6", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE6_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage6, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE7", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE7_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage7, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE8", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE8_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage8, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE9", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE9_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage9, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE1", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE1_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage1, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE2", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage2, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE3_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage3, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE4", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE4_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage4, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE5", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE5_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage5, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE6", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE6_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage6, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE7", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE7_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage7, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE8", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE8_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage8, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE9", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE9_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage9, 0},
		{"WEB_CAM_IMAGE", CARMEN_WEB_CAM_MESSAGE_NAME, (converter_func) carmen_string_to_web_cam_message, &web_cam_message, 0},
		{"BASEMOTION_ACK", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, (converter_func) carmen_string_to_base_ackerman_motion_message, &ackerman_motion_message, 0},
		{"ULTRASONIC_SONAR_SENSOR", CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, (converter_func) carmen_string_to_ultrasonic_message, &ultrasonic_message, 0},

		{"FORD_ESCAPE_STATUS", CARMEN_FORD_ESCAPE_STATUS_NAME, (converter_func) carmen_string_to_ford_escape_estatus_message, &ford_escape_status, 0},

		{"GLOBALPOS_ACK", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, (converter_func) carmen_string_to_globalpos_message, &globalpos, 0},
		{"VARIABLE_VELODYNE_SCAN1", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE1_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan1, 0},
		{"VARIABLE_VELODYNE_SCAN2", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE2_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan2, 0},
		{"VARIABLE_VELODYNE_SCAN3", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE3_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan3, 0},
		{"VELODYNE_VARIABLE_SCAN_IN_FILE1", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE1_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan1, 0},
		{"VELODYNE_VARIABLE_SCAN_IN_FILE2", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE2_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan2, 0},
		{"VELODYNE_VARIABLE_SCAN_IN_FILE3", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE3_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan3, 0},
	};

void gicp_cloudModelAndEmptyTruck()
{
	//simplifica as nuvens usando voxel grid

	PointCloud<PointXYZRGB>::Ptr cloud_emptyBucketLaser_downsampled(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud_emptyBucketModel_downsampled(new PointCloud<PointXYZRGB>);

	pcl::VoxelGrid<pcl::PointXYZRGB> vg;

	vg.setInputCloud(cloud_emptyBucketModel);
	vg.setLeafSize (0.15f, 0.15f, 0.15f);
	vg.filter(*cloud_emptyBucketModel_downsampled);

	vg.setInputCloud(cloud_emptyBucketLaser);
	vg.setLeafSize (0.15f, 0.15f, 0.15f);
	vg.filter(*cloud_emptyBucketLaser_downsampled);

	//executa GICP

	GeneralizedIterativeClosestPoint<PointXYZRGB, PointXYZRGB> gicp;
	gicp.setMaximumIterations(2000);
	gicp.setTransformationEpsilon(1e-3);
	gicp.setMaxCorrespondenceDistance(0.25);
	//gicp.setRotationEpsilon(0.00001);

	gicp.setInputCloud(cloud_emptyBucketModel_downsampled);
    gicp.setInputTarget(cloud_emptyBucketLaser_downsampled);
    gicp.align(*output);
	//std::cout << "HAS CONVERGED:" << gicp.hasConverged() << " SCORE: " << gicp.getFitnessScore() << std::endl;

	Matrix<double, 4, 4> gicp_transformation = gicp.getFinalTransformation().cast<double>();
	gicp_transformation_bucketLaser_to_bucketModel = gicp_transformation.inverse();

    //PARA VISUALIZAR O RESULTADO E VERIFICAR SE A TRANSFORMACAO ESTA REALMENTE ALINHANDO AS NUVENS, DESCOMENTAR O CODIGO ABAIXO
    //LEMBRAR DE MANTER A TRANSFORMACAO ABAIXO COMENTADA APOS VISUALIZAR, POIS ESTA OPERACAO NAO DEVE SER REALIZADA NESTE MOMENTO!
    /*transformPointCloud<PointXYZRGB>(*cloud_emptyBucketLaser, *cloud_emptyBucketLaser, gicp_transformation_bucketLaser_to_bucketModel);
    for(unsigned int i=0; i<cloud_emptyBucketLaser->width; i++)
    {
    	cloud_emptyBucketLaser->at(i).r = 30;
    	cloud_emptyBucketLaser->at(i).g = 30;
    	cloud_emptyBucketLaser->at(i).b = 30;
    }
    visualization::PCLVisualizer viewer_teste;
    viewer_teste.setBackgroundColor(.5, .5, .5);
    viewer_teste.addCoordinateSystem (3.0);
    viewer_teste.removeAllPointClouds();
    viewer_teste.addPointCloud(cloud_emptyBucketLaser, "cloud 1");
    viewer_teste.addPointCloud(cloud_emptyBucketModel, "cloud 2");
    viewer_teste.spin();*/
}

//agrupa nuvens de montagens do v1 e v2
void groupMountageClouds()
{
	int position = 0;

	cloud_mountage_v1v2->width = cloud_mountage_v1->width + cloud_mountage_v2->width;
	cloud_mountage_v1v2->points.resize (cloud_mountage_v1v2->width * cloud_mountage_v1v2->height);

	for (unsigned int i=0; i<cloud_mountage_v1->width; i++)
	{
		cloud_mountage_v1v2->points[position].x = cloud_mountage_v1->points[i].x;
		cloud_mountage_v1v2->points[position].y = cloud_mountage_v1->points[i].y;
		cloud_mountage_v1v2->points[position].z = cloud_mountage_v1->points[i].z;

		cloud_mountage_v1v2->at(position).r = 150;
		cloud_mountage_v1v2->at(position).g = 0;
		cloud_mountage_v1v2->at(position).b = 0;

		position++;
	}

	for (unsigned int i=0; i<cloud_mountage_v2->width; i++)
	{
		cloud_mountage_v1v2->points[position].x = cloud_mountage_v2->points[i].x;
		cloud_mountage_v1v2->points[position].y = cloud_mountage_v2->points[i].y;
		cloud_mountage_v1v2->points[position].z = cloud_mountage_v2->points[i].z;

		cloud_mountage_v1v2->at(position).r = 150;
		cloud_mountage_v1v2->at(position).g = 0;
		cloud_mountage_v1v2->at(position).b = 0;

		position++;
	}
}

void gicp_cloudEmptyTruckAndCloudMountage()
{
	//simplifica as nuvens usando voxel grid

	PointCloud<PointXYZRGB>::Ptr cloud_emptyBucketLaserGICP_downsampled(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud_mountage_v1v2_downsampled(new PointCloud<PointXYZRGB>);

	pcl::VoxelGrid<pcl::PointXYZRGB> vg;

	vg.setInputCloud(cloud_emptyBucketLaserGICP);
	vg.setLeafSize (0.05f, 0.05f, 0.05f);
	vg.filter(*cloud_emptyBucketLaserGICP_downsampled);

	vg.setInputCloud(cloud_mountage_v1v2);
	vg.setLeafSize (0.05f, 0.05f, 0.05f);
	vg.filter(*cloud_mountage_v1v2_downsampled);

	//remove qualquer NaN das nuvens de pontos que possa atrapalhar o GICP

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud_emptyBucketLaserGICP_downsampled, *cloud_emptyBucketLaserGICP_downsampled, indices);
	pcl::removeNaNFromPointCloud(*cloud_mountage_v1v2_downsampled, *cloud_mountage_v1v2_downsampled, indices);

	//executa GICP

	GeneralizedIterativeClosestPoint<PointXYZRGB, PointXYZRGB> gicp;
	gicp.setMaximumIterations(2000);
	gicp.setTransformationEpsilon(1e-3);
	gicp.setMaxCorrespondenceDistance(0.3);
	//gicp.setRotationEpsilon(0.00001);

	gicp.setInputCloud(cloud_emptyBucketLaserGICP_downsampled);
    gicp.setInputTarget(cloud_mountage_v1v2_downsampled);
    gicp.align(*output);
	std::cout << "HAS CONVERGED:" << gicp.hasConverged() << " SCORE: " << gicp.getFitnessScore() << std::endl;

	Matrix<double, 4, 4> gicp_transformation = gicp.getFinalTransformation().cast<double>();
	gicp_transformation_bucketMountage_to_bucketLaserPreviousGICP = gicp_transformation.inverse();

	transformPointCloud<PointXYZRGB>(*cloud_emptyBucketLaserGICP, *cloud_emptyBucketLaserGICP, gicp_transformation);	//apenas para visualizacao, pois a partir daqui nao utilizo mais esta nuvem
	transformPointCloud<PointXYZRGB>(*cloud_bucketAxis, *cloud_bucketAxis, gicp_transformation);

	//posiciona a nuvem da montagem na posicao do eixo antes de executar o gicp
	transformPointCloud<PointXYZRGB>(*cloud_mountage_v1v2, *cloud_mountage_v1v2_transformed, gicp_transformation_bucketMountage_to_bucketLaserPreviousGICP);
	transformPointCloud<PointXYZRGB>(*cloud_mountage_v1, *cloud_mountage_v1_transformed, gicp_transformation_bucketMountage_to_bucketLaserPreviousGICP);
	transformPointCloud<PointXYZRGB>(*cloud_mountage_v2, *cloud_mountage_v2_transformed, gicp_transformation_bucketMountage_to_bucketLaserPreviousGICP);
	//traz a nuvem da montagem para a posicao original da nuvem de pontos do laser
	Matrix<double, 4, 4> transformation_bucketMountage_to_bucketLaser = transformation_bucketLaser_to_bucketMountage.inverse();
	transformPointCloud<PointXYZRGB>(*cloud_mountage_v1v2_transformed, *cloud_mountage_v1v2_transformed, transformation_bucketMountage_to_bucketLaser); //aproveito e ja faco aqui a transformacao para a posicao do modelo 3D
	transformPointCloud<PointXYZRGB>(*cloud_mountage_v1_transformed, *cloud_mountage_v1_transformed, transformation_bucketMountage_to_bucketLaser);
	transformPointCloud<PointXYZRGB>(*cloud_mountage_v2_transformed, *cloud_mountage_v2_transformed, transformation_bucketMountage_to_bucketLaser);
	//leva a nuvem da montagem para a posicao da nuvem do modelo 3d
	transformPointCloud<PointXYZRGB>(*cloud_mountage_v1v2_transformed, *cloud_mountage_v1v2_transformed, gicp_transformation_bucketLaser_to_bucketModel);
	transformPointCloud<PointXYZRGB>(*cloud_mountage_v1_transformed, *cloud_mountage_v1_transformed, gicp_transformation_bucketLaser_to_bucketModel);
	transformPointCloud<PointXYZRGB>(*cloud_mountage_v2_transformed, *cloud_mountage_v2_transformed, gicp_transformation_bucketLaser_to_bucketModel);

	for (unsigned int i=0; i<cloud_emptyBucketLaserGICP->width; i++)
	{
		cloud_emptyBucketLaserGICP->at(i).r = 255;
		cloud_emptyBucketLaserGICP->at(i).g = 255;
		cloud_emptyBucketLaserGICP->at(i).b = 0;
	}
	for (unsigned int i=0; i<cloud_mountage_v1->width; i++)
	{
		cloud_mountage_v1->at(i).r = 0;
		cloud_mountage_v1->at(i).g = 0;
		cloud_mountage_v1->at(i).b = 0;
	}
	for (unsigned int i=0; i<cloud_mountage_v2->width; i++)
	{
		cloud_mountage_v2->at(i).r = 0;
		cloud_mountage_v2->at(i).g = 0;
		cloud_mountage_v2->at(i).b = 0;
	}
	for (unsigned int i=0; i<cloud_mountage_v1_transformed->width; i++)
	{
		cloud_mountage_v1_transformed->at(i).r = 0;
		cloud_mountage_v1_transformed->at(i).g = 0;
		cloud_mountage_v1_transformed->at(i).b = 0;
	}
	for (unsigned int i=0; i<cloud_mountage_v2_transformed->width; i++)
	{
		cloud_mountage_v2_transformed->at(i).r = 0;
		cloud_mountage_v2_transformed->at(i).g = 0;
		cloud_mountage_v2_transformed->at(i).b = 0;
	}
	for (unsigned int i=0; i<cloud_mountage_v1v2_transformed->width; i++)
	{
		cloud_mountage_v1v2_transformed->at(i).r = 0;
		cloud_mountage_v1v2_transformed->at(i).g = 0;
		cloud_mountage_v1v2_transformed->at(i).b = 0;
	}

}

void importManualCalibration()
{
	std::ifstream infile("../../../dados/calibration/manual_calibration.txt");
	infile >> c_rx >> c_ry >> c_rz >> c_tx >> c_ty >> c_tz;
}

void exportManualCalibration()
{
	FILE *fptr;
	fptr = fopen("/dados/calibration/manual_calibration.txt","w");
	fprintf(fptr, "%f%s%f%s%f%s%f%s%f%s%f", c_rx, " ", c_ry, " ", c_rz, " ", c_tx, " ", c_ty, " ", c_tz);
	fclose(fptr);
}

void importFinalTransformation()
{
	std::ifstream infile("../../../dados/calibration/final_transformation.txt");
	float x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16;

	infile >> x1 >> x2 >> x3 >> x4 >> x5 >> x6 >> x7 >> x8 >> x9 >> x10 >> x11 >> x12 >> x13 >> x14 >> x15 >> x16;
	final_transformation << x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16;
}

void exportFinalTransformation()
{
	//Salva o arquivo da calibracao
	FILE *fptr;
	fptr = fopen("/dados/calibration/final_transformation.txt","w");

	fprintf(fptr, "%f%s", *(final_transformation.data() + 0), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 4), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 8), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 12), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 1), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 5), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 9), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 13), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 2), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 6), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 10), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 14), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 3), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 7), " ");
	fprintf(fptr, "%f%s", *(final_transformation.data() + 11), " ");
	fprintf(fptr, "%f", *(final_transformation.data() + 15));

	fclose(fptr);
}

void exportPointsFinalMesh()
{
	//obtem caminho e nome do arquivo .pointcloud e cria um novo com formato .xyz

	//Salva o arquivo da montagem
	FILE *fptr;
	fptr = fopen("/dados/points_mountage_final.xyz","w");

	for (unsigned int i=0; i<cloud_mountage_v2_internal->width; i++)
	{
		fprintf(fptr, "%f%s%f%s%f%s", cloud_mountage_v2_internal->points[i].x, "\t", cloud_mountage_v2_internal->points[i].y, "\t", cloud_mountage_v2_internal->points[i].z, "\n");
	}

	//exporta malha do volume
    pcl::io::saveOBJFile("/dados/montagem_malha.obj", meshReconstruction);
    pcl::io::saveOBJFile("/dados/montagem_malha_with_quadric_decimation.obj", meshReconstructionFinal);

	fclose(fptr);
}

void exportMountagePoints()
{
	//obtem caminho e nome do arquivo .pointcloud e cria um novo com formato .xyz

	//Salva o arquivo da montagem
	FILE *fptr;
	fptr = fopen("/dados/points_mountage_v1v2.xyz","w");

	for (unsigned int i=0; i<cloud_mountage_v1v2->width; i++)
	{
		fprintf(fptr, "%f%s%f%s%f%s", cloud_mountage_v1v2->points[i].x, "\t", cloud_mountage_v1v2->points[i].y, "\t", cloud_mountage_v1v2->points[i].z, "\n");
	}

	fptr = fopen("/dados/points_mountage_v1.xyz","w");

	for (unsigned int i=0; i<cloud_mountage_v1->width; i++)
	{
		fprintf(fptr, "%f%s%f%s%f%s", cloud_mountage_v1->points[i].x, "\t", cloud_mountage_v1->points[i].y, "\t", cloud_mountage_v1->points[i].z, "\n");
	}

	fptr = fopen("/dados/points_mountage_v2.xyz","w");

	for (unsigned int i=0; i<cloud_mountage_v2->width; i++)
	{
		fprintf(fptr, "%f%s%f%s%f%s", cloud_mountage_v2->points[i].x, "\t", cloud_mountage_v2->points[i].y, "\t", cloud_mountage_v2->points[i].z, "\n");
	}

	fclose(fptr);
}

void exportCloudEnvironmentV1()
{
	FILE *fptr;
	fptr = fopen("/dados/calibration/cloud_environment_v1.xyz","w");

	for (unsigned int i=0; i<cloud_environmentRemovalV1->width; i++)
	{
		fprintf(fptr, "%f%s%f%s%f%s", cloud_environmentRemovalV1->points[i].x, "\t", cloud_environmentRemovalV1->points[i].y, "\t", cloud_environmentRemovalV1->points[i].z, "\n");
	}

	fclose(fptr);
}

void exportCloudEnvironmentV2()
{
	FILE *fptr;
	fptr = fopen("/dados/calibration/cloud_environment_v2.xyz","w");

	for (unsigned int i=0; i<cloud_environmentRemovalV2->width; i++)
	{
		fprintf(fptr, "%f%s%f%s%f%s", cloud_environmentRemovalV2->points[i].x, "\t", cloud_environmentRemovalV2->points[i].y, "\t", cloud_environmentRemovalV2->points[i].z, "\n");
	}

	fclose(fptr);
}

void exportLinesData()
{
	FILE *fptr;
	fptr = fopen("/dados/lines.txt","w");

	fprintf(fptr, "%f%s", point_x1, " ");
	fprintf(fptr, "%f%s", point_y1, " ");
	fprintf(fptr, "%f%s", point_z1, " ");
	fprintf(fptr, "%f%s", vector_x1, " ");
	fprintf(fptr, "%f%s", vector_y1, " ");
	fprintf(fptr, "%f%s", vector_z1, " ");
	fprintf(fptr, "%f%s", point_x2, " ");
	fprintf(fptr, "%f%s", point_y2, " ");
	fprintf(fptr, "%f%s", point_z2, " ");
	fprintf(fptr, "%f%s", vector_x2, " ");
	fprintf(fptr, "%f%s", vector_y2, " ");
	fprintf(fptr, "%f%s", vector_z2, " ");
	fprintf(fptr, "%f%s", point_x3, " ");
	fprintf(fptr, "%f%s", point_y3, " ");
	fprintf(fptr, "%f%s", point_z3, " ");
	fprintf(fptr, "%f%s", vector_x3, " ");
	fprintf(fptr, "%f%s", vector_y3, " ");
	fprintf(fptr, "%f", vector_z3);

	fclose(fptr);
}

//salva o tamanho e posicao dos cubos para execucao da calibracao
void exportCubePosition()
{
	FILE *fptr;
	fptr = fopen("/dados/calibration/cube_position.txt","w");

	fprintf(fptr, "%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f",
			cx1, " ", cy1, " ", cz1, " ", rx1, " ", ry1, " ", rz1, " ", tx1, " ", ty1, " ", tz1, " ",
			cx2, " ", cy2, " ", cz2, " ", rx2, " ", ry2, " ", rz2, " ", tx2, " ", ty2, " ", tz2);

	fclose(fptr);
}

//salva o tamanho e posicao dos cubos para execucao da calibracao
void importCubePosition()
{
	std::ifstream infile("../../../dados/calibration/cube_position.txt");

	infile >> cx1 >> cy1 >> cz1 >> rx1 >> ry1 >> rz1 >> tx1 >> ty1 >> tz1
		   >> cx2 >> cy2 >> cz2 >> rx2 >> ry2 >> rz2 >> tx2 >> ty2 >> tz2;
}

void createPointCloudsCalibrated()
{
	double x, y, z, v_angle, range, h_angle;
	int size = 0;

	if (/*velodyne_number == 1*/velodyne_number == 3)
	{
		//velodyne 1
		//double vertical_correction[32] = { -30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0,
		//										-20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };
		double vertical_correction[32] = { -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
										   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

		cloud_v1_world->width = 0;
		cloud_v1_world->points.resize (cloud_v1_world->width * cloud_v1_world->height);

		for (int k = 0; k < velodyne_variable_scan3.number_of_shots; k++)
		{
			for (int l = 0; l < velodyne_variable_scan3.partial_scan->shot_size; l++)
			{
				v_angle = (carmen_degrees_to_radians(vertical_correction[l]));
				range = ((double)velodyne_variable_scan3.partial_scan[k].distance[l]);
				h_angle = (carmen_degrees_to_radians(velodyne_variable_scan3.partial_scan[k].angle));

				x = (range * cos(v_angle) * cos(h_angle)) / 500.0;
				y = (range * cos(v_angle) * sin(h_angle)) / 500.0;
				z = (range * sin(v_angle)) / 500.0;

				if (x != 0.0 && y != 0.0 && z != 0.0)
				{
					size++;

					cloud_v1_world->width = size;
					cloud_v1_world->points.resize (cloud_v1_world->width * cloud_v1_world->height);

					cloud_v1_world->points[cloud_v1_world->width-1].x = x;
					cloud_v1_world->points[cloud_v1_world->width-1].y = y;
					cloud_v1_world->points[cloud_v1_world->width-1].z = z;

					cloud_v1_world->at(cloud_v1_world->width-1).r = 255;
					cloud_v1_world->at(cloud_v1_world->width-1).g = 255;
					cloud_v1_world->at(cloud_v1_world->width-1).b = 0;
				}
			}
		}
	}
	else if (velodyne_number == 2)
	{
		double vertical_correction[32] = { -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

		cloud_v2_world->width = 0;
		cloud_v2_world->points.resize (cloud_v2_world->width * cloud_v2_world->height);

		for (int k = 0; k < velodyne_variable_scan2.number_of_shots; k++)
		{
			for (int l = 0; l < velodyne_variable_scan2.partial_scan->shot_size; l++)
			{
				v_angle = (carmen_degrees_to_radians(vertical_correction[l]));
				range = ((double)velodyne_variable_scan2.partial_scan[k].distance[l]);
				h_angle = (carmen_degrees_to_radians(velodyne_variable_scan2.partial_scan[k].angle));

				x = (range * cos(v_angle) * cos(h_angle)) / 500.0;
				y = (range * cos(v_angle) * sin(h_angle)) / 500.0;
				z = (range * sin(v_angle)) / 500.0;

				if (x != 0.0 && y != 0.0 && z != 0.0)
				{
					size++;

					cloud_v2_world->width = size;
					cloud_v2_world->points.resize (cloud_v2_world->width * cloud_v2_world->height);

					cloud_v2_world->points[cloud_v2_world->width-1].x = x;
					cloud_v2_world->points[cloud_v2_world->width-1].y = y;
					cloud_v2_world->points[cloud_v2_world->width-1].z = z;

					cloud_v2_world->at(cloud_v2_world->width-1).r = 255;
					cloud_v2_world->at(cloud_v2_world->width-1).g = 0;
					cloud_v2_world->at(cloud_v2_world->width-1).b = 255;

				}
			}
		}

		transformPointCloud<PointXYZRGB>(*cloud_v2_world, *cloud_v2_world_transformed, final_transformation);
	    transformPointCloud(*MatrixCube2, *MatrixCube2_transformed, final_transformation);

		for (unsigned int i=0; i<cloud_v2_world_transformed->width; i++)
		{
			//azul
			cloud_v2_world_transformed->at(i).r = 0;
			cloud_v2_world_transformed->at(i).g = 0;
			cloud_v2_world_transformed->at(i).b = 255;
		}
	}
}

void clearClouds()
{
	if (velodyne_number == 3)
	{
		cloud_v1_world->width = 0;
		cloud_v1_world->points.resize (cloud_v1_world->width * cloud_v1_world->height);
		cloud_v1_truck->width = 0;
		cloud_v1_truck->points.resize (cloud_v1_truck->width * cloud_v1_truck->height);
		cloud_v1_truck_single_line->width = 0;
		cloud_v1_truck_single_line->points.resize (cloud_v1_truck_single_line->width * cloud_v1_truck_single_line->height);
	}
	else if (velodyne_number == 2)
	{
		cloud_v2_world->width = 0;
		cloud_v2_world->points.resize (cloud_v2_world->width * cloud_v2_world->height);
		cloud_v2_truck->width = 0;
		cloud_v2_truck->points.resize (cloud_v2_truck->width * cloud_v2_truck->height);

		for (unsigned int i=0; i<cloud_v2_border_right->width; i++)
		{
			cloud_v2_border_right->points[i].x = 0;
			cloud_v2_border_right->points[i].y = 0;
			cloud_v2_border_right->points[i].z = 0;
		}
		for (unsigned int i=0; i<cloud_v2_border_left->width; i++)
		{
			cloud_v2_border_left->points[i].x = 0;
			cloud_v2_border_left->points[i].y = 0;
			cloud_v2_border_left->points[i].z = 0;
		}

		for (unsigned int i=0; i<16; i++)
		{
			vetLasersV2[i] = 0;
		}
	}
}

void accumulateBorders()
{
	for (unsigned int i=0; i<cloud_v2_border_right->width; i++)
	{
		if (cloud_v2_border_right->points[i].x != 0 && cloud_v2_border_right->points[i].y != 0 && cloud_v2_border_right->points[i].z != 0)
		{
			cloud_v2_border_right_accumulated->width = cloud_v2_border_right_accumulated->width + 1;
			cloud_v2_border_right_accumulated->points.resize (cloud_v2_border_right_accumulated->width * cloud_v2_border_right_accumulated->height);

			cloud_v2_border_right_accumulated->points[cloud_v2_border_right_accumulated->width-1].x = cloud_v2_border_right->points[i].x;
			cloud_v2_border_right_accumulated->points[cloud_v2_border_right_accumulated->width-1].y = cloud_v2_border_right->points[i].y;
			cloud_v2_border_right_accumulated->points[cloud_v2_border_right_accumulated->width-1].z = cloud_v2_border_right->points[i].z;

			cloud_v2_border_right_accumulated->at(cloud_v2_border_right_accumulated->width-1).r = 255;
			cloud_v2_border_right_accumulated->at(cloud_v2_border_right_accumulated->width-1).g = 20;
			cloud_v2_border_right_accumulated->at(cloud_v2_border_right_accumulated->width-1).b = 147;
		}
	}

	for (unsigned int i=0; i<cloud_v2_border_left->width; i++)
	{
		if (cloud_v2_border_left->points[i].x != 0 && cloud_v2_border_left->points[i].y != 0 && cloud_v2_border_left->points[i].z != 0)
		{
			cloud_v2_border_left_accumulated->width = cloud_v2_border_left_accumulated->width + 1;
			cloud_v2_border_left_accumulated->points.resize (cloud_v2_border_left_accumulated->width * cloud_v2_border_left_accumulated->height);

			cloud_v2_border_left_accumulated->points[cloud_v2_border_left_accumulated->width-1].x = cloud_v2_border_left->points[i].x;
			cloud_v2_border_left_accumulated->points[cloud_v2_border_left_accumulated->width-1].y = cloud_v2_border_left->points[i].y;
			cloud_v2_border_left_accumulated->points[cloud_v2_border_left_accumulated->width-1].z = cloud_v2_border_left->points[i].z;

			cloud_v2_border_left_accumulated->at(cloud_v2_border_left_accumulated->width-1).r = 255;
			cloud_v2_border_left_accumulated->at(cloud_v2_border_left_accumulated->width-1).g = 165;
			cloud_v2_border_left_accumulated->at(cloud_v2_border_left_accumulated->width-1).b = 0;
		}
	}
}

void checkStartOfProcess2()
{
	if (vetLasersV2[0] >= sizeShotProcess2 &&
		vetLasersV2[1] >= sizeShotProcess2 &&
		vetLasersV2[2] >= sizeShotProcess2 &&
		vetLasersV2[3] >= sizeShotProcess2 &&
		vetLasersV2[4] >= sizeShotProcess2 &&
		vetLasersV2[5] >= sizeShotProcess2 &&
		vetLasersV2[6] >= sizeShotProcess2 &&
		vetLasersV2[7] >= sizeShotProcess2 &&
		vetLasersV2[8] >= sizeShotProcess2 &&
		vetLasersV2[9] <= sizeShotProcess2 &&
		vetLasersV2[10] >= sizeShotProcess2 &&
		vetLasersV2[11] <= sizeShotProcess2 &&
		vetLasersV2[12] >= sizeShotProcess2 &&
		vetLasersV2[13] <= sizeShotProcess2 &&
		vetLasersV2[14] >= sizeShotProcess2 &&
		vetLasersV2[15] <= sizeShotProcess2)
	{
		process_start = true;
	}
}

void checkStartOfProcess3()
{
	if (cloud_v2_truck->width >= sizeToStartProcess3)
	{
		process_start = true;
	}
}

void removeZeroesFromBorders()
{
	cloud_v2_border_right_transformed->width = 0;
	cloud_v2_border_right_transformed->points.resize(cloud_v2_border_right_transformed->width * cloud_v2_border_right_transformed->height);

	cloud_v2_border_left_transformed->width = 0;
	cloud_v2_border_left_transformed->points.resize (cloud_v2_border_left_transformed->width * cloud_v2_border_left_transformed->height);

	for (unsigned int i=0; i<cloud_v2_border_right->width; i++)
	{
		if (cloud_v2_border_right->points[i].x != 0 && cloud_v2_border_right->points[i].x != 0 && cloud_v2_border_right->points[i].x != 0)
		{
			cloud_v2_border_right_transformed->width = cloud_v2_border_right_transformed->width + 1;
			cloud_v2_border_right_transformed->points.resize (cloud_v2_border_right_transformed->width * cloud_v2_border_right_transformed->height);

			cloud_v2_border_right_transformed->points[cloud_v2_border_right_transformed->width-1].x = cloud_v2_border_right->points[i].x;
			cloud_v2_border_right_transformed->points[cloud_v2_border_right_transformed->width-1].y = cloud_v2_border_right->points[i].y;
			cloud_v2_border_right_transformed->points[cloud_v2_border_right_transformed->width-1].z = cloud_v2_border_right->points[i].z;
			cloud_v2_border_right_transformed->at(cloud_v2_border_right_transformed->width-1).r = cloud_v2_border_right->at(i).r;
			cloud_v2_border_right_transformed->at(cloud_v2_border_right_transformed->width-1).g = cloud_v2_border_right->at(i).g;
			cloud_v2_border_right_transformed->at(cloud_v2_border_right_transformed->width-1).b = cloud_v2_border_right->at(i).b;
		}
	}

	for (unsigned int i=0; i<cloud_v2_border_left->width; i++)
	{
		if (cloud_v2_border_left->points[i].x != 0 && cloud_v2_border_left->points[i].x != 0 && cloud_v2_border_left->points[i].x != 0)
		{
			cloud_v2_border_left_transformed->width = cloud_v2_border_left_transformed->width + 1;
			cloud_v2_border_left_transformed->points.resize (cloud_v2_border_left_transformed->width * cloud_v2_border_left_transformed->height);

			cloud_v2_border_left_transformed->points[cloud_v2_border_left_transformed->width-1].x = cloud_v2_border_left->points[i].x;
			cloud_v2_border_left_transformed->points[cloud_v2_border_left_transformed->width-1].y = cloud_v2_border_left->points[i].y;
			cloud_v2_border_left_transformed->points[cloud_v2_border_left_transformed->width-1].z = cloud_v2_border_left->points[i].z;
			cloud_v2_border_left_transformed->at(cloud_v2_border_left_transformed->width-1).r = cloud_v2_border_left->at(i).r;
			cloud_v2_border_left_transformed->at(cloud_v2_border_left_transformed->width-1).g = cloud_v2_border_left->at(i).g;
			cloud_v2_border_left_transformed->at(cloud_v2_border_left_transformed->width-1).b = cloud_v2_border_left->at(i).b;
		}
	}
}

void createPointCloudsCalibratedByKDTree()
{
	double x, y, z, v_angle, range, h_angle;
	bool flagBorderRight = true;

	if (/*velodyne_number == 1*/velodyne_number == 3)
	{
		//velodyne 1
		//double vertical_correction[32] = { -30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0,
		//										-20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };
		double vertical_correction[32] = { -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
										   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		clearClouds();

		for (int k = 0; k < velodyne_variable_scan3.number_of_shots; k++)
		{
			for (int l = 0; l < velodyne_variable_scan3.partial_scan->shot_size; l++)
			{
				v_angle = (carmen_degrees_to_radians(vertical_correction[l]));
				range = ((double)velodyne_variable_scan3.partial_scan[k].distance[l]);
				h_angle = (carmen_degrees_to_radians(velodyne_variable_scan3.partial_scan[k].angle));

				x = (range * cos(v_angle) * cos(h_angle)) / 500.0;
				y = (range * cos(v_angle) * sin(h_angle)) / 500.0;
				z = (range * sin(v_angle)) / 500.0;

				if (x != 0.0 && y != 0.0 && z != 0.0)
				{
					searchPoint.x = x;
					searchPoint.y = y;
					searchPoint.z = z;

					if (kdtree_environmentV1.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					{
						cloud_v1_world->width = cloud_v1_world->width + 1;
						cloud_v1_world->points.resize (cloud_v1_world->width * cloud_v1_world->height);

						cloud_v1_world->points[cloud_v1_world->width-1].x = x;
						cloud_v1_world->points[cloud_v1_world->width-1].y = y;
						cloud_v1_world->points[cloud_v1_world->width-1].z = z;

						cloud_v1_world->at(cloud_v1_world->width-1).r = 0;
						cloud_v1_world->at(cloud_v1_world->width-1).g = 0;
						cloud_v1_world->at(cloud_v1_world->width-1).b = 255;
					}
					else
					{
						cloud_v1_truck->width = cloud_v1_truck->width + 1;
						cloud_v1_truck->points.resize (cloud_v1_truck->width * cloud_v1_truck->height);

						cloud_v1_truck->points[cloud_v1_truck->width-1].x = x;
						cloud_v1_truck->points[cloud_v1_truck->width-1].y = y;
						cloud_v1_truck->points[cloud_v1_truck->width-1].z = z;

						cloud_v1_truck->at(cloud_v1_truck->width-1).r = 0;
						cloud_v1_truck->at(cloud_v1_truck->width-1).g = 255;
						cloud_v1_truck->at(cloud_v1_truck->width-1).b = 255;

						//salva nuvem com linha unica
						if (l == 14)
						{
							cloud_v1_truck_single_line->width = cloud_v1_truck_single_line->width + 1;
							cloud_v1_truck_single_line->points.resize (cloud_v1_truck_single_line->width * cloud_v1_truck_single_line->height);

							cloud_v1_truck_single_line->points[cloud_v1_truck_single_line->width-1].x = x;
							cloud_v1_truck_single_line->points[cloud_v1_truck_single_line->width-1].y = y;
							cloud_v1_truck_single_line->points[cloud_v1_truck_single_line->width-1].z = z;

							cloud_v1_truck_single_line->at(cloud_v1_truck_single_line->width-1).r = 255;
							cloud_v1_truck_single_line->at(cloud_v1_truck_single_line->width-1).g = 255;
							cloud_v1_truck_single_line->at(cloud_v1_truck_single_line->width-1).b = 0;
						}
					}
				}
			}
		}
	}
	else if (velodyne_number == 2)
	{
		double vertical_correction[32] = { -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
										   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		clearClouds();

		for (int k = 0; k < velodyne_variable_scan2.number_of_shots; k++)
		{
			for (int l = 0; l < velodyne_variable_scan2.partial_scan->shot_size; l++)
			{
				v_angle = (carmen_degrees_to_radians(vertical_correction[l]));
				range = ((double)velodyne_variable_scan2.partial_scan[k].distance[l]);
				h_angle = (carmen_degrees_to_radians(velodyne_variable_scan2.partial_scan[k].angle));

				x = (range * cos(v_angle) * cos(h_angle)) / 500.0;
				y = (range * cos(v_angle) * sin(h_angle)) / 500.0;
				z = (range * sin(v_angle)) / 500.0;

				if (x != 0.0 && y != 0.0 && z != 0.0)
				{
					searchPoint.x = x;
					searchPoint.y = y;
					searchPoint.z = z;

					if (kdtree_environmentV2.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					{
						cloud_v2_world->width = cloud_v2_world->width + 1;
						cloud_v2_world->points.resize (cloud_v2_world->width * cloud_v2_world->height);

						cloud_v2_world->points[cloud_v2_world->width-1].x = x;
						cloud_v2_world->points[cloud_v2_world->width-1].y = y;
						cloud_v2_world->points[cloud_v2_world->width-1].z = z;

						cloud_v2_world->at(cloud_v2_world->width-1).r = 0;
						cloud_v2_world->at(cloud_v2_world->width-1).g = 0;
						cloud_v2_world->at(cloud_v2_world->width-1).b = 255;
					}
					else
					{
						cloud_v2_truck->width = cloud_v2_truck->width + 1;
						cloud_v2_truck->points.resize (cloud_v2_truck->width * cloud_v2_truck->height);

						cloud_v2_truck->points[cloud_v2_truck->width-1].x = x;
						cloud_v2_truck->points[cloud_v2_truck->width-1].y = y;
						cloud_v2_truck->points[cloud_v2_truck->width-1].z = z;

						cloud_v2_truck->at(cloud_v2_truck->width-1).r = 0;
						cloud_v2_truck->at(cloud_v2_truck->width-1).g = 255;
						cloud_v2_truck->at(cloud_v2_truck->width-1).b = 0;

						//if (task == 2 || task == 3)
						//{
							if(velodyne_variable_scan2.partial_scan[k].angle > middleAngle && velodyne_variable_scan2.partial_scan[k].angle < middleAngle + rangeAngle)
							{
								cloud_v2_border_right->points[l].x = x;
								cloud_v2_border_right->points[l].y = y;
								cloud_v2_border_right->points[l].z = z;

								cloud_v2_border_right->at(l).r = 255;
								cloud_v2_border_right->at(l).g = 20;
								cloud_v2_border_right->at(l).b = 147;
							}
							else if ((velodyne_variable_scan2.partial_scan[k].angle < middleAngle && velodyne_variable_scan2.partial_scan[k].angle > middleAngle - rangeAngle)
									 && (cloud_v2_border_left->points[l].x == 0 && cloud_v2_border_left->points[l].y == 0 && cloud_v2_border_left->points[l].z == 0))
							{
								cloud_v2_border_left->points[l].x = x;
								cloud_v2_border_left->points[l].y = y;
								cloud_v2_border_left->points[l].z = z;

								cloud_v2_border_left->at(l).r = 255;
								cloud_v2_border_left->at(l).g = 165;
								cloud_v2_border_left->at(l).b = 0;
							}
							vetLasersV2[l]++;		//para indicar que este laser possui algum ponto;
						//}
					}
				}
			}
		}

		transformPointCloud<PointXYZRGB>(*cloud_v2_world, *cloud_v2_world_transformed, final_transformation);
		transformPointCloud<PointXYZRGB>(*cloud_v2_truck, *cloud_v2_truck_transformed, final_transformation);
	    transformPointCloud(*MatrixCube2, *MatrixCube2_transformed, final_transformation);

	    removeZeroesFromBorders();
		transformPointCloud<PointXYZRGB>(*cloud_v2_border_left_transformed, *cloud_v2_border_left_transformed, final_transformation);
		transformPointCloud<PointXYZRGB>(*cloud_v2_border_right_transformed, *cloud_v2_border_right_transformed, final_transformation);

	    if (task == 2)
	    {
	    	if (!process_start)
	    	{
	    		checkStartOfProcess2();
	    	}
	    	else
	    	{
	    		accumulateBorders();
				transformPointCloud<PointXYZRGB>(*cloud_v2_border_right_accumulated, *cloud_v2_border_right_accumulated_final, final_transformation);
				transformPointCloud<PointXYZRGB>(*cloud_v2_border_left_accumulated, *cloud_v2_border_left_accumulated_final, final_transformation);

				if (cloud_v2_truck->width < sizeToEndProcess)
					process_end = true;
	    	}
	    }
	    if (task == 3)
	    {
	    	if (!process_start)
	    	{
	    		checkStartOfProcess3();
	    	}
	    	else
	    	{
	    		if (cloud_v2_truck->width < sizeToEndProcess)
	    			process_end = true;
	    	}
	    }

	}
}

void cleanVelodyneMessagesByCube()
{
	Matrix<double, 4, 1> point;

	if (/*line[30] == '1' ||*/velodyne_number == 3) //velodyne 1
	{
		/*double vertical_correction[32] = {-30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0,
										  -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };*/
		double vertical_correction[32] = {-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
												  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		for (int k = 0; k < velodyne_variable_scan3.number_of_shots; k++)
		{
			for (int l = 0; l < velodyne_variable_scan3.partial_scan->shot_size; l++)
			{
				double v_angle = (carmen_degrees_to_radians(vertical_correction[l]));
				double range = ((double) velodyne_variable_scan3.partial_scan[k].distance[l]);
				double h_angle = (carmen_degrees_to_radians(velodyne_variable_scan3.partial_scan[k].angle));

				double x, y, z;
				x = (range * cos(v_angle) * cos(h_angle)) / 500.0;
				y = (range * cos(v_angle) * sin(h_angle)) / 500.0;
				z = (range * sin(v_angle)) / 500.0;

				//transforma os valores de x, y e z para dentro do cubo na origem
				point << x, y, z, 1;
				point = transformationCubeV1.inverse() * point;

				x = *(point.data() + 0);
				y = *(point.data() + 1);
				z = *(point.data() + 2);

				if (!((x >= -cx1/2.0) && (x <= cx1/2.0) && (y >= -cy1/2.0) && (y <= cy1/2.0) && (z >= -cz1/2.0) && (z <= cz1/2.0)))
				{
					velodyne_variable_scan3.partial_scan[k].distance[l] = 0.0;
					velodyne_variable_scan3.partial_scan[k].intensity[l] = 0.0;
				}
			}
		}
	}
	else if (velodyne_number == 2) //velodyne 2
	{
		double vertical_correction[32] = {-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
										  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		for (int k = 0; k < velodyne_variable_scan2.number_of_shots; k++)
		{
			for (int l = 0; l < velodyne_variable_scan2.partial_scan->shot_size; l++)
			{
				double v_angle = (carmen_degrees_to_radians(vertical_correction[l]));

				double range = ((double) velodyne_variable_scan2.partial_scan[k].distance[l]);

				double h_angle = (carmen_degrees_to_radians(velodyne_variable_scan2.partial_scan[k].angle));

				double x, y, z;
				x = (range * cos(v_angle) * cos(h_angle)) / 500.0;
				y = (range * cos(v_angle) * sin(h_angle)) / 500.0;
				z = (range * sin(v_angle)) / 500.0;

				//transforma os valores de x, y e z para dentro do cubo na origem
				point << x, y, z, 1;
				point = transformationCubeV2.inverse() * point;

				x = *(point.data() + 0);
				y = *(point.data() + 1);
				z = *(point.data() + 2);

				if (!((x >= -cx2/2.0) && (x <= cx2/2.0) && (y >= -cy2/2.0) && (y <= cy2/2.0) && (z >= -cz2/2.0) && (z <= cz2/2.0)))
				{
					velodyne_variable_scan2.partial_scan[k].distance[l] = 0.0;
					velodyne_variable_scan2.partial_scan[k].intensity[l] = 0.0;
				}
			}
		}
	}
}

//ajusta a nuvem do primeiro velodyne para o segundo na matriz de transformacao (ajuste manual)
void configureManualCalibration(bool saveFile)
{
	Matrix<double, 4, 4> rotationX, rotationY, rotationZ, translation;
	float C_RX, C_RY, C_RZ;

	C_RX = carmen_degrees_to_radians(c_rx);
	C_RY = carmen_degrees_to_radians(c_ry);
	C_RZ = carmen_degrees_to_radians(c_rz);

	rotationX << 1,		0,			0,			0,
				 0,		cos(C_RX),	-sin(C_RX), 0,
				 0,		sin(C_RX), 	cos(C_RX), 	0,
				 0, 	0, 			0,			1;

	rotationY << cos(C_RY),	0,		-sin(C_RY),	0,
				 0,			1,		0,			0,
				 sin(C_RY), 0,		cos(C_RY), 	0,
				 0, 		0, 		0,			1;

	rotationZ << cos(C_RZ),	-sin(C_RZ), 0, 		0,
				 sin(C_RZ), cos(C_RZ), 	0, 		0,
				 0, 		0, 			1, 		0,
				 0, 		0, 			0, 		1;

	translation << 1, 0, 0, c_tx,
				   0, 1, 0, c_ty,
				   0, 0, 1, c_tz,
				   0, 0, 0, 1;

    final_transformation = transformationCubeV2 * translation * rotationY * rotationX * rotationZ * transformationCubeV2.inverse();

    if (saveFile)
    {
    	exportManualCalibration();
    	exportFinalTransformation();
    }
}

void calibration_gicp()
{
    //ajusta a nuvem de V2 com GICP após a transformacao manual, para melhorar a calibracao

    GeneralizedIterativeClosestPoint<PointXYZRGB, PointXYZRGB> gicp;
    gicp.setMaximumIterations(2000);
    gicp.setTransformationEpsilon(1e-3);
    gicp.setMaxCorrespondenceDistance(0.3);

    PointCloud<PointXYZRGB>::Ptr output (new PointCloud<PointXYZRGB>);
    Matrix<double, 4, 4> gicp_correction; //matriz de transformacao do gicp

	gicp.setInputCloud(cloud_v2_world_transformed);
	gicp.setInputTarget(cloud_v1_world);
	gicp.align(*output);
    gicp_correction = gicp.getFinalTransformation().cast<double>();

	//transformPointCloud<PointXYZRGB>(*cloud_v2_world_transformed, *cloud_v2_world_transformed, gicp_correction);

	//matriz com a composicao da transformacao manual com a transformacao do gicp
	final_transformation = gicp_correction * final_transformation;

    transformPointCloud(*cloud_v2_world, *cloud_v2_world_transformed, final_transformation);
    transformPointCloud(*MatrixCube2, *MatrixCube2_transformed, final_transformation);

	for (unsigned int i=0; i<cloud_v2_world_transformed->width; i++)
	{
		//ciano
		cloud_v2_world_transformed->at(i).r = 255;
		cloud_v2_world_transformed->at(i).g = 255;
		cloud_v2_world_transformed->at(i).b = 0;
	}

	exportFinalTransformation();
	drawCloudsCalibration(true);

}

void executePoisson()
{
	PointCloud<PointXYZRGB>::Ptr cloud_mountage_volume_filtered(new PointCloud<PointXYZRGB>());
	PassThrough<PointXYZRGB> filter;
	filter.setInputCloud(cloud_mountage_v2_internal);
	filter.filter(*cloud_mountage_volume_filtered);

	/*MovingLeastSquares<PointXYZRGB, PointXYZRGB> mls;
	mls.setInputCloud(cloud_mountage_volume_filtered);
	mls.setSearchRadius(0.1);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(2);
	mls.setUpsamplingMethod(MovingLeastSquares<PointXYZRGB, PointXYZRGB>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius(0.05);
	mls.setUpsamplingStepSize(0.01);

	PointCloud<PointXYZRGB>::Ptr cloud_smoothed (new PointCloud<PointXYZRGB>());
	mls.process(*cloud_smoothed);

	viewer4.addPointCloud(cloud_smoothed, "cloud_mountage_volume_filtered");
	viewer4.spin();*/

	NormalEstimationOMP<PointXYZRGB, Normal> ne;
	ne.setNumberOfThreads (8);
	ne.setInputCloud (cloud_mountage_volume_filtered);
	ne.setRadiusSearch (0.5);
	Eigen::Vector4f centroid;
	compute3DCentroid (*cloud_mountage_volume_filtered, centroid);
	ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
	ne.compute (*cloud_normals);

	for (size_t  i = 0; i < cloud_normals->size(); ++i)
	{
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}

	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals (new PointCloud<PointXYZRGBNormal> ());
	concatenateFields (*cloud_mountage_volume_filtered, *cloud_normals, *cloud_with_normals);

	Poisson<PointXYZRGBNormal> poisson;
	poisson.setDepth (10);
	poisson.setInputCloud (cloud_with_normals);

	poisson.reconstruct (meshReconstruction);
}

void meshSimplification()
{
	//remove vertices nao usados
	/*pcl::surface::SimplificationRemoveUnusedVertices cleaner;
	cleaner.simplify(meshReconstruction, meshReconstruction);*/

	//Executa processo de Quadric Decimation
	pcl::MeshQuadricDecimationVTK mesh_decimator;
	mesh_decimator.setInputMesh(meshReconstructionPointer);
	mesh_decimator.setTargetReductionFactor(0.99);
	mesh_decimator.process(meshReconstructionFinal);

	//vtkSmartPointer<vtkPolyData> pointsPolydata;// = vtkSmartPointer<vtkPolyData>::New();
	//VTKUtils::convertToVTK (meshReconstruction, pointsPolydata);
}

void drawCloudsCalibration(bool waitSpin)
{
	viewer1.setBackgroundColor(.5, .5, .5);
	viewer1.addCoordinateSystem (1.0);
	viewer1.removeAllPointClouds();
	viewer1.removeAllShapes();

	//VISUALIZACAO DO MUNDO
	viewer1.addPointCloud(cloud_v1_world, "world_v1");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "world_v1");

	viewer1.addPointCloud(cloud_v2_world, "world_v2");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "world_v2");

	viewer1.addPointCloud(cloud_v2_world_transformed, "world_v2_transformed");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "world_v2_transformed");

	viewer1.addPointCloud(cloud_pointPickingV1, "pp_v1");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "pp_v1");
	viewer1.addPointCloud(cloud_pointPickingV2, "pp_v2");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "pp_v2");

	PointXYZ pontoInicio, pontoFim;

	//desenha arestas do cubo V1:
	pontoInicio.x = MatrixCube1->points[0].x; pontoInicio.y = MatrixCube1->points[0].y; pontoInicio.z = MatrixCube1->points[0].z;
	pontoFim.x = MatrixCube1->points[1].x; pontoFim.y = MatrixCube1->points[1].y; pontoFim.z = MatrixCube1->points[1].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 0, "1", 0);
	pontoInicio.x = MatrixCube1->points[1].x; pontoInicio.y = MatrixCube1->points[1].y; pontoInicio.z = MatrixCube1->points[1].z;
	pontoFim.x = MatrixCube1->points[2].x; pontoFim.y = MatrixCube1->points[2].y; pontoFim.z = MatrixCube1->points[2].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 255, 0, "2", 0);
	pontoInicio.x = MatrixCube1->points[2].x; pontoInicio.y = MatrixCube1->points[2].y; pontoInicio.z = MatrixCube1->points[2].z;
	pontoFim.x = MatrixCube1->points[3].x; pontoFim.y = MatrixCube1->points[3].y; pontoFim.z = MatrixCube1->points[3].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 255, 0, "3", 0);
	pontoInicio.x = MatrixCube1->points[3].x; pontoInicio.y = MatrixCube1->points[3].y; pontoInicio.z = MatrixCube1->points[3].z;
	pontoFim.x = MatrixCube1->points[0].x; pontoFim.y = MatrixCube1->points[0].y; pontoFim.z = MatrixCube1->points[0].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 0, "4", 0);
	pontoInicio.x = MatrixCube1->points[4].x; pontoInicio.y = MatrixCube1->points[4].y; pontoInicio.z = MatrixCube1->points[4].z;
	pontoFim.x = MatrixCube1->points[5].x; pontoFim.y = MatrixCube1->points[5].y; pontoFim.z = MatrixCube1->points[5].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 255, 0, "5", 0);
	pontoInicio.x = MatrixCube1->points[5].x; pontoInicio.y = MatrixCube1->points[5].y; pontoInicio.z = MatrixCube1->points[5].z;
	pontoFim.x = MatrixCube1->points[6].x; pontoFim.y = MatrixCube1->points[6].y; pontoFim.z = MatrixCube1->points[6].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 255, 0, "6", 0);
	pontoInicio.x = MatrixCube1->points[6].x; pontoInicio.y = MatrixCube1->points[6].y; pontoInicio.z = MatrixCube1->points[6].z;
	pontoFim.x = MatrixCube1->points[7].x; pontoFim.y = MatrixCube1->points[7].y; pontoFim.z = MatrixCube1->points[7].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 255, 0, "7", 0);
	pontoInicio.x = MatrixCube1->points[7].x; pontoInicio.y = MatrixCube1->points[7].y; pontoInicio.z = MatrixCube1->points[7].z;
	pontoFim.x = MatrixCube1->points[4].x; pontoFim.y = MatrixCube1->points[4].y; pontoFim.z = MatrixCube1->points[4].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 255, 0, "8", 0);
	pontoInicio.x = MatrixCube1->points[4].x; pontoInicio.y = MatrixCube1->points[4].y; pontoInicio.z = MatrixCube1->points[4].z;
	pontoFim.x = MatrixCube1->points[0].x; pontoFim.y = MatrixCube1->points[0].y; pontoFim.z = MatrixCube1->points[0].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 0, 255, "9", 0);
	pontoInicio.x = MatrixCube1->points[5].x; pontoInicio.y = MatrixCube1->points[5].y; pontoInicio.z = MatrixCube1->points[5].z;
	pontoFim.x = MatrixCube1->points[1].x; pontoFim.y = MatrixCube1->points[1].y; pontoFim.z = MatrixCube1->points[1].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 255, 0, "10", 0);
	pontoInicio.x = MatrixCube1->points[6].x; pontoInicio.y = MatrixCube1->points[6].y; pontoInicio.z = MatrixCube1->points[6].z;
	pontoFim.x = MatrixCube1->points[2].x; pontoFim.y = MatrixCube1->points[2].y; pontoFim.z = MatrixCube1->points[2].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 255, 0, "11", 0);
	pontoInicio.x = MatrixCube1->points[7].x; pontoInicio.y = MatrixCube1->points[7].y; pontoInicio.z = MatrixCube1->points[7].z;
	pontoFim.x = MatrixCube1->points[3].x; pontoFim.y = MatrixCube1->points[3].y; pontoFim.z = MatrixCube1->points[3].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 255, 0, "12", 0);

	//desenha arestas do cubo V2:
	pontoInicio.x = MatrixCube2->points[0].x; pontoInicio.y = MatrixCube2->points[0].y; pontoInicio.z = MatrixCube2->points[0].z;
	pontoFim.x = MatrixCube2->points[1].x; pontoFim.y = MatrixCube2->points[1].y; pontoFim.z = MatrixCube2->points[1].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 0, "13", 0);
	pontoInicio.x = MatrixCube2->points[1].x; pontoInicio.y = MatrixCube2->points[1].y; pontoInicio.z = MatrixCube2->points[1].z;
	pontoFim.x = MatrixCube2->points[2].x; pontoFim.y = MatrixCube2->points[2].y; pontoFim.z = MatrixCube2->points[2].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 255, "14", 0);
	pontoInicio.x = MatrixCube2->points[2].x; pontoInicio.y = MatrixCube2->points[2].y; pontoInicio.z = MatrixCube2->points[2].z;
	pontoFim.x = MatrixCube2->points[3].x; pontoFim.y = MatrixCube2->points[3].y; pontoFim.z = MatrixCube2->points[3].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 255, "15", 0);
	pontoInicio.x = MatrixCube2->points[3].x; pontoInicio.y = MatrixCube2->points[3].y; pontoInicio.z = MatrixCube2->points[3].z;
	pontoFim.x = MatrixCube2->points[0].x; pontoFim.y = MatrixCube2->points[0].y; pontoFim.z = MatrixCube2->points[0].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 0, "16", 0);
	pontoInicio.x = MatrixCube2->points[4].x; pontoInicio.y = MatrixCube2->points[4].y; pontoInicio.z = MatrixCube2->points[4].z;
	pontoFim.x = MatrixCube2->points[5].x; pontoFim.y = MatrixCube2->points[5].y; pontoFim.z = MatrixCube2->points[5].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 255, "17", 0);
	pontoInicio.x = MatrixCube2->points[5].x; pontoInicio.y = MatrixCube2->points[5].y; pontoInicio.z = MatrixCube2->points[5].z;
	pontoFim.x = MatrixCube2->points[6].x; pontoFim.y = MatrixCube2->points[6].y; pontoFim.z = MatrixCube2->points[6].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 255, "18", 0);
	pontoInicio.x = MatrixCube2->points[6].x; pontoInicio.y = MatrixCube2->points[6].y; pontoInicio.z = MatrixCube2->points[6].z;
	pontoFim.x = MatrixCube2->points[7].x; pontoFim.y = MatrixCube2->points[7].y; pontoFim.z = MatrixCube2->points[7].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 255, "19", 0);
	pontoInicio.x = MatrixCube2->points[7].x; pontoInicio.y = MatrixCube2->points[7].y; pontoInicio.z = MatrixCube2->points[7].z;
	pontoFim.x = MatrixCube2->points[4].x; pontoFim.y = MatrixCube2->points[4].y; pontoFim.z = MatrixCube2->points[4].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 255, "20", 0);
	pontoInicio.x = MatrixCube2->points[4].x; pontoInicio.y = MatrixCube2->points[4].y; pontoInicio.z = MatrixCube2->points[4].z;
	pontoFim.x = MatrixCube2->points[0].x; pontoFim.y = MatrixCube2->points[0].y; pontoFim.z = MatrixCube2->points[0].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 0, 255, "21", 0);
	pontoInicio.x = MatrixCube2->points[5].x; pontoInicio.y = MatrixCube2->points[5].y; pontoInicio.z = MatrixCube2->points[5].z;
	pontoFim.x = MatrixCube2->points[1].x; pontoFim.y = MatrixCube2->points[1].y; pontoFim.z = MatrixCube2->points[1].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 255, "22", 0);
	pontoInicio.x = MatrixCube2->points[6].x; pontoInicio.y = MatrixCube2->points[6].y; pontoInicio.z = MatrixCube2->points[6].z;
	pontoFim.x = MatrixCube2->points[2].x; pontoFim.y = MatrixCube2->points[2].y; pontoFim.z = MatrixCube2->points[2].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 255, "23", 0);
	pontoInicio.x = MatrixCube2->points[7].x; pontoInicio.y = MatrixCube2->points[7].y; pontoInicio.z = MatrixCube2->points[7].z;
	pontoFim.x = MatrixCube2->points[3].x; pontoFim.y = MatrixCube2->points[3].y; pontoFim.z = MatrixCube2->points[3].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 255, "24", 0);

	//desenha arestas do cubo V2 transformado:
	pontoInicio.x = MatrixCube2_transformed->points[0].x; pontoInicio.y = MatrixCube2_transformed->points[0].y; pontoInicio.z = MatrixCube2_transformed->points[0].z;
	pontoFim.x = MatrixCube2_transformed->points[1].x; pontoFim.y = MatrixCube2_transformed->points[1].y; pontoFim.z = MatrixCube2_transformed->points[1].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 0, "25", 0);
	pontoInicio.x = MatrixCube2_transformed->points[1].x; pontoInicio.y = MatrixCube2_transformed->points[1].y; pontoInicio.z = MatrixCube2_transformed->points[1].z;
	pontoFim.x = MatrixCube2_transformed->points[2].x; pontoFim.y = MatrixCube2_transformed->points[2].y; pontoFim.z = MatrixCube2_transformed->points[2].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 255, "26", 0);
	pontoInicio.x = MatrixCube2_transformed->points[2].x; pontoInicio.y = MatrixCube2_transformed->points[2].y; pontoInicio.z = MatrixCube2_transformed->points[2].z;
	pontoFim.x = MatrixCube2_transformed->points[3].x; pontoFim.y = MatrixCube2_transformed->points[3].y; pontoFim.z = MatrixCube2_transformed->points[3].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 255, "27", 0);
	pontoInicio.x = MatrixCube2_transformed->points[3].x; pontoInicio.y = MatrixCube2_transformed->points[3].y; pontoInicio.z = MatrixCube2_transformed->points[3].z;
	pontoFim.x = MatrixCube2_transformed->points[0].x; pontoFim.y = MatrixCube2_transformed->points[0].y; pontoFim.z = MatrixCube2_transformed->points[0].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 0, "28", 0);
	pontoInicio.x = MatrixCube2_transformed->points[4].x; pontoInicio.y = MatrixCube2_transformed->points[4].y; pontoInicio.z = MatrixCube2_transformed->points[4].z;
	pontoFim.x = MatrixCube2_transformed->points[5].x; pontoFim.y = MatrixCube2_transformed->points[5].y; pontoFim.z = MatrixCube2_transformed->points[5].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 255, "29", 0);
	pontoInicio.x = MatrixCube2_transformed->points[5].x; pontoInicio.y = MatrixCube2_transformed->points[5].y; pontoInicio.z = MatrixCube2_transformed->points[5].z;
	pontoFim.x = MatrixCube2_transformed->points[6].x; pontoFim.y = MatrixCube2_transformed->points[6].y; pontoFim.z = MatrixCube2_transformed->points[6].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 255, "30", 0);
	pontoInicio.x = MatrixCube2_transformed->points[6].x; pontoInicio.y = MatrixCube2_transformed->points[6].y; pontoInicio.z = MatrixCube2_transformed->points[6].z;
	pontoFim.x = MatrixCube2_transformed->points[7].x; pontoFim.y = MatrixCube2_transformed->points[7].y; pontoFim.z = MatrixCube2_transformed->points[7].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 255, "31", 0);
	pontoInicio.x = MatrixCube2_transformed->points[7].x; pontoInicio.y = MatrixCube2_transformed->points[7].y; pontoInicio.z = MatrixCube2_transformed->points[7].z;
	pontoFim.x = MatrixCube2_transformed->points[4].x; pontoFim.y = MatrixCube2_transformed->points[4].y; pontoFim.z = MatrixCube2_transformed->points[4].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 255, "32", 0);
	pontoInicio.x = MatrixCube2_transformed->points[4].x; pontoInicio.y = MatrixCube2_transformed->points[4].y; pontoInicio.z = MatrixCube2_transformed->points[4].z;
	pontoFim.x = MatrixCube2_transformed->points[0].x; pontoFim.y = MatrixCube2_transformed->points[0].y; pontoFim.z = MatrixCube2_transformed->points[0].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 0, 255, "33", 0);
	pontoInicio.x = MatrixCube2_transformed->points[5].x; pontoInicio.y = MatrixCube2_transformed->points[5].y; pontoInicio.z = MatrixCube2_transformed->points[5].z;
	pontoFim.x = MatrixCube2_transformed->points[1].x; pontoFim.y = MatrixCube2_transformed->points[1].y; pontoFim.z = MatrixCube2_transformed->points[1].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 255, "34", 0);
	pontoInicio.x = MatrixCube2_transformed->points[6].x; pontoInicio.y = MatrixCube2_transformed->points[6].y; pontoInicio.z = MatrixCube2_transformed->points[6].z;
	pontoFim.x = MatrixCube2_transformed->points[2].x; pontoFim.y = MatrixCube2_transformed->points[2].y; pontoFim.z = MatrixCube2_transformed->points[2].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 255, "35", 0);
	pontoInicio.x = MatrixCube2_transformed->points[7].x; pontoInicio.y = MatrixCube2_transformed->points[7].y; pontoInicio.z = MatrixCube2_transformed->points[7].z;
	pontoFim.x = MatrixCube2_transformed->points[3].x; pontoFim.y = MatrixCube2_transformed->points[3].y; pontoFim.z = MatrixCube2_transformed->points[3].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 255, "36", 0);

	if (waitSpin)
		viewer1.spin();
	else
		viewer1.spinOnce();
}

void drawCloudsTask2(bool waitSpin)
{
	viewer1.setBackgroundColor(.5, .5, .5);
	viewer1.addCoordinateSystem (1.0);
	viewer1.removeAllPointClouds();
	viewer1.removeAllShapes();

	/*viewer1.addPointCloud(cloud_environmentRemovalV1, "cloud_env_v1");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_env_v1");
	viewer1.addPointCloud(cloud_environmentRemovalV2, "cloud_env_v2");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_env_v2");
	viewer1.addPointCloud(cloud_v2_world, "world_v2");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "world_v2");
	viewer1.addPointCloud(cloud_v2_truck, "truck_v2");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "truck_v2");*/
	viewer1.addPointCloud(cloud_v1_world, "world_v1");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "world_v1");
	viewer1.addPointCloud(cloud_v1_truck, "truck_v1");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "truck_v1");
	viewer1.addPointCloud(cloud_v2_world_transformed, "world_v2_transformed");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "world_v2_transformed");
	viewer1.addPointCloud(cloud_v2_truck_transformed, "truck_v2_transformed");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "truck_v2_transformed");

	viewer1.addPointCloud(cloud_v2_border_right_accumulated_final, "border_right");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "border_right");
	viewer1.addPointCloud(cloud_v2_border_left_accumulated_final, "border_left");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "border_left");

	//Visualizacao das retas
	float t = 12;
	PointXYZ pontoInicio, pontoFim;

	//para visualizacao da reta esquerda
	pontoInicio.x = point_x1 + (-t)*vector_x1;
	pontoInicio.y = point_y1 + (-t)*vector_y1;
	pontoInicio.z = point_z1 + (-t)*vector_z1;
	pontoFim.x = point_x1 + (t)*vector_x1;
	pontoFim.y = point_y1 + (t)*vector_y1;
	pontoFim.z = point_z1 + (t)*vector_z1;
	viewer1.addLine (pontoInicio, pontoFim, 255, 255, 0, "line left", 0);
	viewer1.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "line left");

	//para visualizacao da reta direita
	pontoInicio.x = point_x2 + (-t)*vector_x2;
	pontoInicio.y = point_y2 + (-t)*vector_y2;
	pontoInicio.z = point_z2 + (-t)*vector_z2;
	pontoFim.x = point_x2 + (t)*vector_x2;
	pontoFim.y = point_y2 + (t)*vector_y2;
	pontoFim.z = point_z2 + (t)*vector_z2;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 255, "line right", 0);
	viewer1.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "line right");

	//para visualizacao da reta central
	pontoInicio.x = point_x3 + (-t)*vector_x3;
	pontoInicio.y = point_y3 + (-t)*vector_y3;
	pontoInicio.z = point_z3 + (-t)*vector_z3;
	pontoFim.x = point_x3 + (t)*vector_x3;
	pontoFim.y = point_y3 + (t)*vector_y3;
	pontoFim.z = point_z3 + (t)*vector_z3;
	viewer1.addLine (pontoInicio, pontoFim, 0, 0, 0, "line center", 0);
	viewer1.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "line center");
	//viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.25, "line center");

	//viewer1.addPointCloud(cloud_line_points, "line points");
	//viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "line points");


	if (waitSpin)
		viewer1.spin();
	else
		viewer1.spinOnce();
}

void drawCloudsTask3(bool waitSpin)
{
	viewer1.setBackgroundColor(.5, .5, .5);
	viewer1.addCoordinateSystem (1.0);
	viewer1.removeAllPointClouds();
	viewer1.removeAllShapes();

	viewer1.addPointCloud(cloud_v1_world, "world_v1");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "world_v1");
	viewer1.addPointCloud(cloud_v1_truck, "truck_v1");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "truck_v1");
	viewer1.addPointCloud(cloud_v2_world_transformed, "world_v2_transformed");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "world_v2_transformed");
	//viewer1.addPointCloud(cloud_v2_truck_transformed, "truck_v2_transformed");
	//viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "truck_v2_transformed");
	viewer1.addPointCloud(cloud_v2_truck_corrected, "truck_v2_corrected");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "truck_v2_corrected");
	if (projectionMethod == 2)
	{
		viewer1.addPointCloud(cloud_v1_truck_single_line, "truck_v1_single_line");
		viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "truck_v1_single_line");
	}
	//para visualizacao da reta central
	float t = 12;
	PointXYZ pontoInicio, pontoFim;
	pontoInicio.x = point_x3 + (-t)*vector_x3;
	pontoInicio.y = point_y3 + (-t)*vector_y3;
	pontoInicio.z = point_z3 + (-t)*vector_z3;
	pontoFim.x = point_x3 + (t)*vector_x3;
	pontoFim.y = point_y3 + (t)*vector_y3;
	pontoFim.z = point_z3 + (t)*vector_z3;
	viewer1.addLine (pontoInicio, pontoFim, 0, 0, 0, "line center", 0);
	viewer1.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "line center");
	viewer1.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.25, "line center");

	viewer1.addPointCloud(cloud_line_v1_projection, "projection v1");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "projection v1");
	viewer1.addPointCloud(cloud_mostDensePoints_v1_projection, "most dense points v1 projection");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "most dense points v1 projection");
	viewer1.addPointCloud(cloud_onlyMostDensePoint_v1, "most dense point v1");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "most dense point v1");
	viewer1.addPointCloud(cloud_mostDensePoints_v1_projection_correction_v2, "most dense points v2 correction");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "most dense points v2 correction");

	viewer1.addPointCloud(cloud_mountage_v1, "montagem v1");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "montagem v1");
	viewer1.addPointCloud(cloud_mountage_v2, "montagem v2");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "montagem v2");

	//viewer1.addPointCloud(cloud_emptyBucketLaser, "cacamba vazia laser");
	//viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cacamba vazia laser");
	viewer1.addPointCloud(cloud_emptyBucketLaserGICP, "cacamba vazia laser GICP");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cacamba vazia laser GICP");

	//desenha eixos
	viewer1.addPointCloud(cloud_bucketAxis, "eixo cacamba vazia");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "eixo cacamba vazia");
	viewer1.addPointCloud(cloud_mountageAxis, "eixo montagem");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "eixo montagem");
	pontoInicio.x = cloud_bucketAxis->points[0].x; pontoInicio.y = cloud_bucketAxis->points[0].y; pontoInicio.z = cloud_bucketAxis->points[0].z;
	pontoFim.x = cloud_bucketAxis->points[1].x; pontoFim.y = cloud_bucketAxis->points[1].y; pontoFim.z = cloud_bucketAxis->points[1].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 0, "eixo1", 0);
	pontoFim.x = cloud_bucketAxis->points[2].x; pontoFim.y = cloud_bucketAxis->points[2].y; pontoFim.z = cloud_bucketAxis->points[2].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 0, "eixo2", 0);
	pontoFim.x = cloud_bucketAxis->points[3].x; pontoFim.y = cloud_bucketAxis->points[3].y; pontoFim.z = cloud_bucketAxis->points[3].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 0, 255, "eixo3", 0);
	pontoInicio.x = cloud_mountageAxis->points[0].x; pontoInicio.y = cloud_mountageAxis->points[0].y; pontoInicio.z = cloud_mountageAxis->points[0].z;
	pontoFim.x = cloud_mountageAxis->points[1].x; pontoFim.y = cloud_mountageAxis->points[1].y; pontoFim.z = cloud_mountageAxis->points[1].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 255, 0, "eixo4", 0);
	pontoFim.x = cloud_mountageAxis->points[2].x; pontoFim.y = cloud_mountageAxis->points[2].y; pontoFim.z = cloud_mountageAxis->points[2].z;
	viewer1.addLine (pontoInicio, pontoFim, 255, 0, 0, "eixo5", 0);
	pontoFim.x = cloud_mountageAxis->points[3].x; pontoFim.y = cloud_mountageAxis->points[3].y; pontoFim.z = cloud_mountageAxis->points[3].z;
	viewer1.addLine (pontoInicio, pontoFim, 0, 0, 255, "eixo6", 0);

	//bordas
	//viewer1.addPointCloud(cloud_v2_border_right_transformed, "border_right");
	//viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "border_right");
	//viewer1.addPointCloud(cloud_v2_border_left_transformed, "border_left");
	//viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "border_left");
	viewer1.addPointCloud(cloud_v2_border_right_accumulated2, "border_right_accumulated");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "border_right_accumulated");
	viewer1.addPointCloud(cloud_v2_border_left_accumulated2, "border_left_accumulated");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "border_left_accumulated");
	viewer1.addPointCloud(cloud_lateral_points_to_gicp, "border_points");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "border_points");

	viewer1.addPlane (*planeCoefficients, "ground_plane", 0);
	viewer1.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1 /*R,G,B*/, "plane_1", 0);
	viewer1.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "plane_1", 0);
	viewer1.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "plane_1", 0);


	if (waitSpin)
		viewer1.spin();
	else
		viewer1.spinOnce();
}

void drawCloudsTask3_viewer2(bool waitSpin)
{
	viewer2.setBackgroundColor(255, 255, 255);
	viewer2.addCoordinateSystem (1.0);
	//viewer2.removeAllPointClouds();
	//viewer2.removeAllShapes();

	//viewer2.addPointCloud(cloud_mountage_v1v2_transformed, "montagem do volume");
	//viewer2.addPointCloud(cloud_mountage_v1_transformed, "montagem do volume");
	//viewer2.addPointCloud(cloud_mountage_v2_transformed, "montagem do volume");
	viewer2.addPointCloud(cloud_mountage_v2_internal, "parte interna do volume v2");

	viewer2.addPolygonMesh(meshBucket,"mesh_volume", 0);
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "mesh_volume");
	viewer2.addPolygonMesh(meshBucketSimplified,"mesh_volume_simplified", 0);
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "mesh_volume_simplified");

	viewer2.addPointCloud(cloud_floating_points, "pontos flutuantes");
	viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "pontos flutuantes");
	viewer2.spinOnce();

	if (waitSpin)
		viewer2.spin();
	else
		viewer2.spinOnce();
}

void drawCloudsTask3_viewer3(bool waitSpin)
{
	visualization::PCLVisualizer viewer3;
	//malha reconstruida
	viewer3.setBackgroundColor(255, 255, 255);
	//viewer3.addCoordinateSystem (3.0);
	viewer3.removeAllPointClouds();

	viewer3.addPolygonMesh(meshReconstructionFinal,"mesh_reconstruction",0);

	if (waitSpin)
		viewer3.spin();
	else
		viewer3.spinOnce();
}

void removePointsOutOfBucketModel()
{
	float radius = 0.075;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud_modelPoints, *cloud_modelPoints, indices);
	pcl::removeNaNFromPointCloud(*cloud_mountage_v2_transformed, *cloud_mountage_v2_transformed, indices);

	KdTreeFLANN<PointXYZRGB> kdtree_modelo;
	kdtree_modelo.setInputCloud (cloud_modelPoints);

	int acceptedByInternalProduct;
	float produtoInterno;
	PointXYZRGB searchPoint;

	for (unsigned int i = 0; i < cloud_mountage_v2_transformed->width; i++)
	{
		searchPoint.x = cloud_mountage_v2_transformed->points[i].x;
		searchPoint.y = cloud_mountage_v2_transformed->points[i].y;
		searchPoint.z = cloud_mountage_v2_transformed->points[i].z;

		acceptedByInternalProduct = 1;

		for (unsigned j = 0; j<meshPoligonsNormalsCacambaSimplified->points.size(); j+= 3)
		{
			//pontos que formam um triangulo de uma face (so preciso de um ponto)
			pcl::PointXYZ pt1(meshPoligonsNormalsCacambaSimplified->points[j].x,meshPoligonsNormalsCacambaSimplified->points[j].y,meshPoligonsNormalsCacambaSimplified->points[j].z);
			//pcl::PointXYZ pt2(meshPoligonsNormalsCacambaSimplified->points[j+1].x,meshPoligonsNormalsCacambaSimplified->points[j+1].y,meshPoligonsNormalsCacambaSimplified->points[j+1].z);
			//pcl::PointXYZ pt3(meshPoligonsNormalsCacambaSimplified->points[j+2].x,meshPoligonsNormalsCacambaSimplified->points[j+2].y,meshPoligonsNormalsCacambaSimplified->points[j+2].z);

			//obtem uma normal da face (as duas seguintes sao iguais pois pertencem a mesma face)
			Eigen::Vector3f faceNormal(meshPoligonsNormalsCacambaSimplified->points[j].normal_x, meshPoligonsNormalsCacambaSimplified->points[j].normal_y, meshPoligonsNormalsCacambaSimplified->points[j].normal_z);

			//obtenho o vetor que vai do ponto da nuvem de pontos ate um ponto do triangulo
			Eigen::Vector3f vectorPointToFace(meshPoligonsNormalsCacambaSimplified->points[j].x - searchPoint.x,
											  meshPoligonsNormalsCacambaSimplified->points[j].y - searchPoint.y,
											  meshPoligonsNormalsCacambaSimplified->points[j].z - searchPoint.z);

			produtoInterno = faceNormal[0] * vectorPointToFace[0] + faceNormal[1] * vectorPointToFace[1] + faceNormal[2] * vectorPointToFace[2];

			if (produtoInterno < 0)
			{
				acceptedByInternalProduct = 0;
				break;
			}
		}

		if (acceptedByInternalProduct && !(kdtree_modelo.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0))
		{
			cloud_mountage_v2_internal->width = cloud_mountage_v2_internal->width + 1;
			cloud_mountage_v2_internal->points.resize (cloud_mountage_v2_internal->width * cloud_mountage_v2_internal->height);

			cloud_mountage_v2_internal->points[cloud_mountage_v2_internal->width-1].x = cloud_mountage_v2_transformed->points[i].x;
			cloud_mountage_v2_internal->points[cloud_mountage_v2_internal->width-1].y = cloud_mountage_v2_transformed->points[i].y;
			cloud_mountage_v2_internal->points[cloud_mountage_v2_internal->width-1].z = cloud_mountage_v2_transformed->points[i].z;

			cloud_mountage_v2_internal->at(cloud_mountage_v2_internal->width-1).r = 0;
			cloud_mountage_v2_internal->at(cloud_mountage_v2_internal->width-1).g = 0;
			cloud_mountage_v2_internal->at(cloud_mountage_v2_internal->width-1).b = 0;
		}
	}
}

//funcoes utilitarias para criacao da base da cacamba
/* a = b - c */
#define vector(a, b, c) \
	(a)[0] = (b)[0] - (c)[0];	\
	(a)[1] = (b)[1] - (c)[1];	\
	(a)[2] = (b)[2] - (c)[2];

#define crossProduct(a,b,c) \
	(a)[0] = (b)[1] * (c)[2] - (c)[1] * (b)[2]; \
	(a)[1] = (b)[2] * (c)[0] - (c)[2] * (b)[0]; \
	(a)[2] = (b)[0] * (c)[1] - (c)[0] * (b)[1];

#define innerProduct(v,q) \
		((v)[0] * (q)[0] + \
		(v)[1] * (q)[1] + \
		(v)[2] * (q)[2])

void createBucketBasePoints()
{
	int qtdPoints = cloud_mountage_v2_internal->width;

	float p1[3], p2[3], p3[3], pEscolhido[3];
	float distP1, distP2, distP3;

	//uso sempre 3 pontos flutuantes
	p1[0] = cloud_floating_points->points[0].x;
	p1[1] = cloud_floating_points->points[0].y;
	p1[2] = cloud_floating_points->points[0].z;

	p2[0] = cloud_floating_points->points[1].x;
	p2[1] = cloud_floating_points->points[1].y;
	p2[2] = cloud_floating_points->points[1].z;

	p3[0] = cloud_floating_points->points[2].x;
	p3[1] = cloud_floating_points->points[2].y;
	p3[2] = cloud_floating_points->points[2].z;

	for (unsigned int j=0; j<qtdPoints; j++)
	{
		float d[3]; //vetor de direcao do ponto flutuante ate o ponto da montagem de volume

		//verifica o ponto flutuante de menor distancia
		distP1 = sqrt(pow(cloud_mountage_v2_internal->points[j].x - p1[0], 2) + pow(cloud_mountage_v2_internal->points[j].y - p1[1], 2) + pow(cloud_mountage_v2_internal->points[j].z - p1[2], 2));
		distP2 = sqrt(pow(cloud_mountage_v2_internal->points[j].x - p2[0], 2) + pow(cloud_mountage_v2_internal->points[j].y - p2[1], 2) + pow(cloud_mountage_v2_internal->points[j].z - p2[2], 2));
		distP3 = sqrt(pow(cloud_mountage_v2_internal->points[j].x - p3[0], 2) + pow(cloud_mountage_v2_internal->points[j].y - p3[1], 2) + pow(cloud_mountage_v2_internal->points[j].z - p3[2], 2));

		if (distP1 <= distP2 && distP1 <= distP3) //se esta mais proximo de p1, usa p1
		{
			pEscolhido[0] = p1[0];
			pEscolhido[1] = p1[1];
			pEscolhido[2] = p1[2];
		}
		else if (distP2 <= distP1 && distP2 <= distP3) //se esta mais proximo de p2, usa p2
		{
			pEscolhido[0] = p2[0];
			pEscolhido[1] = p2[1];
			pEscolhido[2] = p2[2];
		}
		else //senao, so sobra p3
		{
			pEscolhido[0] = p3[0];
			pEscolhido[1] = p3[1];
			pEscolhido[2] = p3[2];
		}

		d[0] = cloud_mountage_v2_internal->points[j].x - pEscolhido[0];
		d[1] = cloud_mountage_v2_internal->points[j].y - pEscolhido[1];
		d[2] = cloud_mountage_v2_internal->points[j].z - pEscolhido[2];

		for (unsigned k=0; k<meshPoligonsNormalsCacamba->points.size(); k+=3)
		{
			float v0[3], v1[3], v2[3];

			v0[0] = meshPoligonsNormalsCacamba->points[k].x;
			v0[1] = meshPoligonsNormalsCacamba->points[k].y;
			v0[2] = meshPoligonsNormalsCacamba->points[k].z;
			v1[0] = meshPoligonsNormalsCacamba->points[k+1].x;
			v1[1] = meshPoligonsNormalsCacamba->points[k+1].y;
			v1[2] = meshPoligonsNormalsCacamba->points[k+1].z;
			v2[0] = meshPoligonsNormalsCacamba->points[k+2].x;
			v2[1] = meshPoligonsNormalsCacamba->points[k+2].y;
			v2[2] = meshPoligonsNormalsCacamba->points[k+2].z;

			float e1[3],e2[3],h[3],s[3],q[3];
			float a,f,u,v;

			vector(e1, v1, v0);
			vector(e2, v2, v0);

			crossProduct(h,d,e2);
			a = innerProduct(e1,h);

			if (a > -0.00001 && a < 0.00001)
				continue;

			f = 1/a;
			vector(s,pEscolhido,v0);
			u = f * (innerProduct(s,h));

			if (u < 0.0 || u > 1.0)
				continue;

			crossProduct(q,s,e1);
			v = f * innerProduct(d,q);

			if (v < 0.0 || u + v > 1.0)
				continue;

			// at this stage we can compute t to find out where the intersection point is on the line
			float t = f * innerProduct(e2,q);

			if (t > 0.00001) // ray intersection
			{
				ponto.x = pEscolhido[0] + d[0] * t;
				ponto.y = pEscolhido[1] + d[1] * t;
				ponto.z = pEscolhido[2] + d[2] * t;

				ponto.r = 0;
				ponto.g = 0;
				ponto.b = 255;

				cloud_mountage_v2_internal->points.push_back(ponto);//push in points without normals
				cloud_mountage_v2_internal->width = cloud_mountage_v2_internal->width + 1;
				cloud_mountage_v2_internal->points.resize (cloud_mountage_v2_internal->width * cloud_mountage_v2_internal->height);

				//desenha a reta na janela de visualizacao
				PointXYZRGB pontoInicio;
				pontoInicio.x = pEscolhido[0];
				pontoInicio.y = pEscolhido[1];
				pontoInicio.z = pEscolhido[2];
				std::stringstream ss;
				ss<<"line"<<j;
				if (j%100 == 0)
				{
					viewer2.addLine (pontoInicio, ponto, 0, 0, 255, ss.str(), 0);
					viewer2.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.35, ss.str());
				}
			}
			else // this means that there is a line intersection but not a ray intersection
				continue;
		}
	}
}

void findLines() //RANSAC
{
	float distanceThreshold = 0.2;

	//ENCONTRA MELHOR RETA PARA PONTOS DA BORDA ESQUERDA

	pcl::ModelCoefficients mc_line_left;
	pcl::PointIndices::Ptr inliers_left(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distanceThreshold);
	seg.setInputCloud(cloud_v2_border_left_accumulated_final);
	seg.segment(*inliers_left, mc_line_left);

	//ponto na reta
	point_x1 = mc_line_left.values[0];
	point_y1 = mc_line_left.values[1];
	point_z1 = mc_line_left.values[2];
	//vetor direcao da reta
	vector_x1 = mc_line_left.values[3];
	vector_y1 = mc_line_left.values[4];
	vector_z1 = mc_line_left.values[5];

	//ENCONTRA MELHOR RETA PARA PONTOS DA BORDA DIREITA

	pcl::ModelCoefficients mc_line_right;
	pcl::PointIndices::Ptr inliers_right(new pcl::PointIndices);
	seg.setInputCloud(cloud_v2_border_right_accumulated_final);
	seg.segment(*inliers_right, mc_line_right);

	//ponto na reta
	point_x2 = mc_line_right.values[0];
	point_y2 = mc_line_right.values[1];
	point_z2 = mc_line_right.values[2];
	//vetor direcao da reta
	vector_x2 = mc_line_right.values[3];
	vector_y2 = mc_line_right.values[4];
	vector_z2 = mc_line_right.values[5];
}

//retorna a projecao ortogonal de um ponto do espaco em uma reta (parametrica)
//p1x, p1y, p1z: ponto a obter a projecao na reta
//p2x, p2y, p2z, v2x, v2y, v2z: ponto e vetor da reta parametrica
PointXYZRGB getPointProjectionOnLine(float p1x, float p1y, float p1z, float p2x, float p2y, float p2z, float v2x, float v2y, float v2z)
{
	float t;

	t = (v2x*p1x + v2y*p1y + v2z*p1z - (v2x*p2x + v2y*p2y + v2z*p2z)) / (v2x*v2x + v2y*v2y + v2z*v2z);

	PointXYZRGB pointProjection;
	pointProjection.x = p2x + v2x * t;
	pointProjection.y = p2y + v2y * t;
	pointProjection.z = p2z + v2z * t;

	return pointProjection;
}

void findCentralLine()
{
	//Primeiramente obtenho o ponto da projecao ortogonal de um dos pontos da reta da esquerda
	PointXYZRGB pointProjection;
	pointProjection = getPointProjectionOnLine(point_x1, point_y1, point_z1, point_x2, point_y2, point_z2, vector_x2, vector_y2, vector_z2);

	//calculo o ponto e vetor da reta central usando o ponto e vetor medio entre o ponto da reta da esquerda e o ortogonal a ele encontrado acima na reta da direita
	point_x3 = (point_x1 + pointProjection.x) / 2.0;
	point_y3 = (point_y1 + pointProjection.y) / 2.0;
	point_z3 = (point_z1 + pointProjection.z) / 2.0;
	vector_x3 = (vector_x1 + vector_x2) / 2.0;
	vector_y3 = (vector_y1 + vector_y2) / 2.0;
	vector_z3 = (vector_z1 + vector_z2) / 2.0;

	//normalizando vetor da reta central
	float v3_size = sqrt(vector_x3*vector_x3 + vector_y3*vector_y3 + vector_z3*vector_z3);
	vector_x3 = vector_x3 / v3_size;
	vector_y3 = vector_y3 / v3_size;
	vector_z3 = vector_z3 / v3_size;

	//nuvem existente apenas para verificar se a projecao e o ponto central encontrado esta ok
	cloud_line_points->width = 4;
	cloud_line_points->height = 1;
	cloud_line_points->is_dense = false;
	cloud_line_points->points.resize (cloud_line_points->width * cloud_line_points->height);
	cloud_line_points->points[0].x = point_x1;
	cloud_line_points->points[0].y = point_y1;
	cloud_line_points->points[0].z = point_z1;
	cloud_line_points->at(0).r = cloud_line_points->at(0).g = cloud_line_points->at(0).b = 255;
	cloud_line_points->points[1].x = pointProjection.x;
	cloud_line_points->points[1].y = pointProjection.y;
	cloud_line_points->points[1].z = pointProjection.z;
	cloud_line_points->at(1).r = cloud_line_points->at(1).g = cloud_line_points->at(1).b = 255;
	cloud_line_points->points[2].x = point_x3;
	cloud_line_points->points[2].y = point_y3;
	cloud_line_points->points[2].z = point_z3;
	cloud_line_points->at(2).r = cloud_line_points->at(2).g = cloud_line_points->at(2).b = 255;
	cloud_line_points->points[3].x = point_x3 + 1*vector_x3;
	cloud_line_points->points[3].y = point_y3 + 1*vector_y3;
	cloud_line_points->points[3].z = point_z3 + 1*vector_z3;
	cloud_line_points->at(3).r = cloud_line_points->at(3).g = cloud_line_points->at(3).b = 0;
}

void projectPointsOnCentralLine()
{
	PointXYZRGB pointProjection;

	if (projectionMethod == 1)
	{
		cloud_line_v1_projection->width = cloud_v1_truck->width;
		cloud_line_v1_projection->points.resize (cloud_line_v1_projection->width * cloud_line_v1_projection->height);

		for (unsigned int i=0; i<cloud_v1_truck->width; i++)
		{
			pointProjection = getPointProjectionOnLine(cloud_v1_truck->points[i].x, cloud_v1_truck->points[i].y, cloud_v1_truck->points[i].z,
													   point_x3, point_y3, point_z3, vector_x3, vector_y3, vector_z3);

			cloud_line_v1_projection->points[i].x = pointProjection.x;
			cloud_line_v1_projection->points[i].y = pointProjection.y;
			cloud_line_v1_projection->points[i].z = pointProjection.z;
			cloud_line_v1_projection->at(i).r = 255;
			cloud_line_v1_projection->at(i).g = 0;
			cloud_line_v1_projection->at(i).b = 0;
		}
	}
	else if (projectionMethod == 2)
	{
		cloud_line_v1_projection->width = cloud_v1_truck_single_line->width;
		cloud_line_v1_projection->points.resize (cloud_line_v1_projection->width * cloud_line_v1_projection->height);

		for (unsigned int i=0; i<cloud_v1_truck_single_line->width; i++)
		{
			pointProjection = getPointProjectionOnLine(cloud_v1_truck_single_line->points[i].x, cloud_v1_truck_single_line->points[i].y, cloud_v1_truck_single_line->points[i].z,
													   point_x3, point_y3, point_z3, vector_x3, vector_y3, vector_z3);

			cloud_line_v1_projection->points[i].x = pointProjection.x;
			cloud_line_v1_projection->points[i].y = pointProjection.y;
			cloud_line_v1_projection->points[i].z = pointProjection.z;
			cloud_line_v1_projection->at(i).r = 255;
			cloud_line_v1_projection->at(i).g = 0;
			cloud_line_v1_projection->at(i).b = 0;
		}
	}
}

void detectPointWithMostNeighbours()
{
	float radius = 0.15;

	PointXYZRGB point;
	KdTreeFLANN<PointXYZRGB> kdtree;
	kdtree.setInputCloud (cloud_line_v1_projection);

	int mostDensePoint = -1;
	float mostDensityValue = -1;
	float actualDensityValue = -1;

	for (unsigned int i=0; i<cloud_line_v1_projection->width; i++)
	{
		point.x = cloud_line_v1_projection->points[i].x;
		point.y = cloud_line_v1_projection->points[i].y;
		point.z = cloud_line_v1_projection->points[i].z;

		actualDensityValue = kdtree.radiusSearch (point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

		if (actualDensityValue > mostDensityValue)
		{
			mostDensePoint = i;
			mostDensityValue = actualDensityValue;
		}
	}

	cloud_mostDensePoints_v1_projection->width = cloud_mostDensePoints_v1_projection->width + 1;
	cloud_mostDensePoints_v1_projection->points.resize (cloud_mostDensePoints_v1_projection->width * cloud_mostDensePoints_v1_projection->height);

	cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].x = cloud_line_v1_projection->points[mostDensePoint].x;
	cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].y = cloud_line_v1_projection->points[mostDensePoint].y;
	cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].z = cloud_line_v1_projection->points[mostDensePoint].z;

	cloud_mostDensePoints_v1_projection->at(cloud_mostDensePoints_v1_projection->width-1).r = 0;
	cloud_mostDensePoints_v1_projection->at(cloud_mostDensePoints_v1_projection->width-1).g = 0;
	cloud_mostDensePoints_v1_projection->at(cloud_mostDensePoints_v1_projection->width-1).b = 0;

	if (cloud_mostDensePoints_v1_projection->width > 1)
	{
		cloud_mostDensePoints_v1_projection->at(cloud_mostDensePoints_v1_projection->width-2).r = 60;
		cloud_mostDensePoints_v1_projection->at(cloud_mostDensePoints_v1_projection->width-2).g = 60;
		cloud_mostDensePoints_v1_projection->at(cloud_mostDensePoints_v1_projection->width-2).b = 60;
	}

	//salva o ponto mais denso na reta, e o ponto escolhido na traseira do caminhao
	cloud_onlyMostDensePoint_v1->points[0].x = cloud_line_v1_projection->points[mostDensePoint].x;
	cloud_onlyMostDensePoint_v1->points[0].y = cloud_line_v1_projection->points[mostDensePoint].y;
	cloud_onlyMostDensePoint_v1->points[0].z = cloud_line_v1_projection->points[mostDensePoint].z;
	if (projectionMethod == 1)
	{
		cloud_onlyMostDensePoint_v1->points[1].x = cloud_v1_truck->points[mostDensePoint].x;
		cloud_onlyMostDensePoint_v1->points[1].y = cloud_v1_truck->points[mostDensePoint].y;
		cloud_onlyMostDensePoint_v1->points[1].z = cloud_v1_truck->points[mostDensePoint].z;
	}
	else if (projectionMethod == 2)
	{
		cloud_onlyMostDensePoint_v1->points[1].x = cloud_v1_truck_single_line->points[mostDensePoint].x;
		cloud_onlyMostDensePoint_v1->points[1].y = cloud_v1_truck_single_line->points[mostDensePoint].y;
		cloud_onlyMostDensePoint_v1->points[1].z = cloud_v1_truck_single_line->points[mostDensePoint].z;
	}


	//calcula a correcao para o velodyne 2, apenas a partir do segundo ponto. O ponto inicial é adicionado para facilitar as futuras operacoes de retorno das nuvens de V2 ao ponto inicial.

	if (cloud_mostDensePoints_v1_projection->width == 1)
	{
		cloud_mostDensePoints_v1_projection_correction_v2->points[0].x = cloud_mostDensePoints_v1_projection->points[0].x;
		cloud_mostDensePoints_v1_projection_correction_v2->points[0].y = cloud_mostDensePoints_v1_projection->points[0].y;
		cloud_mostDensePoints_v1_projection_correction_v2->points[0].z = cloud_mostDensePoints_v1_projection->points[0].z;
	}
	else if (cloud_mostDensePoints_v1_projection->width > 1)
	{
		cloud_mostDensePoints_v1_projection_correction_v2->width = cloud_mostDensePoints_v1_projection_correction_v2->width + 1;
		cloud_mostDensePoints_v1_projection_correction_v2->points.resize (cloud_mostDensePoints_v1_projection_correction_v2->width * cloud_mostDensePoints_v1_projection_correction_v2->height);

		cloud_mostDensePoints_v1_projection_correction_v2->points[cloud_mostDensePoints_v1_projection_correction_v2->width-1].x = ((1-taxaDePosicao)*cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-2].x + (taxaDePosicao)*cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].x);
		cloud_mostDensePoints_v1_projection_correction_v2->points[cloud_mostDensePoints_v1_projection_correction_v2->width-1].y = ((1-taxaDePosicao)*cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-2].y + (taxaDePosicao)*cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].y);
		cloud_mostDensePoints_v1_projection_correction_v2->points[cloud_mostDensePoints_v1_projection_correction_v2->width-1].z = ((1-taxaDePosicao)*cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-2].z + (taxaDePosicao)*cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].z);
	}
	cloud_mostDensePoints_v1_projection_correction_v2->at(cloud_mostDensePoints_v1_projection_correction_v2->width-1).r = 255;
	cloud_mostDensePoints_v1_projection_correction_v2->at(cloud_mostDensePoints_v1_projection_correction_v2->width-1).g = 0;
	cloud_mostDensePoints_v1_projection_correction_v2->at(cloud_mostDensePoints_v1_projection_correction_v2->width-1).b = 255;
}

void returnCloudsToOriginalDensePoint()
{
	PointXYZ vector;
	int previousSize;

	//MONTAGEM V1
	previousSize = cloud_mountage_v1->width;

	vector.x = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].x - cloud_mostDensePoints_v1_projection->points[0].x;
	vector.y = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].y - cloud_mostDensePoints_v1_projection->points[0].y;
	vector.z = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].z - cloud_mostDensePoints_v1_projection->points[0].z;

	cloud_mountage_v1->width = cloud_mountage_v1->width + cloud_v1_truck->width;
	cloud_mountage_v1->points.resize (cloud_mountage_v1->width * cloud_mountage_v1->height);

	for (unsigned int i=0; i<cloud_v1_truck->width; i++)
	{
		cloud_mountage_v1->points[previousSize+i].x = cloud_v1_truck->points[i].x - vector.x;
		cloud_mountage_v1->points[previousSize+i].y = cloud_v1_truck->points[i].y - vector.y;
		cloud_mountage_v1->points[previousSize+i].z = cloud_v1_truck->points[i].z - vector.z;

		cloud_mountage_v1->at(previousSize+i).r = 0;
		cloud_mountage_v1->at(previousSize+i).g = 0;
		cloud_mountage_v1->at(previousSize+i).b = 0;
	}

	//MONTAGEM V2
	if (cloud_mostDensePoints_v1_projection_correction_v2->width > 1)
	{
		previousSize = cloud_mountage_v2->width;

		vector.x = cloud_mostDensePoints_v1_projection_correction_v2->points[cloud_mostDensePoints_v1_projection_correction_v2->width-1].x - cloud_mostDensePoints_v1_projection_correction_v2->points[0].x;
		vector.y = cloud_mostDensePoints_v1_projection_correction_v2->points[cloud_mostDensePoints_v1_projection_correction_v2->width-1].y - cloud_mostDensePoints_v1_projection_correction_v2->points[0].y;
		vector.z = cloud_mostDensePoints_v1_projection_correction_v2->points[cloud_mostDensePoints_v1_projection_correction_v2->width-1].z - cloud_mostDensePoints_v1_projection_correction_v2->points[0].z;

		cloud_mountage_v2->width = cloud_mountage_v2->width + cloud_v2_truck_corrected->width;
		cloud_mountage_v2->points.resize (cloud_mountage_v2->width * cloud_mountage_v2->height);

		for (unsigned int i=0; i<cloud_v2_truck_corrected->width; i++)
		{
			cloud_mountage_v2->points[previousSize+i].x = cloud_v2_truck_corrected->points[i].x - vector.x;
			cloud_mountage_v2->points[previousSize+i].y = cloud_v2_truck_corrected->points[i].y - vector.y;
			cloud_mountage_v2->points[previousSize+i].z = cloud_v2_truck_corrected->points[i].z - vector.z;

			cloud_mountage_v2->at(previousSize+i).r = 60;
			cloud_mountage_v2->at(previousSize+i).g = 60;
			cloud_mountage_v2->at(previousSize+i).b = 60;
		}

		//Bordas do V2 acumuladas para criacao da cacamba vazia
		for (unsigned int i=0; i<cloud_v2_border_right_transformed->width; i++)
		{
			cloud_v2_border_right_transformed->points[i].x = cloud_v2_border_right_transformed->points[i].x - vector.x;
			cloud_v2_border_right_transformed->points[i].y = cloud_v2_border_right_transformed->points[i].y - vector.y;
			cloud_v2_border_right_transformed->points[i].z = cloud_v2_border_right_transformed->points[i].z - vector.z;
		}
		for (unsigned int i=0; i<cloud_v2_border_left_transformed->width; i++)
		{
			cloud_v2_border_left_transformed->points[i].x = cloud_v2_border_left_transformed->points[i].x - vector.x;
			cloud_v2_border_left_transformed->points[i].y = cloud_v2_border_left_transformed->points[i].y - vector.y;
			cloud_v2_border_left_transformed->points[i].z = cloud_v2_border_left_transformed->points[i].z - vector.z;
		}
	}
}

void removePointsByNeighbors()
{
	//nuvem de pontos auxiliar
	PointCloud<PointXYZRGB>::Ptr cloud_mountage_v2_internal_reduced (new PointCloud<PointXYZRGB>);
	cloud_mountage_v2_internal_reduced->width = 0;
	cloud_mountage_v2_internal_reduced->height = 1;
	cloud_mountage_v2_internal_reduced->is_dense = false;
	cloud_mountage_v2_internal_reduced->points.resize (cloud_mountage_v2_internal_reduced->width * cloud_mountage_v2_internal_reduced->height);

	//KD-TREE PARA ELIMINACAO DO MODELO
	KdTreeFLANN<PointXYZRGB> kdtree_vizinhos;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	kdtree_vizinhos.setInputCloud (cloud_mountage_v2_internal);

	PointXYZRGB searchPointVizinhos;
	int qtdVizinhos = 0;
	int qtdPontos = cloud_mountage_v2_internal->width;
	float media = 0.0;
	float radius = 0.30;
	float porcent_vizinhos = 0.15;
	float qtdVizinhosMinima = 0;

	for (unsigned int i=0; i<cloud_mountage_v2_internal->width; i++)
	{
		searchPointVizinhos.x = cloud_mountage_v2_internal->points[i].x;
		searchPointVizinhos.y = cloud_mountage_v2_internal->points[i].y;
		searchPointVizinhos.z = cloud_mountage_v2_internal->points[i].z;

		qtdVizinhos += kdtree_vizinhos.radiusSearch (searchPointVizinhos, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	}

	if (qtdPontos > 0)
	{
		media = (float)qtdVizinhos / qtdPontos;
	}

	qtdVizinhosMinima = media * porcent_vizinhos;

	for (unsigned int i=0; i<cloud_mountage_v2_internal->width; i++)
	{
		searchPointVizinhos.x = cloud_mountage_v2_internal->points[i].x;
		searchPointVizinhos.y = cloud_mountage_v2_internal->points[i].y;
		searchPointVizinhos.z = cloud_mountage_v2_internal->points[i].z;

		if(kdtree_vizinhos.radiusSearch (searchPointVizinhos, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > qtdVizinhosMinima)
		{
			cloud_mountage_v2_internal_reduced->width = cloud_mountage_v2_internal_reduced->width + 1;
			cloud_mountage_v2_internal_reduced->points.resize (cloud_mountage_v2_internal_reduced->width * cloud_mountage_v2_internal_reduced->height);

			cloud_mountage_v2_internal_reduced->points[cloud_mountage_v2_internal_reduced->width-1].x = searchPointVizinhos.x;
			cloud_mountage_v2_internal_reduced->points[cloud_mountage_v2_internal_reduced->width-1].y = searchPointVizinhos.y;
			cloud_mountage_v2_internal_reduced->points[cloud_mountage_v2_internal_reduced->width-1].z = searchPointVizinhos.z;

			//azul
			cloud_mountage_v2_internal_reduced->at(cloud_mountage_v2_internal_reduced->width-1).r = 0;
			cloud_mountage_v2_internal_reduced->at(cloud_mountage_v2_internal_reduced->width-1).g = 0;
			cloud_mountage_v2_internal_reduced->at(cloud_mountage_v2_internal_reduced->width-1).b = 255;
		}
	}

	//passa os pontos da nuvem auxiliar pra nuvem que utilizo
	cloud_mountage_v2_internal->width = cloud_mountage_v2_internal_reduced->width;
	cloud_mountage_v2_internal->points.resize (cloud_mountage_v2_internal->width * cloud_mountage_v2_internal->height);

	for (unsigned int i=0; i<cloud_mountage_v2_internal_reduced->width; i++)
	{
		cloud_mountage_v2_internal->points[i].x = cloud_mountage_v2_internal_reduced->points[i].x;
		cloud_mountage_v2_internal->points[i].y = cloud_mountage_v2_internal_reduced->points[i].y;
		cloud_mountage_v2_internal->points[i].z = cloud_mountage_v2_internal_reduced->points[i].z;
	}
}

void configBucketAxis()
{
	//calculando a linha central da cacamba:
	double P1[3], P2[3], P3[3], P4[3], CO[3];
	double Pmedio_P1P3[3], Pmedio_P2P4[3];

	P1[0] = cloud_bucketCorners->points[0].x; P1[1] = cloud_bucketCorners->points[0].y; P1[2] = cloud_bucketCorners->points[0].z;
	P2[0] = cloud_bucketCorners->points[1].x; P2[1] = cloud_bucketCorners->points[1].y; P2[2] = cloud_bucketCorners->points[1].z;
	P3[0] = cloud_bucketCorners->points[2].x; P3[1] = cloud_bucketCorners->points[2].y; P3[2] = cloud_bucketCorners->points[2].z;
	P4[0] = cloud_bucketCorners->points[3].x; P4[1] = cloud_bucketCorners->points[3].y; P4[2] = cloud_bucketCorners->points[3].z;

	Pmedio_P1P3[0] = (P1[0] + P3[0]) / 2.0;
	Pmedio_P1P3[1] = (P1[1] + P3[1]) / 2.0;
	Pmedio_P1P3[2] = (P1[2] + P3[2]) / 2.0;
	Pmedio_P2P4[0] = (P2[0] + P4[0]) / 2.0;
	Pmedio_P2P4[1] = (P2[1] + P4[1]) / 2.0;
	Pmedio_P2P4[2] = (P2[2] + P4[2]) / 2.0;

	//Calcula o vetor V1 (x) de direcao da cacamba e normaliza ele
	V1[0] = Pmedio_P2P4[0] - Pmedio_P1P3[0];
	V1[1] = Pmedio_P2P4[1] - Pmedio_P1P3[1];
	V1[2] = Pmedio_P2P4[2] - Pmedio_P1P3[2];
	double V1_norm = sqrt(V1[0]*V1[0] + V1[1]*V1[1] + V1[2]*V1[2]);
	V1[0] = V1[0] / V1_norm;
	V1[1] = V1[1] / V1_norm;
	V1[2] = V1[2] / V1_norm;

	//Calcula o vetor V3 (z) de direcao da cacamba fazendo o produto vetorial de V1 e V2, e normaliza ele tbm
	CO[0] = P3[0] - Pmedio_P1P3[0];
	CO[1] = P3[1] - Pmedio_P1P3[1];
	CO[2] = P3[2] - Pmedio_P1P3[2];
	crossProduct(V3, CO, V1);
	double V3_norm = sqrt(V3[0]*V3[0] + V3[1]*V3[1] + V3[2]*V3[2]);
	V3[0] = V3[0] / V3_norm;
	V3[1] = V3[1] / V3_norm;
	V3[2] = V3[2] / V3_norm;

	//Calcula o vetor V2 (y) de direcao da cacamba e normaliza ele - para calcular ele, projeta o ponto medio de P1 e P3 na reta formada por P3 e P4
	crossProduct(V2, V1, V3);
	double V2_norm = sqrt(V2[0]*V2[0] + V2[1]*V2[1] + V2[2]*V2[2]);
	V2[0] = V2[0] / V2_norm;
	V2[1] = V2[1] / V2_norm;
	V2[2] = V2[2] / V2_norm;

	//Apenas para renomear a origem do sistema de coordenadas do modelo
	O1[0]= Pmedio_P1P3[0];
	O1[1]= Pmedio_P1P3[1];
	O1[2]= Pmedio_P1P3[2];

	cloud_bucketAxis->points[0].x = O1[0];
	cloud_bucketAxis->points[0].y = O1[1];
	cloud_bucketAxis->points[0].z = O1[2];

	cloud_bucketAxis->points[1].x = V1[0] + O1[0];
	cloud_bucketAxis->points[1].y = V1[1] + O1[1];
	cloud_bucketAxis->points[1].z = V1[2] + O1[2];

	cloud_bucketAxis->points[2].x = V2[0] + O1[0];
	cloud_bucketAxis->points[2].y = V2[1] + O1[1];
	cloud_bucketAxis->points[2].z = V2[2] + O1[2];

	cloud_bucketAxis->points[3].x = V3[0] + O1[0];
	cloud_bucketAxis->points[3].y = V3[1] + O1[1];
	cloud_bucketAxis->points[3].z = V3[2] + O1[2];

	cloud_bucketAxis->at(0).r = 255;
	cloud_bucketAxis->at(0).g = 255;
	cloud_bucketAxis->at(0).b = 255;

	cloud_bucketAxis->at(1).r = 0;
	cloud_bucketAxis->at(1).g = 255;
	cloud_bucketAxis->at(1).b = 0;

	cloud_bucketAxis->at(2).r = 255;
	cloud_bucketAxis->at(2).g = 0;
	cloud_bucketAxis->at(2).b = 0;

	cloud_bucketAxis->at(3).r = 0;
	cloud_bucketAxis->at(3).g = 0;
	cloud_bucketAxis->at(3).b = 255;
}

void configMountageAxis()
{
	double CO[3];

	O2[0] = cloud_mostDensePoints_v1_projection->points[0].x;
	O2[1] = cloud_mostDensePoints_v1_projection->points[0].y;
	O2[2] = cloud_mostDensePoints_v1_projection->points[0].z;

	//Calcula o vetor V4 (x) de direcao do caminho do caminhao e normaliza ele
	V4[0] = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].x - O2[0];
	V4[1] = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].y - O2[1];
	V4[2] = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].z - O2[2];
	double V4_norm = sqrt(V4[0]*V4[0] + V4[1]*V4[1] + V4[2]*V4[2]);
	V4[0] = V4[0] / V4_norm;
	V4[1] = V4[1] / V4_norm;
	V4[2] = V4[2] / V4_norm;

	//Calcula o vetor V6 (z) de direcao da montagem fazendo o produto vetorial de V4 e V5, e normaliza ele tbm
	CO[0] = point_x2 - O2[0];
	CO[1] = point_y2 - O2[1];
	CO[2] = point_z2 - O2[2];
	crossProduct(V6, CO, V4);
	double V6_norm = sqrt(V6[0]*V6[0] + V6[1]*V6[1] + V6[2]*V6[2]);
	V6[0] = V6[0] / V6_norm;
	V6[1] = V6[1] / V6_norm;
	V6[2] = V6[2] / V6_norm;

	//Calcula o vetor V5 (y) de direcao da cacamba e normaliza ele - para calcular ele, projeta o ponto mais denso original 1 e P3 na reta formada por P3 e P4
	crossProduct(V5, V4, V6);
	double V5_norm = sqrt(V5[0]*V5[0] + V5[1]*V5[1] + V5[2]*V5[2]);
	V5[0] = V5[0] / V5_norm;
	V5[1] = V5[1] / V5_norm;
	V5[2] = V5[2] / V5_norm;

	cloud_mountageAxis->points[0].x = O2[0];
	cloud_mountageAxis->points[0].y = O2[1];
	cloud_mountageAxis->points[0].z = O2[2];

	cloud_mountageAxis->points[1].x = V4[0] + O2[0];
	cloud_mountageAxis->points[1].y = V4[1] + O2[1];
	cloud_mountageAxis->points[1].z = V4[2] + O2[2];

	cloud_mountageAxis->points[2].x = V5[0] + O2[0];
	cloud_mountageAxis->points[2].y = V5[1] + O2[1];
	cloud_mountageAxis->points[2].z = V5[2] + O2[2];

	cloud_mountageAxis->points[3].x = V6[0] + O2[0];
	cloud_mountageAxis->points[3].y = V6[1] + O2[1];
	cloud_mountageAxis->points[3].z = V6[2] + O2[2];

	cloud_mountageAxis->at(0).r = 255;
	cloud_mountageAxis->at(0).g = 255;
	cloud_mountageAxis->at(0).b = 255;

	cloud_mountageAxis->at(1).r = 0;
	cloud_mountageAxis->at(1).g = 255;
	cloud_mountageAxis->at(1).b = 0;

	cloud_mountageAxis->at(2).r = 255;
	cloud_mountageAxis->at(2).g = 0;
	cloud_mountageAxis->at(2).b = 0;

	cloud_mountageAxis->at(3).r = 0;
	cloud_mountageAxis->at(3).g = 0;
	cloud_mountageAxis->at(3).b = 255;
}

void configTransformationMatrix()
{
	Matrix<double, 4, 4> T_MONTAGEM, T_CACAMBA;

	T_CACAMBA << V1[0],	V2[0],	V3[0],	O1[0],
	  		     V1[1],	V2[1],	V3[1], 	O1[1],
				 V1[2],	V2[2], 	V3[2], 	O1[2],
				 0,		0, 			0,	1;

	T_MONTAGEM << V4[0],	V5[0],	V6[0],	O2[0],
	  		      V4[1],	V5[1],	V6[1], 	O2[1],
				  V4[2],	V5[2], 	V6[2], 	O2[2],
				  0,		0, 		0,		1;

	transformation_bucketLaser_to_bucketMountage =  T_MONTAGEM * T_CACAMBA.inverse();

	//transformPointCloud(*cloud_emptyBucketLaser, *cloud_emptyBucketLaser, transformation_bucketLaser_to_bucketMountage);
	transformPointCloud(*cloud_bucketAxis, *cloud_bucketAxis, transformation_bucketLaser_to_bucketMountage);
	transformPointCloud<PointXYZRGB>(*cloud_emptyBucketLaserGICP, *cloud_emptyBucketLaserGICP, transformation_bucketLaser_to_bucketMountage);
	//transformPointCloud<PointXYZRGB>(*cloud_bucketCorners, *cloud_bucketCorners, transformation_bucketLaser_to_bucketMountage);
	//transformPointCloud(*cloud_mountageAxis, *cloud_mountageAxis, TRANSFORMACAO);

}

void configAxis()
{
	configBucketAxis();
	configMountageAxis();
}

void saveEnvironmentPoints()
{
	int size;

	if (velodyne_number == 3/*1*/)
	{
		if (numberOfMessagesV1 > 0)
		{
			size = cloud_environmentRemovalV1->width;

			cloud_environmentRemovalV1->width = cloud_environmentRemovalV1->width + cloud_v1_world->width;
			cloud_environmentRemovalV1->points.resize (cloud_environmentRemovalV1->width * cloud_environmentRemovalV1->height);

			for (unsigned int i=0; i<cloud_v1_world->size(); i++)
			{
				cloud_environmentRemovalV1->points[size+i].x = cloud_v1_world->points[i].x;
				cloud_environmentRemovalV1->points[size+i].y = cloud_v1_world->points[i].y;
				cloud_environmentRemovalV1->points[size+i].z = cloud_v1_world->points[i].z;
			}

			numberOfMessagesV1--;
		}
	}
	else if (velodyne_number == 2)
	{
		if (numberOfMessagesV2 > 0)
		{
			size = cloud_environmentRemovalV2->width;

			cloud_environmentRemovalV2->width = cloud_environmentRemovalV2->width + cloud_v2_world_transformed->width;
			cloud_environmentRemovalV2->points.resize (cloud_environmentRemovalV2->width * cloud_environmentRemovalV2->height);

			for (unsigned int i=0; i<cloud_v2_world_transformed->size(); i++)
			{
				cloud_environmentRemovalV2->points[size+i].x = cloud_v2_world_transformed->points[i].x;
				cloud_environmentRemovalV2->points[size+i].y = cloud_v2_world_transformed->points[i].y;
				cloud_environmentRemovalV2->points[size+i].z = cloud_v2_world_transformed->points[i].z;
			}

			numberOfMessagesV2--;
		}
	}

	if (numberOfMessagesV1 == 0 && numberOfMessagesV2 == 0)
	{
		exportCloudEnvironmentV1();
		exportCloudEnvironmentV2();

		flagSaveEnvironment = false;
	}
}

void correctV2PointCloud()
{
	double vertical_correction[32] = { -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
									   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	double x, y, z, v_angle, range, h_angle;

	cloud_v2_truck_corrected->width = 0;
	cloud_v2_truck_corrected->points.resize (cloud_v2_truck_corrected->width * cloud_v2_truck_corrected->height);

	for (int k = 0; k < velodyne_variable_scan2.number_of_shots; k++)
	{
		for (int l = 0; l < velodyne_variable_scan2.partial_scan->shot_size; l++)
		{
			v_angle = (carmen_degrees_to_radians(vertical_correction[l]));
			range = ((double)velodyne_variable_scan2.partial_scan[k].distance[l]);
			h_angle = (carmen_degrees_to_radians(velodyne_variable_scan2.partial_scan[k].angle));

			x = (range * cos(v_angle) * cos(h_angle)) / 500.0;
			y = (range * cos(v_angle) * sin(h_angle)) / 500.0;
			z = (range * sin(v_angle)) / 500.0;

			if (x != 0.0 && y != 0.0 && z != 0.0)
			{
				searchPoint.x = x;
				searchPoint.y = y;
				searchPoint.z = z;

				if (!(kdtree_environmentV2.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0))
				{
					//inicio da correcao da nuvem de v2
					if(cloud_mostDensePoints_v1_projection->width > 1)
					{
						PointXYZ p1, p2, vectorVM, X2, X1;
						p1.x = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-2].x;
						p1.y = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-2].y;
						p1.z = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-2].z;
						p2.x = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].x;
						p2.y = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].y;
						p2.z = cloud_mostDensePoints_v1_projection->points[cloud_mostDensePoints_v1_projection->width-1].z;

						float dt = (playback_timestamp_v1 - playback_timestamp_v1_ant); //tempo

						vectorVM.x = (p2.x-p1.x)/dt;
						vectorVM.y = (p2.y-p1.y)/dt;
						vectorVM.z = (p2.z-p1.z)/dt;

						cloud_v2_truck_corrected->width = cloud_v2_truck_corrected->width + 1;
						cloud_v2_truck_corrected->points.resize (cloud_v2_truck_corrected->width * cloud_v2_truck_corrected->height);

						pointToTransform->points[0].x = x;
						pointToTransform->points[0].y = y;
						pointToTransform->points[0].z = z;
						transformPointCloud<PointXYZRGB>(*pointToTransform, *pointToTransform, final_transformation);

						X1.x = pointToTransform->points[0].x;
						X1.y = pointToTransform->points[0].y;
						X1.z = pointToTransform->points[0].z;

						X2.x = X1.x + vectorVM.x * dt;
						X2.y = X1.y + vectorVM.y * dt;
						X2.z = X1.z + vectorVM.z * dt;

						float t = k/(float)velodyne_variable_scan2.number_of_shots;

						cloud_v2_truck_corrected->points[cloud_v2_truck_corrected->width-1].x = X1.x - (X2.x-X1.x) * t;
						cloud_v2_truck_corrected->points[cloud_v2_truck_corrected->width-1].y = X1.y - (X2.y-X1.y) * t;
						cloud_v2_truck_corrected->points[cloud_v2_truck_corrected->width-1].z = X1.z - (X2.z-X1.z) * t;

						cloud_v2_truck_corrected->at(cloud_v2_truck_corrected->width-1).r = 0;
						cloud_v2_truck_corrected->at(cloud_v2_truck_corrected->width-1).g = 255;
						cloud_v2_truck_corrected->at(cloud_v2_truck_corrected->width-1).b = 0;
					}
				}
			}
		}
	}
}

float signedVolumeOfTriangle(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3)
{
    float v321 = p3.x*p2.y*p1.z;
    float v231 = p2.x*p3.y*p1.z;
    float v312 = p3.x*p1.y*p2.z;
    float v132 = p1.x*p3.y*p2.z;
    float v213 = p2.x*p1.y*p3.z;
    float v123 = p1.x*p2.y*p3.z;
    return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
}

float volumeOfMesh(pcl::PolygonMesh mesh)
{
    float vols = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud,*cloud);
    for(int triangle=0;triangle<mesh.polygons.size();triangle++)
    {
        pcl::PointXYZ pt1 = cloud->points[mesh.polygons[triangle].vertices[0]];
        pcl::PointXYZ pt2 = cloud->points[mesh.polygons[triangle].vertices[1]];
        pcl::PointXYZ pt3 = cloud->points[mesh.polygons[triangle].vertices[2]];
        vols += signedVolumeOfTriangle(pt1, pt2, pt3);
    }
    return fabs(vols);
}

void calculateMeshVolume()
{
	pcl::PolygonMesh mesh; //carrego a malha novamente do arquivo salvo só para facilitar testes

	pcl::io::loadPolygonFileOBJ("/dados/montagem_malha.obj", mesh);

	float meshVolume = volumeOfMesh(mesh);

	printf("MESH VOLUME: %f\n", meshVolume);
}

void removeIsolatedPieces()
{
	//pcl::io::loadPolygonFileOBJ("/dados/montagem_malha.obj", meshTeste);
	vtkSmartPointer<vtkPolyData> meshPolydata = vtkSmartPointer<vtkPolyData>::New();
	VTKUtils::convertToVTK (meshReconstruction, meshPolydata);

	vtkSmartPointer<vtkPolyData> pointsPolydataExtracted = vtkSmartPointer<vtkPolyData>::New();

	//para manter apenas a maior regiao conectada

	vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivityFilter = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
	connectivityFilter->SetInput(meshPolydata);

	connectivityFilter->SetExtractionModeToLargestRegion();
	connectivityFilter->Update();
	cout << "PolyData has " << connectivityFilter->GetNumberOfExtractedRegions() << " regions.\n";

	pointsPolydataExtracted = connectivityFilter->GetOutput();

	//------------- PARA VISUALIZAR AS REGIOES EXCLUIDAS (Visualizador da VTK) -------------//

	// Create a mapper and actor for original data
	vtkSmartPointer<vtkPolyDataMapper> originalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	originalMapper->SetInput(meshPolydata);
	originalMapper->Update();

	vtkSmartPointer<vtkActor> originalActor = vtkSmartPointer<vtkActor>::New();
	originalActor->GetProperty()->SetColor(1,0,0);
	originalActor->SetMapper(originalMapper);

	// Create a mapper and actor for extracted data
	vtkSmartPointer<vtkPolyDataMapper> extractedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	extractedMapper->SetInputConnection(connectivityFilter->GetOutputPort());
	extractedMapper->Update();

	vtkSmartPointer<vtkActor> extractedActor = vtkSmartPointer<vtkActor>::New();
	extractedActor->SetMapper(extractedMapper);

	// Visualization
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(originalActor);
	renderer->AddActor(extractedActor);

	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	interactor->SetRenderWindow(renderWindow);
	interactor->Initialize();
	interactor->Start();

	//----------------------------------------------------------------//

	VTKUtils::convertToPCL(pointsPolydataExtracted, meshReconstruction);

	/*visualization::PCLVisualizer viewerTeste;
	viewerTeste.setBackgroundColor(255, 255, 255);
	viewerTeste.addPolygonMesh(meshReconstruction, "meshReconstruction", 0);
	viewerTeste.spin();*/
}

void cleanPolyData()
{
	//pcl::io::loadPolygonFileOBJ("/dados/montagem_malha.obj", meshTeste);
	vtkSmartPointer<vtkPolyData> meshPolydata = vtkSmartPointer<vtkPolyData>::New();
	VTKUtils::convertToVTK (meshReconstruction, meshPolydata);

	vtkSmartPointer<vtkPolyData> pointsPolydataExtracted = vtkSmartPointer<vtkPolyData>::New();

	std::cout << "Input mesh has " << meshPolydata->GetNumberOfPoints() << " vertices." << std::endl;
	std::cout << "Input mesh has " << meshPolydata->GetNumberOfPolys() << " polys." << std::endl;
	std::cout << "Input mesh has " << meshPolydata->GetNumberOfLines() << " lines." << std::endl;

	//limpa a malha, unindo vertices
	vtkSmartPointer<vtkCleanPolyData> cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
	cleaner->SetInput(meshPolydata);
	cleaner->Update();

	std::cout << "Cleaned mesh has " << cleaner->GetOutput()->GetNumberOfPoints() << " vertices." << std::endl;
	std::cout << "Cleaned mesh has " << cleaner->GetOutput()->GetNumberOfPolys() << " polys." << std::endl;
	std::cout << "Cleaned mesh has " << cleaner->GetOutput()->GetNumberOfLines() << " lines." << std::endl;

	pointsPolydataExtracted = cleaner->GetOutput();

	//----------------------------------------------------------------//

	VTKUtils::convertToPCL(pointsPolydataExtracted, meshReconstruction);

	/*visualization::PCLVisualizer viewerTeste;
	viewerTeste.setBackgroundColor(255, 255, 255);
	viewerTeste.addPolygonMesh(meshReconstruction, "meshReconstruction", 0);
	viewerTeste.spin();*/
}

void massProperties()
{
	vtkSmartPointer<vtkPolyData> meshPolydata = vtkSmartPointer<vtkPolyData>::New();
	VTKUtils::convertToVTK (meshReconstruction, meshPolydata);

	vtkSmartPointer<vtkMassProperties> massProperties = vtkSmartPointer<vtkMassProperties>::New();
	massProperties->SetInput(meshPolydata);
	massProperties->Update();
	std::cout << "Volume: " << massProperties->GetVolume() << std::endl
	          << "    VolumeX: " << massProperties->GetVolumeX() << std::endl
	          << "    VolumeY: " << massProperties->GetVolumeY() << std::endl
	          << "    VolumeZ: " << massProperties->GetVolumeZ() << std::endl
	          << "Area:   " << massProperties->GetSurfaceArea() << std::endl
	          << "    MinCellArea: " << massProperties->GetMinCellArea() << std::endl
	          << "    MinCellArea: " << massProperties->GetMaxCellArea() << std::endl
	          << "NormalizedShapeIndex: " << massProperties->GetNormalizedShapeIndex() << std::endl;
}

void fillHoles()
{
	vtkSmartPointer<vtkPolyData> meshPolydata = vtkSmartPointer<vtkPolyData>::New();
	VTKUtils::convertToVTK (meshReconstruction, meshPolydata);

	vtkSmartPointer<vtkFillHolesFilter> fillHolesFilter = vtkSmartPointer<vtkFillHolesFilter>::New();
	fillHolesFilter->SetInput(meshPolydata);
	fillHolesFilter->SetHoleSize(0.3);
	fillHolesFilter->Update();

	vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
	triangleFilter->SetInputConnection(fillHolesFilter->GetOutputPort());
	triangleFilter->Update();

	// Make the triangle windong order consistent
	vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
	normals->SetInputConnection(triangleFilter->GetOutputPort());
	normals->ConsistencyOn();
	normals->SplittingOff();
	normals->Update();

	vtkSmartPointer<vtkPolyData> pointsPolydataExtracted = vtkSmartPointer<vtkPolyData>::New();
	pointsPolydataExtracted = normals->GetOutput();

	VTKUtils::convertToPCL(pointsPolydataExtracted, meshReconstruction);
}

void featureEdges()
{
	vtkSmartPointer<vtkPolyData> meshPolydata = vtkSmartPointer<vtkPolyData>::New();
	VTKUtils::convertToVTK (meshReconstruction, meshPolydata);

	vtkSmartPointer<vtkFeatureEdges> featureEdges = vtkSmartPointer<vtkFeatureEdges>::New();
	featureEdges->SetInput(meshPolydata);
	featureEdges->BoundaryEdgesOn();
	featureEdges->FeatureEdgesOn();
	featureEdges->ManifoldEdgesOn();
	featureEdges->NonManifoldEdgesOff();
	featureEdges->Update();

	vtkSmartPointer<vtkPolyData> pointsPolydataExtracted = vtkSmartPointer<vtkPolyData>::New();
	pointsPolydataExtracted = featureEdges->GetOutput();

	VTKUtils::convertToPCL(pointsPolydataExtracted, meshReconstruction);
}

void accumulateBorders2()
{
	int previousSize;

	previousSize = cloud_v2_border_right_accumulated2->width;

	cloud_v2_border_right_accumulated2->width = cloud_v2_border_right_accumulated2->width + cloud_v2_border_right_transformed->width;
	cloud_v2_border_right_accumulated2->points.resize (cloud_v2_border_right_accumulated2->width * cloud_v2_border_right_accumulated2->height);

	for (unsigned int i=0; i<cloud_v2_border_right_transformed->width; i++)
	{
		cloud_v2_border_right_accumulated2->points[previousSize+i].x = cloud_v2_border_right_transformed->points[i].x;
		cloud_v2_border_right_accumulated2->points[previousSize+i].y = cloud_v2_border_right_transformed->points[i].y;
		cloud_v2_border_right_accumulated2->points[previousSize+i].z = cloud_v2_border_right_transformed->points[i].z;

		cloud_v2_border_right_accumulated2->at(previousSize+i).r = cloud_v2_border_right_transformed->at(i).r;
		cloud_v2_border_right_accumulated2->at(previousSize+i).g = cloud_v2_border_right_transformed->at(i).g;
		cloud_v2_border_right_accumulated2->at(previousSize+i).b = cloud_v2_border_right_transformed->at(i).b;
	}

	previousSize = cloud_v2_border_left_accumulated2->width;

	cloud_v2_border_left_accumulated2->width = cloud_v2_border_left_accumulated2->width + cloud_v2_border_left_transformed->width;
	cloud_v2_border_left_accumulated2->points.resize (cloud_v2_border_left_accumulated2->width * cloud_v2_border_left_accumulated2->height);

	for (unsigned int i=0; i<cloud_v2_border_left_transformed->width; i++)
	{
		cloud_v2_border_left_accumulated2->points[previousSize+i].x = cloud_v2_border_left_transformed->points[i].x;
		cloud_v2_border_left_accumulated2->points[previousSize+i].y = cloud_v2_border_left_transformed->points[i].y;
		cloud_v2_border_left_accumulated2->points[previousSize+i].z = cloud_v2_border_left_transformed->points[i].z;

		cloud_v2_border_left_accumulated2->at(previousSize+i).r = cloud_v2_border_left_transformed->at(i).r;
		cloud_v2_border_left_accumulated2->at(previousSize+i).g = cloud_v2_border_left_transformed->at(i).g;
		cloud_v2_border_left_accumulated2->at(previousSize+i).b = cloud_v2_border_left_transformed->at(i).b;
	}
}

void generateLateralPointCloud()
{
	//agrupa as nuvens das bordas direita e esquerda
	for (unsigned int i=0; i<cloud_v2_border_right_accumulated2->width; i++)
	{
		cloud_lateral_borders->width = cloud_lateral_borders->width + 1;
		cloud_lateral_borders->points.resize (cloud_lateral_borders->width * cloud_lateral_borders->height);

		cloud_lateral_borders->points[cloud_lateral_borders->width-1].x = cloud_v2_border_right_accumulated2->points[i].x;
		cloud_lateral_borders->points[cloud_lateral_borders->width-1].y = cloud_v2_border_right_accumulated2->points[i].y;
		cloud_lateral_borders->points[cloud_lateral_borders->width-1].z = cloud_v2_border_right_accumulated2->points[i].z;
	}

	for (unsigned int i=0; i<cloud_v2_border_left_accumulated2->width; i++)
	{
		cloud_lateral_borders->width = cloud_lateral_borders->width + 1;
		cloud_lateral_borders->points.resize (cloud_lateral_borders->width * cloud_lateral_borders->height);

		cloud_lateral_borders->points[cloud_lateral_borders->width-1].x = cloud_v2_border_left_accumulated2->points[i].x;
		cloud_lateral_borders->points[cloud_lateral_borders->width-1].y = cloud_v2_border_left_accumulated2->points[i].y;
		cloud_lateral_borders->points[cloud_lateral_borders->width-1].z = cloud_v2_border_left_accumulated2->points[i].z;
	}

	float cutHeight = -0.40;

	//pega os pontos de V2 próximos aos pontos das laterais e adiciona na nuvem lateral
	KdTreeFLANN<PointXYZRGB> kdtree_lateralBorders;
	kdtree_lateralBorders.setInputCloud (cloud_lateral_borders);

	float radius = 0.15;

	for (unsigned int i=0; i<cloud_mountage_v2->width; i++)
	{
		//if (cloud_mountage_v2->points[i].z > cutHeight) //adicionar o plano de altura aqui
		//{
			searchPoint.x = cloud_mountage_v2->points[i].x;
			searchPoint.y = cloud_mountage_v2->points[i].y;
			searchPoint.z = cloud_mountage_v2->points[i].z;

			if ((kdtree_lateralBorders.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0))
			{
				cloud_lateral_points_to_gicp->width = cloud_lateral_points_to_gicp->width + 1;
				cloud_lateral_points_to_gicp->points.resize (cloud_lateral_points_to_gicp->width * cloud_lateral_points_to_gicp->height);

				cloud_lateral_points_to_gicp->points[cloud_lateral_points_to_gicp->width-1].x = searchPoint.x;
				cloud_lateral_points_to_gicp->points[cloud_lateral_points_to_gicp->width-1].y = searchPoint.y;
				cloud_lateral_points_to_gicp->points[cloud_lateral_points_to_gicp->width-1].z = searchPoint.z;

				cloud_lateral_points_to_gicp->at(cloud_lateral_points_to_gicp->width-1).r = 255;
				cloud_lateral_points_to_gicp->at(cloud_lateral_points_to_gicp->width-1).g = 0;
				cloud_lateral_points_to_gicp->at(cloud_lateral_points_to_gicp->width-1).b = 0;
			}
		//}
	}

	//adiciona pontos laterais da montagem de V1:

	for (unsigned int i=0; i<cloud_mountage_v1->width; i++)
	{
		//if (cloud_mountage_v1->points[i].z > cutHeight) //adicionar o plano de altura aqui
		//{
			cloud_lateral_points_to_gicp->width = cloud_lateral_points_to_gicp->width + 1;
			cloud_lateral_points_to_gicp->points.resize (cloud_lateral_points_to_gicp->width * cloud_lateral_points_to_gicp->height);

			cloud_lateral_points_to_gicp->points[cloud_lateral_points_to_gicp->width-1].x = cloud_mountage_v1->points[i].x;
			cloud_lateral_points_to_gicp->points[cloud_lateral_points_to_gicp->width-1].y = cloud_mountage_v1->points[i].y;
			cloud_lateral_points_to_gicp->points[cloud_lateral_points_to_gicp->width-1].z = cloud_mountage_v1->points[i].z;

			cloud_lateral_points_to_gicp->at(cloud_lateral_points_to_gicp->width-1).r = 255;
			cloud_lateral_points_to_gicp->at(cloud_lateral_points_to_gicp->width-1).g = 0;
			cloud_lateral_points_to_gicp->at(cloud_lateral_points_to_gicp->width-1).b = 0;
		//}
	}



}

void exportLateralPointCloud()
{
	//obtem caminho e nome do arquivo .pointcloud e cria um novo com formato .xyz

	//Salva o arquivo da montagem
	FILE *fptr;
	fptr = fopen("/dados/lateralPointCloudToGICP.xyz","w");

	for (unsigned int i=0; i<cloud_lateral_points_to_gicp->width; i++)
	{
		fprintf(fptr, "%f%s%f%s%f%s", cloud_lateral_points_to_gicp->points[i].x, "\t", cloud_lateral_points_to_gicp->points[i].y, "\t", cloud_lateral_points_to_gicp->points[i].z, "\n");
	}

	fclose(fptr);
}

int read_message(int message_num, int publish, int no_wait)
{
	//  char *line[MAX_LINE_LENGTH];
	char *line;
	char *current_pos;
	int i, j;
	char command[100];
	static double last_update = 0;
	double current_time;

	//dados do arquivo de log
	line = (char *) malloc(MAX_LINE_LENGTH * sizeof(char));
	if (line == NULL)
		carmen_die("Could not alloc memory in playback.c:read_message()\n");

	carmen_logfile_read_line(logfile_index, logfile, message_num,
	MAX_LINE_LENGTH, line);
	current_pos = carmen_next_word(line);

	for (i = 0; i < (int) (sizeof(logger_callbacks) / sizeof(logger_callback_t)); i++)
	{
		/* copy the command over */
		j = 0;
		while (line[j] != ' ')
		{
			command[j] = line[j];
			j++;
		}
		command[j] = '\0';

		if (strncmp(command, logger_callbacks[i].logger_message_name, j) == 0)
		{
			if (!basic_messages || !logger_callbacks[i].interpreted)
			{
				current_pos = logger_callbacks[i].conv_func(current_pos, logger_callbacks[i].message_data);
				playback_timestamp = atof(current_pos);
				//printf("command = %s, playback_timestamp = %lf\n", command, playback_timestamp);
				if (publish)
				{
					current_time = carmen_get_time();
					if (current_time - last_update > 0.2)
					{
						print_playback_status();
						last_update = current_time;
					}
					if (!no_wait)
						wait_for_timestamp(playback_timestamp);

					if (line[30] == '2')
						velodyne_number = 2;
					else if (line[30] == '3')
						velodyne_number = 3;

					if (velodyne_number == 3)
					{
						playback_timestamp_v1 = playback_timestamp;
						taxaDePosicao = (playback_timestamp_v2 - playback_timestamp_v1_ant)/(playback_timestamp_v1 - playback_timestamp_v1_ant);
					}
					else if (velodyne_number == 2)
					{
						playback_timestamp_v1_ant = playback_timestamp_v1;
						playback_timestamp_v2 = playback_timestamp;
					}

					int do_not_publish = !g_publish_odometry && (strcmp(logger_callbacks[i].ipc_message_name, CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME) == 0);

					if (!do_not_publish)
					{
						if (task == 0)
						{
							createPointCloudsCalibrated();

							drawCloudsCalibration(false);
						}
						else if (task == 1) //CALIBRACAO
						{
							cleanVelodyneMessagesByCube();		//elimina das mensagens dos velodynes, os pontos fora dos cubos de regiao de interesse
							createPointCloudsCalibrated();		//cria os objetos das nuvens de pontos do mundo de V1 e V2 Transformado para a posicao de V1

							if (flagSaveEnvironment)
								saveEnvironmentPoints();

							drawCloudsCalibration(waitSpin);
						}
						if (task == 2 || task == 3) //GERACAO DAS RETAS
						{
							cleanVelodyneMessagesByCube();			//elimina das mensagens dos velodynes, os pontos fora dos cubos de regiao de interesse
							createPointCloudsCalibratedByKDTree();	//cria os objetos das nuvens de pontos do mundo de V1 e V2 Transformado para a posicao de V1
						}
						if (task == 3 && process_start && velodyne_number == 3)
						{
							projectPointsOnCentralLine();
							detectPointWithMostNeighbours();
							correctV2PointCloud();
							returnCloudsToOriginalDensePoint();
							accumulateBorders2();					//esta funcao agrupa pontos das bordas na task 3 para criacao das bordas da cacamba vazia
						}

						IPC_publishData(logger_callbacks[i].ipc_message_name, logger_callbacks[i].message_data);

						//para exibicao do processo no decorrer das mensagens

						if (task == 3/* && process_start*/)
							drawCloudsTask3(waitSpin);

						if (task == 2 && process_start)
							drawCloudsTask2(waitSpin);
						if (task == 2 && process_end == 1)
						{
							findLines();
							findCentralLine();
							exportLinesData();

							drawCloudsTask2(true);
						}
						else if (task == 3 && process_end == 1)
						{
							generateLateralPointCloud();
							exportLateralPointCloud();

							groupMountageClouds(); 					//agrupa nuvem de pontos do v1 e v2 montadas
							exportMountagePoints();					//exporta a montagem das nuvens
							//gicp_cloudModelAndEmptyTruck();			//alinha a nuvem de pontos do caminhao vazio com a nuvem de pontos do modelo 3 construido
							configAxis();							//cria os eixos da nuvem de pontos da montagem e da cacamba
							drawCloudsTask3(true);
							configTransformationMatrix();			//translada a nuvem de pontos da cacamba vazia para o eixo da nuvem de pontos da cacamba montada
							drawCloudsTask3(true);
							gicp_cloudEmptyTruckAndCloudMountage();	//calcula a transformacao da nuvem de pontos da cacamba vazia com a nuvem da montagem do caminhao
							drawCloudsTask3(true);
							//removePointsOutOfBucketModel();			//elimina pontos fora dos limites do modelo da cacamba e tbm os proximos aos pontos do modelo
							//removePointsByNeighbors();				//elimina pontos que possuem poucos vizinhos apos calcular a media de vizinhos de todos os pontos
							//createBucketBasePoints();				//cria os pontos da base da cacamba
							//drawCloudsTask3_viewer2(false);

							//executePoisson();						//executa o método poisson para reconstrucao do modelo da carga (PCL)
						    //pcl::io::saveOBJFile("/dados/1_montagem_malha_POISSON.obj", meshReconstruction);

							//removeIsolatedPieces();					//mantem apenas o maior elemento conexo da mesh (VTK)
						    //pcl::io::saveOBJFile("/dados/2_montagem_malha_REMOVE_ISOLATED_PIECES.obj", meshReconstruction);

						    //featureEdges();
						    //pcl::io::saveOBJFile("/dados/3_montagem_malha_FEATURE_EDGES.obj", meshReconstruction);

						    //cleanPolyData();						//limpa a mesh (VTK) - remove pontos duplicados e outras coisas que preciso verificar
						    //pcl::io::saveOBJFile("/dados/4_montagem_malha_CLEAN_POLY_DATA.obj", meshReconstruction);

						    //fillHoles();							//fecha buracos (VTK)
						    //pcl::io::saveOBJFile("/dados/5_montagem_malha_FILL_HOLES.obj", meshReconstruction);

						    //massProperties(); 						//calcula volume (VTK)

							//meshSimplification();
							//exportPointsFinalMesh();

							//calculateMeshVolume();
							//drawCloudsTask3_viewer3(false);
						}
					}
				}
				/* return 1 if it is a front laser message */

				free(line);
				return (strcmp(command, "FLASER") == 0);
			}
		}
	}

	free(line);
	return 0;
}

void publish_info_message()
{
	IPC_RETURN_TYPE err;
	carmen_playback_info_message playback_info_message;

	playback_info_message.message_number = current_position;
	playback_info_message.message_timestamp = playback_timestamp;
	playback_info_message.message_timestamp_difference = playback_starttime + playback_timestamp;
	playback_info_message.playback_speed = playback_speed;

	err = IPC_publishData (CARMEN_PLAYBACK_INFO_MESSAGE_NAME, &playback_info_message);
	carmen_test_ipc (err, "Could not publish", CARMEN_PLAYBACK_INFO_MESSAGE_NAME);

	timestamp_last_message_published = playback_timestamp;
}

void main_playback_loop(void)
{
	print_playback_status();

	while (1)
	{
		// eu faco esse teste para evitar uma sobrecarga de mensagens sobre o central e o
		// playback_control. vale ressaltar que essa mensagem so possuir carater informativo
		if (fabs(timestamp_last_message_published - playback_timestamp) > 0.05)
			publish_info_message();

		if(advance_frame || rewind_frame)
			paused = 1;

		if(offset != 0) {
			playback_starttime = 0.0;
			current_position += offset;
			if(current_position < 0)
				current_position = 0;
			if(current_position >= logfile_index->num_messages)
				current_position = logfile_index->num_messages - 1;
			offset = 0;
		}

		if(!paused && (current_position >= logfile_index->num_messages || current_position >= final_position)) {
			paused = 1;
			current_position = 0;
			playback_starttime = 0.0;
			playback_timestamp = 0;
			print_playback_status();
		}
		else if(!paused && current_position < logfile_index->num_messages) {
			read_message(current_position, 1, 0);
			current_position++;
			if(current_position > stop_position) {
				offset = 0;
				paused = 1;
				print_playback_status();
				stop_position = MAX_SIGNED_INT;
			}
		}
		else if(paused && advance_frame) {
			//      laser = 0;
			//      while(current_position < logfile_index->num_messages - 1 && !laser) {
			//	laser = read_message(current_position, 1);
			//	current_position++;
			//      }
			if (current_position < logfile_index->num_messages) {
				read_message(current_position, 1, 0);
				publish_info_message();
				current_position++;
			}
			print_playback_status();
			advance_frame = 0;
		}
		else if(paused && rewind_frame) {
			//      laser = 0;
			//      while(current_position > 0 && !laser) {
			//	current_position--;
			//	laser = read_message(current_position, 0);
			//      }
			//      laser = 0;
			//      while(current_position > 0 && !laser) {
			//	current_position--;
			//	laser = read_message(current_position, 0);
			//      }
			//      read_message(current_position, 1);
			//      current_position++;
			if (current_position > 0) {
				current_position--;
				read_message(current_position, 1, 0);
				publish_info_message();
			}
			print_playback_status();
			rewind_frame = 0;
		}
		if(paused)
			carmen_ipc_sleep(0.01);
		if(fast)
			carmen_ipc_sleep(0.000001);
		else
			carmen_ipc_sleep(0.0001);
	}
}

void usage(char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);

	fprintf(stderr, "Usage: playback <log_file_name> [args]\n");
	fprintf(stderr, "[args]: -fast {on|off} -autostart {on|off} -basic {on|off} -play_message <num> -stop_message <num>\n");
	exit(-1);
}

void read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "robot",		(char *) "publish_odometry",	CARMEN_PARAM_ONOFF,	&(g_publish_odometry),	0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	carmen_param_t param_list2[] =
	{
		{(char *) "commandline",	(char *) "fast",		CARMEN_PARAM_ONOFF,	&(fast),		0, NULL},
		{(char *) "commandline",	(char *) "autostart",		CARMEN_PARAM_ONOFF, 	&(autostart),		0, NULL},
		{(char *) "commandline",	(char *) "basic",		CARMEN_PARAM_ONOFF, 	&(basic_messages),	0, NULL},
		{(char *) "commandline",	(char *) "play_message",	CARMEN_PARAM_INT, 	&(current_position),	0, NULL},
		{(char *) "commandline",	(char *) "stop_message",	CARMEN_PARAM_INT, 	&(stop_position),	0, NULL},
		{(char *) "commandline",	(char *) "save_velodyne_xyz",		CARMEN_PARAM_ONOFF, 	&(save_velodyne_xyz),		0, NULL},

	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_list2, sizeof(param_list2) / sizeof(param_list2[0]));

	paused = !(autostart);
}

void shutdown_playback_module(int sig)
{
	if(sig == SIGINT) {
		fprintf(stderr, "\n");
		exit(1);
	}
}

carmen_logfile_index_p load_logindex_file(char *index_file_name)
{
	FILE *index_file;
	carmen_logfile_index_p logfile_index;

	index_file = fopen(index_file_name, "r");
	if (index_file == NULL)
		carmen_die("Error: could not open file %s for reading.\n", index_file_name);

	logfile_index = (carmen_logfile_index_p) malloc(sizeof(carmen_logfile_index_t));
	carmen_test_alloc(logfile_index);
	fread(logfile_index, sizeof(carmen_logfile_index_t), 1, index_file);

	// carmen_logfile_index_messages() (in readlog.c) set file size as last offset
	// so, offset array contains one element more than messages
	// it is required by carmen_logfile_read_line to read the last line
	logfile_index->offset = (off_t *) malloc((logfile_index->num_messages + 1) * sizeof(off_t));
	fread(logfile_index->offset, sizeof(off_t), logfile_index->num_messages + 1, index_file);
	logfile_index->current_position = 0;

	fclose(index_file);
	return (logfile_index);
}

void save_logindex_file(carmen_logfile_index_p logfile_index, char *index_file_name)
{
	FILE *index_file;

	index_file = fopen(index_file_name, "w");
	if (index_file == NULL)
		carmen_die("Error: could not open file %s for writing.\n", index_file_name);

	fwrite(logfile_index, sizeof(carmen_logfile_index_t), 1, index_file);

	// carmen_logfile_index_messages() (in readlog.c) set file size as last offset
	// so, offset array contains one element more than messages
	// it is required by carmen_logfile_read_line to read the last line
	fwrite(logfile_index->offset, sizeof(off_t), logfile_index->num_messages+1, index_file);

	fclose(index_file);
}

int index_file_older_than_log_file(FILE *index_file, carmen_FILE *logfile)
{
	struct stat stat_buf;
	time_t last_log_modification, last_index_modification;

	fstat(fileno(index_file), &stat_buf);
	last_index_modification = stat_buf.st_mtime;

	fstat(fileno(logfile->fp), &stat_buf);
	last_log_modification = stat_buf.st_mtime;

	if (last_log_modification > last_index_modification)
		return (1);
	else
		return (0);
}

//carrega o arquivo de nuvem de pontos para o GICP
void load_point_cloud_xyz_empty_bucket_laser_gicp()
{
	float x, y, z;
	std::ifstream infile("bin/modelos/cacamba_gicp.xyz");

	//conta o numero de linhas (pontos) do arquivo
	int numLines = 0;
	std::string unused;
	while ( std::getline(infile, unused) )
	   ++numLines;

	//retorna pro inicio do arquivo
	infile.clear();
	infile.seekg(0, ios::beg);

	cloud_emptyBucketLaserGICP->width = numLines;
	cloud_emptyBucketLaserGICP->points.resize (cloud_emptyBucketLaserGICP->width * cloud_emptyBucketLaserGICP->height);

	int i = 0;

	//percorre o arquivo e adiciona os pontos na nuvem
	while (infile >> x >> y >> z)
	{
		cloud_emptyBucketLaserGICP->points[i].x = x;
		cloud_emptyBucketLaserGICP->points[i].y = y;
		cloud_emptyBucketLaserGICP->points[i].z = z;

		cloud_emptyBucketLaserGICP->at(i).r = 0;
		cloud_emptyBucketLaserGICP->at(i).g = 0;
		cloud_emptyBucketLaserGICP->at(i).b = 255;

		i++;
	}
}

void load_point_cloud_xyz_empty_bucket_laser()
{
	float x, y, z;
	std::ifstream infile("bin/modelos/cacamba_original.xyz");

	//conta o numero de linhas (pontos) do arquivo
	int numLines = 0;
	std::string unused;
	while ( std::getline(infile, unused) )
	   ++numLines;

	//retorna pro inicio do arquivo
	infile.clear();
	infile.seekg(0, ios::beg);

	//iniciliza tamanho da nuvem de pontos
	cloud_emptyBucketLaser->width = numLines;
	cloud_emptyBucketLaser->points.resize (cloud_emptyBucketLaser->width * cloud_emptyBucketLaser->height);

	int i = 0;

	//percorre o arquivo e adiciona os pontos na nuvem
	while (infile >> x >> y >> z)
	{
		cloud_emptyBucketLaser->points[i].x = x;
		cloud_emptyBucketLaser->points[i].y = y;
		cloud_emptyBucketLaser->points[i].z = z;

		cloud_emptyBucketLaser->at(i).r = 255;
		cloud_emptyBucketLaser->at(i).g = 255;
		cloud_emptyBucketLaser->at(i).b = 100;

		i++;
	}
}

void load_point_cloud_xyz_bucket_corners()
{
	float x, y, z;
	std::ifstream infile("bin/modelos/cacamba_pontos_bordas.xyz");

	//conta o numero de linhas (pontos) do arquivo
	int numLines = 0;
	std::string unused;
	while ( std::getline(infile, unused) )
	   ++numLines;

	//retorna pro inicio do arquivo
	infile.clear();
	infile.seekg(0, ios::beg);

	//iniciliza tamanho da nuvem de pontos
	cloud_bucketCorners->width = numLines;
	cloud_bucketCorners->points.resize (cloud_bucketCorners->width * cloud_bucketCorners->height);

	int i = 0;

	//percorre o arquivo e adiciona os pontos na nuvem
	while (infile >> x >> y >> z)
	{
		cloud_bucketCorners->points[i].x = x;
		cloud_bucketCorners->points[i].y = y;
		cloud_bucketCorners->points[i].z = z;

		cloud_bucketCorners->at(i).r = 0;
		cloud_bucketCorners->at(i).g = 255;
		cloud_bucketCorners->at(i).b = 0;

		i++;
	}
}

//abre o arquivo de nuvem de pontos com os pontos flutuantes acima da cacamba para montagem da base da cacamba
void load_point_cloud_xyz_floating_points()
{
	float x, y, z;
	std::ifstream infile("bin/modelos/cacamba_pontos_flutuantes.xyz");

	//conta o numero de linhas (pontos) do arquivo
	int numLines = 0;
	std::string unused;
	while ( std::getline(infile, unused) )
	   ++numLines;

	//retorna pro inicio do arquivo
	infile.clear();
	infile.seekg(0, ios::beg);

	//nuvem dos pontos flutuantes:
	cloud_floating_points->width = numLines;
	cloud_floating_points->points.resize (cloud_floating_points->width * cloud_floating_points->height);

	int i = 0;

	//percorre o arquivo e adiciona os pontos na nuvem
	while (infile >> x >> y >> z)
	{
		cloud_floating_points->points[i].x = x;
		cloud_floating_points->points[i].y = y;
		cloud_floating_points->points[i].z = z;

		//azul
		cloud_floating_points->at(i).r = 0;
		cloud_floating_points->at(i).g = 0;
		cloud_floating_points->at(i).b = 255;

		i++;
	}
}

void load_point_cloud_xyz_model_points()
{
	float x, y, z;
	std::ifstream infile("bin/modelos/cacamba_tesselate_para_remocao_de_pontos_da_mesh.xyz");

	//conta o numero de linhas (pontos) do arquivo
	int numLines = 0;
	std::string unused;
	while ( std::getline(infile, unused) )
	   ++numLines;

	//retorna pro inicio do arquivo
	infile.clear();
	infile.seekg(0, ios::beg);

	cloud_modelPoints->width = numLines;
	cloud_modelPoints->points.resize (cloud_modelPoints->width * cloud_modelPoints->height);

	int i = 0;

	while (infile >> x >> y >> z)
	{
		cloud_modelPoints->points[i].x = x;
		cloud_modelPoints->points[i].y = y;
		cloud_modelPoints->points[i].z = z;

		cloud_modelPoints->at(i).r = 255;
		cloud_modelPoints->at(i).g = 255;
		cloud_modelPoints->at(i).b = 0;

		i++;
	}
}

void load_point_cloud_xyz_empty_bucket_model()
{
	float x, y, z;
	std::ifstream infile("bin/modelos/cacamba_tesselate_para_localizacao_da_mesh.xyz");

	//conta o numero de linhas (pontos) do arquivo
	int numLines = 0;
	std::string unused;
	while ( std::getline(infile, unused) )
	   ++numLines;

	//retorna pro inicio do arquivo
	infile.clear();
	infile.seekg(0, ios::beg);

	//nuvem inicial mais densa (nao sera alterada):
	cloud_emptyBucketModel->width = numLines;
	cloud_emptyBucketModel->points.resize (cloud_emptyBucketModel->width * cloud_emptyBucketModel->height);

	int i = 0;

	//percorre o arquivo e adiciona os pontos na nuvem
	while (infile >> x >> y >> z)
	{
		cloud_emptyBucketModel->points[i].x = x;
		cloud_emptyBucketModel->points[i].y = y;
		cloud_emptyBucketModel->points[i].z = z;

		cloud_emptyBucketModel->at(i).r = 0;
		cloud_emptyBucketModel->at(i).g = 255;
		cloud_emptyBucketModel->at(i).b = 0;

		i++;
	}
}

void load_bucket_mesh()

{
	pcl::io::loadPolygonFileOBJ("bin/modelos/cacamba_para_reconstrucao_da_base.obj", meshBucket);

	int polygonCount = meshBucket.polygons.size();

	pcl::PointCloud<pcl::PointXYZ> objCloud;
	pcl::PCLPointCloud2 ptCloud2 = meshBucket.cloud;
	pcl::fromPCLPointCloud2(ptCloud2,objCloud);

	for(int i=0; i<polygonCount; i++)
	{
		pcl::Vertices currentPoly = meshBucket.polygons[i];

		for(int j=0; j<currentPoly.vertices.size(); j++) //deve ser 3 (x, y, z)
		{
			pcl::PointNormal currentPt = pcl::PointNormal();
			currentPt.x = objCloud[currentPoly.vertices[j]].x;
			currentPt.y = objCloud[currentPoly.vertices[j]].y;
			currentPt.z = objCloud[currentPoly.vertices[j]].z;

			meshPoligonsNormalsCacamba->points.push_back(currentPt);//push in points without normals
		}

		//suponho 3 vertices para cada poligono
		int index = meshPoligonsNormalsCacamba->points.size()-1;
		pcl::PointXYZ pt3(meshPoligonsNormalsCacamba->points[index].x,meshPoligonsNormalsCacamba->points[index].y,meshPoligonsNormalsCacamba->points[index].z);
		index--;
		pcl::PointXYZ pt2(meshPoligonsNormalsCacamba->points[index].x,meshPoligonsNormalsCacamba->points[index].y,meshPoligonsNormalsCacamba->points[index].z);
		index--;
		pcl::PointXYZ pt1(meshPoligonsNormalsCacamba->points[index].x,meshPoligonsNormalsCacamba->points[index].y,meshPoligonsNormalsCacamba->points[index].z);

		Eigen::Vector3f vec12(pt2.x-pt1.x,pt2.y-pt1.y,pt2.z-pt1.z);
		Eigen::Vector3f vec23(pt3.x-pt2.x,pt3.y-pt2.y,pt3.z-pt2.z);
		Eigen::Vector3f vecNorm = vec12.cross(vec23);
		vecNorm.normalize();

		for(int k=0; k<3; k++)
		{
			meshPoligonsNormalsCacamba->points[index+k].normal_x = vecNorm[0];
			meshPoligonsNormalsCacamba->points[index+k].normal_y = vecNorm[1];
			meshPoligonsNormalsCacamba->points[index+k].normal_z = vecNorm[2];
		}

	}

	//apenas para visualizar se o carregamento ocorreu normalmente
	/*int pointsoffaces = meshPoligonsNormalsCacamba->points.size(); //numero de pontos deve ser 3x o numero de faces

	pcl::visualization::PCLVisualizer viewer_normals_from_model("Viewer Normals from Model");

	viewer_normals_from_model.setBackgroundColor(.5, .5, .5);
	viewer_normals_from_model.addCoordinateSystem (3.0);
	viewer_normals_from_model.removeAllPointClouds();

	viewer_normals_from_model.addPointCloud<pcl::PointNormal>(meshPoligonsNormalsCacamba, "poligons_and_normals");
	viewer_normals_from_model.addPointCloudNormals<pcl::PointNormal, pcl::PointNormal> (meshPoligonsNormalsCacamba, meshPoligonsNormalsCacamba, 1, 0.15, "normals");

	viewer_normals_from_model.spin();*/

}

//carrega o modelo utilizado para eliminacao de pontos externos ao volume, e cria um objeto com
void load_bucket_mesh_simplified()
{
	//pcl::io::loadPolygonFileOBJ("bin/modelos/cacamba_volume_simplificado.obj", meshBucketSimplified);
	pcl::io::loadPolygonFileOBJ("bin/modelos/cacamba_simplificada_para_eliminacao_3.obj", meshBucketSimplified);

	int polygonCount = meshBucketSimplified.polygons.size();

	pcl::PointCloud<pcl::PointXYZ> objCloud;
    pcl::PCLPointCloud2 ptCloud2 = meshBucketSimplified.cloud;
    pcl::fromPCLPointCloud2(ptCloud2,objCloud);

	for(int i=0; i<polygonCount; i++)
	{
		pcl::Vertices currentPoly = meshBucketSimplified.polygons[i];

		for(int j=0; j<currentPoly.vertices.size(); j++) //deve ser 3 (x, y, z)
		{
			pcl::PointNormal currentPt = pcl::PointNormal();
			currentPt.x = objCloud[currentPoly.vertices[j]].x;
			currentPt.y = objCloud[currentPoly.vertices[j]].y;
			currentPt.z = objCloud[currentPoly.vertices[j]].z;

			meshPoligonsNormalsCacambaSimplified->points.push_back(currentPt);//push in points without normals
		}

        //suponho 3 vertices para cada poligono
        int index = meshPoligonsNormalsCacambaSimplified->points.size()-1;
        pcl::PointXYZ pt3(meshPoligonsNormalsCacambaSimplified->points[index].x,meshPoligonsNormalsCacambaSimplified->points[index].y,meshPoligonsNormalsCacambaSimplified->points[index].z);
        index--;
        pcl::PointXYZ pt2(meshPoligonsNormalsCacambaSimplified->points[index].x,meshPoligonsNormalsCacambaSimplified->points[index].y,meshPoligonsNormalsCacambaSimplified->points[index].z);
        index--;
        pcl::PointXYZ pt1(meshPoligonsNormalsCacambaSimplified->points[index].x,meshPoligonsNormalsCacambaSimplified->points[index].y,meshPoligonsNormalsCacambaSimplified->points[index].z);

        Eigen::Vector3f vec12(pt2.x-pt1.x,pt2.y-pt1.y,pt2.z-pt1.z);
		Eigen::Vector3f vec23(pt3.x-pt2.x,pt3.y-pt2.y,pt3.z-pt2.z);
		Eigen::Vector3f vecNorm = vec12.cross(vec23);
		vecNorm.normalize();

		for(int k=0; k<3; k++)
		{
			meshPoligonsNormalsCacambaSimplified->points[index+k].normal_x = vecNorm[0];
			meshPoligonsNormalsCacambaSimplified->points[index+k].normal_y = vecNorm[1];
			meshPoligonsNormalsCacambaSimplified->points[index+k].normal_z = vecNorm[2];
		}

	}

	//montaBaseCacamba();
	/*
	//apenas para visualizar se o carregamento ocorreu normalmente
	int pointsoffaces = meshPoligonsNormalsCacambaSimplified->points.size(); //numero de pontos deve ser 3x o numero de faces

	pcl::visualization::PCLVisualizer viewer_normals_from_model("Viewer Normals from Model");

	viewer_normals_from_model.setBackgroundColor(.5, .5, .5);
	viewer_normals_from_model.addCoordinateSystem (3.0);
	viewer_normals_from_model.removeAllPointClouds();

	viewer_normals_from_model.addPointCloud<pcl::PointNormal>(meshPoligonsNormalsCacambaSimplified, "poligons_and_normals");
	viewer_normals_from_model.addPointCloudNormals<pcl::PointNormal, pcl::PointNormal> (meshPoligonsNormalsCacambaSimplified, meshPoligonsNormalsCacambaSimplified, 1, 0.5, "normals");

	viewer_normals_from_model.spin();
	*/
}

void importLinesData()
{
	std::ifstream infile("../../../dados/lines.txt");

	infile >> point_x1 >> point_y1 >> point_z1 >> vector_x1 >> vector_y1 >> vector_z1 >>
			  point_x2 >> point_y2 >> point_z2 >> vector_x2 >> vector_y2 >> vector_z2 >>
			  point_x3 >> point_y3 >> point_z3 >> vector_x3 >> vector_y3 >> vector_z3;

}

//apenas inicializa as nuvens de pontos dos velodynes
void configClouds()
{
    MatrixCube1->width = 8;
	MatrixCube1->height = 1;
	MatrixCube1->is_dense = false;
	MatrixCube1->points.resize (MatrixCube1->width * MatrixCube1->height);

    MatrixCube2->width = 8;
	MatrixCube2->height = 1;
	MatrixCube2->is_dense = false;
	MatrixCube2->points.resize (MatrixCube2->width * MatrixCube2->height);

    MatrixCube2_transformed->width = 8;
    MatrixCube2_transformed->height = 1;
    MatrixCube2_transformed->is_dense = false;
	MatrixCube2_transformed->points.resize (MatrixCube2_transformed->width * MatrixCube2_transformed->height);

    cloud_pointPickingV1->width = 0;
    cloud_pointPickingV1->height = 1;
    cloud_pointPickingV1->is_dense = false;
    cloud_pointPickingV1->points.resize (cloud_pointPickingV1->width * cloud_pointPickingV1->height);

    cloud_pointPickingV2->width = 0;
    cloud_pointPickingV2->height = 1;
    cloud_pointPickingV2->is_dense = false;
    cloud_pointPickingV2->points.resize (cloud_pointPickingV2->width * cloud_pointPickingV2->height);

	cloud_v1_world->width = 0;
	cloud_v1_world->height = 1;
	cloud_v1_world->is_dense = false;
	cloud_v1_world->points.resize (cloud_v1_world->width * cloud_v1_world->height);

	cloud_v2_world->width = 0;
	cloud_v2_world->height = 1;
	cloud_v2_world->is_dense = false;
	cloud_v2_world->points.resize (cloud_v2_world->width * cloud_v2_world->height);

	cloud_v1_truck->width = 0;
	cloud_v1_truck->height = 1;
	cloud_v1_truck->is_dense = false;
	cloud_v1_truck->points.resize (cloud_v1_truck->width * cloud_v1_truck->height);

	cloud_v2_truck->width = 0;
	cloud_v2_truck->height = 1;
	cloud_v2_truck->is_dense = false;
	cloud_v2_truck->points.resize (cloud_v2_truck->width * cloud_v2_truck->height);

	cloud_v2_truck_transformed->width = 0;
	cloud_v2_truck_transformed->height = 1;
	cloud_v2_truck_transformed->is_dense = false;
	cloud_v2_truck_transformed->points.resize (cloud_v2_truck_transformed->width * cloud_v2_truck_transformed->height);

	cloud_v2_truck_corrected->width = 0;
	cloud_v2_truck_corrected->height = 1;
	cloud_v2_truck_corrected->is_dense = false;
	cloud_v2_truck_corrected->points.resize (cloud_v2_truck_corrected->width * cloud_v2_truck_corrected->height);

    cloud_environmentRemovalV1->width = 0;
    cloud_environmentRemovalV1->height = 1;
    cloud_environmentRemovalV1->is_dense = false;
    cloud_environmentRemovalV1->points.resize (cloud_environmentRemovalV1->width * cloud_environmentRemovalV1->height);

    cloud_environmentRemovalV2->width = 0;
    cloud_environmentRemovalV2->height = 1;
    cloud_environmentRemovalV2->is_dense = false;
    cloud_environmentRemovalV2->points.resize (cloud_environmentRemovalV2->width * cloud_environmentRemovalV2->height);

	cloud_v2_border_left->width = 16;
	cloud_v2_border_left->height = 1;
	cloud_v2_border_left->is_dense = false;
	cloud_v2_border_left->points.resize (cloud_v2_border_left->width * cloud_v2_border_left->height);

	cloud_v2_border_right->width = 16;
	cloud_v2_border_right->height = 1;
	cloud_v2_border_right->is_dense = false;
	cloud_v2_border_right->points.resize (cloud_v2_border_right->width * cloud_v2_border_right->height);

	cloud_v2_border_left_accumulated->width = 0;
	cloud_v2_border_left_accumulated->height = 1;
	cloud_v2_border_left_accumulated->is_dense = false;
	cloud_v2_border_left_accumulated->points.resize (cloud_v2_border_left_accumulated->width * cloud_v2_border_left_accumulated->height);

	cloud_v2_border_right_accumulated->width = 0;
	cloud_v2_border_right_accumulated->height = 1;
	cloud_v2_border_right_accumulated->is_dense = false;
	cloud_v2_border_right_accumulated->points.resize (cloud_v2_border_right_accumulated->width * cloud_v2_border_right_accumulated->height);

	cloud_line_v1_projection->width = 0;
	cloud_line_v1_projection->height = 1;
	cloud_line_v1_projection->is_dense = false;
	cloud_line_v1_projection->points.resize (cloud_line_v1_projection->width * cloud_line_v1_projection->height);

	cloud_mostDensePoints_v1_projection->width = 0;
	cloud_mostDensePoints_v1_projection->height = 1;
	cloud_mostDensePoints_v1_projection->is_dense = false;
	cloud_mostDensePoints_v1_projection->points.resize (cloud_mostDensePoints_v1_projection->width * cloud_mostDensePoints_v1_projection->height);

	cloud_onlyMostDensePoint_v1->width = 2;
	cloud_onlyMostDensePoint_v1->height = 1;
	cloud_onlyMostDensePoint_v1->is_dense = false;
	cloud_onlyMostDensePoint_v1->points.resize (cloud_onlyMostDensePoint_v1->width * cloud_onlyMostDensePoint_v1->height);
	cloud_onlyMostDensePoint_v1->at(0).r = 0; cloud_onlyMostDensePoint_v1->at(0).g = 0; cloud_onlyMostDensePoint_v1->at(0).b = 0;
	cloud_onlyMostDensePoint_v1->at(1).r = 255; cloud_onlyMostDensePoint_v1->at(1).g = 0; cloud_onlyMostDensePoint_v1->at(1).b = 0;

	cloud_mostDensePoints_v1_projection_correction_v2->width = 1;
	cloud_mostDensePoints_v1_projection_correction_v2->height = 1;
	cloud_mostDensePoints_v1_projection_correction_v2->is_dense = false;
	cloud_mostDensePoints_v1_projection_correction_v2->points.resize (cloud_mostDensePoints_v1_projection_correction_v2->width * cloud_mostDensePoints_v1_projection_correction_v2->height);

	cloud_mountage_v1->width = 0;
	cloud_mountage_v1->height = 1;
	cloud_mountage_v1->is_dense = false;
	cloud_mountage_v1->points.resize (cloud_mountage_v1->width * cloud_mountage_v1->height);

	cloud_mountage_v2->width = 0;
	cloud_mountage_v2->height = 1;
	cloud_mountage_v2->is_dense = false;
	cloud_mountage_v2->points.resize (cloud_mountage_v2->width * cloud_mountage_v2->height);

	cloud_mountage_v1v2->width = 0;
	cloud_mountage_v1v2->height = 1;
	cloud_mountage_v1v2->is_dense = false;
	cloud_mountage_v1v2->points.resize (cloud_mountage_v1v2->width * cloud_mountage_v1v2->height);

	cloud_mountage_v2_internal->width = 0;
	cloud_mountage_v2_internal->height = 1;
	cloud_mountage_v2_internal->is_dense = false;
	cloud_mountage_v2_internal->points.resize (cloud_mountage_v2_internal->width * cloud_mountage_v2_internal->height);

	cloud_emptyBucketLaser->width = 0;
	cloud_emptyBucketLaser->height = 1;
	cloud_emptyBucketLaser->is_dense = false;
	cloud_emptyBucketLaser->points.resize (cloud_emptyBucketLaser->width * cloud_emptyBucketLaser->height);

	cloud_emptyBucketLaserGICP->width = 0;
	cloud_emptyBucketLaserGICP->height = 1;
	cloud_emptyBucketLaserGICP->is_dense = false;
	cloud_emptyBucketLaserGICP->points.resize (cloud_emptyBucketLaserGICP->width * cloud_emptyBucketLaserGICP->height);

	cloud_emptyBucketModel->width = 0;
	cloud_emptyBucketModel->height = 1;
	cloud_emptyBucketModel->is_dense = false;
	cloud_emptyBucketModel->points.resize (cloud_emptyBucketModel->width * cloud_emptyBucketModel->height);

	cloud_bucketCorners->width = 0;
	cloud_bucketCorners->height = 1;
	cloud_bucketCorners->is_dense = false;
	cloud_bucketCorners->points.resize (cloud_bucketCorners->width * cloud_bucketCorners->height);

	cloud_bucketAxis->width = 4;
	cloud_bucketAxis->height = 1;
	cloud_bucketAxis->is_dense = false;
	cloud_bucketAxis->points.resize (cloud_bucketAxis->width * cloud_bucketAxis->height);

	cloud_mountageAxis->width = 4;
	cloud_mountageAxis->height = 1;
	cloud_mountageAxis->is_dense = false;
	cloud_mountageAxis->points.resize (cloud_mountageAxis->width * cloud_mountageAxis->height);

	cloud_modelPoints->width = 0;
	cloud_modelPoints->height = 1;
	cloud_modelPoints->is_dense = false;
	cloud_modelPoints->points.resize (cloud_modelPoints->width * cloud_modelPoints->height);

	cloud_floating_points->width = 0;
	cloud_floating_points->height = 1;
	cloud_floating_points->is_dense = false;
	cloud_floating_points->points.resize (cloud_floating_points->width * cloud_floating_points->height);

	pointToTransform->width = 1;
	pointToTransform->height = 1;
	pointToTransform->is_dense = false;
	pointToTransform->points.resize (pointToTransform->width * pointToTransform->height);

	cloud_v1_truck_single_line->width = 0;
	cloud_v1_truck_single_line->height = 1;
	cloud_v1_truck_single_line->is_dense = false;
	cloud_v1_truck_single_line->points.resize (cloud_v1_truck_single_line->width * cloud_v1_truck_single_line->height);

	cloud_v2_border_left_accumulated2->width = 0;
	cloud_v2_border_left_accumulated2->height = 1;
	cloud_v2_border_left_accumulated2->is_dense = false;
	cloud_v2_border_left_accumulated2->points.resize (cloud_v2_border_left_accumulated2->width * cloud_v2_border_left_accumulated2->height);

	cloud_v2_border_right_accumulated2->width = 0;
	cloud_v2_border_right_accumulated2->height = 1;
	cloud_v2_border_right_accumulated2->is_dense = false;
	cloud_v2_border_right_accumulated2->points.resize (cloud_v2_border_right_accumulated2->width * cloud_v2_border_right_accumulated2->height);

	cloud_v2_border_right_transformed->width = 0;
	cloud_v2_border_right_transformed->height = 1;
	cloud_v2_border_right_transformed->is_dense = false;
	cloud_v2_border_right_transformed->points.resize (cloud_v2_border_right_transformed->width * cloud_v2_border_right_transformed->height);

	cloud_v2_border_left_transformed->width = 0;
	cloud_v2_border_left_transformed->height = 1;
	cloud_v2_border_left_transformed->is_dense = false;
	cloud_v2_border_left_transformed->points.resize (cloud_v2_border_left_transformed->width * cloud_v2_border_left_transformed->height);

	cloud_lateral_borders->width = 0;
	cloud_lateral_borders->height = 1;
	cloud_lateral_borders->is_dense = false;
	cloud_lateral_borders->points.resize (cloud_lateral_borders->width * cloud_lateral_borders->height);

	cloud_lateral_points_to_gicp->width = 0;
	cloud_lateral_points_to_gicp->height = 1;
	cloud_lateral_points_to_gicp->is_dense = false;
	cloud_lateral_points_to_gicp->points.resize (cloud_lateral_points_to_gicp->width * cloud_lateral_points_to_gicp->height);
}

//altera dimensoes da caixa de regiao interesse
void configMatrixCube_size()
{
	//Matriz do Velodyne 1:

	MatrixCube1->points[0].x = -cx1/2.0;
	MatrixCube1->points[0].y = -cy1/2.0;
	MatrixCube1->points[0].z = -cz1/2.0;
	MatrixCube1->points[1].x = cx1/2.0;
	MatrixCube1->points[1].y = -cy1/2.0;
	MatrixCube1->points[1].z = -cz1/2.0;
	MatrixCube1->points[2].x = cx1/2.0;
	MatrixCube1->points[2].y = cy1/2.0;
	MatrixCube1->points[2].z = -cz1/2.0;
	MatrixCube1->points[3].x = -cx1/2.0;
	MatrixCube1->points[3].y = cy1/2.0;
	MatrixCube1->points[3].z = -cz1/2.0;
	MatrixCube1->points[4].x = -cx1/2.0;
	MatrixCube1->points[4].y = -cy1/2.0;
	MatrixCube1->points[4].z = cz1/2.0;
	MatrixCube1->points[5].x = cx1/2.0;
	MatrixCube1->points[5].y = -cy1/2.0;
	MatrixCube1->points[5].z = cz1/2.0;
	MatrixCube1->points[6].x = cx1/2.0;
	MatrixCube1->points[6].y = cy1/2.0;
	MatrixCube1->points[6].z = cz1/2.0;
	MatrixCube1->points[7].x = -cx1/2.0;
	MatrixCube1->points[7].y = cy1/2.0;
	MatrixCube1->points[7].z = cz1/2.0;

    //Matriz do Velodyne 2:

	MatrixCube2->points[0].x = -cx2/2.0;
	MatrixCube2->points[0].y = -cy2/2.0;
	MatrixCube2->points[0].z = -cz2/2.0;
	MatrixCube2->points[1].x = cx2/2.0;
	MatrixCube2->points[1].y = -cy2/2.0;
	MatrixCube2->points[1].z = -cz2/2.0;
	MatrixCube2->points[2].x = cx2/2.0;
	MatrixCube2->points[2].y = cy2/2.0;
	MatrixCube2->points[2].z = -cz2/2.0;
	MatrixCube2->points[3].x = -cx2/2.0;
	MatrixCube2->points[3].y = cy2/2.0;
	MatrixCube2->points[3].z = -cz2/2.0;
	MatrixCube2->points[4].x = -cx2/2.0;
	MatrixCube2->points[4].y = -cy2/2.0;
	MatrixCube2->points[4].z = cz2/2.0;
	MatrixCube2->points[5].x = cx2/2.0;
	MatrixCube2->points[5].y = -cy2/2.0;
	MatrixCube2->points[5].z = cz2/2.0;
	MatrixCube2->points[6].x = cx2/2.0;
	MatrixCube2->points[6].y = cy2/2.0;
	MatrixCube2->points[6].z = cz2/2.0;
	MatrixCube2->points[7].x = -cx2/2.0;
	MatrixCube2->points[7].y = cy2/2.0;
	MatrixCube2->points[7].z = cz2/2.0;
}

void configMatrixCube_position()
{
	Matrix<double, 4, 4> rotationX, rotationY, rotationZ, translation;
	float RX, RY, RZ;

    RX = carmen_degrees_to_radians(rx1);
    RY = carmen_degrees_to_radians(ry1);
    RZ = carmen_degrees_to_radians(rz1);

    rotationX << 1,		0,			0,			0,
    			 0,		cos(RX),	-sin(RX), 	0,
				 0,		sin(RX), 	cos(RX), 	0,
				 0, 	0, 			0,			1;

    rotationY << cos(RY),	0,		-sin(RY),	0,
    			 0,			1,		0,			0,
    			 sin(RY), 	0,		cos(RY), 	0,
				 0, 		0, 		0,			1;

    rotationZ << cos(RZ),	-sin(RZ), 	0, 		0,
    			 sin(RZ), 	cos(RZ), 	0, 		0,
				 0, 		0, 			1, 		0,
				 0, 		0, 			0, 		1;

    translation << 1, 0, 0, tx1,
    			   0, 1, 0, ty1,
				   0, 0, 1, tz1,
				   0, 0, 0, 1;

    transformationCubeV1 = translation * rotationZ * rotationY * rotationX;

    transformPointCloud(*MatrixCube1, *MatrixCube1, transformationCubeV1);

	RX = carmen_degrees_to_radians(rx2);
	RY = carmen_degrees_to_radians(ry2);
	RZ = carmen_degrees_to_radians(rz2);

	rotationX << 1,		0,			0,			0,
				 0,		cos(RX),	-sin(RX), 	0,
				 0,		sin(RX), 	cos(RX), 	0,
				 0, 	0, 			0,			1;

	rotationY << cos(RY),	0,		-sin(RY),	0,
				 0,			1,		0,			0,
				 sin(RY), 	0,		cos(RY), 	0,
				 0, 		0, 		0,			1;

	rotationZ << cos(RZ),	-sin(RZ), 	0, 		0,
				 sin(RZ), 	cos(RZ), 	0, 		0,
				 0, 		0, 			1, 		0,
				 0, 		0, 			0, 		1;

	translation << 1, 0, 0, tx2,
				   0, 1, 0, ty2,
				   0, 0, 1, tz2,
				   0, 0, 0, 1;

	transformationCubeV2 = translation * rotationZ * rotationY * rotationX;

	transformPointCloud(*MatrixCube2, *MatrixCube2, transformationCubeV2);

	//printf("\n\nDados de calibracao - Posicionamento dos cubos:\n");
	//printf("cx1: %f, cy1: %f, cz1: %f, rx1: %f, ry1: %f, rz1: %f, tx1: %f, ty1: %f, tz1: %f\n", cx1, cy1, cy1, rx1, ry1, rz1, tx1, ty1, tz1);
	//printf("cx2: %f, cy2: %f, cz2: %f, rx2: %f, ry2: %f, rz2: %f, tx2: %f, ty2: %f, tz2: %f\n\n", cx2, cy2, cy2, rx2, ry2, rz2, tx2, ty2, tz2);

}

void load_point_cloud_testPoisson()
{
	float x, y, z;
	std::ifstream infile("/dados/points_mountage_v2.xyz");

	//conta o numero de linhas (pontos) do arquivo
	int numLines = 0;
	std::string unused;
	while ( std::getline(infile, unused) )
	   ++numLines;

	//retorna pro inicio do arquivo
	infile.clear();
	infile.seekg(0, ios::beg);

	//iniciliza tamanho da nuvem de pontos
	cloud_testPoisson->width = numLines;
	cloud_testPoisson->height = 1;
	cloud_testPoisson->is_dense = false;
	cloud_testPoisson->points.resize (cloud_testPoisson->width * cloud_testPoisson->height);

	int i = 0;

	//percorre o arquivo e adiciona os pontos na nuvem
	while (infile >> x >> y >> z)
	{
		cloud_testPoisson->points[i].x = x;
		cloud_testPoisson->points[i].y = y;
		cloud_testPoisson->points[i].z = z;

		cloud_testPoisson->at(i).r = 0;
		cloud_testPoisson->at(i).g = 0;
		cloud_testPoisson->at(i).b = 255;

		i++;
	}
}

void PoissonFromPointCloudsOnFolder()
{
	load_point_cloud_testPoisson();

	PointCloud<PointXYZRGB>::Ptr cloud_mountage_volume_filtered(new PointCloud<PointXYZRGB>());
	PassThrough<PointXYZRGB> filter;
	filter.setInputCloud(cloud_testPoisson);
	filter.filter(*cloud_mountage_volume_filtered);

	/*MovingLeastSquares<PointXYZRGB, PointXYZRGB> mls;
	mls.setInputCloud(cloud_mountage_volume_filtered);
	mls.setSearchRadius(0.1);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(2);
	mls.setUpsamplingMethod(MovingLeastSquares<PointXYZRGB, PointXYZRGB>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius(0.05);
	mls.setUpsamplingStepSize(0.01);

	PointCloud<PointXYZRGB>::Ptr cloud_smoothed (new PointCloud<PointXYZRGB>());
	mls.process(*cloud_smoothed);

	viewer4.addPointCloud(cloud_smoothed, "cloud_mountage_volume_filtered");
	viewer4.spin();*/

	NormalEstimationOMP<PointXYZRGB, Normal> ne;
	ne.setNumberOfThreads (8);
	ne.setInputCloud (cloud_mountage_volume_filtered);
	ne.setRadiusSearch (0.5);
	Eigen::Vector4f centroid;
	compute3DCentroid (*cloud_mountage_volume_filtered, centroid);
	ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
	ne.compute (*cloud_normals);

	for (size_t  i = 0; i < cloud_normals->size(); ++i)
	{
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}

	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals (new PointCloud<PointXYZRGBNormal> ());
	concatenateFields (*cloud_mountage_volume_filtered, *cloud_normals, *cloud_with_normals);

	Poisson<PointXYZRGBNormal> poisson;
	poisson.setDepth (10);
	poisson.setInputCloud (cloud_with_normals);

	//poisson.setManifold(true);

	poisson.reconstruct (meshReconstruction);

	//obtem caminho e nome do arquivo .pointcloud e cria um novo com formato .xyz

	//exporta malha do volume
    pcl::io::saveOBJFile("/dados/montagem_malha_teste.obj", meshReconstruction);

    visualization::PCLVisualizer viewer_teste;
    viewer_teste.setBackgroundColor(.5, .5, .5);
    viewer_teste.addCoordinateSystem (3.0);
    viewer_teste.addPolygonMesh(meshReconstruction,"mesh_teste_resconstrucao", 0);
    //viewer_teste.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "mesh_teste_resconstrucao");
    viewer_teste.spin();
}

void configureCubePosition(bool saveFile)
{
	configMatrixCube_size();
	configMatrixCube_position();

	if (saveFile)
		exportCubePosition();
}

//calibracao manual executada pela selecao de tres pontos de cada nuvem (selecionar os pontos aproximando a camera e segurando SHIFT)
//selecionar 3 pontos da nuvem origem (V2) e 3 pontos da nuvem destino (V1), selecionando os pontos equivalentes na mesma ordem
void pointPicking_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	float x, y, z;
	event.getPoint(x, y, z);
	//std::cout << "X: " << x << " Y: " << y << " Z: " << z << std::endl;
	contPoints++;

	if (contPoints <= 3)
	{
		cloud_pointPickingV2->width = cloud_pointPickingV2->width + 1;
		cloud_pointPickingV2->points.resize (cloud_pointPickingV2->width * cloud_pointPickingV2->height);
		cloud_pointPickingV2->points[cloud_pointPickingV2->size()-1].x = x;
		cloud_pointPickingV2->points[cloud_pointPickingV2->size()-1].y = y;
		cloud_pointPickingV2->points[cloud_pointPickingV2->size()-1].z = z;
		cloud_pointPickingV2->at(cloud_pointPickingV2->size()-1).r = 0;
		cloud_pointPickingV2->at(cloud_pointPickingV2->size()-1).g = 255;
		cloud_pointPickingV2->at(cloud_pointPickingV2->size()-1).b = 0;

		viewer1.updatePointCloud(cloud_pointPickingV2, "pp_v2");
	}
	else if (contPoints <= 6)
	{
		cloud_pointPickingV1->width = cloud_pointPickingV1->width + 1;
		cloud_pointPickingV1->points.resize (cloud_pointPickingV1->width * cloud_pointPickingV1->height);
		cloud_pointPickingV1->points[cloud_pointPickingV1->size()-1].x = x;
		cloud_pointPickingV1->points[cloud_pointPickingV1->size()-1].y = y;
		cloud_pointPickingV1->points[cloud_pointPickingV1->size()-1].z = z;
		cloud_pointPickingV1->at(cloud_pointPickingV1->size()-1).r = 255;
		cloud_pointPickingV1->at(cloud_pointPickingV1->size()-1).g = 0;
		cloud_pointPickingV1->at(cloud_pointPickingV1->size()-1).b = 0;

		viewer1.updatePointCloud(cloud_pointPickingV1, "pp_v1");
	}

	if (contPoints == 6)
	{
		Matrix4f transformation;
		const pcl::registration::TransformationEstimationSVD<PointXYZRGB, PointXYZRGB> transformationEstimation;
		transformationEstimation.estimateRigidTransformation(*cloud_pointPickingV2, *cloud_pointPickingV1, transformation);

	    transformPointCloud(*cloud_pointPickingV2, *cloud_pointPickingV2, transformation);

	    final_transformation = transformation.cast<double>();

	    exportFinalTransformation();
	}
}

void importCloudEnvironmentV1()
{
	float x, y, z;
	std::ifstream infile("/dados/calibration/cloud_environment_v1.xyz");

	//conta o numero de linhas (pontos) do arquivo
	int numLines = 0;
	std::string unused;
	while ( std::getline(infile, unused) )
	   ++numLines;

	//retorna pro inicio do arquivo
	infile.clear();
	infile.seekg(0, ios::beg);

	//iniciliza tamanho da nuvem de pontos
	cloud_environmentRemovalV1->width = numLines;
	cloud_environmentRemovalV1->points.resize (cloud_environmentRemovalV1->width * cloud_environmentRemovalV1->height);

	int i = 0;

	//percorre o arquivo e adiciona os pontos na nuvem
	while (infile >> x >> y >> z)
	{
		cloud_environmentRemovalV1->points[i].x = x;
		cloud_environmentRemovalV1->points[i].y = y;
		cloud_environmentRemovalV1->points[i].z = z;

		//cinza
		cloud_environmentRemovalV1->at(i).r = 0;
		cloud_environmentRemovalV1->at(i).g = 0;
		cloud_environmentRemovalV1->at(i).b = 0;

		i++;
	}
}

void importCloudEnvironmentV2()
{
	float x, y, z;
	std::ifstream infile("/dados/calibration/cloud_environment_v2.xyz");

	//conta o numero de linhas (pontos) do arquivo
	int numLines = 0;
	std::string unused;
	while ( std::getline(infile, unused) )
	   ++numLines;

	//retorna pro inicio do arquivo
	infile.clear();
	infile.seekg(0, ios::beg);

	//iniciliza tamanho da nuvem de pontos
	cloud_environmentRemovalV2->width = numLines;
	cloud_environmentRemovalV2->points.resize (cloud_environmentRemovalV2->width * cloud_environmentRemovalV2->height);

	int i = 0;

	//percorre o arquivo e adiciona os pontos na nuvem
	while (infile >> x >> y >> z)
	{
		cloud_environmentRemovalV2->points[i].x = x;
		cloud_environmentRemovalV2->points[i].y = y;
		cloud_environmentRemovalV2->points[i].z = z;

		//cinza
		cloud_environmentRemovalV2->at(i).r = 0;
		cloud_environmentRemovalV2->at(i).g = 0;
		cloud_environmentRemovalV2->at(i).b = 0;

		i++;
	}
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
	if ((event.getKeySym() == "r" || event.getKeySym() == "R") && event.keyDown())
	{
		contPoints = 0;
		cloud_pointPickingV1->width = 0;
		cloud_pointPickingV1->points.resize (cloud_pointPickingV1->width * cloud_pointPickingV1->height);
		cloud_pointPickingV2->width = 0;
		cloud_pointPickingV2->points.resize (cloud_pointPickingV2->width * cloud_pointPickingV2->height);

		viewer1.updatePointCloud(cloud_pointPickingV1, "pp_v1");
		viewer1.updatePointCloud(cloud_pointPickingV2, "pp_v2");
	}
	else if ((event.getKeySym() == "y" || event.getKeySym() == "Y") && event.keyDown())
	{
		final_transformation << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //transformacao previamente calculada
		exportFinalTransformation();
	}
	else if ((event.getKeySym() == "p" || event.getKeySym() == "P") && event.keyDown())
	{
		cx1 = 40; cy1 = 40; cz1 = 40; rx1 = 0; ry1 = 0; rz1 = 0; tx1 = 0; ty1 = 0; tz1 = 0;
		cx2 = 30; cy2 = 30; cz2 = 30; rx2 = 0; ry2 = 0; rz2 = 0; tx2 = 0; ty2 = 0; tz2 = 0;
		configureCubePosition(true);
	}
	else if ((event.getKeySym() == "u" || event.getKeySym() == "U") && event.keyDown())
	{
		waitSpin = !waitSpin;
	}
	else if ((event.getKeySym() == "k" || event.getKeySym() == "K") && event.keyDown())
	{
		flagSaveEnvironment = true;
	}
}

void testeVTK()
{
	pcl::io::loadPolygonFileOBJ("/dados/montagem_malha.obj", meshTeste);
	vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
	VTKUtils::convertToVTK (meshTeste, pointsPolydata);

	vtkSmartPointer<vtkPolyData> pointsPolydataExtracted = vtkSmartPointer<vtkPolyData>::New();

	//mantem apenas o maior elemento conexo
	/*
	//para visualizar a quantidade de poligonos de cada regiao

	vtkPolyDataConnectivityFilter *connectivityFilter = vtkPolyDataConnectivityFilter::New();

	connectivityFilter->SetInput(pointsPolydata);
	connectivityFilter->SetExtractionModeToAllRegions();
	connectivityFilter->Update();
	connectivityFilter->SetExtractionModeToSpecifiedRegions();

	for (int i=0; i<connectivityFilter->GetNumberOfExtractedRegions(); i++)
	{
		connectivityFilter->AddSpecifiedRegion(i);
		connectivityFilter->Update();

		cout << "Region " << i << " has " << connectivityFilter->GetOutput()->GetNumberOfPolys() << " polys.\n";
		// if region shall be dropped
		if (1) connectivityFilter->DeleteSpecifiedRegion(i);
	}
	*/

	//para manter apenas a maior regiao conectada

	vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivityFilter = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
	connectivityFilter->SetInput(pointsPolydata);

	connectivityFilter->SetExtractionModeToLargestRegion();
	connectivityFilter->Update();
	cout << "PolyData has " << connectivityFilter->GetNumberOfExtractedRegions() << " regions.\n";
	pointsPolydataExtracted = connectivityFilter->GetOutput();

	/*
	// Create a mapper and actor for original data
	vtkSmartPointer<vtkPolyDataMapper> originalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	originalMapper->SetInput(pointsPolydata);
	originalMapper->Update();

	vtkSmartPointer<vtkActor> originalActor = vtkSmartPointer<vtkActor>::New();
	originalActor->SetMapper(originalMapper);

	// Create a mapper and actor for extracted data
	vtkSmartPointer<vtkPolyDataMapper> extractedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	extractedMapper->SetInputConnection(connectivityFilter->GetOutputPort());
	extractedMapper->Update();

	vtkSmartPointer<vtkActor> extractedActor = vtkSmartPointer<vtkActor>::New();
	extractedActor->GetProperty()->SetColor(1,0,0);
	extractedActor->SetMapper(extractedMapper);

	// Visualization
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(originalActor);
	renderer->AddActor(extractedActor);

	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	interactor->SetRenderWindow(renderWindow);
	interactor->Initialize();
	interactor->Start();
	*/

	VTKUtils::convertToPCL(pointsPolydataExtracted, meshTesteFinal);

	visualization::PCLVisualizer viewerTeste;
	viewerTeste.setBackgroundColor(255, 255, 255);
	viewerTeste.addPolygonMesh(meshTesteFinal, "meshTesteFinal", 0);
	viewerTeste.spin();

}

void calculateBestFitPlane()
{
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud_environmentRemovalV1);
	seg.segment (*inliers, *planeCoefficients);

	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return;
	}

	std::cerr << "Model coefficients: " << planeCoefficients->values[0] << " "
	                                    << planeCoefficients->values[1] << " "
	                                    << planeCoefficients->values[2] << " "
	                                    << planeCoefficients->values[3] << std::endl;

	//planeCoefficients->values[3] = 0;

	/*std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
	for (size_t i = 0; i < inliers->indices.size (); ++i)
		std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
											   << cloud->points[inliers->indices[i]].y << " "
											   << cloud->points[inliers->indices[i]].z << std::endl;*/

}

int main(int argc, char **argv)
{
	FILE *index_file;
	char index_file_name[2000];
	char *log_file_name;

	memset(&odometry_ackerman, 0, sizeof(odometry_ackerman));
	memset(&velocity_ackerman, 0, sizeof(velocity_ackerman));
	
	memset(&visual_odometry, 0, sizeof(visual_odometry));
	memset(&imu, 0, sizeof(imu));
	memset(&truepos_ackerman, 0, sizeof(truepos_ackerman));
	memset(&laser_ackerman1, 0, sizeof(laser_ackerman1));
	memset(&laser_ackerman2, 0, sizeof(laser_ackerman2));
	memset(&laser_ackerman3, 0, sizeof(laser_ackerman3));
	memset(&laser_ackerman4, 0, sizeof(laser_ackerman4));
	memset(&laser_ackerman5, 0, sizeof(laser_ackerman5));
	memset(&laser_ldmrs, 0, sizeof(laser_ldmrs));
	memset(&laser_ldmrs_new, 0, sizeof(laser_ldmrs_new));
	memset(&laser_ldmrs_objects, 0, sizeof(laser_ldmrs_objects));
	memset(&laser_ldmrs_objects_data, 0, sizeof(laser_ldmrs_objects_data));
	memset(&rawlaser1, 0, sizeof(rawlaser1));
	memset(&rawlaser2, 0, sizeof(rawlaser2));
	memset(&rawlaser3, 0, sizeof(rawlaser3));
	memset(&rawlaser4, 0, sizeof(rawlaser4));
	memset(&rawlaser5, 0, sizeof(rawlaser5));
	memset(&gpsgga, 0, sizeof(gpsgga));
	memset(&gpshdt, 0, sizeof(gpshdt));
	memset(&gpsrmc, 0, sizeof(gpsrmc));
	memset(&raw_depth_kinect_0, 0, sizeof(raw_depth_kinect_0));
	memset(&raw_depth_kinect_1, 0, sizeof(raw_depth_kinect_1));
	memset(&raw_video_kinect_0, 0, sizeof(raw_video_kinect_0));
	memset(&raw_video_kinect_1, 0, sizeof(raw_video_kinect_1));
	memset(&velodyne_partial_scan, 0, sizeof(velodyne_partial_scan));
	memset(&velodyne_variable_scan, 0, sizeof(velodyne_variable_scan));
	memset(&velodyne_gps, 0, sizeof(velodyne_gps));
	memset(&xsens_euler, 0, sizeof(xsens_euler));
	memset(&xsens_quat, 0, sizeof(xsens_quat));
	memset(&xsens_matrix, 0, sizeof(xsens_matrix));
	memset(&xsens_mtig, 0, sizeof(xsens_mtig));
	memset(&bumblebee_basic_stereoimage1, 0, sizeof(bumblebee_basic_stereoimage1));
	memset(&bumblebee_basic_stereoimage2, 0, sizeof(bumblebee_basic_stereoimage2));
	memset(&bumblebee_basic_stereoimage3, 0, sizeof(bumblebee_basic_stereoimage3));
	memset(&bumblebee_basic_stereoimage4, 0, sizeof(bumblebee_basic_stereoimage4));
	memset(&bumblebee_basic_stereoimage5, 0, sizeof(bumblebee_basic_stereoimage5));
	memset(&bumblebee_basic_stereoimage6, 0, sizeof(bumblebee_basic_stereoimage6));
	memset(&bumblebee_basic_stereoimage7, 0, sizeof(bumblebee_basic_stereoimage7));
	memset(&bumblebee_basic_stereoimage8, 0, sizeof(bumblebee_basic_stereoimage8));
	memset(&bumblebee_basic_stereoimage9, 0, sizeof(bumblebee_basic_stereoimage9));
	memset(&web_cam_message, 0, sizeof(web_cam_message));
	memset(&ackerman_motion_message, 0, sizeof(ackerman_motion_message));
	memset(&ultrasonic_message, 0, sizeof(ultrasonic_message));
	memset(&ford_escape_status, 0, sizeof(ford_escape_status));
	memset(&globalpos, 0, sizeof(globalpos));

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	if (argc < 2)
		carmen_die("Error: wrong number of parameters: program requires 1 parameter and received %d parameter(s).\n"
				"Usage:\n %s <log_file_name>\n", argc - 1, argv[0]);

	if (strcmp(argv[1], "-h") == 0)
		usage(NULL);

    log_file_name = argv[1];
	logfile = carmen_fopen(log_file_name, "r");
	if (logfile == NULL)
		carmen_die("Error: could not open file %s for reading.\n", log_file_name);

	int fadvise_error = posix_fadvise(fileno(logfile->fp), 0, 0, POSIX_FADV_SEQUENTIAL);
	if (fadvise_error)
		carmen_die("Could not advise POSIX_FADV_SEQUENTIAL on playback.\n");

	strcpy(index_file_name, log_file_name);
	strcat(index_file_name, ".index");
	index_file = fopen(index_file_name, "r");
	if (index_file == NULL)
	{
		logfile_index = carmen_logfile_index_messages(logfile);
		save_logindex_file(logfile_index, index_file_name);
	}
	else if (index_file_older_than_log_file(index_file, logfile))
	{
		fclose(index_file);
		logfile_index = carmen_logfile_index_messages(logfile);
		save_logindex_file(logfile_index, index_file_name);
	}
	else
	{
		logfile_index = load_logindex_file(index_file_name);
	}

	read_parameters (argc, argv);
	define_ipc_messages();
	carmen_playback_define_messages();

	signal(SIGINT, shutdown_playback_module);

	configClouds();							//inicializa nuvens de pontos dos velodynes com zero
											//configuracao da posicao dos cubos para cada velodyne
	importCubePosition();					//carrega comprimento e posicao (rotacao e translacao) dos cubos de regiao de interesse
	configureCubePosition(false);			//aplica as transformacoes nos cubos com os dados de posicao, rotacao e translacao para posiciona-los no mundo
											//configuracao da posicao de V2 em relacao a V1
	importManualCalibration();				//carrega dados de rotacao e translacao manual do cubo de V2 para aproximar da posicao de V1
	configureManualCalibration(false);		//aplica as transformacoes manuais no cubo de V2

	importFinalTransformation();			//importa a matriz de transformacao final

	if (task == 1)
	{
		viewer1.registerPointPickingCallback(pointPicking_callback, (void*)&viewer1);
		viewer1.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer1);
	}
	else if (task == 2 || task == 3)
	{
		importCloudEnvironmentV1();			//importa nuvem do ambiente de V1
		importCloudEnvironmentV2();			//importa nuvem do ambiente de V2

		kdtree_environmentV1.setInputCloud (cloud_environmentRemovalV1);
		Matrix<double, 4, 4> cloud_environmentRemovalV2_inverse = final_transformation.inverse();
	    transformPointCloud<PointXYZRGB>(*cloud_environmentRemovalV2, *cloud_environmentRemovalV2, cloud_environmentRemovalV2_inverse);
		kdtree_environmentV2.setInputCloud (cloud_environmentRemovalV2);
		transformPointCloud<PointXYZRGB>(*cloud_environmentRemovalV2, *cloud_environmentRemovalV2, final_transformation);

		calculateBestFitPlane();
	}
	if (task == 3)
	{
		importLinesData();									//carregar os dados das retas que já devem ter sidos salvos numa primeira passagem do log

		//load_point_cloud_xyz_empty_bucket_laser();			//carrega nuvem de pontos da cacamba gerada pelo laser vazia (para alinhar com a nuvem do modelo 3D)
		//load_point_cloud_xyz_empty_bucket_model();   		//carrega nuvem de pontos da cacamba que sera usada para elimitar pontos próximos
		load_point_cloud_xyz_bucket_corners();				//nuvem de pontos apenas com pontos das laterais superiores
		load_point_cloud_xyz_empty_bucket_laser_gicp();		//carrega nuvem de pontos da cacamba que sera usada para o GICP
		//load_point_cloud_xyz_model_points();				//carrega nuvem de pontos da cacamba que sera usada para elimitar pontos próximos
		//load_point_cloud_xyz_floating_points(); 			//carrega os pontos flutuantes acima da cacamba para montagem da base da cacamba

		//load_bucket_mesh();									//carrega mesh da cacamba
		//load_bucket_mesh_simplified(); 						//carrega mesh da cacamba simplificada
	}
	if (task == 4)
	{
		PoissonFromPointCloudsOnFolder();
	}

	//calculateMeshVolume(); //TESTE
	//gicp_cloudModelAndEmptyTruck() //TESTE
	//testeVTK();

	main_playback_loop();

	return 0;
}

