#include <epik/mtt/MovBox2DTracker.h>
#include <epik/mtt/PlotList.h>
#include <epik/mtt/TrackList.h>
#include <epik/tracker/TrackerConfig.h>
#include <epik/utils/TimeUtils.h>
#include <epik/utils/MiscUtils.h>

#include <opencv2/highgui/highgui.hpp>

#include <carmen/carmen.h>
#include <carmen/virtual_scan_interface.h>
#include <carmen/global_graphics.h>
#include <carmen/map_server_interface.h>
#include <carmen/map.h>
#include <carmen/grid_mapping.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_transforms.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/matrix.h>
#include <carmen/kalman.h>

#include "virtual_scan.h"

#define DEFAULT_SCAN_RATE 5.0 // [Hz]
#define DEFAULT_SCAN_PERIOD 1.0/DEFAULT_SCAN_RATE // [s]
#define DEFAULT_SURVEILLANCE_REGION_CENTER Point2D(50, 50) // [m]x[m]
#define DEFAULT_SURVEILLANCE_REGION_SIZE Point2D(100, 100) // [m]x[m]
#define DEFAULT_SENSOR_COVERAGE_CENTER Point2D(50, 50) // [m]x[m]
#define DEFAULT_SENSOR_COVERAGE_RADIUS 100.0 // [m]

#define PLOTS_WINDOW_TITLE "DetectedVirtualScanSegments"
#define PLOTS_WINDOW_WIDTH 384
#define PLOTS_WINDOW_HEIGHT 384
#define PLOTS_WINDOW_POSITION_X 0
#define PLOTS_WINDOW_POSITION_Y 0

#define TRACKS_WINDOW_TITLE "TrackedMovingObjects"
#define TRACKS_WINDOW_WIDTH 384
#define TRACKS_WINDOW_HEIGHT 384
#define TRACKS_WINDOW_POSITION_X 384
#define TRACKS_WINDOW_POSITION_Y 0

MovBox2DTracker g_tracker;
PlotList *g_plots = NULL;
Box2D *g_surveillance_region = NULL;
Circle *g_sensor_coverage = NULL;
TrackList *g_tracks = NULL;
double g_initial_time = 0.0;
double g_current_time = 0.0;
double g_previous_time = 0.0;
double g_delta_time = 0.0;
virtual_scan_track_set_t *best_track_set = NULL;
extern carmen_point_t g_initial_pos;
extern carmen_point_t g_current_pos;
extern double x_origin;
extern double y_origin;
extern double x_size;
extern double y_size;
extern double map_resolution;

///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Main functions																			 //
//																							 //
///////////////////////////////////////////////////////////////////////////////////////////////


void
virtual_scan_tracker_initialize(void)
{
	TrackerConfig::Initialize();
	
	g_plots = new PlotList();	
	g_surveillance_region = new Box2D(DEFAULT_SURVEILLANCE_REGION_CENTER, DEFAULT_SURVEILLANCE_REGION_SIZE);
	g_sensor_coverage = new Circle(DEFAULT_SENSOR_COVERAGE_CENTER, DEFAULT_SENSOR_COVERAGE_RADIUS);
	
	g_tracker.SetInput("PlotList", &g_plots);
	g_tracker.SetInput("SurveillanceRegion", &g_surveillance_region);	
	g_tracker.SetInput("SensorCoverage", &g_sensor_coverage);
	
	g_tracks = *((TrackList **) g_tracker.GetOutputByName("OutputTrackList"));
		
	g_tracker.Initialize();
	
	g_initial_time = g_current_time = g_previous_time = TimeUtils::GetCurrentTime();
	
	best_track_set = (virtual_scan_track_set_t *) malloc(sizeof(virtual_scan_track_set_t));
	best_track_set->size = 0;
	best_track_set->tracks = (virtual_scan_track_t **) malloc(MAX_NUMBER_OF_TRACKS * sizeof(virtual_scan_track_t *));
	for (int i = 0; i < MAX_NUMBER_OF_TRACKS; i++)
	{
		best_track_set->tracks[i] = (virtual_scan_track_t *) malloc(sizeof(virtual_scan_track_t));
		best_track_set->tracks[i]->track_id = 0;
		best_track_set->tracks[i]->size = 1;
		best_track_set->tracks[i]->box_model_hypothesis = (virtual_scan_box_model_hypothesis_t *) malloc(sizeof(virtual_scan_box_model_hypothesis_t));
	}
	best_track_set->vertex_selected = NULL;	
}


void
virtual_scan_tracker_finalize(void)
{
	g_tracker.Finalize();
	
	TrackerConfig::Finalize();
	
	g_plots->clear();
	
	delete g_plots;
	delete g_surveillance_region;	
	delete g_sensor_coverage;
}


virtual_scan_track_set_t *
virtual_scan_infer_moving_objects(carmen_mapper_virtual_scan_message *virtual_scan_extended, virtual_scan_segment_classes_t *virtual_scan_segment_classes, double frame_timestamp)
{
// 	g_delta_time = TimeUtils::GetCurrentTime() - g_previous_time;
// 	
// 	if (g_delta_time < DEFAULT_SCAN_PERIOD)
// 		usleep((DEFAULT_SCAN_PERIOD-g_delta_time)*1000000.0);
	
	// Get the current timestamp in seconds since UNIX epoch GMT
	g_current_time = TimeUtils::GetCurrentTime();
	
	// Set the surveillance region as the current map extension
	g_surveillance_region->size.x = map_resolution*x_size;
	g_surveillance_region->size.y = map_resolution*y_size;
	g_surveillance_region->center.x = x_origin + 0.5*g_surveillance_region->size.x;
	g_surveillance_region->center.y = y_origin + 0.5*g_surveillance_region->size.y;	
// 	printf("Surveillance region (%f, %f, %f, %f)\n", g_surveillance_region->center.x, g_surveillance_region->center.y, g_surveillance_region->size.x, g_surveillance_region->size.y);

	
	for (int s = 0; s < virtual_scan_extended->num_sensors; s++)
	{
		int sensor_id = virtual_scan_extended->virtual_scan_sensor[s].sensor_id;
		
		// Set the sensor coverage accordingly
		g_sensor_coverage->center.x = g_current_pos.x;
		g_sensor_coverage->center.y = g_current_pos.y;	
// 		printf("Sensor coverage (%f, %f, %f)\n", g_sensor_coverage->center.x, g_sensor_coverage->center.y, g_sensor_coverage->radius);

		// Encode the virtual segments as plots
		g_plots->clear();
		g_plots->m_sensorIdx = s;
		for (int i = 0; i < virtual_scan_segment_classes->num_segments; i++)
		{
			if (virtual_scan_segment_classes->segment[i].sensor_id != sensor_id)
				continue;
			
			Plot plot;
			plot.measurement.center.x = virtual_scan_segment_classes->segment[i].centroid.x;
			plot.measurement.center.y = virtual_scan_segment_classes->segment[i].centroid.y;
			plot.measurement.size.x = virtual_scan_segment_classes->segment_features[i].width;
			plot.measurement.size.y = virtual_scan_segment_classes->segment_features[i].length;
			plot.measurement.category = virtual_scan_segment_classes->segment_features[i].segment_class;
			
			carmen_point_t first_point = virtual_scan_segment_classes->segment_features[i].first_point;
			carmen_point_t last_point = virtual_scan_segment_classes->segment_features[i].last_point;
			carmen_point_t farthest_point = virtual_scan_segment_classes->segment_features[i].farthest_point;
						
			switch (virtual_scan_segment_classes->segment_features[i].segment_class)
			{
				case L_SHAPED: 
				{
					double l = DIST2D(farthest_point, first_point);
					double w = DIST2D(farthest_point, last_point);
					if (l > w)
						plot.measurement.orientation = carmen_normalize_theta(atan2(first_point.y - farthest_point.y, first_point.x - farthest_point.x));
					else
						plot.measurement.orientation = carmen_normalize_theta(atan2(last_point.y - farthest_point.y, last_point.x - farthest_point.x));
				}
				break;
				
				case I_SHAPED: 
				{
					plot.measurement.orientation = carmen_normalize_theta(atan2(last_point.y - first_point.y, last_point.x - first_point.x));
				}
				break;
				
				case MASS_POINT:
				{
					plot.measurement.orientation = .0;				
				}
				break;
				
				default:
				{
					plot.measurement.orientation = .0;
				}
				break;
			}
		
			
			plot.timeStamp = g_current_time;
			g_plots->push_back(plot);		
// 			printf("Plot %d (%f, %f, %f, %f, %d, %lf)\n", i, plot.measurement.center.x, plot.measurement.center.y, plot.measurement.size.x, plot.measurement.size.y, plot.measurement.category, plot.timeStamp);
		}
		
		// Show the plots for debugging purposes
// 		virtual_scan_show_plots(sensor_id);

		// Run one tracking iteration
		g_tracker.Process(g_current_time);
	}
			
	// Show the tracks for debugging purposes
// 	virtual_scan_show_tracks();
	
	// Encode the output tracks as box models
	best_track_set->size = g_tracks->size();
	for (size_t i = 0; i < g_tracks->size(); i++)
	{		
		best_track_set->tracks[i]->size = 1;
		best_track_set->tracks[i]->track_id = g_tracks->at(i).id;
		best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis_state.x = g_tracks->at(i).state.center.x;
		best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis_state.y = g_tracks->at(i).state.center.y;
		carmen_rect_to_polar(g_tracks->at(i).state.vel.x,g_tracks->at(i).state.vel.y, 
							 &(best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis_state.v),
							 &(best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis_state.theta));
		switch (g_tracks->at(i).state.category)
		{
			case L_SHAPED: // L-shape segments segments will generate bus and car hypotheses
			{
				if (g_tracks->at(i).state.size.x >= 2.4 && g_tracks->at(i).state.size.x <= 2.6 && g_tracks->at(i).state.size.y >= 10.0 && g_tracks->at(i).state.size.y <= 14.0)
					best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.c = BUS;
				else
					best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.c = CAR;
			}
			break;
			
			case I_SHAPED: // I-shape segments segments will generate bus, car and bike hypotheses
			{
				if (g_tracks->at(i).state.size.x >= 2.4 && g_tracks->at(i).state.size.x <= 2.6 && g_tracks->at(i).state.size.y >= 10.0 && g_tracks->at(i).state.size.y <= 14.0)
					best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.c = BUS;
				else if (g_tracks->at(i).state.size.x >= 1.8 && g_tracks->at(i).state.size.x <= 2.1 && g_tracks->at(i).state.size.y >= 3.9 && g_tracks->at(i).state.size.y <= 5.3)
					best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.c = CAR;
				else
					best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.c = BIKE;
			}
			break;
			
			case MASS_POINT:
			{
				best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.c = PEDESTRIAN;
			}
			break;
			
			default:
			{
				best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.c = CAR;
			}
			break;
		}		
		best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.x = g_tracks->at(i).state.center.x;
		best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.y = g_tracks->at(i).state.center.y;
		best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.width = g_tracks->at(i).state.size.x;
		best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.length = g_tracks->at(i).state.size.y;
		best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.theta = g_tracks->at(i).state.orientation;
		best_track_set->tracks[i]->box_model_hypothesis[0].frame_timestamp = frame_timestamp;
// 		printf("Track %d (%d, %f, %f, %f, %f, %f, %lf)\n", best_track_set->tracks[i]->track_id, best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.c, best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.x, best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.y, best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.width, best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.length, best_track_set->tracks[i]->box_model_hypothesis[0].hypothesis.theta, best_track_set->tracks[i]->box_model_hypothesis[0].frame_timestamp);
	}

	// Wait for while
	cv::waitKey(1); 
	
	return best_track_set;
}


void 
virtual_scan_show_tracks()
{
	static bool flag = true;
	cv::namedWindow(TRACKS_WINDOW_TITLE, cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
	if (flag)
	{
		cv::moveWindow(TRACKS_WINDOW_TITLE, TRACKS_WINDOW_POSITION_X, TRACKS_WINDOW_POSITION_Y);
		flag = false;
	}
	
	cv::Mat image(TRACKS_WINDOW_WIDTH, TRACKS_WINDOW_HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
	
	double sx =  (double) (TRACKS_WINDOW_WIDTH-1)/g_surveillance_region->size.x;
	double sy = -(double) (TRACKS_WINDOW_HEIGHT-1)/g_surveillance_region->size.y;
	
	double ox = -g_surveillance_region->center.x + 0.5* g_surveillance_region->size.x;
	double oy = -g_surveillance_region->center.y - 0.5* g_surveillance_region->size.y;
	
	// Draw targets
	size_t nMovingTracks = 0;
	size_t nStaticTracks = 0;
	for (TrackList::iterator it = g_tracks->begin(); it != g_tracks->end(); it++)
	{
		it->Draw(image, sx, sy, ox, oy);
		if (it->moving)
			nMovingTracks++;
		else
			nStaticTracks++;
	}
	
	// Draw info
	cv::putText(image, "Number of moving tracks = " + MiscUtils::toStr(nMovingTracks), cv::Point(10,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, CV_AA);
	cv::putText(image, "Number of static tracks = " + MiscUtils::toStr(nStaticTracks), cv::Point(10,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, CV_AA);
	cv::putText(image, "Scan rate = " + MiscUtils::toStr(SCAN_RATE) + " Hz", cv::Point(10,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, CV_AA);
	
	// Draw surveillance region
	g_surveillance_region->Draw(image, sx, sy, ox, oy, cv::Scalar(200,200,200));
	
	// Draw sensor coverage area
	g_sensor_coverage->Draw(image, sx, sy, ox, oy, cv::Scalar(0,200,0));
	
	cv::imshow(TRACKS_WINDOW_TITLE, image);
}


void 
virtual_scan_show_plots(int sensor_id)
{
	string window_title = PLOTS_WINDOW_TITLE+MiscUtils::toStr(sensor_id);	
	cv::namedWindow(window_title.c_str(), cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);

	static bool flag = true;
	if (flag)
	{
		cv::moveWindow(window_title.c_str(), PLOTS_WINDOW_POSITION_X, PLOTS_WINDOW_POSITION_Y);
		flag = false;
	}
	
	cv::Mat image(PLOTS_WINDOW_WIDTH, PLOTS_WINDOW_HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
	
	double sx =  (double) (PLOTS_WINDOW_WIDTH-1)/g_surveillance_region->size.x;
	double sy = -(double) (PLOTS_WINDOW_HEIGHT-1)/g_surveillance_region->size.y;
	
	double ox = -g_surveillance_region->center.x + 0.5* g_surveillance_region->size.x;
	double oy = -g_surveillance_region->center.y - 0.5* g_surveillance_region->size.y;
	
	// Draw detections
	for (PlotList::iterator it = g_plots->begin(); it != g_plots->end(); it++)
	{
		it->Draw(image, sx, sy, ox, oy);
	}
	
	// Draw info
	cv::putText(image, "Number of plots = " + MiscUtils::toStr(g_plots->size()), cv::Point(10,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, CV_AA);
	
	// Draw surveillance region
	g_surveillance_region->Draw(image, sx, sy, ox, oy, cv::Scalar(200,200,200));
	
	// Draw sensor coverage area
	g_sensor_coverage->Draw(image, sx, sy, ox, oy, cv::Scalar(0,200,0));
	
	cv::imshow(window_title.c_str(), image);
}


void
virtual_scan_free_moving_objects(carmen_moving_objects_point_clouds_message *moving_objects)
{
	if (moving_objects != NULL)
	{
		free(moving_objects->point_clouds);
		free(moving_objects);
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
