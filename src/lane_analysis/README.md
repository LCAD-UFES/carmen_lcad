##Ego-Lane Analysis System

**Input Messages:**

- carmen_bumblebee_basic_stereoimage_message (only one image, usually the left one)

**Output Messages:**

1. carmen_elas_**lane_estimation**_message
2. carmen_elas_**lane_markings_type**_message
3. carmen_elas_**adjacent_lanes**_message

----

####Usage Notes:

- Camera number is required: `./lane-analysis <camera_number>`
- Output messages are timestamped based on the input `timestamp`
- A new format is defined: `IPC_defineFormat("vector_2D", "{double,double}");`

----

#####1. carmen_elas_lane_estimation_message:

	typedef struct {
		int num_outputs_left, num_outputs_right;
		carmen_vector_2D_t *left, *right;
		carmen_vector_2D_t point_bottom, point_top, direction;
		double lane_width;
		int trustworthy_height;
		double timestamp;
		char *host;
	} carmen_elas_lane_estimation_message;


#####2. carmen_elas_lane_markings_type_message:

	typedef struct {
		int left, right;
		double timestamp;
		char *host;
	} carmen_elas_lane_markings_type_message;

#####3. carmen_elas_adjacent_lanes_message:

	typedef struct {
		int left, right;
		double timestamp;
		char *host;
	} carmen_elas_adjacent_lanes_message;

----

####Development Notes:

- A calibration process will be developed to enable control over the visible distance in meters;
- A visualization will be developed on the Viewer3D (requires another calibration), as demanded;
