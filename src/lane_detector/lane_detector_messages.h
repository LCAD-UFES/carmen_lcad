/*********************************************************
 Lane Analysis Module
 *********************************************************/

#ifndef CARMEN_LANE_DETECTOR_MESSAGES_H
#define CARMEN_LANE_DETECTOR_MESSAGES_H

#include <carmen/carmen.h>
#include <carmen/global.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Lane Markings Types
enum LMT {
	NONE = 0,
	SCB = 1, // LMS-1	-> [S]imples	[C]ont�nua		[B]ranca
	SSB = 2, // LMS-2	-> [S]imples	[S]eccionada	[B]ranca
	SCA = 3, // LFO-1	-> [S]imples	[C]ont�nua		[A]marela
	SSA = 4, // LFO-2	-> [S]imples	[S]eccionada	[A]marela
	DCA = 5, // LFO-3	-> [D]upla		[C]ont�nua		[A]marela
	DSC = 6, // LFO-4b	-> [D]upla		[S]eccionada	[C]ontinua		(esquerda [S] | [C] direita) Amarela
	DCS = 7 // LFO-4a	-> [D]upla		[C]ontinua		[S]eccionada	(esquerda [C] | [S] direita) Amarela
};

typedef struct {
	int lane_class;                           //
	carmen_position_t lane_segment_position;  // Lane Segment position on the map
}lane;


typedef struct {
	int lane_vector_size;
	lane* lane_vector;
	double timestamp;
	char *host;
} carmen_lane_message;

#define CARMEN_LANE_NAME "carmen_lane_message"
#define CARMEN_LANE_FMT "{int,<{int,{double,double}}>,double,string}"


#ifdef __cplusplus
}
#endif

#endif
