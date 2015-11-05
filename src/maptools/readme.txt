carmen_linemapping: Incremental learning of line models

This pragram is an approach to learn/extract line maps from laser
range data.  The key idea is to extract these line segments and
integrate them with the line segments stored in a map.

We use the split-and-merge algorithm to extract line segments from
individual range scans. This algorithm recursively subdivides the scan
into sets of neighbouring beams that can accurately be approximated by
lines.  To extract line segments from individual range scans, use the
function:
'carmen_linemapping_get_segments_from_scan(const carmen_robot_laser_message*,
int)'


To fit a line to a set of neighbouring points (from laser data) we use
a line fitting algorithm. In this approach the line that approximates
a set of points best is computed.

Our approach for incrementel learning line models uses the
split-and-merge algorithm to extract line segments from range scans
and afterwards combines these line segments with a given line map.  To
update such a map with a new range scan, use the function:
'carmen_linemapping_update_linemap(carmen_linemapping_segment_set*,
const carmen_robot_laser_message*)'

To integrate a new line segments from a single scan into a line map we
search the "closest" line segment in this map. If both end points of
the new or old line segment are close enough to the other line segment
AND if these two line segments overlaps, then we merge these two line
segments. Othwewise the new line segment will be added to the line
map.

-----
Parameters:


# linemapping parameters
#
# opening angle of the laser (180 deg for SICKs)
linemapping_laser_opening_angle        3.141592653589793

# the maximum beam length, which should be used 
# to extract lines 
linemapping_laser_max_length           6.0

# angle of the first beam relative to the robot pose
# this value is -0.5 PI for all SICKs
linemapping_laser_start_angle          -1.5707963267949   # - 0.5 * PI

# Split'n'Merge: max. distance of a point to a segment in the "split"-step
# (e.g.: if distance>sam_tolerance then the point set will be splited)
# with smaler values you get less line segments, but more accurate ones
linemapping_sam_tolerance              0.1

# Split'n'Merge:  max. distance of neighbouring points, so that a segment 
# will be created. 
# (E.g. if 'distance>sam_max_gap' then the point set will be splited)
linemapping_sam_max_gap                0.3

# Split'n'Merge: minimum length of a line segment
linemapping_sam_min_length             0.4

# Split'n'Merge: minimun number of points on a line segment 
linemapping_sam_min_num                5

# use the fitting algorithm when merging lines
linemapping_sam_use_fit_split          off

# max. distance of the two end points of a line segment for merging
linemapping_merge_max_dist             0.1

# the minimum overlap between to lines before they get merged (relative)
linemapping_merge_min_relative_overlap  0.2

# the minimum overlap between to lines before they get merged (in m)
linemapping_merge_overlap_min_length   0.2

# when mergeing, distribute points over the linesegment for re-computation
linemapping_merge_uniformly_distribute_dist 0.05




