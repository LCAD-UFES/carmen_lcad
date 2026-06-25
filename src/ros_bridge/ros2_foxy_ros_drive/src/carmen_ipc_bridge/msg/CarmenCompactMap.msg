# Compact representation of a CARMEN occupancy map cell update.
# Mirrors carmen_compact_map_t — only changed cells are transmitted.

std_msgs/Header header

# Map metadata
float64 resolution     # metres per cell
float64 origin_x       # global X of cell (0,0)
float64 origin_y       # global Y of cell (0,0)
int32   x_size
int32   y_size

# Sparse cell updates
int32[]   cell_x
int32[]   cell_y
float32[] occupancy    # log-odds value

float64 timestamp
