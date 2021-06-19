#include "movable_object.h"


double
no_out_mean(vector<double> values)// calculates the mean value removing outlayers(values > 1.5*std_dev)
{
	if (values.size() == 0)
		return 0.0;
	double mean = 0;
	for (unsigned int i=0; i<values.size(); i++)
		mean += values[i];
	mean /= values.size();
	double std = 0;
	for (unsigned int i=0; i<values.size(); i++)
		std += abs(mean-values[i]);
	std /= values.size();
	double sum = 0;
	int elements = 0;
	for (unsigned int i=0; i<values.size(); i++)
	{
		if (abs(mean-values[i]) < 1.5*std) //change outlayer treshold by changing this
		{
			sum += values[i];
			elements++;
		}
	}
	if (elements == 0)
		return 0.0;
	return sum/elements;
}

double 
slope_angle(const vector<double>& x, const vector<double>& y)
{
	if (x.size() < 2)
		return 0.0;

    double n = x.size();

    double avgX = accumulate(x.begin(), x.end(), 0.0) / n;
    double avgY = accumulate(y.begin(), y.end(), 0.0) / n;

    double numerator = 0.0;
    double denominator = 0.0;

    for(int i=0; i<n; ++i){
        numerator += (x[i] - avgX) * (y[i] - avgY);
        denominator += (x[i] - avgX) * (x[i] - avgX);
    }

    if(denominator == 0){
        return 0.0;
    }
    return atan2(numerator,denominator);
}

void 
update_world_position(movable_object* p, double new_x, double new_y, double new_timestamp)
{
	p->circular_idx = (p->circular_idx + 1) % P_BUFF_SIZE;
	p->timestamp[p->circular_idx] = new_timestamp;
	p->x_world[p->circular_idx] = new_x;
	p->y_world[p->circular_idx] = new_y;

	p->velocity = 0.0;
	p->orientation = 0.0;

	vector<double> velx_vect;
	vector<double> vely_vect;
	vector<double> valid_x;
	vector<double> valid_y;
	vector<double> ori;

	int i = 0;
	for (i = 0; i < P_BUFF_SIZE-1; i++)
	{
		int idx = (p->circular_idx+P_BUFF_SIZE-i) % P_BUFF_SIZE;
		int prev_idx = (p->circular_idx+P_BUFF_SIZE-i-1) % P_BUFF_SIZE;
		if (p->x_world[prev_idx] == -999.0 && p->y_world[prev_idx] == -999.0)
			break;
		double delta_x = p->x_world[idx]-p->x_world[prev_idx];
		double delta_y = p->y_world[idx]-p->y_world[prev_idx];
		double delta_t = p->timestamp[idx]-p->timestamp[prev_idx];

		//printf("DELTAS: %f ; %f ; %f --- Vel = %f\n",delta_x, delta_y, delta_t,new_vel);
		velx_vect.push_back(delta_x/delta_t);
		vely_vect.push_back(delta_y/delta_t);
		valid_x.push_back(p->x_world[idx]);
		valid_y.push_back(p->y_world[idx]);
		ori.push_back(atan2(delta_y,delta_x));
	}

	double slope = carmen_normalize_theta(slope_angle(valid_x,valid_y));
	double mean_ori = carmen_normalize_theta(no_out_mean(ori));
	if (abs(carmen_normalize_theta(mean_ori-slope)) < abs(carmen_normalize_theta(mean_ori-slope-M_PI)))
		p->orientation = slope;
	else
		p->orientation = carmen_normalize_theta(slope-M_PI);
	double velx = no_out_mean(velx_vect);
	double vely = no_out_mean(vely_vect);
	p->velocity = sqrt(velx*velx+vely*vely);
}

void
update_movable_object_bbox(movable_object* p,short* bbox_vector)
{
	p->x = bbox_vector[0];
	p->y = bbox_vector[1];
	p->w = bbox_vector[2];
	p->h = bbox_vector[3];

	p->active = true;
}

double
get_movable_object_x(movable_object p)
{
	return p.x_world[p.circular_idx];
}

double
get_movable_object_y(movable_object p)
{
	return p.y_world[p.circular_idx];
}


movable_object 
create_movable_object(int track_id)
{
	movable_object p;

	p.circular_idx = P_BUFF_SIZE-1;
	p.track_id = track_id;

	for(int i = 0; i<P_BUFF_SIZE; i++)
	{
		p.timestamp[i] = 0;
		p.x_world[i] = -999.0;
		p.y_world[i] = -999.0;
	}
	p.velocity = 0.0;
	p.orientation = 0.0;
	p.active = true;
	p.last_timestamp = 0;
	return p;
}

