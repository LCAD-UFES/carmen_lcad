#include "mvog_model/mapper.h"

#define SCAN_RANGE_MIN 0.5
#define SCAN_RANGE_MAX 30.0

namespace MVOG
{

Mapper::Mapper(double resolution, double sizeXmeters, double sizeYmeters):
        map_(resolution, sizeXmeters, sizeYmeters)
{
  modelNegativeSpace_  = true;

}

Mapper::~Mapper ()
{


}

void Mapper::setModelNegativeSpace(bool modelNegativeSpace)
{
  modelNegativeSpace_ = modelNegativeSpace;
}

bool Mapper::getModelNegativeSpace() const
{
  return modelNegativeSpace_;
}

Map * Mapper::getMap()
{
  return &map_;
}

carmen_vector_3D_t get_laser_reading_position_car_reference(double angle, double range)
{
	carmen_vector_3D_t laser_reference;

	laser_reference.x = cos(angle) * range;
	laser_reference.y = sin(angle) * range;
	laser_reference.z = 0;

	double sinRoll = sin(0.0);
	double cosRoll = cos(0.0);
	double sinPitch = sin(-0.25);
	double cosPitch = cos(-0.25);
	double sinYaw = sin(0.0);
	double cosYaw = cos(0.0);

	double rotation[3][3];
	rotation[0][0] = cosPitch*cosYaw;
	rotation[0][1] = sinRoll*sinPitch*cosYaw-cosRoll*sinYaw;
	rotation[0][2] = cosRoll*sinPitch*cosYaw+sinRoll*sinYaw;
	rotation[1][0] = cosPitch*sinYaw;
	rotation[1][1] = sinRoll*sinPitch*sinYaw+cosRoll*cosYaw;
	rotation[1][2] = cosRoll*sinPitch*sinYaw-sinRoll*cosYaw;
	rotation[2][0] = -sinPitch;
	rotation[2][1] = sinRoll*cosPitch;
	rotation[2][2] = cosRoll*cosPitch;

	carmen_vector_3D_t car_reference;

	car_reference.x = rotation[0][0]*laser_reference.x + rotation[1][0]*laser_reference.y + rotation[2][0]*laser_reference.z;
	car_reference.y = rotation[0][1]*laser_reference.x + rotation[1][1]*laser_reference.y + rotation[2][1]*laser_reference.z;
	car_reference.z = rotation[0][2]*laser_reference.x + rotation[1][2]*laser_reference.y + rotation[2][2]*laser_reference.z;

	car_reference.x = car_reference.x + 2.25;
	car_reference.y = car_reference.y + 0.0;
	car_reference.z = car_reference.z + 0.2;

	carmen_vector_3D_t reading;
	reading.x = car_reference.x;
	reading.y = car_reference.y;
	reading.z = car_reference.z;

	return reading;

}


void Mapper::removeLaserData (carmen_laser_laser_message* scan, const btTransform& w2l)
{
  boost::mutex::scoped_lock(map_.mutex_);

  // position of the laser in the world coordinates
	btVector3 origin = w2l * btVector3(0.0, 0.0, 0.0);

  double scanAngle = scan->config.start_angle;
  unsigned int num_readings = scan->num_readings;
  for (size_t i = 10; i < num_readings-10; i++)
  {
#ifdef SCAN_RANGE_MAX
    if (scan->range[i] > SCAN_RANGE_MIN && scan->range[i] < SCAN_RANGE_MAX)
    {
#else
	if (scan->range[i] > SCAN_RANGE_MIN && scan->range[i] < scan->config.maximum_range)
	{
#endif
      // valid, in range reading

		  carmen_vector_3D_t v = get_laser_reading_position_car_reference(scanAngle, scan->range[i]);
		  //btVector3 obstacle = w2l * btVector3(cos(scanAngle)*scan->range[i], sin(scanAngle)*scan->range[i], sin(carmen_degrees_to_radians(25.0)*scan->range[i]-1.0));
		  btVector3 obstacle = w2l * btVector3(v.x, v.y, v.z);
		  removeBeamReading(origin, obstacle);
    }
		else if (scan->range[i] > SCAN_RANGE_MAX || scan->range[i] == 0)
		{
      // out of range reading
    }
    else
    {
      // invalid reading - too close, or error

    }

    // increment scan angle
    scanAngle += scan->config.angular_resolution;
  }
}


void Mapper::addLaserData (carmen_laser_laser_message* scan, const btTransform& w2l)
{
  boost::mutex::scoped_lock(map_.mutex_);

  // position of the laser in the world coordinates
	btVector3 origin = w2l * btVector3(0.0, 0.0, 0.0);

  double scanAngle = scan->config.start_angle;
  unsigned int num_readings = scan->num_readings;
  for (size_t i = 10; i < num_readings-10; i++)
  {
#ifdef SCAN_RANGE_MAX
    if (scan->range[i] > SCAN_RANGE_MIN && scan->range[i] < SCAN_RANGE_MAX)
    {
#else
	if (scan->range[i] > SCAN_RANGE_MIN && scan->range[i] < scan->config.maximum_range)
	{
#endif
      // valid, in range reading

		  carmen_vector_3D_t v = get_laser_reading_position_car_reference(scanAngle, scan->range[i]);
		  btVector3 obstacle = w2l * btVector3(v.x, v.y, v.z);
		  addBeamReading(origin, obstacle);
    }
		else if (scan->range[i] > SCAN_RANGE_MAX || scan->range[i] == 0)
		{
      // out of range reading
    }
    else
    {
      // invalid reading - too close, or error

    }

    // increment scan angle
    scanAngle += scan->config.angular_resolution;
  }
}

void Mapper::addBumblebeeData (float* scan, int width, int height, const btTransform& w2l, double ahfov, int sample)
{
	boost::mutex::scoped_lock(map_.mutex_);

	// position of the laser in the world coordinates
	btVector3 origin = w2l * btVector3(0.0, 0.0, 0.0);

	unsigned int num_readings_x = width;
	unsigned int num_readings_y = height;

	double hfov = ahfov;
	double vfov = hfov * ((double)height/(double)width);

	double scan_angle_y = 0.0;
	double scan_angle_x = 0.0;

	double focal_length_in_pixels = ((double)width ) / (2.0 * tan(carmen_degrees_to_radians(hfov/ 2.0)));

	for(size_t j=0; j < num_readings_y; j+=3*sample)
	{
		//scan_angle_x = hfov/2.0;
		scan_angle_y = -atan((j - (((double)height)/2.0))/focal_length_in_pixels);

		for (size_t i = 0; i < num_readings_x; i+=4*sample)
		{
			scan_angle_x = -atan((i - (((double)width)/2.0))/focal_length_in_pixels);

			int k = j * num_readings_x + i;

			if (scan[k] > SCAN_RANGE_MIN && scan[k] < SCAN_RANGE_MAX)
			{
				// valid, in range reading
				btVector3 obstacle = w2l * btVector3(cos(scan_angle_x)*scan[k]/cos(scan_angle_x), sin(scan_angle_x)*scan[k]/cos(scan_angle_x), sin(scan_angle_y)*scan[k]/cos(scan_angle_y));
				addBeamReading(origin, obstacle);
			}
			else if (scan[k] > SCAN_RANGE_MAX || scan[k] == 0)
			{
				// out of range reading
			}
			else
			{
				// invalid reading - too close, or error

			}

			//printf("sax: %6.2f, say: %6.2f\n", scan_angle_x, scan_angle_y);
			// increment scan angle
			//scan_angle_x -= scan_step_x*sample;

		}
		//scan_angle_y -= scan_step_y*sample;
	}
}

void Mapper::addKinectData(carmen_kinect_depth_message* scan, const btTransform& w2l, double ahfov, int sample)
{
	boost::mutex::scoped_lock(map_.mutex_);

	// position of the laser in the world coordinates
	btVector3 origin = w2l * btVector3(0.0, 0.0, 0.0);

	unsigned int num_readings_x = scan->width;
	unsigned int num_readings_y = scan->height;

	double hfov = ahfov;
	double vfov = hfov * ((double)scan->height/(double)scan->width);

	//double scan_step_x = hfov/((double)scan->width);
	//double scan_step_y = vfov/((double)scan->height);

	double scan_angle_y = 0.0;
	double scan_angle_x = 0.0;

	double focal_length_in_pixels = ((double)scan->width ) / (2.0 * tan(carmen_degrees_to_radians(hfov/ 2.0)));

	for(size_t j=0; j < num_readings_y; j+=3*sample)
	{
		//scan_angle_x = hfov/2.0;
		scan_angle_y = -atan((j - (((double)scan->height)/2.0))/focal_length_in_pixels);

		for (size_t i = 0; i < num_readings_x; i+=4*sample)
		{
			scan_angle_x = -atan((i - (((double)scan->width)/2.0))/focal_length_in_pixels);

			int k = j * num_readings_x + i;

			if (scan->depth[k] > SCAN_RANGE_MIN && scan->depth[k] < SCAN_RANGE_MAX)
			{
				// valid, in range reading
				btVector3 obstacle = w2l * btVector3(cos(scan_angle_x)*scan->depth[k]/cos(scan_angle_x), sin(scan_angle_x)*scan->depth[k]/cos(scan_angle_x), sin(scan_angle_y)*scan->depth[k]/cos(scan_angle_y));
				addBeamReading(origin, obstacle);
			}
			else if (scan->depth[k] > SCAN_RANGE_MAX || scan->depth[k] == 0)
			{
				// out of range reading
			}
			else
			{
				// invalid reading - too close, or error

			}

			//printf("sax: %6.2f, say: %6.2f\n", scan_angle_x, scan_angle_y);
			// increment scan angle
			//scan_angle_x -= scan_step_x*sample;

		}
		//scan_angle_y -= scan_step_y*sample;
	}
}

void Mapper::addBeamReading(btVector3 origin, btVector3 obstacle)
{
  // **** convert to grid scale
  origin   /= map_.getResolution();
  obstacle /= map_.getResolution();

  addPositiveBeamSpace(obstacle);
  if (modelNegativeSpace_) addNegativeBeamSpace(origin, obstacle);
}

void Mapper::removeBeamReading(btVector3 origin, btVector3 obstacle)
{
  // **** convert to grid scale
  origin   /= map_.getResolution();
  obstacle /= map_.getResolution();

  map_.clearCell(obstacle.getX(), obstacle.getY());
}

void Mapper::addPositiveBeamSpace(btVector3 obstacle)
{
  // **** add a positive volume
  map_.getCell(obstacle.getX(), obstacle.getY())->
    addPVolume(obstacle.getZ() - 0.5, obstacle.getZ() + 0.5);
}

void Mapper::addNegativeBeamSpace(btVector3 origin, btVector3 obstacle)
{
  // **** precalculate some variables

  btVector3 beam = obstacle - origin;

  double cx    = origin.getX();
  double cy    = origin.getY();
  double cz    = origin.getZ();
  double cd    = 0.0;

  double beamLength = beam.length();

  double slopeYX  = beam.getY() / beam.getX();
  double slopeZX  = beam.getZ() / beam.getX();
  double slopeZY  = beam.getZ() / beam.getY();
  double slopeXY  = beam.getX() / beam.getY();
  double slopeZYX = beam.getZ() / sqrt(beam.getX()*beam.getX() + beam.getY()*beam.getY());
  double slopeDX  = beamLength / beam.getX();
  double slopeDY  = beamLength / beam.getY();

  // **** rasterize

  Cell * currentCell;
  double pz;

  while(1)
  {
    pz = cz;

    // **** rasterize in the appropriate quadrant

	  if      (beam.getX() >= 0 && beam.getY() >= 0)	  // **** Q1
    {
      double dx = floor(cx) + 1.0 - cx; // distance to right cell wall
      double dy = floor(cy) + 1.0 - cy; // distance to top cell wall

      currentCell = map_.getCell(cx, cy);

      if (dy > dx * slopeYX)
      {
        cx += dx;
        cy += dx * slopeYX;
        cz += dx * slopeZX;
        cd += dx * slopeDX;
      }
      else
      {
        cx += dy * slopeXY;
        cy += dy;
        cz += dy * slopeZY;
        cd += dy * slopeDY;
      }
    }
	  else if (beam.getX() <  0 && beam.getY() >= 0)		// **** Q2
    {
      double dx = floor(cx) - cx;       // distance to left cell wall
      double dy = floor(cy) + 1.0 - cy; // distance to top cell wall

      if (dx == 0.0) dx = -1.0;

      currentCell = map_.getCell(cx + dx, cy);

      if (dy > dx * slopeYX)
      {
        cx += dx;
        cy += dx * slopeYX;
        cz += dx * slopeZX;
        cd += dx * slopeDX;
      }
      else
      {
        cx += dy * slopeXY;
        cy += dy;
        cz += dy * slopeZY;
        cd += dy * slopeDY;
      }
    }
    else if (beam.getX() <= 0 && beam.getY() <  0)		// **** Q3
    {
      double dx = floor(cx) - cx; // distance to right cell wall
      double dy = floor(cy) - cy; // distance to top cell wall

      if (dx == 0) dx = -1.0;
      if (dy == 0) dy = -1.0;

      currentCell = map_.getCell(cx + dx, cy+dy);

      if (dy < dx * slopeYX)
      {
        cx += dx;
        cy += dx * slopeYX;
        cz += dx * slopeZX;
        cd += dx * slopeDX;
      }
      else
      {
        cx += dy * slopeXY;
        cy += dy;
        cz += dy * slopeZY;
        cd += dy * slopeDY;
      }
    }
    else                                           	// **** Q4
    {
      double dx = floor(cx) + 1.0 - cx; // distance to right cell wall
      double dy = floor(cy) - cy; // distance to top cell wall

      if (dy == 0.0) dy = -1.0;

      currentCell = map_.getCell(cx, cy + dy);

      if (dy < dx * slopeYX)
      {
        cx += dx;
        cy += dx * slopeYX;
        cz += dx * slopeZX;
        cd += dx * slopeDX;
      }
      else
      {
        cx += dy * slopeXY;
        cy += dy;
        cz += dy * slopeZY;
        cd += dy * slopeDY;
      }
    }

    // check if we reached the end
    if (cd < beamLength)
    {
      currentCell->addNVolume(pz, cz);
    }
    else
    {
      if(slopeZYX > 1.0)
      {
        double ez = obstacle.getZ() - 0.5;
        if (ez > pz) currentCell->addNVolume(pz, ez);
      }
      else if(slopeZYX < -1.0)
      {
        double ez = obstacle.getZ() + 0.5;
        if (ez < pz) currentCell->addNVolume(ez, pz);
      }
      break;
    }
  }
}

} // namespace MVOG
