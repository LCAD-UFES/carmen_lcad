#include "fast.h"

int descriptor_lookup[] = {
  0,42,10,41,20,37,29,31,36,22,40,13,42,2,42,-8,38,-18,33,-27,25,-34,15,-39,5,-42,-5,-42,-15,-39,-25,-34,-33,-27,-38,-18,-42,-8,-42,2,-40,13,-36,22,-29,31,-20,37,-10,41,
  0,57,14,55,27,50,39,41,48,30,54,17,57,3,56,-10,51,-24,44,-36,33,-46,21,-53,7,-56,-7,-56,-21,-53,-33,-46,-44,-36,-51,-24,-56,-10,-57,3,-54,17,-48,30,-39,41,-27,50,-14,55,
  0,71,17,69,34,62,48,52,60,38,67,22,71,4,70,-13,64,-30,55,-45,41,-57,26,-66,8,-70,-8,-70,-26,-66,-41,-57,-55,-45,-64,-30,-70,-13,-71,4,-67,22,-60,38,-48,52,-34,62,-17,69,
  0,85,21,83,41,75,58,62,72,45,81,26,85,5,84,-16,77,-36,66,-54,50,-69,31,-79,10,-85,-10,-85,-31,-79,-50,-69,-66,-54,-77,-36,-84,-16,-85,5,-81,26,-72,45,-58,62,-41,75,-21,83,
  0,100,24,96,48,87,68,72,84,53,95,30,99,6,98,-18,90,-42,77,-63,58,-80,36,-92,12,-99,-12,-99,-36,-92,-58,-80,-77,-63,-90,-42,-98,-18,-99,6,-95,30,-84,53,-68,72,-48,87,-24,96,
};

int descriptor_direction_buf[FAST_DESCRIPTOR_WIDTH];

fast::fast() {
  scores = new int[FAST_MAX_CORNERS];
  corners = new xy[FAST_MAX_CORNERS];
  nonmax = new xy[FAST_MAX_CORNERS_PREVIOUS];
  row_start = new int[FAST_MAX_IMAGE_HEIGHT];
  img_mono = NULL;
  prev_img_mono = NULL;
  previous_corners = new xy*[FAST_PREVIOUS_BUFFER];
  previous_no_of_corners = new int[FAST_PREVIOUS_BUFFER];
  temporal_matches = new unsigned char*[FAST_PREVIOUS_BUFFER];
  previous_interocular_disparity = new unsigned short*[FAST_PREVIOUS_BUFFER];
  int i;
  for (i = 0; i < FAST_PREVIOUS_BUFFER; i++) {
    previous_corners[i] = new xy[FAST_MAX_CORNERS_PREVIOUS];
    temporal_matches[i] = new unsigned char[FAST_MAX_CORNERS_PREVIOUS];
    previous_interocular_disparity[i] = new unsigned short[FAST_MAX_CORNERS_PREVIOUS];
    previous_no_of_corners[i] = 0;
  }
  interocular_disparity = NULL;
  threshold = 100;
  num_nonmax = 0;

  //int* descriptor_lookup2 = new int[FAST_DESCRIPTOR_RADIUS*FAST_DESCRIPTOR_WIDTH*2];
  //create_descriptor_lookup(FAST_DESCRIPTOR_RADIUS, FAST_DESCRIPTOR_WIDTH, descriptor_lookup2);
  //delete[] descriptor_lookup2;
}

fast::~fast() {
  delete[] scores;
  delete[] corners;
  delete[] nonmax;
  delete[] row_start;
  if (interocular_disparity != NULL) {
    delete[] interocular_disparity;
  }
  if (img_mono != NULL) {
    delete[] img_mono;
    delete[] prev_img_mono;
  }
  int i;
  for (i = 0; i < FAST_PREVIOUS_BUFFER; i++) {
    delete[] previous_corners[i];
    delete[] temporal_matches[i];
    delete[] previous_interocular_disparity[i];
  }
  delete[] previous_corners;
  delete[] temporal_matches;
  delete[] previous_interocular_disparity;
  delete[] previous_no_of_corners;
}

int fast::corner_score(
		       unsigned char* p,
		       int* pixel,
		       int bstart)
{
  int bmin = bstart;
  int bmax = 255;
  int b = (bmax + bmin)/2;

  /*Compute the score using binary search*/
  for(;;)
    {
      int cb = *p + b;
      int c_b= *p - b;

      if( p[pixel[0]] > cb)
	if( p[pixel[1]] > cb)
          if( p[pixel[2]] > cb)
	    if( p[pixel[3]] > cb)
	      if( p[pixel[4]] > cb)
		if( p[pixel[5]] > cb)
		  if( p[pixel[6]] > cb)
		    if( p[pixel[7]] > cb)
		      if( p[pixel[8]] > cb)
			goto is_a_corner;
		      else
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  goto is_not_a_corner;
		    else if( p[pixel[7]] < c_b)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  goto is_not_a_corner;
		      else if( p[pixel[14]] < c_b)
			if( p[pixel[8]] < c_b)
			  if( p[pixel[9]] < c_b)
			    if( p[pixel[10]] < c_b)
			      if( p[pixel[11]] < c_b)
				if( p[pixel[12]] < c_b)
				  if( p[pixel[13]] < c_b)
				    if( p[pixel[15]] < c_b)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else if( p[pixel[6]] < c_b)
		    if( p[pixel[15]] > cb)
		      if( p[pixel[13]] > cb)
			if( p[pixel[14]] > cb)
			  goto is_a_corner;
			else
			  goto is_not_a_corner;
		      else if( p[pixel[13]] < c_b)
			if( p[pixel[7]] < c_b)
			  if( p[pixel[8]] < c_b)
			    if( p[pixel[9]] < c_b)
			      if( p[pixel[10]] < c_b)
				if( p[pixel[11]] < c_b)
				  if( p[pixel[12]] < c_b)
				    if( p[pixel[14]] < c_b)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      if( p[pixel[7]] < c_b)
			if( p[pixel[8]] < c_b)
			  if( p[pixel[9]] < c_b)
			    if( p[pixel[10]] < c_b)
			      if( p[pixel[11]] < c_b)
				if( p[pixel[12]] < c_b)
				  if( p[pixel[13]] < c_b)
				    if( p[pixel[14]] < c_b)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[13]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else if( p[pixel[13]] < c_b)
		      if( p[pixel[7]] < c_b)
			if( p[pixel[8]] < c_b)
			  if( p[pixel[9]] < c_b)
			    if( p[pixel[10]] < c_b)
			      if( p[pixel[11]] < c_b)
				if( p[pixel[12]] < c_b)
				  if( p[pixel[14]] < c_b)
				    if( p[pixel[15]] < c_b)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else if( p[pixel[5]] < c_b)
		  if( p[pixel[14]] > cb)
		    if( p[pixel[12]] > cb)
		      if( p[pixel[13]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  if( p[pixel[10]] > cb)
				    if( p[pixel[11]] > cb)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else if( p[pixel[12]] < c_b)
		      if( p[pixel[6]] < c_b)
			if( p[pixel[7]] < c_b)
			  if( p[pixel[8]] < c_b)
			    if( p[pixel[9]] < c_b)
			      if( p[pixel[10]] < c_b)
				if( p[pixel[11]] < c_b)
				  if( p[pixel[13]] < c_b)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else if( p[pixel[14]] < c_b)
		    if( p[pixel[7]] < c_b)
		      if( p[pixel[8]] < c_b)
			if( p[pixel[9]] < c_b)
			  if( p[pixel[10]] < c_b)
			    if( p[pixel[11]] < c_b)
			      if( p[pixel[12]] < c_b)
				if( p[pixel[13]] < c_b)
				  if( p[pixel[6]] < c_b)
				    goto is_a_corner;
				  else
				    if( p[pixel[15]] < c_b)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    if( p[pixel[6]] < c_b)
		      if( p[pixel[7]] < c_b)
			if( p[pixel[8]] < c_b)
			  if( p[pixel[9]] < c_b)
			    if( p[pixel[10]] < c_b)
			      if( p[pixel[11]] < c_b)
				if( p[pixel[12]] < c_b)
				  if( p[pixel[13]] < c_b)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  if( p[pixel[12]] > cb)
		    if( p[pixel[13]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  if( p[pixel[10]] > cb)
				    if( p[pixel[11]] > cb)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else if( p[pixel[12]] < c_b)
		    if( p[pixel[7]] < c_b)
		      if( p[pixel[8]] < c_b)
			if( p[pixel[9]] < c_b)
			  if( p[pixel[10]] < c_b)
			    if( p[pixel[11]] < c_b)
			      if( p[pixel[13]] < c_b)
				if( p[pixel[14]] < c_b)
				  if( p[pixel[6]] < c_b)
				    goto is_a_corner;
				  else
				    if( p[pixel[15]] < c_b)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else if( p[pixel[4]] < c_b)
		if( p[pixel[13]] > cb)
		  if( p[pixel[11]] > cb)
		    if( p[pixel[12]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  if( p[pixel[10]] > cb)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  if( p[pixel[10]] > cb)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else if( p[pixel[11]] < c_b)
		    if( p[pixel[5]] < c_b)
		      if( p[pixel[6]] < c_b)
			if( p[pixel[7]] < c_b)
			  if( p[pixel[8]] < c_b)
			    if( p[pixel[9]] < c_b)
			      if( p[pixel[10]] < c_b)
				if( p[pixel[12]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else if( p[pixel[13]] < c_b)
		  if( p[pixel[7]] < c_b)
		    if( p[pixel[8]] < c_b)
		      if( p[pixel[9]] < c_b)
			if( p[pixel[10]] < c_b)
			  if( p[pixel[11]] < c_b)
			    if( p[pixel[12]] < c_b)
			      if( p[pixel[6]] < c_b)
				if( p[pixel[5]] < c_b)
				  goto is_a_corner;
				else
				  if( p[pixel[14]] < c_b)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
			      else
				if( p[pixel[14]] < c_b)
				  if( p[pixel[15]] < c_b)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  if( p[pixel[5]] < c_b)
		    if( p[pixel[6]] < c_b)
		      if( p[pixel[7]] < c_b)
			if( p[pixel[8]] < c_b)
			  if( p[pixel[9]] < c_b)
			    if( p[pixel[10]] < c_b)
			      if( p[pixel[11]] < c_b)
				if( p[pixel[12]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else
		if( p[pixel[11]] > cb)
		  if( p[pixel[12]] > cb)
		    if( p[pixel[13]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  if( p[pixel[10]] > cb)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  if( p[pixel[10]] > cb)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else if( p[pixel[11]] < c_b)
		  if( p[pixel[7]] < c_b)
		    if( p[pixel[8]] < c_b)
		      if( p[pixel[9]] < c_b)
			if( p[pixel[10]] < c_b)
			  if( p[pixel[12]] < c_b)
			    if( p[pixel[13]] < c_b)
			      if( p[pixel[6]] < c_b)
				if( p[pixel[5]] < c_b)
				  goto is_a_corner;
				else
				  if( p[pixel[14]] < c_b)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
			      else
				if( p[pixel[14]] < c_b)
				  if( p[pixel[15]] < c_b)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	    else if( p[pixel[3]] < c_b)
	      if( p[pixel[10]] > cb)
		if( p[pixel[11]] > cb)
		  if( p[pixel[12]] > cb)
		    if( p[pixel[13]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else if( p[pixel[10]] < c_b)
		if( p[pixel[7]] < c_b)
		  if( p[pixel[8]] < c_b)
		    if( p[pixel[9]] < c_b)
		      if( p[pixel[11]] < c_b)
			if( p[pixel[6]] < c_b)
			  if( p[pixel[5]] < c_b)
			    if( p[pixel[4]] < c_b)
			      goto is_a_corner;
			    else
			      if( p[pixel[12]] < c_b)
				if( p[pixel[13]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			  else
			    if( p[pixel[12]] < c_b)
			      if( p[pixel[13]] < c_b)
				if( p[pixel[14]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			else
			  if( p[pixel[12]] < c_b)
			    if( p[pixel[13]] < c_b)
			      if( p[pixel[14]] < c_b)
				if( p[pixel[15]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      if( p[pixel[10]] > cb)
		if( p[pixel[11]] > cb)
		  if( p[pixel[12]] > cb)
		    if( p[pixel[13]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				if( p[pixel[9]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else if( p[pixel[10]] < c_b)
		if( p[pixel[7]] < c_b)
		  if( p[pixel[8]] < c_b)
		    if( p[pixel[9]] < c_b)
		      if( p[pixel[11]] < c_b)
			if( p[pixel[12]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[5]] < c_b)
			      if( p[pixel[4]] < c_b)
				goto is_a_corner;
			      else
				if( p[pixel[13]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			    else
			      if( p[pixel[13]] < c_b)
				if( p[pixel[14]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			  else
			    if( p[pixel[13]] < c_b)
			      if( p[pixel[14]] < c_b)
				if( p[pixel[15]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
          else if( p[pixel[2]] < c_b)
	    if( p[pixel[9]] > cb)
	      if( p[pixel[10]] > cb)
		if( p[pixel[11]] > cb)
		  if( p[pixel[12]] > cb)
		    if( p[pixel[13]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[3]] > cb)
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else if( p[pixel[9]] < c_b)
	      if( p[pixel[7]] < c_b)
		if( p[pixel[8]] < c_b)
		  if( p[pixel[10]] < c_b)
		    if( p[pixel[6]] < c_b)
		      if( p[pixel[5]] < c_b)
			if( p[pixel[4]] < c_b)
			  if( p[pixel[3]] < c_b)
			    goto is_a_corner;
			  else
			    if( p[pixel[11]] < c_b)
			      if( p[pixel[12]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			else
			  if( p[pixel[11]] < c_b)
			    if( p[pixel[12]] < c_b)
			      if( p[pixel[13]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[11]] < c_b)
			  if( p[pixel[12]] < c_b)
			    if( p[pixel[13]] < c_b)
			      if( p[pixel[14]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[11]] < c_b)
			if( p[pixel[12]] < c_b)
			  if( p[pixel[13]] < c_b)
			    if( p[pixel[14]] < c_b)
			      if( p[pixel[15]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else
	    if( p[pixel[9]] > cb)
	      if( p[pixel[10]] > cb)
		if( p[pixel[11]] > cb)
		  if( p[pixel[12]] > cb)
		    if( p[pixel[13]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[3]] > cb)
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      if( p[pixel[8]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else if( p[pixel[9]] < c_b)
	      if( p[pixel[7]] < c_b)
		if( p[pixel[8]] < c_b)
		  if( p[pixel[10]] < c_b)
		    if( p[pixel[11]] < c_b)
		      if( p[pixel[6]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[4]] < c_b)
			    if( p[pixel[3]] < c_b)
			      goto is_a_corner;
			    else
			      if( p[pixel[12]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			  else
			    if( p[pixel[12]] < c_b)
			      if( p[pixel[13]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			else
			  if( p[pixel[12]] < c_b)
			    if( p[pixel[13]] < c_b)
			      if( p[pixel[14]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[12]] < c_b)
			  if( p[pixel[13]] < c_b)
			    if( p[pixel[14]] < c_b)
			      if( p[pixel[15]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
	else if( p[pixel[1]] < c_b)
          if( p[pixel[8]] > cb)
	    if( p[pixel[9]] > cb)
	      if( p[pixel[10]] > cb)
		if( p[pixel[11]] > cb)
		  if( p[pixel[12]] > cb)
		    if( p[pixel[13]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[3]] > cb)
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  if( p[pixel[2]] > cb)
		    if( p[pixel[3]] > cb)
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
	    if( p[pixel[7]] < c_b)
	      if( p[pixel[9]] < c_b)
		if( p[pixel[6]] < c_b)
		  if( p[pixel[5]] < c_b)
		    if( p[pixel[4]] < c_b)
		      if( p[pixel[3]] < c_b)
			if( p[pixel[2]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[10]] < c_b)
			    if( p[pixel[11]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[10]] < c_b)
			  if( p[pixel[11]] < c_b)
			    if( p[pixel[12]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[10]] < c_b)
			if( p[pixel[11]] < c_b)
			  if( p[pixel[12]] < c_b)
			    if( p[pixel[13]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[10]] < c_b)
		      if( p[pixel[11]] < c_b)
			if( p[pixel[12]] < c_b)
			  if( p[pixel[13]] < c_b)
			    if( p[pixel[14]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  if( p[pixel[10]] < c_b)
		    if( p[pixel[11]] < c_b)
		      if( p[pixel[12]] < c_b)
			if( p[pixel[13]] < c_b)
			  if( p[pixel[14]] < c_b)
			    if( p[pixel[15]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else
	    goto is_not_a_corner;
	else
          if( p[pixel[8]] > cb)
	    if( p[pixel[9]] > cb)
	      if( p[pixel[10]] > cb)
		if( p[pixel[11]] > cb)
		  if( p[pixel[12]] > cb)
		    if( p[pixel[13]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[15]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[3]] > cb)
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  if( p[pixel[2]] > cb)
		    if( p[pixel[3]] > cb)
		      if( p[pixel[4]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[7]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
	    if( p[pixel[7]] < c_b)
	      if( p[pixel[9]] < c_b)
		if( p[pixel[10]] < c_b)
		  if( p[pixel[6]] < c_b)
		    if( p[pixel[5]] < c_b)
		      if( p[pixel[4]] < c_b)
			if( p[pixel[3]] < c_b)
			  if( p[pixel[2]] < c_b)
			    goto is_a_corner;
			  else
			    if( p[pixel[11]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			else
			  if( p[pixel[11]] < c_b)
			    if( p[pixel[12]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[11]] < c_b)
			  if( p[pixel[12]] < c_b)
			    if( p[pixel[13]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[11]] < c_b)
			if( p[pixel[12]] < c_b)
			  if( p[pixel[13]] < c_b)
			    if( p[pixel[14]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[11]] < c_b)
		      if( p[pixel[12]] < c_b)
			if( p[pixel[13]] < c_b)
			  if( p[pixel[14]] < c_b)
			    if( p[pixel[15]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else
	    goto is_not_a_corner;
      else if( p[pixel[0]] < c_b)
	if( p[pixel[1]] > cb)
          if( p[pixel[8]] > cb)
	    if( p[pixel[7]] > cb)
	      if( p[pixel[9]] > cb)
		if( p[pixel[6]] > cb)
		  if( p[pixel[5]] > cb)
		    if( p[pixel[4]] > cb)
		      if( p[pixel[3]] > cb)
			if( p[pixel[2]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[10]] > cb)
			    if( p[pixel[11]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[10]] > cb)
			  if( p[pixel[11]] > cb)
			    if( p[pixel[12]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[10]] > cb)
			if( p[pixel[11]] > cb)
			  if( p[pixel[12]] > cb)
			    if( p[pixel[13]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[10]] > cb)
		      if( p[pixel[11]] > cb)
			if( p[pixel[12]] > cb)
			  if( p[pixel[13]] > cb)
			    if( p[pixel[14]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  if( p[pixel[10]] > cb)
		    if( p[pixel[11]] > cb)
		      if( p[pixel[12]] > cb)
			if( p[pixel[13]] > cb)
			  if( p[pixel[14]] > cb)
			    if( p[pixel[15]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
	    if( p[pixel[9]] < c_b)
	      if( p[pixel[10]] < c_b)
		if( p[pixel[11]] < c_b)
		  if( p[pixel[12]] < c_b)
		    if( p[pixel[13]] < c_b)
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[3]] < c_b)
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  if( p[pixel[2]] < c_b)
		    if( p[pixel[3]] < c_b)
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else
	    goto is_not_a_corner;
	else if( p[pixel[1]] < c_b)
          if( p[pixel[2]] > cb)
	    if( p[pixel[9]] > cb)
	      if( p[pixel[7]] > cb)
		if( p[pixel[8]] > cb)
		  if( p[pixel[10]] > cb)
		    if( p[pixel[6]] > cb)
		      if( p[pixel[5]] > cb)
			if( p[pixel[4]] > cb)
			  if( p[pixel[3]] > cb)
			    goto is_a_corner;
			  else
			    if( p[pixel[11]] > cb)
			      if( p[pixel[12]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			else
			  if( p[pixel[11]] > cb)
			    if( p[pixel[12]] > cb)
			      if( p[pixel[13]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[11]] > cb)
			  if( p[pixel[12]] > cb)
			    if( p[pixel[13]] > cb)
			      if( p[pixel[14]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[11]] > cb)
			if( p[pixel[12]] > cb)
			  if( p[pixel[13]] > cb)
			    if( p[pixel[14]] > cb)
			      if( p[pixel[15]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else if( p[pixel[9]] < c_b)
	      if( p[pixel[10]] < c_b)
		if( p[pixel[11]] < c_b)
		  if( p[pixel[12]] < c_b)
		    if( p[pixel[13]] < c_b)
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[3]] < c_b)
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else if( p[pixel[2]] < c_b)
	    if( p[pixel[3]] > cb)
	      if( p[pixel[10]] > cb)
		if( p[pixel[7]] > cb)
		  if( p[pixel[8]] > cb)
		    if( p[pixel[9]] > cb)
		      if( p[pixel[11]] > cb)
			if( p[pixel[6]] > cb)
			  if( p[pixel[5]] > cb)
			    if( p[pixel[4]] > cb)
			      goto is_a_corner;
			    else
			      if( p[pixel[12]] > cb)
				if( p[pixel[13]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			  else
			    if( p[pixel[12]] > cb)
			      if( p[pixel[13]] > cb)
				if( p[pixel[14]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			else
			  if( p[pixel[12]] > cb)
			    if( p[pixel[13]] > cb)
			      if( p[pixel[14]] > cb)
				if( p[pixel[15]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else if( p[pixel[10]] < c_b)
		if( p[pixel[11]] < c_b)
		  if( p[pixel[12]] < c_b)
		    if( p[pixel[13]] < c_b)
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else if( p[pixel[3]] < c_b)
	      if( p[pixel[4]] > cb)
		if( p[pixel[13]] > cb)
		  if( p[pixel[7]] > cb)
		    if( p[pixel[8]] > cb)
		      if( p[pixel[9]] > cb)
			if( p[pixel[10]] > cb)
			  if( p[pixel[11]] > cb)
			    if( p[pixel[12]] > cb)
			      if( p[pixel[6]] > cb)
				if( p[pixel[5]] > cb)
				  goto is_a_corner;
				else
				  if( p[pixel[14]] > cb)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
			      else
				if( p[pixel[14]] > cb)
				  if( p[pixel[15]] > cb)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else if( p[pixel[13]] < c_b)
		  if( p[pixel[11]] > cb)
		    if( p[pixel[5]] > cb)
		      if( p[pixel[6]] > cb)
			if( p[pixel[7]] > cb)
			  if( p[pixel[8]] > cb)
			    if( p[pixel[9]] > cb)
			      if( p[pixel[10]] > cb)
				if( p[pixel[12]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else if( p[pixel[11]] < c_b)
		    if( p[pixel[12]] < c_b)
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  if( p[pixel[10]] < c_b)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  if( p[pixel[10]] < c_b)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  if( p[pixel[5]] > cb)
		    if( p[pixel[6]] > cb)
		      if( p[pixel[7]] > cb)
			if( p[pixel[8]] > cb)
			  if( p[pixel[9]] > cb)
			    if( p[pixel[10]] > cb)
			      if( p[pixel[11]] > cb)
				if( p[pixel[12]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else if( p[pixel[4]] < c_b)
		if( p[pixel[5]] > cb)
		  if( p[pixel[14]] > cb)
		    if( p[pixel[7]] > cb)
		      if( p[pixel[8]] > cb)
			if( p[pixel[9]] > cb)
			  if( p[pixel[10]] > cb)
			    if( p[pixel[11]] > cb)
			      if( p[pixel[12]] > cb)
				if( p[pixel[13]] > cb)
				  if( p[pixel[6]] > cb)
				    goto is_a_corner;
				  else
				    if( p[pixel[15]] > cb)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else if( p[pixel[14]] < c_b)
		    if( p[pixel[12]] > cb)
		      if( p[pixel[6]] > cb)
			if( p[pixel[7]] > cb)
			  if( p[pixel[8]] > cb)
			    if( p[pixel[9]] > cb)
			      if( p[pixel[10]] > cb)
				if( p[pixel[11]] > cb)
				  if( p[pixel[13]] > cb)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else if( p[pixel[12]] < c_b)
		      if( p[pixel[13]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  if( p[pixel[10]] < c_b)
				    if( p[pixel[11]] < c_b)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    if( p[pixel[6]] > cb)
		      if( p[pixel[7]] > cb)
			if( p[pixel[8]] > cb)
			  if( p[pixel[9]] > cb)
			    if( p[pixel[10]] > cb)
			      if( p[pixel[11]] > cb)
				if( p[pixel[12]] > cb)
				  if( p[pixel[13]] > cb)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else if( p[pixel[5]] < c_b)
		  if( p[pixel[6]] > cb)
		    if( p[pixel[15]] < c_b)
		      if( p[pixel[13]] > cb)
			if( p[pixel[7]] > cb)
			  if( p[pixel[8]] > cb)
			    if( p[pixel[9]] > cb)
			      if( p[pixel[10]] > cb)
				if( p[pixel[11]] > cb)
				  if( p[pixel[12]] > cb)
				    if( p[pixel[14]] > cb)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else if( p[pixel[13]] < c_b)
			if( p[pixel[14]] < c_b)
			  goto is_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      if( p[pixel[7]] > cb)
			if( p[pixel[8]] > cb)
			  if( p[pixel[9]] > cb)
			    if( p[pixel[10]] > cb)
			      if( p[pixel[11]] > cb)
				if( p[pixel[12]] > cb)
				  if( p[pixel[13]] > cb)
				    if( p[pixel[14]] > cb)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else if( p[pixel[6]] < c_b)
		    if( p[pixel[7]] > cb)
		      if( p[pixel[14]] > cb)
			if( p[pixel[8]] > cb)
			  if( p[pixel[9]] > cb)
			    if( p[pixel[10]] > cb)
			      if( p[pixel[11]] > cb)
				if( p[pixel[12]] > cb)
				  if( p[pixel[13]] > cb)
				    if( p[pixel[15]] > cb)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else if( p[pixel[7]] < c_b)
		      if( p[pixel[8]] < c_b)
			goto is_a_corner;
		      else
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[13]] > cb)
		      if( p[pixel[7]] > cb)
			if( p[pixel[8]] > cb)
			  if( p[pixel[9]] > cb)
			    if( p[pixel[10]] > cb)
			      if( p[pixel[11]] > cb)
				if( p[pixel[12]] > cb)
				  if( p[pixel[14]] > cb)
				    if( p[pixel[15]] > cb)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else if( p[pixel[13]] < c_b)
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  if( p[pixel[12]] > cb)
		    if( p[pixel[7]] > cb)
		      if( p[pixel[8]] > cb)
			if( p[pixel[9]] > cb)
			  if( p[pixel[10]] > cb)
			    if( p[pixel[11]] > cb)
			      if( p[pixel[13]] > cb)
				if( p[pixel[14]] > cb)
				  if( p[pixel[6]] > cb)
				    goto is_a_corner;
				  else
				    if( p[pixel[15]] > cb)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else if( p[pixel[12]] < c_b)
		    if( p[pixel[13]] < c_b)
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  if( p[pixel[10]] < c_b)
				    if( p[pixel[11]] < c_b)
				      goto is_a_corner;
				    else
				      goto is_not_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else
		if( p[pixel[11]] > cb)
		  if( p[pixel[7]] > cb)
		    if( p[pixel[8]] > cb)
		      if( p[pixel[9]] > cb)
			if( p[pixel[10]] > cb)
			  if( p[pixel[12]] > cb)
			    if( p[pixel[13]] > cb)
			      if( p[pixel[6]] > cb)
				if( p[pixel[5]] > cb)
				  goto is_a_corner;
				else
				  if( p[pixel[14]] > cb)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
			      else
				if( p[pixel[14]] > cb)
				  if( p[pixel[15]] > cb)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else if( p[pixel[11]] < c_b)
		  if( p[pixel[12]] < c_b)
		    if( p[pixel[13]] < c_b)
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  if( p[pixel[10]] < c_b)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  if( p[pixel[10]] < c_b)
				    goto is_a_corner;
				  else
				    goto is_not_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	    else
	      if( p[pixel[10]] > cb)
		if( p[pixel[7]] > cb)
		  if( p[pixel[8]] > cb)
		    if( p[pixel[9]] > cb)
		      if( p[pixel[11]] > cb)
			if( p[pixel[12]] > cb)
			  if( p[pixel[6]] > cb)
			    if( p[pixel[5]] > cb)
			      if( p[pixel[4]] > cb)
				goto is_a_corner;
			      else
				if( p[pixel[13]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			    else
			      if( p[pixel[13]] > cb)
				if( p[pixel[14]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			  else
			    if( p[pixel[13]] > cb)
			      if( p[pixel[14]] > cb)
				if( p[pixel[15]] > cb)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else if( p[pixel[10]] < c_b)
		if( p[pixel[11]] < c_b)
		  if( p[pixel[12]] < c_b)
		    if( p[pixel[13]] < c_b)
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				if( p[pixel[9]] < c_b)
				  goto is_a_corner;
				else
				  goto is_not_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
          else
	    if( p[pixel[9]] > cb)
	      if( p[pixel[7]] > cb)
		if( p[pixel[8]] > cb)
		  if( p[pixel[10]] > cb)
		    if( p[pixel[11]] > cb)
		      if( p[pixel[6]] > cb)
			if( p[pixel[5]] > cb)
			  if( p[pixel[4]] > cb)
			    if( p[pixel[3]] > cb)
			      goto is_a_corner;
			    else
			      if( p[pixel[12]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			  else
			    if( p[pixel[12]] > cb)
			      if( p[pixel[13]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			else
			  if( p[pixel[12]] > cb)
			    if( p[pixel[13]] > cb)
			      if( p[pixel[14]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[12]] > cb)
			  if( p[pixel[13]] > cb)
			    if( p[pixel[14]] > cb)
			      if( p[pixel[15]] > cb)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else if( p[pixel[9]] < c_b)
	      if( p[pixel[10]] < c_b)
		if( p[pixel[11]] < c_b)
		  if( p[pixel[12]] < c_b)
		    if( p[pixel[13]] < c_b)
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[3]] < c_b)
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      if( p[pixel[8]] < c_b)
				goto is_a_corner;
			      else
				goto is_not_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
	else
          if( p[pixel[8]] > cb)
	    if( p[pixel[7]] > cb)
	      if( p[pixel[9]] > cb)
		if( p[pixel[10]] > cb)
		  if( p[pixel[6]] > cb)
		    if( p[pixel[5]] > cb)
		      if( p[pixel[4]] > cb)
			if( p[pixel[3]] > cb)
			  if( p[pixel[2]] > cb)
			    goto is_a_corner;
			  else
			    if( p[pixel[11]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			else
			  if( p[pixel[11]] > cb)
			    if( p[pixel[12]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[11]] > cb)
			  if( p[pixel[12]] > cb)
			    if( p[pixel[13]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[11]] > cb)
			if( p[pixel[12]] > cb)
			  if( p[pixel[13]] > cb)
			    if( p[pixel[14]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[11]] > cb)
		      if( p[pixel[12]] > cb)
			if( p[pixel[13]] > cb)
			  if( p[pixel[14]] > cb)
			    if( p[pixel[15]] > cb)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
	    if( p[pixel[9]] < c_b)
	      if( p[pixel[10]] < c_b)
		if( p[pixel[11]] < c_b)
		  if( p[pixel[12]] < c_b)
		    if( p[pixel[13]] < c_b)
		      if( p[pixel[14]] < c_b)
			if( p[pixel[15]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[3]] < c_b)
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  if( p[pixel[2]] < c_b)
		    if( p[pixel[3]] < c_b)
		      if( p[pixel[4]] < c_b)
			if( p[pixel[5]] < c_b)
			  if( p[pixel[6]] < c_b)
			    if( p[pixel[7]] < c_b)
			      goto is_a_corner;
			    else
			      goto is_not_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else
		goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else
	    goto is_not_a_corner;
      else
	if( p[pixel[7]] > cb)
          if( p[pixel[8]] > cb)
	    if( p[pixel[9]] > cb)
	      if( p[pixel[6]] > cb)
		if( p[pixel[5]] > cb)
		  if( p[pixel[4]] > cb)
		    if( p[pixel[3]] > cb)
		      if( p[pixel[2]] > cb)
			if( p[pixel[1]] > cb)
			  goto is_a_corner;
			else
			  if( p[pixel[10]] > cb)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[10]] > cb)
			  if( p[pixel[11]] > cb)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[10]] > cb)
			if( p[pixel[11]] > cb)
			  if( p[pixel[12]] > cb)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[10]] > cb)
		      if( p[pixel[11]] > cb)
			if( p[pixel[12]] > cb)
			  if( p[pixel[13]] > cb)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  if( p[pixel[10]] > cb)
		    if( p[pixel[11]] > cb)
		      if( p[pixel[12]] > cb)
			if( p[pixel[13]] > cb)
			  if( p[pixel[14]] > cb)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else
		if( p[pixel[10]] > cb)
		  if( p[pixel[11]] > cb)
		    if( p[pixel[12]] > cb)
		      if( p[pixel[13]] > cb)
			if( p[pixel[14]] > cb)
			  if( p[pixel[15]] > cb)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else
	    goto is_not_a_corner;
	else if( p[pixel[7]] < c_b)
          if( p[pixel[8]] < c_b)
	    if( p[pixel[9]] < c_b)
	      if( p[pixel[6]] < c_b)
		if( p[pixel[5]] < c_b)
		  if( p[pixel[4]] < c_b)
		    if( p[pixel[3]] < c_b)
		      if( p[pixel[2]] < c_b)
			if( p[pixel[1]] < c_b)
			  goto is_a_corner;
			else
			  if( p[pixel[10]] < c_b)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
		      else
			if( p[pixel[10]] < c_b)
			  if( p[pixel[11]] < c_b)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		    else
		      if( p[pixel[10]] < c_b)
			if( p[pixel[11]] < c_b)
			  if( p[pixel[12]] < c_b)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		  else
		    if( p[pixel[10]] < c_b)
		      if( p[pixel[11]] < c_b)
			if( p[pixel[12]] < c_b)
			  if( p[pixel[13]] < c_b)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		else
		  if( p[pixel[10]] < c_b)
		    if( p[pixel[11]] < c_b)
		      if( p[pixel[12]] < c_b)
			if( p[pixel[13]] < c_b)
			  if( p[pixel[14]] < c_b)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
	      else
		if( p[pixel[10]] < c_b)
		  if( p[pixel[11]] < c_b)
		    if( p[pixel[12]] < c_b)
		      if( p[pixel[13]] < c_b)
			if( p[pixel[14]] < c_b)
			  if( p[pixel[15]] < c_b)
			    goto is_a_corner;
			  else
			    goto is_not_a_corner;
			else
			  goto is_not_a_corner;
		      else
			goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;
		else
		  goto is_not_a_corner;
	    else
	      goto is_not_a_corner;
          else
	    goto is_not_a_corner;
	else
          goto is_not_a_corner;

    is_a_corner:
      bmin=b;
      goto end_if;

    is_not_a_corner:
      bmax=b;
      goto end_if;

    end_if:

      if(bmin == bmax - 1 || bmin == bmax)
	return bmin;
      b = (bmin + bmax) / 2;
    }
}

void fast::make_offsets(
			int* pixel,
			int row_stride)
{
  pixel[0] = 0 + row_stride * 3;
  pixel[1] = 1 + row_stride * 3;
  pixel[2] = 2 + row_stride * 2;
  pixel[3] = 3 + row_stride * 1;
  pixel[4] = 3 + row_stride * 0;
  pixel[5] = 3 + row_stride * -1;
  pixel[6] = 2 + row_stride * -2;
  pixel[7] = 1 + row_stride * -3;
  pixel[8] = 0 + row_stride * -3;
  pixel[9] = -1 + row_stride * -3;
  pixel[10] = -2 + row_stride * -2;
  pixel[11] = -3 + row_stride * -1;
  pixel[12] = -3 + row_stride * 0;
  pixel[13] = -3 + row_stride * 1;
  pixel[14] = -2 + row_stride * 2;
  pixel[15] = -1 + row_stride * 3;
}

void fast::score(
		 unsigned char* i,
		 int stride,
		 xy* corners,
		 int num_corners,
		 int b)
{
  int n;

  int pixel[16];
  make_offsets(pixel, stride);

  for(n=0; n < num_corners; n++)
    scores[n] = corner_score(i + corners[n].y*stride + corners[n].x, pixel, b);
}

void fast::detect(
		  unsigned char* im,
		  int xsize,
		  int ysize,
		  int stride,
		  int b,
		  int* ret_num_corners)
{
  int num_corners=0;
  int pixel[16];
  int x, y;

  make_offsets(pixel, stride);

  for(y=3; y < ysize - 3; y++)
    for(x=3; x < xsize - 3; x++)
      {
	const unsigned char* p = im + y*stride + x;

	int cb = *p + b;
	int c_b= *p - b;
        if(p[pixel[0]] > cb)
	  if(p[pixel[1]] > cb)
	    if(p[pixel[2]] > cb)
	      if(p[pixel[3]] > cb)
		if(p[pixel[4]] > cb)
		  if(p[pixel[5]] > cb)
		    if(p[pixel[6]] > cb)
		      if(p[pixel[7]] > cb)
			if(p[pixel[8]] > cb)
			  {}
			else
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    continue;
		      else if(p[pixel[7]] < c_b)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    continue;
			else if(p[pixel[14]] < c_b)
			  if(p[pixel[8]] < c_b)
			    if(p[pixel[9]] < c_b)
			      if(p[pixel[10]] < c_b)
				if(p[pixel[11]] < c_b)
				  if(p[pixel[12]] < c_b)
				    if(p[pixel[13]] < c_b)
				      if(p[pixel[15]] < c_b)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    continue;
			else
			  continue;
		    else if(p[pixel[6]] < c_b)
		      if(p[pixel[15]] > cb)
			if(p[pixel[13]] > cb)
			  if(p[pixel[14]] > cb)
			    {}
			  else
			    continue;
			else if(p[pixel[13]] < c_b)
			  if(p[pixel[7]] < c_b)
			    if(p[pixel[8]] < c_b)
			      if(p[pixel[9]] < c_b)
				if(p[pixel[10]] < c_b)
				  if(p[pixel[11]] < c_b)
				    if(p[pixel[12]] < c_b)
				      if(p[pixel[14]] < c_b)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			if(p[pixel[7]] < c_b)
			  if(p[pixel[8]] < c_b)
			    if(p[pixel[9]] < c_b)
			      if(p[pixel[10]] < c_b)
				if(p[pixel[11]] < c_b)
				  if(p[pixel[12]] < c_b)
				    if(p[pixel[13]] < c_b)
				      if(p[pixel[14]] < c_b)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[13]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    continue;
			else
			  continue;
		      else if(p[pixel[13]] < c_b)
			if(p[pixel[7]] < c_b)
			  if(p[pixel[8]] < c_b)
			    if(p[pixel[9]] < c_b)
			      if(p[pixel[10]] < c_b)
				if(p[pixel[11]] < c_b)
				  if(p[pixel[12]] < c_b)
				    if(p[pixel[14]] < c_b)
				      if(p[pixel[15]] < c_b)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else if(p[pixel[5]] < c_b)
		    if(p[pixel[14]] > cb)
		      if(p[pixel[12]] > cb)
			if(p[pixel[13]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    if(p[pixel[10]] > cb)
				      if(p[pixel[11]] > cb)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  continue;
		      else if(p[pixel[12]] < c_b)
			if(p[pixel[6]] < c_b)
			  if(p[pixel[7]] < c_b)
			    if(p[pixel[8]] < c_b)
			      if(p[pixel[9]] < c_b)
				if(p[pixel[10]] < c_b)
				  if(p[pixel[11]] < c_b)
				    if(p[pixel[13]] < c_b)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else if(p[pixel[14]] < c_b)
		      if(p[pixel[7]] < c_b)
			if(p[pixel[8]] < c_b)
			  if(p[pixel[9]] < c_b)
			    if(p[pixel[10]] < c_b)
			      if(p[pixel[11]] < c_b)
				if(p[pixel[12]] < c_b)
				  if(p[pixel[13]] < c_b)
				    if(p[pixel[6]] < c_b)
				      {}
				    else
				      if(p[pixel[15]] < c_b)
					{}
				      else
					continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      if(p[pixel[6]] < c_b)
			if(p[pixel[7]] < c_b)
			  if(p[pixel[8]] < c_b)
			    if(p[pixel[9]] < c_b)
			      if(p[pixel[10]] < c_b)
				if(p[pixel[11]] < c_b)
				  if(p[pixel[12]] < c_b)
				    if(p[pixel[13]] < c_b)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    if(p[pixel[12]] > cb)
		      if(p[pixel[13]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    if(p[pixel[10]] > cb)
				      if(p[pixel[11]] > cb)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  continue;
		      else
			continue;
		    else if(p[pixel[12]] < c_b)
		      if(p[pixel[7]] < c_b)
			if(p[pixel[8]] < c_b)
			  if(p[pixel[9]] < c_b)
			    if(p[pixel[10]] < c_b)
			      if(p[pixel[11]] < c_b)
				if(p[pixel[13]] < c_b)
				  if(p[pixel[14]] < c_b)
				    if(p[pixel[6]] < c_b)
				      {}
				    else
				      if(p[pixel[15]] < c_b)
					{}
				      else
					continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else if(p[pixel[4]] < c_b)
		  if(p[pixel[13]] > cb)
		    if(p[pixel[11]] > cb)
		      if(p[pixel[12]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    if(p[pixel[10]] > cb)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    if(p[pixel[10]] > cb)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			continue;
		    else if(p[pixel[11]] < c_b)
		      if(p[pixel[5]] < c_b)
			if(p[pixel[6]] < c_b)
			  if(p[pixel[7]] < c_b)
			    if(p[pixel[8]] < c_b)
			      if(p[pixel[9]] < c_b)
				if(p[pixel[10]] < c_b)
				  if(p[pixel[12]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else if(p[pixel[13]] < c_b)
		    if(p[pixel[7]] < c_b)
		      if(p[pixel[8]] < c_b)
			if(p[pixel[9]] < c_b)
			  if(p[pixel[10]] < c_b)
			    if(p[pixel[11]] < c_b)
			      if(p[pixel[12]] < c_b)
				if(p[pixel[6]] < c_b)
				  if(p[pixel[5]] < c_b)
				    {}
				  else
				    if(p[pixel[14]] < c_b)
				      {}
				    else
				      continue;
				else
				  if(p[pixel[14]] < c_b)
				    if(p[pixel[15]] < c_b)
				      {}
				    else
				      continue;
				  else
				    continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else
		    if(p[pixel[5]] < c_b)
		      if(p[pixel[6]] < c_b)
			if(p[pixel[7]] < c_b)
			  if(p[pixel[8]] < c_b)
			    if(p[pixel[9]] < c_b)
			      if(p[pixel[10]] < c_b)
				if(p[pixel[11]] < c_b)
				  if(p[pixel[12]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else
		  if(p[pixel[11]] > cb)
		    if(p[pixel[12]] > cb)
		      if(p[pixel[13]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    if(p[pixel[10]] > cb)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    if(p[pixel[10]] > cb)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			continue;
		    else
		      continue;
		  else if(p[pixel[11]] < c_b)
		    if(p[pixel[7]] < c_b)
		      if(p[pixel[8]] < c_b)
			if(p[pixel[9]] < c_b)
			  if(p[pixel[10]] < c_b)
			    if(p[pixel[12]] < c_b)
			      if(p[pixel[13]] < c_b)
				if(p[pixel[6]] < c_b)
				  if(p[pixel[5]] < c_b)
				    {}
				  else
				    if(p[pixel[14]] < c_b)
				      {}
				    else
				      continue;
				else
				  if(p[pixel[14]] < c_b)
				    if(p[pixel[15]] < c_b)
				      {}
				    else
				      continue;
				  else
				    continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else
		    continue;
	      else if(p[pixel[3]] < c_b)
		if(p[pixel[10]] > cb)
		  if(p[pixel[11]] > cb)
		    if(p[pixel[12]] > cb)
		      if(p[pixel[13]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      continue;
		  else
		    continue;
		else if(p[pixel[10]] < c_b)
		  if(p[pixel[7]] < c_b)
		    if(p[pixel[8]] < c_b)
		      if(p[pixel[9]] < c_b)
			if(p[pixel[11]] < c_b)
			  if(p[pixel[6]] < c_b)
			    if(p[pixel[5]] < c_b)
			      if(p[pixel[4]] < c_b)
				{}
			      else
				if(p[pixel[12]] < c_b)
				  if(p[pixel[13]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			    else
			      if(p[pixel[12]] < c_b)
				if(p[pixel[13]] < c_b)
				  if(p[pixel[14]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			  else
			    if(p[pixel[12]] < c_b)
			      if(p[pixel[13]] < c_b)
				if(p[pixel[14]] < c_b)
				  if(p[pixel[15]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else
		    continue;
		else
		  continue;
	      else
		if(p[pixel[10]] > cb)
		  if(p[pixel[11]] > cb)
		    if(p[pixel[12]] > cb)
		      if(p[pixel[13]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  if(p[pixel[9]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      continue;
		  else
		    continue;
		else if(p[pixel[10]] < c_b)
		  if(p[pixel[7]] < c_b)
		    if(p[pixel[8]] < c_b)
		      if(p[pixel[9]] < c_b)
			if(p[pixel[11]] < c_b)
			  if(p[pixel[12]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[5]] < c_b)
				if(p[pixel[4]] < c_b)
				  {}
				else
				  if(p[pixel[13]] < c_b)
				    {}
				  else
				    continue;
			      else
				if(p[pixel[13]] < c_b)
				  if(p[pixel[14]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			    else
			      if(p[pixel[13]] < c_b)
				if(p[pixel[14]] < c_b)
				  if(p[pixel[15]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else
		    continue;
		else
		  continue;
	    else if(p[pixel[2]] < c_b)
	      if(p[pixel[9]] > cb)
		if(p[pixel[10]] > cb)
		  if(p[pixel[11]] > cb)
		    if(p[pixel[12]] > cb)
		      if(p[pixel[13]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[3]] > cb)
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    continue;
		else
		  continue;
	      else if(p[pixel[9]] < c_b)
		if(p[pixel[7]] < c_b)
		  if(p[pixel[8]] < c_b)
		    if(p[pixel[10]] < c_b)
		      if(p[pixel[6]] < c_b)
			if(p[pixel[5]] < c_b)
			  if(p[pixel[4]] < c_b)
			    if(p[pixel[3]] < c_b)
			      {}
			    else
			      if(p[pixel[11]] < c_b)
				if(p[pixel[12]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			  else
			    if(p[pixel[11]] < c_b)
			      if(p[pixel[12]] < c_b)
				if(p[pixel[13]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[11]] < c_b)
			    if(p[pixel[12]] < c_b)
			      if(p[pixel[13]] < c_b)
				if(p[pixel[14]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[11]] < c_b)
			  if(p[pixel[12]] < c_b)
			    if(p[pixel[13]] < c_b)
			      if(p[pixel[14]] < c_b)
				if(p[pixel[15]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      continue;
		  else
		    continue;
		else
		  continue;
	      else
		continue;
	    else
	      if(p[pixel[9]] > cb)
		if(p[pixel[10]] > cb)
		  if(p[pixel[11]] > cb)
		    if(p[pixel[12]] > cb)
		      if(p[pixel[13]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[3]] > cb)
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				if(p[pixel[8]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    continue;
		else
		  continue;
	      else if(p[pixel[9]] < c_b)
		if(p[pixel[7]] < c_b)
		  if(p[pixel[8]] < c_b)
		    if(p[pixel[10]] < c_b)
		      if(p[pixel[11]] < c_b)
			if(p[pixel[6]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[4]] < c_b)
			      if(p[pixel[3]] < c_b)
				{}
			      else
				if(p[pixel[12]] < c_b)
				  {}
				else
				  continue;
			    else
			      if(p[pixel[12]] < c_b)
				if(p[pixel[13]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			  else
			    if(p[pixel[12]] < c_b)
			      if(p[pixel[13]] < c_b)
				if(p[pixel[14]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[12]] < c_b)
			    if(p[pixel[13]] < c_b)
			      if(p[pixel[14]] < c_b)
				if(p[pixel[15]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			continue;
		    else
		      continue;
		  else
		    continue;
		else
		  continue;
	      else
		continue;
	  else if(p[pixel[1]] < c_b)
	    if(p[pixel[8]] > cb)
	      if(p[pixel[9]] > cb)
		if(p[pixel[10]] > cb)
		  if(p[pixel[11]] > cb)
		    if(p[pixel[12]] > cb)
		      if(p[pixel[13]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[3]] > cb)
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    if(p[pixel[2]] > cb)
		      if(p[pixel[3]] > cb)
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else
		  continue;
	      else
		continue;
	    else if(p[pixel[8]] < c_b)
	      if(p[pixel[7]] < c_b)
		if(p[pixel[9]] < c_b)
		  if(p[pixel[6]] < c_b)
		    if(p[pixel[5]] < c_b)
		      if(p[pixel[4]] < c_b)
			if(p[pixel[3]] < c_b)
			  if(p[pixel[2]] < c_b)
			    {}
			  else
			    if(p[pixel[10]] < c_b)
			      if(p[pixel[11]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[10]] < c_b)
			    if(p[pixel[11]] < c_b)
			      if(p[pixel[12]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[10]] < c_b)
			  if(p[pixel[11]] < c_b)
			    if(p[pixel[12]] < c_b)
			      if(p[pixel[13]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[10]] < c_b)
			if(p[pixel[11]] < c_b)
			  if(p[pixel[12]] < c_b)
			    if(p[pixel[13]] < c_b)
			      if(p[pixel[14]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    if(p[pixel[10]] < c_b)
		      if(p[pixel[11]] < c_b)
			if(p[pixel[12]] < c_b)
			  if(p[pixel[13]] < c_b)
			    if(p[pixel[14]] < c_b)
			      if(p[pixel[15]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else
		  continue;
	      else
		continue;
	    else
	      continue;
	  else
	    if(p[pixel[8]] > cb)
	      if(p[pixel[9]] > cb)
		if(p[pixel[10]] > cb)
		  if(p[pixel[11]] > cb)
		    if(p[pixel[12]] > cb)
		      if(p[pixel[13]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[15]] > cb)
			    {}
			  else
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[3]] > cb)
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    if(p[pixel[2]] > cb)
		      if(p[pixel[3]] > cb)
			if(p[pixel[4]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[7]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else
		  continue;
	      else
		continue;
	    else if(p[pixel[8]] < c_b)
	      if(p[pixel[7]] < c_b)
		if(p[pixel[9]] < c_b)
		  if(p[pixel[10]] < c_b)
		    if(p[pixel[6]] < c_b)
		      if(p[pixel[5]] < c_b)
			if(p[pixel[4]] < c_b)
			  if(p[pixel[3]] < c_b)
			    if(p[pixel[2]] < c_b)
			      {}
			    else
			      if(p[pixel[11]] < c_b)
				{}
			      else
				continue;
			  else
			    if(p[pixel[11]] < c_b)
			      if(p[pixel[12]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[11]] < c_b)
			    if(p[pixel[12]] < c_b)
			      if(p[pixel[13]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[11]] < c_b)
			  if(p[pixel[12]] < c_b)
			    if(p[pixel[13]] < c_b)
			      if(p[pixel[14]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[11]] < c_b)
			if(p[pixel[12]] < c_b)
			  if(p[pixel[13]] < c_b)
			    if(p[pixel[14]] < c_b)
			      if(p[pixel[15]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    continue;
		else
		  continue;
	      else
		continue;
	    else
	      continue;
        else if(p[pixel[0]] < c_b)
	  if(p[pixel[1]] > cb)
	    if(p[pixel[8]] > cb)
	      if(p[pixel[7]] > cb)
		if(p[pixel[9]] > cb)
		  if(p[pixel[6]] > cb)
		    if(p[pixel[5]] > cb)
		      if(p[pixel[4]] > cb)
			if(p[pixel[3]] > cb)
			  if(p[pixel[2]] > cb)
			    {}
			  else
			    if(p[pixel[10]] > cb)
			      if(p[pixel[11]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[10]] > cb)
			    if(p[pixel[11]] > cb)
			      if(p[pixel[12]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[10]] > cb)
			  if(p[pixel[11]] > cb)
			    if(p[pixel[12]] > cb)
			      if(p[pixel[13]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[10]] > cb)
			if(p[pixel[11]] > cb)
			  if(p[pixel[12]] > cb)
			    if(p[pixel[13]] > cb)
			      if(p[pixel[14]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    if(p[pixel[10]] > cb)
		      if(p[pixel[11]] > cb)
			if(p[pixel[12]] > cb)
			  if(p[pixel[13]] > cb)
			    if(p[pixel[14]] > cb)
			      if(p[pixel[15]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else
		  continue;
	      else
		continue;
	    else if(p[pixel[8]] < c_b)
	      if(p[pixel[9]] < c_b)
		if(p[pixel[10]] < c_b)
		  if(p[pixel[11]] < c_b)
		    if(p[pixel[12]] < c_b)
		      if(p[pixel[13]] < c_b)
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[3]] < c_b)
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    if(p[pixel[2]] < c_b)
		      if(p[pixel[3]] < c_b)
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else
		  continue;
	      else
		continue;
	    else
	      continue;
	  else if(p[pixel[1]] < c_b)
	    if(p[pixel[2]] > cb)
	      if(p[pixel[9]] > cb)
		if(p[pixel[7]] > cb)
		  if(p[pixel[8]] > cb)
		    if(p[pixel[10]] > cb)
		      if(p[pixel[6]] > cb)
			if(p[pixel[5]] > cb)
			  if(p[pixel[4]] > cb)
			    if(p[pixel[3]] > cb)
			      {}
			    else
			      if(p[pixel[11]] > cb)
				if(p[pixel[12]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			  else
			    if(p[pixel[11]] > cb)
			      if(p[pixel[12]] > cb)
				if(p[pixel[13]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[11]] > cb)
			    if(p[pixel[12]] > cb)
			      if(p[pixel[13]] > cb)
				if(p[pixel[14]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[11]] > cb)
			  if(p[pixel[12]] > cb)
			    if(p[pixel[13]] > cb)
			      if(p[pixel[14]] > cb)
				if(p[pixel[15]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      continue;
		  else
		    continue;
		else
		  continue;
	      else if(p[pixel[9]] < c_b)
		if(p[pixel[10]] < c_b)
		  if(p[pixel[11]] < c_b)
		    if(p[pixel[12]] < c_b)
		      if(p[pixel[13]] < c_b)
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[3]] < c_b)
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    continue;
		else
		  continue;
	      else
		continue;
	    else if(p[pixel[2]] < c_b)
	      if(p[pixel[3]] > cb)
		if(p[pixel[10]] > cb)
		  if(p[pixel[7]] > cb)
		    if(p[pixel[8]] > cb)
		      if(p[pixel[9]] > cb)
			if(p[pixel[11]] > cb)
			  if(p[pixel[6]] > cb)
			    if(p[pixel[5]] > cb)
			      if(p[pixel[4]] > cb)
				{}
			      else
				if(p[pixel[12]] > cb)
				  if(p[pixel[13]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			    else
			      if(p[pixel[12]] > cb)
				if(p[pixel[13]] > cb)
				  if(p[pixel[14]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			  else
			    if(p[pixel[12]] > cb)
			      if(p[pixel[13]] > cb)
				if(p[pixel[14]] > cb)
				  if(p[pixel[15]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else
		    continue;
		else if(p[pixel[10]] < c_b)
		  if(p[pixel[11]] < c_b)
		    if(p[pixel[12]] < c_b)
		      if(p[pixel[13]] < c_b)
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      continue;
		  else
		    continue;
		else
		  continue;
	      else if(p[pixel[3]] < c_b)
		if(p[pixel[4]] > cb)
		  if(p[pixel[13]] > cb)
		    if(p[pixel[7]] > cb)
		      if(p[pixel[8]] > cb)
			if(p[pixel[9]] > cb)
			  if(p[pixel[10]] > cb)
			    if(p[pixel[11]] > cb)
			      if(p[pixel[12]] > cb)
				if(p[pixel[6]] > cb)
				  if(p[pixel[5]] > cb)
				    {}
				  else
				    if(p[pixel[14]] > cb)
				      {}
				    else
				      continue;
				else
				  if(p[pixel[14]] > cb)
				    if(p[pixel[15]] > cb)
				      {}
				    else
				      continue;
				  else
				    continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else if(p[pixel[13]] < c_b)
		    if(p[pixel[11]] > cb)
		      if(p[pixel[5]] > cb)
			if(p[pixel[6]] > cb)
			  if(p[pixel[7]] > cb)
			    if(p[pixel[8]] > cb)
			      if(p[pixel[9]] > cb)
				if(p[pixel[10]] > cb)
				  if(p[pixel[12]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else if(p[pixel[11]] < c_b)
		      if(p[pixel[12]] < c_b)
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    if(p[pixel[10]] < c_b)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    if(p[pixel[10]] < c_b)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			continue;
		    else
		      continue;
		  else
		    if(p[pixel[5]] > cb)
		      if(p[pixel[6]] > cb)
			if(p[pixel[7]] > cb)
			  if(p[pixel[8]] > cb)
			    if(p[pixel[9]] > cb)
			      if(p[pixel[10]] > cb)
				if(p[pixel[11]] > cb)
				  if(p[pixel[12]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else if(p[pixel[4]] < c_b)
		  if(p[pixel[5]] > cb)
		    if(p[pixel[14]] > cb)
		      if(p[pixel[7]] > cb)
			if(p[pixel[8]] > cb)
			  if(p[pixel[9]] > cb)
			    if(p[pixel[10]] > cb)
			      if(p[pixel[11]] > cb)
				if(p[pixel[12]] > cb)
				  if(p[pixel[13]] > cb)
				    if(p[pixel[6]] > cb)
				      {}
				    else
				      if(p[pixel[15]] > cb)
					{}
				      else
					continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else if(p[pixel[14]] < c_b)
		      if(p[pixel[12]] > cb)
			if(p[pixel[6]] > cb)
			  if(p[pixel[7]] > cb)
			    if(p[pixel[8]] > cb)
			      if(p[pixel[9]] > cb)
				if(p[pixel[10]] > cb)
				  if(p[pixel[11]] > cb)
				    if(p[pixel[13]] > cb)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else if(p[pixel[12]] < c_b)
			if(p[pixel[13]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    if(p[pixel[10]] < c_b)
				      if(p[pixel[11]] < c_b)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  continue;
		      else
			continue;
		    else
		      if(p[pixel[6]] > cb)
			if(p[pixel[7]] > cb)
			  if(p[pixel[8]] > cb)
			    if(p[pixel[9]] > cb)
			      if(p[pixel[10]] > cb)
				if(p[pixel[11]] > cb)
				  if(p[pixel[12]] > cb)
				    if(p[pixel[13]] > cb)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else if(p[pixel[5]] < c_b)
		    if(p[pixel[6]] > cb)
		      if(p[pixel[15]] < c_b)
			if(p[pixel[13]] > cb)
			  if(p[pixel[7]] > cb)
			    if(p[pixel[8]] > cb)
			      if(p[pixel[9]] > cb)
				if(p[pixel[10]] > cb)
				  if(p[pixel[11]] > cb)
				    if(p[pixel[12]] > cb)
				      if(p[pixel[14]] > cb)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else if(p[pixel[13]] < c_b)
			  if(p[pixel[14]] < c_b)
			    {}
			  else
			    continue;
			else
			  continue;
		      else
			if(p[pixel[7]] > cb)
			  if(p[pixel[8]] > cb)
			    if(p[pixel[9]] > cb)
			      if(p[pixel[10]] > cb)
				if(p[pixel[11]] > cb)
				  if(p[pixel[12]] > cb)
				    if(p[pixel[13]] > cb)
				      if(p[pixel[14]] > cb)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else if(p[pixel[6]] < c_b)
		      if(p[pixel[7]] > cb)
			if(p[pixel[14]] > cb)
			  if(p[pixel[8]] > cb)
			    if(p[pixel[9]] > cb)
			      if(p[pixel[10]] > cb)
				if(p[pixel[11]] > cb)
				  if(p[pixel[12]] > cb)
				    if(p[pixel[13]] > cb)
				      if(p[pixel[15]] > cb)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    continue;
			else
			  continue;
		      else if(p[pixel[7]] < c_b)
			if(p[pixel[8]] < c_b)
			  {}
			else
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    continue;
		      else
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[13]] > cb)
			if(p[pixel[7]] > cb)
			  if(p[pixel[8]] > cb)
			    if(p[pixel[9]] > cb)
			      if(p[pixel[10]] > cb)
				if(p[pixel[11]] > cb)
				  if(p[pixel[12]] > cb)
				    if(p[pixel[14]] > cb)
				      if(p[pixel[15]] > cb)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else if(p[pixel[13]] < c_b)
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    if(p[pixel[12]] > cb)
		      if(p[pixel[7]] > cb)
			if(p[pixel[8]] > cb)
			  if(p[pixel[9]] > cb)
			    if(p[pixel[10]] > cb)
			      if(p[pixel[11]] > cb)
				if(p[pixel[13]] > cb)
				  if(p[pixel[14]] > cb)
				    if(p[pixel[6]] > cb)
				      {}
				    else
				      if(p[pixel[15]] > cb)
					{}
				      else
					continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else if(p[pixel[12]] < c_b)
		      if(p[pixel[13]] < c_b)
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    if(p[pixel[10]] < c_b)
				      if(p[pixel[11]] < c_b)
					{}
				      else
					continue;
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else
		  if(p[pixel[11]] > cb)
		    if(p[pixel[7]] > cb)
		      if(p[pixel[8]] > cb)
			if(p[pixel[9]] > cb)
			  if(p[pixel[10]] > cb)
			    if(p[pixel[12]] > cb)
			      if(p[pixel[13]] > cb)
				if(p[pixel[6]] > cb)
				  if(p[pixel[5]] > cb)
				    {}
				  else
				    if(p[pixel[14]] > cb)
				      {}
				    else
				      continue;
				else
				  if(p[pixel[14]] > cb)
				    if(p[pixel[15]] > cb)
				      {}
				    else
				      continue;
				  else
				    continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else if(p[pixel[11]] < c_b)
		    if(p[pixel[12]] < c_b)
		      if(p[pixel[13]] < c_b)
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    if(p[pixel[10]] < c_b)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    if(p[pixel[10]] < c_b)
				      {}
				    else
				      continue;
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			continue;
		    else
		      continue;
		  else
		    continue;
	      else
		if(p[pixel[10]] > cb)
		  if(p[pixel[7]] > cb)
		    if(p[pixel[8]] > cb)
		      if(p[pixel[9]] > cb)
			if(p[pixel[11]] > cb)
			  if(p[pixel[12]] > cb)
			    if(p[pixel[6]] > cb)
			      if(p[pixel[5]] > cb)
				if(p[pixel[4]] > cb)
				  {}
				else
				  if(p[pixel[13]] > cb)
				    {}
				  else
				    continue;
			      else
				if(p[pixel[13]] > cb)
				  if(p[pixel[14]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			    else
			      if(p[pixel[13]] > cb)
				if(p[pixel[14]] > cb)
				  if(p[pixel[15]] > cb)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else
		    continue;
		else if(p[pixel[10]] < c_b)
		  if(p[pixel[11]] < c_b)
		    if(p[pixel[12]] < c_b)
		      if(p[pixel[13]] < c_b)
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  if(p[pixel[9]] < c_b)
				    {}
				  else
				    continue;
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      continue;
		  else
		    continue;
		else
		  continue;
	    else
	      if(p[pixel[9]] > cb)
		if(p[pixel[7]] > cb)
		  if(p[pixel[8]] > cb)
		    if(p[pixel[10]] > cb)
		      if(p[pixel[11]] > cb)
			if(p[pixel[6]] > cb)
			  if(p[pixel[5]] > cb)
			    if(p[pixel[4]] > cb)
			      if(p[pixel[3]] > cb)
				{}
			      else
				if(p[pixel[12]] > cb)
				  {}
				else
				  continue;
			    else
			      if(p[pixel[12]] > cb)
				if(p[pixel[13]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			  else
			    if(p[pixel[12]] > cb)
			      if(p[pixel[13]] > cb)
				if(p[pixel[14]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[12]] > cb)
			    if(p[pixel[13]] > cb)
			      if(p[pixel[14]] > cb)
				if(p[pixel[15]] > cb)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			continue;
		    else
		      continue;
		  else
		    continue;
		else
		  continue;
	      else if(p[pixel[9]] < c_b)
		if(p[pixel[10]] < c_b)
		  if(p[pixel[11]] < c_b)
		    if(p[pixel[12]] < c_b)
		      if(p[pixel[13]] < c_b)
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[3]] < c_b)
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				if(p[pixel[8]] < c_b)
				  {}
				else
				  continue;
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    continue;
		else
		  continue;
	      else
		continue;
	  else
	    if(p[pixel[8]] > cb)
	      if(p[pixel[7]] > cb)
		if(p[pixel[9]] > cb)
		  if(p[pixel[10]] > cb)
		    if(p[pixel[6]] > cb)
		      if(p[pixel[5]] > cb)
			if(p[pixel[4]] > cb)
			  if(p[pixel[3]] > cb)
			    if(p[pixel[2]] > cb)
			      {}
			    else
			      if(p[pixel[11]] > cb)
				{}
			      else
				continue;
			  else
			    if(p[pixel[11]] > cb)
			      if(p[pixel[12]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[11]] > cb)
			    if(p[pixel[12]] > cb)
			      if(p[pixel[13]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[11]] > cb)
			  if(p[pixel[12]] > cb)
			    if(p[pixel[13]] > cb)
			      if(p[pixel[14]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[11]] > cb)
			if(p[pixel[12]] > cb)
			  if(p[pixel[13]] > cb)
			    if(p[pixel[14]] > cb)
			      if(p[pixel[15]] > cb)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    continue;
		else
		  continue;
	      else
		continue;
	    else if(p[pixel[8]] < c_b)
	      if(p[pixel[9]] < c_b)
		if(p[pixel[10]] < c_b)
		  if(p[pixel[11]] < c_b)
		    if(p[pixel[12]] < c_b)
		      if(p[pixel[13]] < c_b)
			if(p[pixel[14]] < c_b)
			  if(p[pixel[15]] < c_b)
			    {}
			  else
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			else
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[3]] < c_b)
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    if(p[pixel[2]] < c_b)
		      if(p[pixel[3]] < c_b)
			if(p[pixel[4]] < c_b)
			  if(p[pixel[5]] < c_b)
			    if(p[pixel[6]] < c_b)
			      if(p[pixel[7]] < c_b)
				{}
			      else
				continue;
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else
		  continue;
	      else
		continue;
	    else
	      continue;
        else
	  if(p[pixel[7]] > cb)
	    if(p[pixel[8]] > cb)
	      if(p[pixel[9]] > cb)
		if(p[pixel[6]] > cb)
		  if(p[pixel[5]] > cb)
		    if(p[pixel[4]] > cb)
		      if(p[pixel[3]] > cb)
			if(p[pixel[2]] > cb)
			  if(p[pixel[1]] > cb)
			    {}
			  else
			    if(p[pixel[10]] > cb)
			      {}
			    else
			      continue;
			else
			  if(p[pixel[10]] > cb)
			    if(p[pixel[11]] > cb)
			      {}
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[10]] > cb)
			  if(p[pixel[11]] > cb)
			    if(p[pixel[12]] > cb)
			      {}
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[10]] > cb)
			if(p[pixel[11]] > cb)
			  if(p[pixel[12]] > cb)
			    if(p[pixel[13]] > cb)
			      {}
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    if(p[pixel[10]] > cb)
		      if(p[pixel[11]] > cb)
			if(p[pixel[12]] > cb)
			  if(p[pixel[13]] > cb)
			    if(p[pixel[14]] > cb)
			      {}
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else
		  if(p[pixel[10]] > cb)
		    if(p[pixel[11]] > cb)
		      if(p[pixel[12]] > cb)
			if(p[pixel[13]] > cb)
			  if(p[pixel[14]] > cb)
			    if(p[pixel[15]] > cb)
			      {}
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else
		    continue;
	      else
		continue;
	    else
	      continue;
	  else if(p[pixel[7]] < c_b)
	    if(p[pixel[8]] < c_b)
	      if(p[pixel[9]] < c_b)
		if(p[pixel[6]] < c_b)
		  if(p[pixel[5]] < c_b)
		    if(p[pixel[4]] < c_b)
		      if(p[pixel[3]] < c_b)
			if(p[pixel[2]] < c_b)
			  if(p[pixel[1]] < c_b)
			    {}
			  else
			    if(p[pixel[10]] < c_b)
			      {}
			    else
			      continue;
			else
			  if(p[pixel[10]] < c_b)
			    if(p[pixel[11]] < c_b)
			      {}
			    else
			      continue;
			  else
			    continue;
		      else
			if(p[pixel[10]] < c_b)
			  if(p[pixel[11]] < c_b)
			    if(p[pixel[12]] < c_b)
			      {}
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		    else
		      if(p[pixel[10]] < c_b)
			if(p[pixel[11]] < c_b)
			  if(p[pixel[12]] < c_b)
			    if(p[pixel[13]] < c_b)
			      {}
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		  else
		    if(p[pixel[10]] < c_b)
		      if(p[pixel[11]] < c_b)
			if(p[pixel[12]] < c_b)
			  if(p[pixel[13]] < c_b)
			    if(p[pixel[14]] < c_b)
			      {}
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		else
		  if(p[pixel[10]] < c_b)
		    if(p[pixel[11]] < c_b)
		      if(p[pixel[12]] < c_b)
			if(p[pixel[13]] < c_b)
			  if(p[pixel[14]] < c_b)
			    if(p[pixel[15]] < c_b)
			      {}
			    else
			      continue;
			  else
			    continue;
			else
			  continue;
		      else
			continue;
		    else
		      continue;
		  else
		    continue;
	      else
		continue;
	    else
	      continue;
	  else
	    continue;
	if(num_corners < FAST_MAX_CORNERS) {
	  corners[num_corners].x = x;
	  corners[num_corners].y = y;
	  num_corners++;
	}

      }

  //printf("num_corners2 %d\n", num_corners);
  *ret_num_corners = num_corners;
}

void fast::nonmax_suppression(
			      xy* corners,
			      int* scores,
			      int num_corners)
{
  num_nonmax=0;
  int last_row;
  int i, j;
  const int sz = (int)num_corners;

  /*Point above points (roughly) to the pixel above the one of interest, if there
    is a feature there.*/
  int point_above = 0;
  int point_below = 0;

  if(num_corners > 0) {

    //memset((void*)row_start, '\0', FAST_MAX_IMAGE_HEIGHT*sizeof(int));

    /* Find where each row begins
       (the corners are output in raster scan order). A beginning of -1 signifies
       that there are no corners on that row. */
    last_row = corners[num_corners-1].y;

    for (i = 0; i < last_row+1; i++)
      row_start[i] = -1;

    int prev_row = -1;
    for (i = 0; i < num_corners; i++) {
      if(corners[i].y != prev_row)
	{
	  row_start[corners[i].y] = i;
	  prev_row = corners[i].y;
	}
    }

    for (i = 0; i < sz; i++)
      {
	int score = scores[i];
	xy pos = corners[i];

	/*Check left */
	if(i > 0)
	  if(corners[i-1].x == pos.x-1 && corners[i-1].y == pos.y && Compare(scores[i-1], score))
	    continue;

	/*Check right*/
	if(i < (sz - 1))
	  if(corners[i+1].x == pos.x+1 && corners[i+1].y == pos.y && Compare(scores[i+1], score))
	    continue;

	/*Check above (if there is a valid row above)*/
	if(pos.y != 0 && row_start[pos.y - 1] != -1)
	  {
	    /*Make sure that current point_above is one
	      row above.*/
	    if(corners[point_above].y < pos.y - 1)
	      point_above = row_start[pos.y-1];

	    /*Make point_above point to the first of the pixels above the current point,
	      if it exists.*/
	    for(; corners[point_above].y < pos.y && corners[point_above].x < pos.x - 1; point_above++)
	      {}


	    for(j=point_above; corners[j].y < pos.y && corners[j].x <= pos.x + 1; j++)
	      {
		int x = corners[j].x;
		if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && Compare(scores[j], score))
		  goto cont;
	      }

	  }

	/*Check below (if there is anything below)*/
	if(pos.y != last_row && row_start[pos.y + 1] != -1 && point_below < sz) /*Nothing below*/
	  {
	    if(corners[point_below].y < pos.y + 1)
	      point_below = row_start[pos.y+1];

	    /* Make point below point to one of the pixels belowthe current point, if it
	       exists.*/
	    for(; point_below < sz && corners[point_below].y == pos.y+1 && corners[point_below].x < pos.x - 1; point_below++)
	      {}

	    for(j=point_below; j < sz && corners[j].y == pos.y+1 && corners[j].x <= pos.x + 1; j++)
	      {
		int x = corners[j].x;
		if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && Compare(scores[j],score))
		  goto cont;
	      }
	  }

	if ((corners[i].x < 0) || (corners[i].x > 320)) {
	  printf("corner out of range %d\n", corners[i].x);
	}
	if (num_nonmax < FAST_MAX_CORNERS_PREVIOUS) {
	  nonmax[num_nonmax++] = corners[i];
	}
      cont:
	;
      }

  }
}

void fast::detect_nonmax(
			 unsigned char* im,
			 int xsize,
			 int ysize,
			 int stride,
			 int b,
			 int* ret_num_corners __attribute__((unused)))
{
  int num_corners=0;

  detect(im, xsize, ysize, stride, b, &num_corners);
  score(im, stride, corners, num_corners, b);
  nonmax_suppression(corners, scores, num_corners);
}

/* updates FAST corners */
int fast::update(
		 unsigned char* img,
		 int img_width,
		 int img_height,
		 int desired_features,
		 int use_tracking)
{
  int i,n;

  if (img_mono == NULL) {
    img_mono = new unsigned char[img_width*img_height];
    prev_img_mono = new unsigned char[img_width*img_height];
  }

  /* remember previous corner positions so that we can track them */
  if (num_nonmax > FAST_MAX_CORNERS_PREVIOUS) num_nonmax = FAST_MAX_CORNERS_PREVIOUS;
  if (num_nonmax > FAST_MIN_CORNERS) {
    for (i = FAST_PREVIOUS_BUFFER-1; i > 0; i--) {
      if (previous_no_of_corners[i-1] > 0) {
	memcpy(previous_corners[i], previous_corners[i-1], previous_no_of_corners[i-1]*sizeof(xy));
	memcpy(temporal_matches[i], temporal_matches[i-1], previous_no_of_corners[i-1]);
	memcpy(previous_interocular_disparity[i], previous_interocular_disparity[i-1], previous_no_of_corners[i-1]*sizeof(unsigned short));
      }
      previous_no_of_corners[i] = previous_no_of_corners[i-1];
    }
    memcpy(previous_corners[0], nonmax, num_nonmax*sizeof(xy));
    memcpy(previous_interocular_disparity[0], interocular_disparity, num_nonmax*sizeof(unsigned short));
    previous_no_of_corners[0] = num_nonmax;
  }

  n = img_width*img_height-1;
  for (i = img_width*img_height*3-1; i >= 2; i -= 3, n--) {
    img_mono[n] = img[i];
  }

  int ret_num_corners = 0;
  detect_nonmax(img_mono, img_width, img_height, img_width, threshold, &ret_num_corners);
  ret_num_corners = num_nonmax;
  if (ret_num_corners < desired_features-10) {
    threshold -= 10;
    if (threshold < 2) threshold=2;
    detect_nonmax(img_mono, img_width, img_height, img_width, threshold, &ret_num_corners);
    if (ret_num_corners < desired_features-10) {
      threshold -= 10;
      if (threshold < 2) threshold=2;
      detect_nonmax(img_mono, img_width, img_height, img_width, threshold, &ret_num_corners);
      if (ret_num_corners < desired_features-10) {
	threshold -= 10;
	if (threshold < 2) threshold=2;
      }
    }
  }
  if (ret_num_corners > desired_features+10) {
    threshold += 8;
    if (threshold > 300) threshold=300;
    detect_nonmax(img_mono, img_width, img_height, img_width, threshold, &ret_num_corners);
    if (ret_num_corners > desired_features+10) {
      threshold += 8;
      if (threshold > 300) threshold=300;
    }
  }

  if (num_nonmax > FAST_MAX_CORNERS_PREVIOUS) num_nonmax = FAST_MAX_CORNERS_PREVIOUS;

  /* track corners from one frame to the next */
  if ((use_tracking != 0) && (num_nonmax > FAST_MIN_CORNERS)) {
    match_temporal(img_mono, img_width, img_height, num_nonmax, nonmax, previous_no_of_corners[0], previous_corners[0], temporal_matches[0], 16);
  }

  return (ret_num_corners);
}

/* creates a set of "true scale" binary descriptors for each feature */
void fast::update_descriptors(
			      unsigned char *img,
			      int img_width,
			      int img_height,
			      unsigned int* descriptor,
			      unsigned char* descriptor_colour,
			      unsigned char* descriptor_direction)
{
  int f, radius, disp;

  for (f = 0; f < num_nonmax; f++) {
    disp = (int)interocular_disparity[f] - 1;
    if (disp > 0) {
      radius = disp/(FAST_SUBPIXEL*6);
      if (radius < 10) radius = 10;
    }
    else {
      radius = 10;
    }
    compute_descriptor(
		       img, img_width, img_height,
		       nonmax[f].x, nonmax[f].y,
		       radius, f, descriptor,
		       descriptor_colour,
		       descriptor_direction);
  }
}

/*
 * creates a binary descriptor for an area with the given radius
 * The descriptor consists of 4 integers, each containing a 5x5 array of bits
 */
void fast::compute_descriptor(
			      unsigned char *img,
			      int img_width,
			      int img_height,
			      int x, int y,
			      int radius,
			      int descriptor_index,
			      unsigned int* descriptor,
			      unsigned char* descriptor_colour,
			      unsigned char* descriptor_direction)
{
  int mean = 0;
  int max = img_width * img_height*3-3;
  int xx,yy,r,w,w2,n2,n=0,tot,max2,w3;
  int tot_r=0,tot_g=0,tot_b=0;
  int dir = 0;
  for (r = 1; r <= FAST_DESCRIPTOR_RADIUS; r++) {
    for (w = 0; w < FAST_DESCRIPTOR_WIDTH; w++, n += 2) {
      xx = x + (descriptor_lookup[n] * radius / 100);
      yy = y + (descriptor_lookup[n+1] * radius / 100);
      n2 = (yy*img_width + xx) * 3;
      if ((n2 > -1) && (n2 < max)) {
	mean += img[n2] + img[n2+1] + img[n2+2];
	tot_r += img[n2+2];
	tot_g += img[n2+1];
	tot_b += img[n2];
      }
    }
  }
  mean /= (FAST_DESCRIPTOR_RADIUS*FAST_DESCRIPTOR_WIDTH);
  int offset = descriptor_index*3;
  descriptor_colour[offset] = (unsigned char)(tot_r / (FAST_DESCRIPTOR_RADIUS*FAST_DESCRIPTOR_WIDTH));
  descriptor_colour[offset+1] = (unsigned char)(tot_g / (FAST_DESCRIPTOR_RADIUS*FAST_DESCRIPTOR_WIDTH));
  descriptor_colour[offset+2] = (unsigned char)(tot_b / (FAST_DESCRIPTOR_RADIUS*FAST_DESCRIPTOR_WIDTH));

  for (w = 0; w < FAST_DESCRIPTOR_WIDTH; w++) {
    descriptor_direction_buf[w] = 0;
    for (r = 1; r <= FAST_DESCRIPTOR_RADIUS; r++) {
      n = r * FAST_DESCRIPTOR_RADIUS + w;
      xx = x + (descriptor_lookup[n] * radius / 100);
      yy = y + (descriptor_lookup[n+1] * radius / 100);
      n2 = (yy*img_width + xx) * 3;
      if ((n2 > -1) && (n2 < max)) {
	if (img[n2] + img[n2+1] + img[n2+2] > mean) {
	  descriptor_direction_buf[w]++;
	}
      }
    }
  }
  max2 = 0;
  int rr = FAST_DESCRIPTOR_WIDTH/6;
  for (w = 0; w < FAST_DESCRIPTOR_WIDTH; w++) {
    tot=0;
    for (w2 = -rr; w2 <= rr; w2++) {
      w3 = w + w2;
      if (w3 < 0) w3 += FAST_DESCRIPTOR_WIDTH;
      if (w3 >= FAST_DESCRIPTOR_WIDTH) w3 -=FAST_DESCRIPTOR_WIDTH;
      tot += descriptor_direction_buf[w3] * (1 + rr - w2);
    }
    if (tot > max2) {
      max2 = tot;
      dir = w;
    }
  }
  descriptor_direction[descriptor_index] = dir;

  int bit = 1;
  int bitno = 0;
  offset = descriptor_index*4;
  descriptor[offset] = 0;
  descriptor[offset+1] = 0;
  descriptor[offset+2] = 0;
  descriptor[offset+3] = 0;
  for (r = 1; r <= FAST_DESCRIPTOR_RADIUS; r++) {
    for (w = 0; w < FAST_DESCRIPTOR_WIDTH; w++) {
      w2 = w + dir;
      if (w2 >= FAST_DESCRIPTOR_WIDTH-1) w2 -= FAST_DESCRIPTOR_WIDTH;
      n = (r * FAST_DESCRIPTOR_WIDTH) + w2;
      xx = x + (descriptor_lookup[n] * radius / 100);
      yy = y + (descriptor_lookup[n+1] * radius / 100);
      n2 = (yy*img_width + xx) * 3;
      if ((n2 > -1) && (n2 < max)) {
	if (img[n2] + img[n2+1] + img[n2+2] > mean) {
	  descriptor[offset] |= bit;
	}
      }
      if (bitno == 31) {
	bit = 1;
	bitno = 0;
	offset++;
      }
    }
  }
}

/* shows FAST corners */
void fast::show(
		unsigned char *outbuf,
		int img_width,
		int img_height,
		int show_tracking)
{
  int f,x,y,x2,y2,n,dx,dy,xx,yy,idx,disp,i;
  int r,g,b;

  for (f = 0; f < num_nonmax; f++) {
    idx = (int)temporal_matches[0][f] - 1;
    if ((show_tracking == 0) ||
	(num_nonmax < FAST_MIN_CORNERS) ||
	((idx > -1) &&
	 (idx < previous_no_of_corners[0]))) {

      x = nonmax[f].x;
      y = nonmax[f].y;

      r = 255;
      g = 0;
      b = 0;

      /* draw edge */
      n = (y*img_width+x)*3;
      outbuf[n++] = b;
      outbuf[n++] = g;
      outbuf[n] = r;
      n = ((y-1)*img_width+x)*3;
      outbuf[n++] = b;
      outbuf[n++] = g;
      outbuf[n] = r;
      n = ((y-2)*img_width+x)*3;
      outbuf[n++] = b;
      outbuf[n++] = g;
      outbuf[n] = r;
      n = ((y+1)*img_width+x)*3;
      outbuf[n++] = b;
      outbuf[n++] = g;
      outbuf[n] = r;
      n = ((y+2)*img_width+x)*3;
      outbuf[n++] = b;
      outbuf[n++] = g;
      outbuf[n] = r;
      n = (y*img_width+x-1)*3;
      outbuf[n++] = b;
      outbuf[n++] = g;
      outbuf[n] = r;
      n = (y*img_width+x+1)*3;
      outbuf[n++] = b;
      outbuf[n++] = g;
      outbuf[n] = r;
      n = (y*img_width+x-2)*3;
      outbuf[n++] = b;
      outbuf[n++] = g;
      outbuf[n] = r;
      n = (y*img_width+x+2)*3;
      outbuf[n++] = b;
      outbuf[n++] = g;
      outbuf[n] = r;

      if ((show_tracking != 0) &&
	  (num_nonmax > FAST_MIN_CORNERS)) {

	if ((interocular_disparity != NULL)) {
	  disp = (int)interocular_disparity[f] - 1;
	  if (disp > -1) {
	    disp = disp / FAST_SUBPIXEL;
	    for (xx = x; xx >= x-disp; xx--) {
	      if (xx > 0) {
		n = ((y-1)*img_width+xx)*3;
		outbuf[n++] = 0;
		outbuf[n++] = 255;
		outbuf[n] = 0;
	      }
	      else {
		break;
	      }
	    }
	  }
	}

	r = 255;
	g = 0;
	b = 0;
	i=0;
	while ((i < FAST_PREVIOUS_BUFFER-1) &&
	       (idx > -1)) {
	  x2 = previous_corners[i][idx].x;
	  y2 = previous_corners[i][idx].y;

	  dx = x2 - x;
	  dy = y2 - y;
	  if (abs(dx) > abs(dy)) {
	    xx = x;
	    while (xx != x2) {
	      xx += dx/abs(dx);
	      yy = y + ((xx-x)*dy/dx);
	      if ((xx > 0) && (xx < img_width) &&
		  (yy > 0) && (yy < img_height)) {
		n = (yy*img_width+xx)*3;
		outbuf[n++] = b;
		outbuf[n++] = g;
		outbuf[n] = r;
	      }
	    }
	  }
	  else {
	    if (dy != 0) {
	      yy = y;
	      while (yy != y2) {
		yy += dy/abs(dy);
		xx = x + ((yy-y)*dx/dy);
		if ((xx > 0) && (xx < img_width) &&
		    (yy > 0) && (yy < img_height)) {
		  n = (yy*img_width+xx)*3;
		  outbuf[n++] = b;
		  outbuf[n++] = g;
		  outbuf[n] = r;
		}
	      }
	    }
	  }

	  idx = (int)temporal_matches[i+1][idx] - 1;
	  x = x2;
	  y = y2;
	  i++;
	}

      }
    }
  }

}


void fast::match_temporal(
			  unsigned char* img_mono __attribute__((unused)),
			  int img_width,
			  int img_height,
			  int current_no_of_corners,
			  xy* current_corners,
			  int prev_no_of_corners,
			  xy* prev_corners,
			  unsigned char* matches,
			  int max_disparity __attribute__((unused)))
{
  /* First we make an estimate of the overall movement of all features.
     This enables us to handle sudden jumps caused by dropped frames,
     delays or sudden changes in pose */

  unsigned char idx;
  int offset_x,offset_y,gradient,i,j,n;
  int hits,hits_left,hits_right,x,y;
  int best_offset_x=0,best_offset_y=0,best_gradient=0;
  int best_offset_x_left=0,best_offset_y_left=0,best_gradient_left=0;
  int best_offset_x_right=0,best_offset_y_right=0,best_gradient_right=0;
  int max_offset_x = img_width/5;
  int max_offset_y = img_height/5;
  int max_hits=0,max_hits_left=0,max_hits_right=0;
  int hits_threshold = current_no_of_corners * 95/100;
  const int radius = 2;
  const int gradient_divisor = 100;

  memset((void*)prev_img_mono, '\0', img_width*img_height);
  memset((void*)matches, '\0', FAST_MAX_CORNERS_PREVIOUS);

  for (i = prev_no_of_corners-1; i >= 0; i--) {
    if ((prev_corners[i].x > radius) && (prev_corners[i].x < img_width-radius-1) &&
	(prev_corners[i].y > radius) && (prev_corners[i].y < img_height-radius-1)) {
      j = prev_corners[i].y * img_width + prev_corners[i].x;
      idx = (unsigned char)(1 + i);
      for (y = -radius; y <= radius; y++) {
	j = (prev_corners[i].y+y) * img_width + prev_corners[i].x - radius;
	for (x = -radius; x <= radius; x++, j++) {
	  prev_img_mono[j] = idx;
	}
      }
    }
  }

  int half_width = img_width/2;

  for (gradient = -5; gradient < 5; gradient++) {
    offset_y = -max_offset_y;
    while (offset_y < max_offset_y) {
      offset_x = -max_offset_x;
      while (offset_x < max_offset_x) {
	hits = 0;
	hits_left = 0;
	hits_right = 0;

	for (i = current_no_of_corners-1; i >= 0; i--) {
	  x = current_corners[i].x - offset_x;
	  y = current_corners[i].y - offset_y;
	  if (gradient != 0) {
	    y += ((current_corners[i].x - half_width) * gradient / gradient_divisor);
	  }
	  if ((x > 0) && (x < img_width) &&
	      (y > 0) && (y < img_height)) {
	    n = y * img_width + x;
	    if (prev_img_mono[n] != 0) {
	      hits++;
	      if (current_corners[i].x < half_width) {
		hits_left++;
	      }
	      else {
		hits_right++;
	      }
	    }
	  }
	}

	if (hits_left > max_hits_left) {
	  max_hits_left = hits_left;
	  best_offset_x_left = offset_x;
	  best_offset_y_left = offset_y;
	  best_gradient_left = gradient;
	}

	if (hits_right > max_hits_right) {
	  max_hits_right = hits_right;
	  best_offset_x_right = offset_x;
	  best_offset_y_right = offset_y;
	  best_gradient_right = gradient;
	}

	if (hits > max_hits) {
	  max_hits = hits;
	  best_offset_x = offset_x;
	  best_offset_y = offset_y;
	  best_gradient = gradient;
	  if (hits > hits_threshold) {
	    offset_x = max_offset_x;
	    offset_y = max_offset_y;
	    gradient = 9999;
	  }
	}

	if ((x <-30) || (x > 30)) {
	  offset_x+=4;
	}
	else {
	  if ((x <-10) || (x > 10)) {
	    offset_x+=3;
	  }
	  else {
	    if ((x <-5) || (x > 5))
	      offset_x+=2;
	    else
	      offset_x++;
	  }
	}
      }
      if ((y <-30) || (y > 30)) {
	offset_y+=4;
      }
      else {
	if ((y <-10) || (y > 10)) {
	  offset_y+=3;
	}
	else {
	  if ((y <-5) || (y > 5))
	    offset_y+=2;
	  else
	    offset_y++;
	}
      }
    }
  }

  if (current_no_of_corners > 0) {
    if (max_hits*100/current_no_of_corners > 20) {
      for (i = current_no_of_corners-1; i >= 0; i--) {
	x = current_corners[i].x - best_offset_x;
	y = current_corners[i].y - best_offset_y;
	if (best_gradient != 0) {
	  y += ((current_corners[i].x - half_width) * best_gradient / gradient_divisor);
	}
	if ((x > 0) && (x < img_width) &&
	    (y > 0) && (y < img_height)) {
	  n = y * img_width + x;
	  matches[i] = prev_img_mono[n];
	}
      }
    }
    if (max_hits_left*100/current_no_of_corners > 20) {
      for (i = current_no_of_corners-1; i >= 0; i--) {
	if ((matches[i] == 0) &&
	    (current_corners[i].x < half_width)) {
	  x = current_corners[i].x - best_offset_x_left;
	  y = current_corners[i].y - best_offset_y_left;
	  if (best_gradient_left != 0) {
	    y += ((current_corners[i].x - half_width) * best_gradient_left / gradient_divisor);
	  }
	  if ((x > 0) && (x < img_width) &&
	      (y > 0) && (y < img_height)) {
	    n = y * img_width + x;
	    matches[i] = prev_img_mono[n];
	  }
	}
      }
    }
    if (max_hits_right*100/current_no_of_corners > 20) {
      for (i = current_no_of_corners-1; i >= 0; i--) {
	if ((matches[i] == 0) &&
	    (current_corners[i].x >= half_width)) {
	  x = current_corners[i].x - best_offset_x_right;
	  y = current_corners[i].y - best_offset_y_right;
	  if (best_gradient_right != 0) {
	    y += ((current_corners[i].x - half_width) * best_gradient_right / gradient_divisor);
	  }
	  if ((x > 0) && (x < img_width) &&
	      (y > 0) && (y < img_height)) {
	    n = y * img_width + x;
	    matches[i] = prev_img_mono[n];
	  }
	}
      }
    }
  }

  temporal_offset_x = best_offset_x;
  temporal_offset_y = best_offset_y;

}

/* saves stereo matches to file for use by other programs */
void fast::save_matches(
			std::string filename, /* filename to save as */
			unsigned char* img, /* left image data */
			int img_width,
			bool colour) { /* whether to additionally save colour of each match */

  FILE *file = fopen(filename.c_str(), "wb");
  if (file != NULL) {

    int i,ctr,max = 0;
    for (i = 0; i < num_nonmax; i++) {
      if (interocular_disparity[i] > 0) max++;
    }

    if (max > 0) {

      if (!colour) {

	struct MatchData {
	  unsigned short int probability;
	  unsigned short int x;
	  unsigned short int y;
	  unsigned short int disparity;
	};

	MatchData *m = new MatchData[FAST_MAX_CORNERS_PREVIOUS];
	ctr = 0;
	for (i = 0; i < num_nonmax; i++) {
	  if (interocular_disparity[i] > 0) {
	    m[ctr].probability = (unsigned short int)1000;
	    m[ctr].x = (unsigned short int)nonmax[i].x*FAST_SUBPIXEL;
	    m[ctr].y = (unsigned short int)nonmax[i].y;
	    m[ctr].disparity = (unsigned short int)(interocular_disparity[i]-1);
	    ctr++;
	  }
	}
	if (ctr < FAST_MAX_CORNERS_PREVIOUS) {
	  m[ctr].x = 9999;
	  m[ctr].y = 9999;
	}

	fwrite(m, sizeof(MatchData), FAST_MAX_CORNERS_PREVIOUS, file);
	delete[] m;
      } else {
	struct MatchDataColour {
	  unsigned short int probability;
	  unsigned short int x;
	  unsigned short int y;
	  unsigned short int disparity;
	  unsigned char r, g, b;
	  unsigned char pack;
	};

	int n;
	ctr=0;
	MatchDataColour *m = new MatchDataColour[FAST_MAX_CORNERS_PREVIOUS];
	for (i = 0; i < num_nonmax; i++) {
	  if (interocular_disparity[i] > 0) {
	    m[ctr].probability = (unsigned short int)1000;
	    m[ctr].x = (unsigned short int)nonmax[i].x*FAST_SUBPIXEL;
	    m[ctr].y = (unsigned short int)nonmax[i].y;
	    m[ctr].disparity = (unsigned short int)(interocular_disparity[i]-1);
	    n = ((m[i].y * img_width) + nonmax[i].x) * 3;
	    m[ctr].r = img[n + 2];
	    m[ctr].g = img[n + 1];
	    m[ctr].b = img[n];
	    ctr++;
	  }
	}
	if (ctr < FAST_MAX_CORNERS_PREVIOUS) {
	  m[ctr].x = 9999;
	  m[ctr].y = 9999;
	}

	fwrite(m, sizeof(MatchDataColour), FAST_MAX_CORNERS_PREVIOUS, file);
	delete[] m;
      }

    }

    printf("%d stereo corner matches saved to %s\n", max, filename.c_str());

    fclose(file);
  }
}

/* saves feature descriptors to file for use by other programs */
int fast::save_descriptors(
			   std::string filename, /* filename to save as */
			   unsigned char *img,
			   int img_width,
			   int img_height)
{
  int max = 0;
  int i,ctr;

  for (i = 0; i < num_nonmax; i++) {
    if (interocular_disparity[i] > 0) max++;
  }

  if (max > 40) {
    FILE *file = fopen(filename.c_str(), "wb");
    if (file != NULL) {
      struct MatchData {
	unsigned short int probability;
	unsigned short int x;
	unsigned short int y;
	unsigned short int disparity;
	unsigned char r, g, b;
	unsigned char direction;
	unsigned int descriptor0;
	unsigned int descriptor1;
	unsigned int descriptor2;
	unsigned int descriptor3;
      };

      unsigned int* descriptor = new unsigned int[4];
      unsigned char* descriptor_colour = new unsigned char[3];
      unsigned char* descriptor_direction = new unsigned char[1];
      MatchData *m = new MatchData[FAST_MAX_CORNERS_PREVIOUS];
      ctr = 0;
      for (i = 0; i < num_nonmax; i++) {
	if (interocular_disparity[i] > 0) {
	  m[ctr].probability = (unsigned short int)1000;
	  m[ctr].x = (unsigned short int)nonmax[i].x*FAST_SUBPIXEL;
	  m[ctr].y = (unsigned short int)nonmax[i].y;
	  m[ctr].disparity = (unsigned short int)(interocular_disparity[i]-1);

	  int radius = (int)interocular_disparity[i] - 1;
	  radius /= FAST_SUBPIXEL*6;
	  if (radius < 10) radius = 10;
	  compute_descriptor(
			     img, img_width, img_height,
			     nonmax[i].x, nonmax[i].y,
			     radius,
			     0, descriptor,
			     descriptor_colour,
			     descriptor_direction);
	  m[ctr].descriptor0 = descriptor[0];
	  m[ctr].descriptor1 = descriptor[1];
	  m[ctr].descriptor2 = descriptor[2];
	  m[ctr].descriptor3 = descriptor[3];
	  m[ctr].r = descriptor_colour[0];
	  m[ctr].g = descriptor_colour[1];
	  m[ctr].b = descriptor_colour[2];
	  m[ctr].direction = descriptor_direction[0];

	  ctr++;
	}
      }
      if (ctr < FAST_MAX_CORNERS_PREVIOUS) {
	m[ctr].x = 9999;
	m[ctr].y = 9999;
      }

      fwrite(m, sizeof(MatchData), FAST_MAX_CORNERS_PREVIOUS, file);
      delete[] m;
      delete[] descriptor;
      delete[] descriptor_colour;
      delete[] descriptor_direction;

      printf("%d feature descriptors saved to %s\n", max, filename.c_str());
      fclose(file);
    }
  }

  return(max);
}

/*!
 * \brief returns true if the given file exists
 * \param filename name of the file
 */
bool fast::FileExists(
		      std::string filename)
{
  std::ifstream inf;

  bool flag = false;
  inf.open(filename.c_str());
  if (inf.good()) flag = true;
  inf.close();
  return(flag);
}

void fast::create_descriptor_lookup(
				    int radius,
				    int width,
				    int* lookup)
{
  int r,w,n=0;

  printf("int descriptor_lookup[] = {\n");
  for (r = 1; r <= radius; r++) {
    for (w = 0; w < width; w++, n += 2) {
      lookup[n] = (int)((r+2) * 100 * sin(w * 3.1415927*2 / width) / (radius+2));
      lookup[n+1] = (int)((r+2) * 100 * cos(w * 3.1415927*2 / width) / (radius+2));
      printf("%d,%d,", lookup[n], lookup[n+1]);
    }
    printf("\n");
  }
  printf("};\n");
}

/* loads stereo matches from file */
void fast::load_matches(
			std::string filename, /* filename to load from */
			bool colour) { /* whether to additionally save colour of each match */

  if (FileExists(filename)) {

    FILE *file = fopen(filename.c_str(), "rb");
    if (file != NULL) {

      int i;

      /* create an array to store interocular matches */
      if (interocular_disparity == NULL) {
	interocular_disparity = new unsigned short[FAST_MAX_CORNERS_PREVIOUS];
      }

      if (!colour) {

	struct MatchData {
	  unsigned short int probability;
	  unsigned short int x;
	  unsigned short int y;
	  unsigned short int disparity;
	};

	MatchData *m = new MatchData[FAST_MAX_CORNERS_PREVIOUS];
	if (fread(m, sizeof(MatchData), FAST_MAX_CORNERS_PREVIOUS, file)>0) {
	  num_nonmax = 0;
	  for (i = 0; i < FAST_MAX_CORNERS_PREVIOUS; i++, num_nonmax++) {
	    if ((m[i].x ==  9999) && (m[i].y == 9999)) {
	      break;
	    }
	    nonmax[i].x = (int)m[i].x;
	    nonmax[i].y = (int)m[i].y;
	    interocular_disparity[i] = m[i].disparity;
	  }
	}
	delete[] m;				
      } else {
	struct MatchDataColour {
	  unsigned short int probability;
	  unsigned short int x;
	  unsigned short int y;
	  unsigned short int disparity;
	  unsigned char r, g, b;
	  unsigned char pack;
	};

	MatchDataColour *m = new MatchDataColour[FAST_MAX_CORNERS_PREVIOUS];
	if (fread(m, sizeof(MatchDataColour), FAST_MAX_CORNERS_PREVIOUS, file)>0) {
	  num_nonmax = 0;
	  for (i = 0; i < FAST_MAX_CORNERS_PREVIOUS; i++, num_nonmax++) {
	    if ((m[i].x ==  9999) && (m[i].y == 9999)) {
	      break;
	    }
	    nonmax[i].x = (int)m[i].x;
	    nonmax[i].y = (int)m[i].y;
	    interocular_disparity[i] = m[i].disparity;
	  }
	}
	delete[] m;
      }

      printf("%d stereo corner matches loaded from %s\n", num_nonmax, filename.c_str());

      fclose(file);
    }
  }
  else {
    printf("File %s not found\n", filename.c_str());
  }
}


void fast::match_interocular(
			     int img_width __attribute__((unused)),
			     int img_height __attribute__((unused)),
			     int no_of_stereo_matches,
			     unsigned int* stereo_matches)
{
  /* create an array to store interocular matches */
  if (interocular_disparity == NULL) {
    interocular_disparity = new unsigned short[FAST_MAX_CORNERS_PREVIOUS];
  }

  memset((void*)interocular_disparity, '\0', num_nonmax*sizeof(unsigned short));

  int i, j, x0,y0,x1,y1,dx,dy,idx;
  int tot_disp, hits;
  const int radius = 8;
  for (j = 0; j < num_nonmax; j++) {
    x0 = nonmax[j].x;
    y0 = nonmax[j].y;
    tot_disp = 0;
    hits = 0;
    for (i = 0; i < no_of_stereo_matches; i++) {
      if (stereo_matches[i*5] > 0) {
	x1 = (int)stereo_matches[i*5 + 1]/FAST_SUBPIXEL;
	y1 = (int)stereo_matches[i*5 + 2];

	dx = x0 - x1;
	if ((dx > -radius) && (dx < radius)) {
	  dy = y0 - y1;
	  if ((dy > -radius) && (dy < radius)) {
	    tot_disp += (int)stereo_matches[i*5 + 3];
	    hits++;
	  }
	}
      }
    }
    if (hits > 0) {

      idx = (int)temporal_matches[0][j] - 1;
      if (idx > -1) {
	if (previous_interocular_disparity[0][idx] > 0) {
	  interocular_disparity[j] = (unsigned short)(1 + (( (tot_disp*2/hits) + (previous_interocular_disparity[0][idx]*8))/10) );
	}
	else {
	  interocular_disparity[j] = (unsigned short)(1 + (tot_disp/hits));
	}
      }
      else {
	interocular_disparity[j] = (unsigned short)(1 + (tot_disp/hits));
      }
    }
  }
}

void fast::estimate_pan_tilt(
			     int img_width,
			     int img_height __attribute__((unused)),
			     int fov_degrees,
			     int angle_multiplier)
{
  int i, disp, idx,dx,dy;
  int pan=0, tilt=0;
  int pan_hits=0, tilt_hits=0;
  for (i = 0; i < num_nonmax; i++) {
    idx = (int)temporal_matches[0][i] - 1;
    if (idx > -1) {
      disp = (int)interocular_disparity[i] - 1;
      if (disp > -1) {
	disp = disp * fov_degrees / img_width;
	dx = (nonmax[i].x - previous_corners[0][idx].x) * fov_degrees * angle_multiplier / img_width;
	dy = (nonmax[i].y - previous_corners[0][idx].y) * fov_degrees * angle_multiplier / img_width;
	if (disp == 0) disp = 1;

	if (dx != 0) {
	  pan += dx/disp;
	  pan_hits++;
	}
	if (dy != 0) {
	  tilt += dy/disp;
	  tilt_hits++;
	}
      }
    }
  }
  if (pan_hits > 0) {
    pan /= pan_hits;
  }
  if (tilt_hits > 0) {
    tilt /= tilt_hits;
  }
  printf("pan = %f   tilt = %f\n", (pan / (float)angle_multiplier) * 180 / 3.1415927f, (tilt / (float)angle_multiplier) * 180 / 3.1415927f);
}

int fast::get_no_of_corners()
{
  return(num_nonmax);
}

int fast::get_previous_no_of_corners()
{
  return(previous_no_of_corners[0]);
}

int* fast::get_corners()
{
  return((int*)nonmax);
}

int* fast::get_previous_corners()
{
  return((int*)previous_corners[0]);
}

unsigned char* fast::get_temporal_matches()
{
  return(temporal_matches[0]);
}

int fast::get_no_of_disparities()
{
  int i, max = 0;
  for (i = 0; i < num_nonmax; i++) {
    if (interocular_disparity[i] > 0) max++;
  }
  return(max);
}
