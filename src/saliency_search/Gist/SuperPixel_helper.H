// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Chin-Kai Chang <chinkai@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/SuperPixel_helper.H $
// $Id: SuperPixel_helper.H 12962 2010-03-06 02:13:53Z irock $
//
//////////////////////////////////////////////////////////////////////////
//
// Implementation of the segmentation algorithm described in:
//
// Efficient Graph-Based Image Segmentation
// Pedro F. Felzenszwalb and Daniel P. Huttenlocher
// International Journal of Computer Vision, 59(2) September 2004.
// Copyright (C) 2006 Pedro Felzenszwalb

#ifndef SUPERPIXEL_HELPER
#define SUPERPIXEL_HELPER
#include <vector>
#include <cmath>
#include <algorithm>

#define WIDTH 4.0//for filter
#define THRESHOLD(size, c) (c/size)//for segment_graph

//////////////////////////////////////////////////////////////////////////////
// disjoint-set forests using union-by-rank and path compression (sort of).

typedef struct {
  int rank;
  int p;
  int size;
} uni_elt;

class universe {
public:
  universe(int elements);
  ~universe();
  int find(int x);  
  void join(int x, int y);
  int size(int x) const { return elts[x].size; }
  int num_sets() const { return num; }

private:
  uni_elt *elts;
  int num;
};

inline universe::universe(int elements) {
  elts = new uni_elt[elements];
  num = elements;
  for (int i = 0; i < elements; i++) {
    elts[i].rank = 0;
    elts[i].size = 1;
    elts[i].p = i;
  }
}

inline 
universe::~universe() {
  delete [] elts;
}

inline 
int universe::find(int x) {
  int y = x;
  while (y != elts[y].p)
    y = elts[y].p;
  elts[x].p = y;
  return y;
}

inline 
void universe::join(int x, int y) {
  if (elts[x].rank > elts[y].rank) {
    elts[y].p = x;
    elts[x].size += elts[y].size;
  } else {
    elts[x].p = y;
    elts[y].size += elts[x].size;
    if (elts[x].rank == elts[y].rank)
      elts[y].rank++;
  }
  num--;
}
//end disjoint set
//////////////////////////////////////////////////////////////////////////////
//start filter code
/* convolve src with mask.  dst is flipped! */
inline void convolve_even(Image<float> src, Image<float> &dst,
                   std::vector<float> &mask)
{
  int width = src.getWidth();
  int height = src.getHeight();
  int len = mask.size();

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      float sum = mask[0] * src.getVal(x, y);
      for (int i = 1; i < len; i++) {
        sum += mask[i] *
          (src.getVal(std::max(x-i,0), y) +
           src.getVal(std::min(x+i, width-1), y));
      }
      dst.setVal(y, x, sum);
    }
  }
}

// /* convolve src with mask.  dst is flipped! */
// static void convolve_odd(image<float> *src, image<float> *dst,
//                          std::vector<float> &mask) {
//   int width = src->width();
//   int height = src->height();
//   int len = mask.size();

//   for (int y = 0; y < height; y++) {
//     for (int x = 0; x < width; x++) {
//       float sum = mask[0] * imRef(src, x, y);
//       for (int i = 1; i < len; i++) {
//         sum += mask[i] *
//           (imRef(src, std::max(x-i,0), y) -
//            imRef(src, std::min(x+i, width-1), y));
//       }
//       imRef(dst, y, x) = sum;
//     }
//   }
// }



/* normalize mask so it integrates to one */
static void normalize(std::vector<float> &mask) {
  int len = mask.size();
  float sum = 0;
  for (int i = 1; i < len; i++) {
    sum += fabs(mask[i]);
  }
  sum = 2*sum + fabs(mask[0]);
  for (int i = 0; i < len; i++) {
    mask[i] /= sum;
  }
}

/* make filters */
#define MAKE_FILTER(name, fun)                                \
static std::vector<float> make_ ## name (float sigma) {       \
  sigma = std::max(sigma, 0.01F);                              \
  int len = (int)ceil(sigma * WIDTH) + 1;                     \
  std::vector<float> mask(len);                               \
  for (int i = 0; i < len; i++) {                             \
    mask[i] = fun;                                            \
  }                                                           \
  return mask;                                                \
}

MAKE_FILTER(fgauss, exp(-0.5* pow(i/sigma, 2.0)));

/* convolve image with gaussian filter */
inline Image<float> smooth(Image<float> src, float sigma)
{
  std::vector<float> mask = make_fgauss(sigma);
  normalize(mask);

  Image<float> tmp(src.getHeight(), src.getWidth(), NO_INIT);
  Image<float> dst(src.getWidth(), src.getHeight(), NO_INIT);
  convolve_even(src, tmp, mask);
  convolve_even(tmp, dst, mask);

  return dst;
}

// /* convolve image with gaussian filter */
// Image<float> smooth(Image<byte> src, float sigma)
// {
//   Image<float> tmp = imageUCHARtoFLOAT(src);
//   Image<float> dst = smooth(tmp, sigma);
//   delete tmp;
//   return dst;
// }

// /* compute laplacian */
// static image<float> *laplacian(image<float> *src) {
//   int width = src->width();
//   int height = src->height();
//   image<float> *dst = new image<float>(width, height);

//   for (int y = 1; y < height-1; y++) {
//     for (int x = 1; x < width-1; x++) {
//       float d2x = imRef(src, x-1, y) + imRef(src, x+1, y) -
//         2*imRef(src, x, y);
//       float d2y = imRef(src, x, y-1) + imRef(src, x, y+1) -
//         2*imRef(src, x, y);
//       imRef(dst, x, y) = d2x + d2y;
//     }
//   }
//   return dst;
// }

//end filter code
//////////////////////////////////////////////////////////////////////////////
//start segment graph code
typedef struct {
  float w;
  int a, b;
} edge;

inline bool operator<(const edge &a, const edge &b) {
  return a.w < b.w;
}

/*
 * Segment a graph
 *
 * Returns a disjoint-set forest representing the segmentation.
 *
 * num_vertices: number of vertices in graph.
 * num_edges: number of edges in graph
 * edges: array of edges.
 * c: constant for treshold function.
 */

inline universe *segment_graph(int num_vertices, int num_edges, edge *edges, 
                        float c) { 
  // sort edges by weight
  std::sort(edges, edges + num_edges);

  // make a disjoint-set forest
  universe *u = new universe(num_vertices);

  // init thresholds
  float *threshold = new float[num_vertices];
  for (int i = 0; i < num_vertices; i++)
    threshold[i] = THRESHOLD(1,c);

  // for each edge, in non-decreasing weight order...
  for (int i = 0; i < num_edges; i++) {
    edge *pedge = &edges[i];
    
    // components conected by this edge
    int a = u->find(pedge->a);
    int b = u->find(pedge->b);
    if (a != b) {
      if ((pedge->w <= threshold[a]) &&
          (pedge->w <= threshold[b])) {
        u->join(a, b);
        a = u->find(a);
        threshold[a] = pedge->w + THRESHOLD(u->size(a), c);
      }
    }
  }

  // free up
  delete threshold;
  return u;
}

//////////////////////////////////////////////////////////////////////////////
//end segment graph code

#endif 
