/*
// Author: Josias Alexandre Oliveira
// josiasalexandre@gmail.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef KD_TREE_TEMPLATE_HPP
#define KD_TREE_TEMPLATE_HPP

#include <iostream>
#include <vector>
#include <limits>

#include "KDTreeNode2D.hpp"

class KDTree2D {

private:

    static const unsigned int D = 2;

    // private members
    // the root pointer
    KDTreeNode2D* root;

    // the min distance
    double minDistance;

    // private methods
    // quick select approach
    unsigned int quickSelect(std::vector<Point2D> &input, unsigned int start, unsigned int end, unsigned int nth, unsigned int axis);

    // build the kdtree from a given array
    KDTreeNode2D* buildKDTree2D(std::vector<Point2D>& input, unsigned int start, unsigned int end, unsigned int axis);

    // compare two points
    bool diffPoints(Point2D& a, Point2D&b);

    // destroy a given kdtree
    void removeSubTree(KDTreeNode2D* n);

    // distance between two nodes
    inline double distance(KDTreeNode2D* a, KDTreeNode2D* b);

    // distance between two points
    inline double distance(const Point2D& a, const Point2D& b);

    // find the nearest point inside the kdtree
    KDTreeNode2D* getNearest(KDTreeNode2D* n, const Point2D& p, unsigned int axis, KDTreeNode2D* bestNode, double& bestDist);

public:

    // empty constructor
    KDTree2D();

    // basic constructor
    KDTree2D(std::vector<Point2D>& input);

    // basic destructor
    ~KDTree2D();

    // clear the kdtree
    void clear();

    // rebuild the entire kdtree
    void rebuild(std::vector<Point2D> &input);

    // add a point to the kdtree
    void add(Point2D& p);

    // add a list of pointers
    void addPoints(std::vector<Point2D>& input);

    // find the nearest neighbour
    Point2D nearest(const Point2D &p);

    // remove a point from the kdtree
    void remove(const Point2D& p);

    // find the k nearest points
    std::vector<Point2D> nearests(const Point2D& p, unsigned int k);

    // balance the KDTree
    void balance();

    //verify if KDTree was build
    bool empty();


};

#endif


