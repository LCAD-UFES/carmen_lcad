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

#include "KDTree2D.hpp"

// private methods
// quick select approach
unsigned int KDTree2D::quickSelect(std::vector<Point2D> &input, unsigned int start, unsigned int end, unsigned int nth, unsigned int axis) {

    // special case
    if (end == start + 1) {

        return start;

    }

    // the pivot element
    double pivot;

    // the auxiliar index
    unsigned int store;

    // find the nth element
    while(true) {

        // get the pivot value
        pivot = input[nth].position[axis];

        // swap the nth and end - 1 elements
        std::swap(input[nth], input[end - 1]);

        // find the median
        for (unsigned int i = store = start; i < end; i++) {

            // compare to the pivot element
            if (input[i].position[axis] < pivot) {

                if (i != store) {

                    // swap the elements
                    std::swap(input[i], input[store]);

                }

                // update the store counter
                store += 1;

            }

        }

        // swap the elements
        std::swap(input[store], input[end - 1]);

        // compare
        if (input[store].position[axis] == input[nth].position[axis]) {

            return nth;

        }

        if (store > nth) {

            end = store;

        } else {

            start = store;

        }

    }

}

// build the kdtree from a given array
KDTreeNode2D* KDTree2D::buildKDTree2D(std::vector<Point2D>& input, unsigned int start, unsigned int end, unsigned int axis) {

    if (start < end) {

        // get the median point
        unsigned int mid = quickSelect(input, start, end, (start + end)/2, axis);

        // build a new node
        KDTreeNode2D* n = new KDTreeNode2D(input[mid]);

        // update the axis
        axis += 1;
        if (D <= axis) {

            axis = 0;

        }

        // build the left subtree
        n->left = buildKDTree2D(input, start, mid, axis);

        // build the right subtree
        n->right = buildKDTree2D(input, mid + 1, end, axis);

        // return the valid node
        return n;

    }

    // invalid
    return NULL;

}

// compare two points
bool KDTree2D::diffPoints(Point2D& a, Point2D&b) {

    // compare
    for (unsigned int i = 0; i < D; i++) {

        if (a.position[i] != b.position[i]) {

            return true;

        }

    }

    return false;

}

// destroy a given kdtree
void KDTree2D::removeSubTree(KDTreeNode2D* n) {

    if (NULL != n->left) {

        // remove the left subtree
        removeSubTree(n->left);

    }

    if (NULL != n->right) {

        // remove the right subtree
        removeSubTree(n->right);

    }

    // finally, remove the current node
    delete n;

}

// distance between two nodes
double KDTree2D::distance(KDTreeNode2D* a, KDTreeNode2D* b) {

    // helpers
    double t, d = 0.0;

    for (unsigned int axis = 0; axis < D; axis++) {

        // get the difference
        t = a->point.position[axis] - b->point.position[axis];

        // power of 2
        d += t*t;

    }


    return d;

}

// distance between two points
double KDTree2D::distance(const Point2D& a, const Point2D& b) {

    // helpers
    double t, d = 0.0;

    for (unsigned int axis = 0; axis < D; axis++) {

        // get the difference
        t = a.position[axis] - b.position[axis];

        // power of 2
        d += t*t;

    }

    // return the result
    return d;

}

// find the nearest point inside the kdtree
KDTreeNode2D* KDTree2D::getNearest(KDTreeNode2D* n, const Point2D& p, unsigned int axis, KDTreeNode2D* bestNode, double& bestDist) {

    // auxiliar vars
    double dist, deltaAxis, deltaAxis2;

    // get the distance between the current node->point and the given point
    dist = distance(n->point, p);

    // verify the the match case
    if (minDistance > dist) {

        // the current node is the best one
        return n;

    }

    // get the axis diff
    deltaAxis = n->point.position[axis] - p.position[axis];

    // get the deltaAxis ^2
    deltaAxis2 = deltaAxis*deltaAxis;

    // is the current distance closer than the current bestDist?
    if (dist < bestDist || NULL == bestNode) {

        // update the new bestDist
        bestDist = dist;

        // the current node is the best one
        bestNode = n;

    }

    // update the axis
    axis += 1;
    if (D <= axis) {

        axis = 0;

    }

    // should we search the left or right subtree?
    if (0 < dist && NULL != n->left) {

        // left subtree
        bestNode = getNearest(n->left, p, axis, bestNode, bestDist);

    } else if (NULL != n->right) {

        // right subtree
        bestNode = getNearest(n->right, p, axis, bestNode, bestDist);

    }

    // are we done?
    if (deltaAxis2 >= bestDist) {

        return bestNode;

    }

    // search the other side
    if (0 < dist && NULL != n->right) {

        // right subtree
        bestNode = getNearest(n->right, p, axis, bestNode, bestDist);

    } else if (NULL != n->left) {

        // left subtree
        bestNode = getNearest(n->left, p, axis, bestNode, bestDist);

    }

    return bestNode;


}

// empty constructor
KDTree2D::KDTree2D() : root(NULL), minDistance(0.005) {}

// basic constructor
KDTree2D::KDTree2D(std::vector<Point2D>& input) : root(NULL), minDistance(0.0005){

    rebuild(input);

}

// basic destructor
KDTree2D::~KDTree2D() {

    clear();

}

// clear the kdtree
void KDTree2D::clear() {

    if (NULL != root) {

        // remove the entire tree
        removeSubTree(root);

    }

}

// rebuild the entire kdtree
void KDTree2D::rebuild(std::vector<Point2D> &input) {

    clear();

    // build a KDTree from the a given array
    root = buildKDTree2D(input, 0, input.size(), 0);

}

// add a point to the kdtree
void KDTree2D::add(Point2D& p) {

    if (NULL != root) {

        // a helper pointer
        KDTreeNode2D *tmp = root;

        // the axis
        unsigned int axis = 0;

        while(NULL != tmp) {

            if (tmp->point.position[axis] > p.position[axis]) {

                // verify if there is a left subtree
                if (NULL != tmp->left) {

                    // update the tmp pointer
                    tmp = tmp->left;

                } else {

                    // add the new point to the left
                    tmp->left = new KDTreeNode2D(p);

                    // exit the while loop
                    return;

                }

            } else if (tmp->point.position[axis] < p.position[axis] || diffPoints(tmp->point, p)){

                // verify if there is a right subtree
                if (NULL != tmp->right) {

                    // update the tmp pointer
                    tmp = tmp->right;

                } else {

                    // add the new point to the right
                    tmp->right = new KDTreeNode2D(p);

                    // exit the while loop
                    return;

                }

            } else {

                // same point
                return;

            }

        }

    } else {

        // append the new node
        root = new KDTreeNode2D(p);

    }

}

// add a list of pointers
void KDTree2D::addPoints(std::vector<Point2D>& input) {

    // is it an empty kdtree?
    if (NULL == root) {

        // build a kdtree from the given array
        root = buildKDTree2D(input, 0, input.size(), 0);

        return;

    }

    // get the vector size
    unsigned int v_size = input.size();

    // append each element
    for (unsigned int i = 0; i < v_size; i++) {

        // insert the element
        add(input[i]);

    }


}

// find the nearest neighbour
Point2D KDTree2D::nearest(const Point2D &p) {

    // is it a valid kdtree?
    if (NULL != root) {

        // set the best distance
        double bestDist;

        // get the best node
        KDTreeNode2D* bestNode = getNearest(root, p, 0, NULL, bestDist);

        // return the point
        return bestNode->point;

    } else {

        return Point2D();
    }

}

// remove a point from the kdtree
void KDTree2D::remove(const Point2D& p) {

    /* TODO */

}

// find the k nearest points
std::vector<Point2D> KDTree2D::nearests(const Point2D& p, unsigned int k) {

    /* TODO */
    return std::vector<Point2D>();

}

// balance the KDTree
void balance() {

    /* TODO */
    return;

}

bool KDTree2D::empty(){

	if (NULL == root)
		return true;
	else
		return false;
}
