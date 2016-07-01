#ifndef KD_TREE_NODE_TEMPLATE_HPP
#define KD_TREE_NODE_TEMPLATE_HPP

#include "Point/Point2D.hpp"

class KDTreeNode2D {

public:

    // public members

    // the point stored
    Point2D point;

    // the left pointer
    KDTreeNode2D *left;

    // the left pointer
    KDTreeNode2D *right;

    // basic constructor
    KDTreeNode2D(const Point2D& p);

    // basic destructor
    ~KDTreeNode2D();

};

#endif
