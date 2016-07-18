#include "KDTreeNode2D.hpp"

#include <stdlib.h>

// basic construtor
KDTreeNode2D::KDTreeNode2D(const Point2D& p) : point(p), left(NULL), right(NULL) {}

// basic destructor
KDTreeNode2D::~KDTreeNode2D() {

    // update the pointers
    left = right = NULL;

}
