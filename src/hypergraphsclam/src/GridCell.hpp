#ifndef HYPERGRAPH_SLAM_GRID_CELL_HPP
#define HYPERGRAPH_SLAM_GRID_CELL_HPP

namespace hyper {

template<typename T>
class GridCell {

    public:

        // the occupancy probability
        T occupancy;

        // hits
        unsigned nocc;

        // empty
        unsigned nfree;

        // basic constructor
        GridCell() : occupancy(0.5), nocc(0), nfree(0)  {}

        // assignement operator overloading
        void operator=(const GridCell<T> &c) {

            // copy the values
            occupancy = c.occupancy;
            nocc = c.nocc;
            nfree = c.nfree;

        }

};

// syntactic sugar
template<typename T>
using GridCellPtr = GridCell<T>*;

// syntactic sugar
template<typename T>
using GridCellRef = GridCell<T>&;

// syntactic sugar
template<typename T>
using GridCellMap2D = GridCell<T>**;

// syntactic sugar
template<typename T>
using GridCellMap3D = GridCell<T>***;

class GridCellIndex2D {

    public:

        // the coordinates
        unsigned x,  y;

        // default constructor
        GridCellIndex2D() : x(0), y(0) {}

        // explicit constructor
        GridCellIndex2D(unsigned x_, unsigned y_) : x(x_), y(y_) {}

};

class GridCellIndex3D {

    public:

        // the coordinates
        unsigned x,  y,  z;

        // default constructor
        GridCellIndex3D() : x(0), y(0), z(0) {}

        // explicit constructor
        GridCellIndex3D(unsigned x_, unsigned y_, unsigned z_) : x(x_), y(y_), z(z_) {}

        // assignement operator overloading
        void operator=(const GridCellIndex3D &index) {

            // copy the coordinates
            x = index.x;
            y = index.y;
            z = index.z;

        }

};

// syntactic sugar
typedef GridCellIndex3D* GridCellIndex3DPtr;
typedef GridCellIndex3D& GridCellIndex3DRef;

}

#endif