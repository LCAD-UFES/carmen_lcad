#ifndef GRID_CELL_INDEX_HPP
#define GRID_CELL_INDEX_HPP

namespace smoother {

/*! A light-weight integer point with fields x,y */
class GridCellIndex {

    public:

        // public attributes
        unsigned int row, col;

        // basic constructors
        GridCellIndex() : row(0), col(0) {}
        GridCellIndex(unsigned int _r, unsigned int _c) : row(_r), col(_c) {}

};

typedef GridCellIndex* GridCellIndexPtr;
typedef GridCellIndex& GridCellIndexRef;

}

#endif
