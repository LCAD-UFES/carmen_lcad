#ifndef HYPERGRAPHSLAM_CUSTOM_VERTEX_SE2_HPP
#define HYPERGRAPHSLAM_CUSTOM_VERTEX_SE2_HPP

#include <g2o/types/slam2d/vertex_se2.h>

namespace hyper
{
    class CustomVertexSE2 : public g2o::VertexSE2
    {
        private:
            unsigned gid;

        public:

            CustomVertexSE2(unsigned _gid);

            unsigned getGid() { return gid; };

    };
}

#endif