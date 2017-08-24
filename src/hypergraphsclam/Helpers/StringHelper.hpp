#ifndef HYPERGRAPHSLAM_STRING_HELPER_HPP
#define HYPERGRAPHSLAM_STRING_HELPER_HPP

#include <fstream>
#include <sstream>

namespace hyper {

class StringHelper {

    public:

        // read a line from an input file
        static int ReadLine(std::istream &is, std::stringstream& line);

        // get the stringstream size
        static unsigned GetStringStreamSize(std::stringstream &ss);

};

}

#endif
