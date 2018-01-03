#include <StringHelper.hpp>

using namespace hyper;

// read a line from an input file
// read the next line
// copy the resulting line to the output line string
// return the  size of the output string
int StringHelper::ReadLine(std::istream &is, std::stringstream& line) {

    // have we reached the end of the input file?
    if (is.eof())
        return -1;

    // clear the output string
    line.str(std::string());
    line.clear();

    // read the current line
    is.get(*line.rdbuf());

    // the fail flag is set on empty lines
    if (is.fail()) {

        // clear the istream flags
        is.clear();

    }

    // just a helper
    char c = ' ';

    // discards '\n' not readed by get()
    while('\n' != c && is.good() && !is.eof()) {

        is.get(c);

    }

    return static_cast<int>(line.str().size());

}


// get the stringstream size
unsigned StringHelper::GetStringStreamSize(std::stringstream &ss) {

    // the original position
    std::streampos original = ss.tellg();

    // go to the last position
    ss.seekg(0, ss.end);

    // get the size
    unsigned size = ss.tellg();

    // reset it
    ss.seekg(original, ss.beg);

    return size;

}
