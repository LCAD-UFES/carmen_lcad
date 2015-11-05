#include <istream>
#include <ostream>
#include <fstream>
//#include <sstream>
//#include <stdlib.h>
#include <string>
//#include <cstdlib>

using namespace std;

class ReadTemp
{
 public:
  ReadTemp();
  ~ReadTemp();
  const ushort getTemp() const;
 private:
  //! File to open to get temperatur
  std::string itsTempFile;
  //! offset into file for reading
  char        itsTempCol;
};



