#include "ReadTemp.H"

ReadTemp::ReadTemp()
{
  itsTempFile = "/proc/acpi/thermal_zone/THRM";
  itsTempCol  = 2;
}

ReadTemp::~ReadTemp()
{}

const ushort ReadTemp::getTemp() const
{
  const char *filename = itsTempFile.c_str();
  std::ifstream infile(filename,std::ios::in);
  char count = 1;
  std::string in;
  while(infile >> in)
  {
    if(count == itsTempCol)
    {
      break;
    }
    count++;
  }
  return static_cast<ushort>(atoi(in.c_str()));
};
