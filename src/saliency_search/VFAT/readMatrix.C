/*! @file VFAT/readMatrix.C [put description here] */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/readMatrix.C $
// $Id: readMatrix.C 6182 2006-01-31 18:41:41Z rjpeters $

#include "VFAT/readMatrix.H"

#include "Util/log.H"
#include "Util/readConfig.H"

#include <iostream>
#include <fstream>
#include <string>

readMatrix::readMatrix(const char* fileName)
{
  int row = 0;
  bool firstLine = false;
  bool comment = false;
  std::vector<std::vector<double> >::iterator rowIter;
  std::vector<double>::iterator columnIter;
  std::ifstream inFile(fileName,std::ios::in);
  std::string in;
  LINFO("LOAD MATRIX %s",fileName);
  while (inFile >> in)
  {
    if(!in.compare("#")) //comment code # found
    {
      if(!comment)
      {
        comment = true;
      }
      else              //end of comment
      {
        comment = false;
      }
    }
    if((!comment) && in.compare("#")) //real line found
    {
      // the first line contains the diminsions
      // resize the vector
      if(firstLine == false)
      {
        sizeX = (int)atof(in.c_str());
        inFile >> in;
        sizeY = (int)atof(in.c_str());
        firstLine = true;
        std::vector<double> tvec(sizeX,0.0F);
        vectorIn.resize(sizeY,tvec);
        rowIter = vectorIn.begin();
        columnIter = rowIter->begin();
      }
      else
      {
        *columnIter = atof(in.c_str());
        ++columnIter;
        // if end of column reached, return
        if(columnIter == rowIter->end())
        {
          row++;
          ++rowIter;
          if(row < sizeY)
            columnIter = rowIter->begin();
          //ASSERT(sizeY > row);
        }
      }
    }
  }
}

readMatrix::~readMatrix()
{}

std::vector<std::vector<double> >  readMatrix::returnMatrix()
{
  return vectorIn;
}

Image<float> readMatrix::returnMatrixAsImage()
{
  std::vector<std::vector<double> >::iterator rowIter;
  std::vector<double>::iterator columnIter;
  Image<float> intermed;
  intermed.resize(sizeY,sizeX);
  int x = 0;
  for(rowIter = vectorIn.begin(); rowIter != vectorIn.end(); ++rowIter, x++)
  {
    int y = 0;
    for(columnIter = rowIter->begin(); columnIter != rowIter->end();
        ++columnIter, y++)
    {
      float whatthefuck;
      whatthefuck = *columnIter;
      intermed.setVal(x,y,whatthefuck);
    }
  }
  outImage = intermed;
  return outImage;
}

void readMatrix::echoMatrix()
{
  std::vector<std::vector<double> >::iterator rowIter;
  std::vector<double>::iterator columnIter;
  for(rowIter = vectorIn.begin(); rowIter != vectorIn.end(); ++rowIter)
  {
    for(columnIter = rowIter->begin(); columnIter != rowIter->end();
        ++columnIter)
    {
      std::cerr << *columnIter << " ";
    }
    std::cerr << "\n";
  }
}

