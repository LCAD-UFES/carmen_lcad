 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef CLFREADER_H
#define CLFREADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <deque>
#include "point.h"

#define CLF_MAX_LINE_LENGHT (100000)

namespace CLFReader{
	
using namespace std;

/////////////////////////////////////////////////////////////////

class CLFRecord{
 public:
  CLFRecord();
  virtual ~CLFRecord();
  virtual void read(istream& is)=0;
  virtual void write(ostream& os)const =0 ;
  virtual string id() const{ return "NO-TYPE"; };

  unsigned int dim;
  double time;
};

/////////////////////////////////////////////////////////////////

class CLFCommentRecord: public CLFRecord {
 public:
  CLFCommentRecord();
  virtual void read(istream& is);
  virtual void write(ostream& os) const ;
  virtual string id() const{ return "COMMENT"; };
  string text;
};


/////////////////////////////////////////////////////////////////

class CLFTruePoseRecord: public CLFRecord{
 public:
  CLFTruePoseRecord();
  void read(istream& is);
  virtual void write(ostream& os)const ;
  virtual string id() const{ return "TRUEPOS"; };

  OrientedPoint truePose;
  OrientedPoint odomPose;
};

/////////////////////////////////////////////////////////////////

class CLFOdometryRecord: public CLFRecord{
 public:
  CLFOdometryRecord();
  virtual void read(istream& is);
  virtual void write(ostream& os)const ;
  virtual string id() const{ return "ODOM"; };

  OrientedPoint pose;
  double tv, rv, acceleration;
};

/////////////////////////////////////////////////////////////////

class CLFLaserRecord: public CLFRecord{
 public:
  CLFLaserRecord(int laserid);
  virtual void read(istream& is);
  virtual void write(ostream& os)const ;
  virtual string id() const;

  vector<double> readings;
  OrientedPoint laserPose;
  OrientedPoint odomPose;
  int laserID;
};

/////////////////////////////////////////////////////////////////

class CLFRecordList: public deque<CLFRecord*>{
 public:
  CLFRecordList(bool showDebug = false);
  virtual ~CLFRecordList() {};
  istream& read(istream& is);
  virtual CLFRecord* createRecord(string recordType);
  
  bool showDebug;
};

/////////////////////////////////////////////////////////////////

} //end namespace

#endif
