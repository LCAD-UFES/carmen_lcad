#include "clfreader.h"

namespace CLFReader{

CLFRecord::CLFRecord(){
}

CLFRecord::~CLFRecord(){
}

/////////////////////////////////////////////////////////////////

CLFCommentRecord::CLFCommentRecord(){
}

void CLFCommentRecord::read(istream& is){
  char buf[CLF_MAX_LINE_LENGHT];
  memset(buf,0, CLF_MAX_LINE_LENGHT*sizeof(char));
  is.getline(buf, CLF_MAX_LINE_LENGHT);
  text=string(buf);
}

void CLFCommentRecord::write(ostream& os) const {
  os << "## " << text << endl;
}

/////////////////////////////////////////////////////////////////

CLFTruePoseRecord::CLFTruePoseRecord() {
};

void CLFTruePoseRecord::read(istream& is){
  is >> truePose.x >> truePose.y >> truePose.theta;
  is >> odomPose.x >> odomPose.y >> odomPose.theta;
  time = 0;
  if (is)
    is >> time;
}
void CLFTruePoseRecord::write(ostream& os) const {
  os << "TRUEPOS ";
  os << truePose.x << " " << truePose.y << " " << truePose.theta << " ";
  os << odomPose.x << " " << odomPose.y << " " << odomPose.theta << " " << time << " clfwrite " << time << endl;
}

/////////////////////////////////////////////////////////////////

CLFOdometryRecord:: CLFOdometryRecord() {
}

void CLFOdometryRecord::read(istream& is){
  is >> pose.x >> pose.y >> pose.theta;
  is >> tv >> rv >> acceleration;
  time = 0;
  if (is)
    is >> time;
}

void CLFOdometryRecord::write(ostream& os)const {
  os << "ODOM ";
  os << pose.x << " " << pose.y << " " << pose.theta << " ";
  os << tv << " " << rv << " " << acceleration << " " << time << " clfwrite " << time << endl;
}


/////////////////////////////////////////////////////////////////

CLFLaserRecord::CLFLaserRecord(int laserid) {
  laserID = laserid;
}

void CLFLaserRecord::read(istream& is){
  is >> dim;
  for (unsigned int i=0; i< dim; i++){
    double r;
    is >> r;
    readings.push_back(r);
  }
  is >> laserPose.x;
  is >> laserPose.y;
  is >> laserPose.theta;
  is >> odomPose.x;
  is >> odomPose.y;
  is >> odomPose.theta;
  time = 0;
  if (is)
    is >> time;
}

void CLFLaserRecord::write(ostream& os)const {
  if (laserID == 1) {
    os << "FLASER " <<  dim;
  }
  else  if (laserID == 2) {
    os << "RLASER " <<  dim;
  }
  else  if (laserID == 3) {
    os << "LASER3 " <<  dim;
  }
  else  if (laserID == 4) {
    os << "LASER4 " <<  dim;
  }
  else  {
    os << "FLASER " <<  dim;
  }
  
  for (unsigned int i=0; i< dim; i++){
    os <<" "<< readings[i] ;
  }
  os <<" "<< laserPose.x;
  os <<" "<< laserPose.y;
  os <<" "<< laserPose.theta;
  os <<" "<< odomPose.x;
  os <<" "<< odomPose.y;
  os <<" "<< odomPose.theta;
  os <<" "<< time <<  " clfwrite " << time << endl;
};

string CLFLaserRecord::id() const { 
  if (laserID == 1) {
    return "FLASER";
  }
  else  if (laserID == 2) {
    return "RLASER";
  }
  else  if (laserID == 3) {
    return "LASER3";
  }
  else  if (laserID == 4) {
    return "LASER4";
  }
  else  {
    return "FLASER";
  }
};

/////////////////////////////////////////////////////////////////

CLFRecordList::CLFRecordList(bool showDebug) {
  this->showDebug = showDebug;
}

CLFRecord* CLFRecordList::createRecord(string recordType) {
  CLFRecord* rec = NULL;

  if (recordType=="FLASER"){
    rec=new CLFLaserRecord(1);
    if (showDebug)
      cerr << "l" << flush;
  }
  else if (recordType=="RLASER"){
    rec=new CLFLaserRecord(2);
    if (showDebug)
      cerr << "r" << flush;
  }
  else if (recordType=="LASER3"){
    rec=new CLFLaserRecord(3);
    if (showDebug)
      cerr << "3" << flush;
  }
  else if (recordType=="LASER4"){
    rec=new CLFLaserRecord(4);
    if (showDebug)
      cerr << "4" << flush;
  }
  else if (recordType=="ODOM"){
    rec=new CLFOdometryRecord;
    if (showDebug)
      cerr << "o" << flush;
  }
  else if (recordType=="TRUEPOS"){
    rec=new CLFTruePoseRecord;
    if (showDebug)
      cerr << "t" << flush;
  }
  else if (recordType=="COMMENT"){
    rec=new CLFCommentRecord;
    if (showDebug)
      cerr << "c" << flush;
  }

  return rec;
}

istream& CLFRecordList::read(istream& is){

  char buf[CLF_MAX_LINE_LENGHT];
  while(is){
    buf[0]='\0';
    is.getline(buf, CLF_MAX_LINE_LENGHT);
    
    istringstream lineStream(buf);
    
    string recordType;
    lineStream >> recordType;

    CLFRecord* rec = createRecord(recordType);
    if (rec){
      rec->read(lineStream);
      push_back(rec);
    }
  }
  return is;
}
 
//////////////////////////////////////////////////////////

}

