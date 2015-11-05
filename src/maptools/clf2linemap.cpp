#include <carmen/carmen.h>
#include "clfreader.h"

#ifdef __cplusplus
extern "C" {
#endif
#include <carmen/linemapping.h>
#ifdef __cplusplus
}
#endif

using namespace std;
using namespace CLFReader;

int main(int argc, char** argv) {

  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <carmen-log-file>" << endl;
    exit(0);
  }

  carmen_ipc_initialize(argc, argv);
  carmen_linemapping_init(argc, argv);

  cerr << "reading clf " << argv[argc-1] << " ...";
  ifstream log(argv[argc-1]);
  if (!log.is_open()) {
    cerr << "unable to open file!" << endl;
    exit(0);
  }
  CLFRecordList rec;
  rec.read(log);
  cerr << "done!" << endl;
  
  cerr << "collecting and dumping scans ...";

  int num_entries=(int) rec.size();
  int num_lasers = 0;
  for (int i=0; i<num_entries; i++) {
    if (rec[i]->id() == "FLASER") 
      num_lasers++;
  }

  int laser_cnt=0;
  carmen_robot_laser_message* lasers = new carmen_robot_laser_message[num_lasers];
  int i=0;
  while (i<num_entries) {

    if (rec[i]->id() == "FLASER") {
      const CLFLaserRecord* laser =  
	dynamic_cast<const CLFLaserRecord*> (rec[i]);
      assert(laser);

      carmen_robot_laser_message ltmp;
      ltmp.num_readings = (int) laser->readings.size();
      ltmp.range = new float[ltmp.num_readings];
      for (int j=0; j<ltmp.num_readings; j++) 
	ltmp.range[j] = laser->readings[j];
      ltmp.laser_pose.x      = laser->laserPose.x;
      ltmp.laser_pose.y      = laser->laserPose.y;
      ltmp.laser_pose.theta  = laser->laserPose.theta;
      ltmp.robot_pose.x      = laser->odomPose.x;
      ltmp.robot_pose.y      = laser->odomPose.y;      
      ltmp.robot_pose.theta  = laser->odomPose.theta;

      lasers[laser_cnt++] = ltmp;

      laser->write(cout);

    }

    i++;
  }
  cerr << "done!" << endl;


  cerr << "building linemap...";
  carmen_linemapping_segment_set_t linemap = 
    carmen_linemapping_get_segments_from_scans(lasers, laser_cnt);
  cerr << "done" << endl;

  cerr << "writing lines to log output...";
  
  for (int j=0; j<linemap.num_segs; j++) {
    cout << "MARKER [color=red; line=" << linemap.segs[j].p1.x << ", " << linemap.segs[j].p1.y << ", "
	 << linemap.segs[j].p2.x << ", " << linemap.segs[j].p2.y << "]" << endl;
    cout << "MARKER [color=blue; circle=" << linemap.segs[j].p1.x << ", " << linemap.segs[j].p1.y << ", "
	 << 0.1 << "]" << endl;
    cout << "MARKER [color=blue; circle=" << linemap.segs[j].p2.x << ", " << linemap.segs[j].p2.y << ", "
	 << 0.1 << "]" << endl;
  }

  cerr <<  "done!" << endl;


  return 0;
}
