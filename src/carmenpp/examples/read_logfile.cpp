#include <carmen/cpp_carmen.h>

int main(int argc, char** argv) {

  if (argc != 2)
    carmen_die("SYNTAX: %s <carmen log file>\n", argv[0]);
  
  carmen_FILE *carmen_stdout = new carmen_FILE;
  carmen_stdout->compressed = 0;
  carmen_stdout->fp = stdout;
  
  LogFile log;
  log.load(argv[1]);
  
  int cnt_rawlaser=0;
  int cnt_robotlaser=0;
  int cnt_odom=0;
  int cnt_other=0;

  for (LogFile::Collection::const_iterator it = log.begin();
       it != log.end(); ++it) {    

    //   this would dump the messages to stdout
    //   (*it)->save(carmen_stdout, (*it)->getTimestamp());
   
    const char* msg_id = (*it)->getMessageID();
 
    if (!strcmp( msg_id, "RAWLASER"))
      cnt_rawlaser++;
    else if (!strcmp( msg_id, "ROBOTLASER"))
      cnt_robotlaser++;
    else if (!strcmp( msg_id, "ODOM"))
      cnt_odom++;
    else 
      cnt_other++;
  }
  
  carmen_warn("\nThis file contains:\n");
  carmen_warn(" # rawlaser messages   : %d\n", cnt_rawlaser);
  carmen_warn(" # robotlaser messages : %d\n", cnt_robotlaser);
  carmen_warn(" # odometry  messages  : %d\n", cnt_odom);
  carmen_warn(" # other messages      : %d\n", cnt_other);

  return 0;
}
