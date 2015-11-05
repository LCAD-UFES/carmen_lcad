package Carmen;

import IPC.*;

/**  Carmen HeartbeatHandler's message
   */

public class HeartbeatMessage extends Message {
  public String module;
  public int pid;

  private static final String CARMEN_HEARTBEAT_NAME = "carmen_heartbeat";
  private static final String CARMEN_HEARTBEAT_FMT =
    "{string, int, double, string}";

  public static void subscribe(HeartbeatHandler handler)
  {
    subscribe(CARMEN_HEARTBEAT_NAME, CARMEN_HEARTBEAT_FMT, handler, 
	      HeartbeatMessage.class, "handle");
  }

  public void publish(String module)
  {
    HeartbeatMessage msg = new HeartbeatMessage();
    msg.module = module;
    msg.pid = (int)IPC.getPID();
    msg.publish(CARMEN_HEARTBEAT_NAME, CARMEN_HEARTBEAT_FMT, this);
  }

}

