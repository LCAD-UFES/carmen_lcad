package Carmen;
/**
 * Declaration of Carmen event handler for odometry.
 *
 * A Carmen module will implement this.
 * @author RSS-CARMEN development team 
 * @version 1.0, Mar 1, 2005
 */

public interface HeartbeatHandler {
  /**
   *  event handler for odometry messages
   */
  public void handle (HeartbeatMessage message);
}
