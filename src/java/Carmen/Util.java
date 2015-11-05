package Carmen;
/** Carmen class with utilities that provide unit conversion, basic trig, and host information retrieval
 */

public class Util {
  public static double getTime() {
    return (double)(System.currentTimeMillis() / 1000.0);
  }
  /** returns the length of hypotenuse of right angled triangle */
  public static double hypot(double x, double y) {
    return Math.sqrt(x*x+y*y);
  }

  /** ensures that theta is in the range -PI to PI */

  public static double normalizeTheta(double theta) {
    int multiplier;
    
    if (theta >= -Math.PI && theta < Math.PI)
      return theta;
    
    multiplier = (int)(theta / (2*Math.PI));
    theta = theta - multiplier*2*Math.PI;
    if (theta >= Math.PI)
      theta -= 2*Math.PI;
    if (theta < -Math.PI)
      theta += 2*Math.PI;
    
    return theta;
  }

  /** returns host name */
  public static String getHostName() {
    String hostname = null;

    try {
      java.net.InetAddress localMachine =
	java.net.InetAddress.getLocalHost();	
      hostname = java.net.InetAddress.getLocalHost().toString();
    }
    catch(java.net.UnknownHostException uhe) {
      System.err.println("Could not recover hostname.\n"+
			 "Is the network functioning properly? Is the"+
			 "nameserver working?");
      System.exit(-1);
    }

    return hostname;    
  }
}

