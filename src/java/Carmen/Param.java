package Carmen;

import java.util.HashMap;
import IPC.IPC;

/**  Carmen class for handling robot parameters in file carmen.ini 
   */
public class Param {
  private static boolean started = false;
  private static HashMap handlerMap;

  private static final String CARMEN_PARAM_VARIABLE_CHANGE_NAME = 
    "carmen_param_variable_change";
  private static final String CARMEN_PARAM_VARIABLE_CHANGE_FMT =
    "{string, string, string, int, int, double, string}";

  private static final String CARMEN_PARAM_QUERY_NAME =
    "carmen_param_query_string";
  private static final String CARMEN_PARAM_QUERY_FMT = 
    "{string, string, double, string}";

  private static final String CARMEN_PARAM_RESPONSE_STRING_NAME =
    "carmen_param_respond_string";
  private static final String CARMEN_PARAM_RESPONSE_STRING_FMT =
    "{string, string, string, int, int, double, string}";

  private static final String CARMEN_PARAM_RESPONSE_INT_NAME = 
	"carmen_param_respond_int";
  private static final String CARMEN_PARAM_RESPONSE_INT_FMT = 
	"{string, string, int, int, int, double, string}";
  
  private static final String CARMEN_PARAM_SET_NAME =
    "carmen_param_set";
  private static final String CARMEN_PARAM_SET_FMT =
    "{string, string, string, double, string}";

  /** Class method that handles query of current parameter values by module and variable */
  public static class ParamQuery extends Message {
    public String moduleName;
    public String variableName;

    ParamQuery(String moduleName, String variableName) {
      this.moduleName = moduleName;
      this.variableName = variableName;
    }
  }
  /** inner class of Param */
  public static class ParamResponse extends Message {
    public String moduleName;
    public String variableName;
    public String value;
    public int expert;
    public int status;
  }

  /** inner class of Param */
  public static class ParamSet extends Message {
    public String moduleName;
    public String variableName;
    public String value;

    ParamSet(String moduleName, String variableName, String value) {
      this.moduleName = moduleName;
      this.variableName = variableName;
      this.value = value;
    }
  }

  public static int CARMEN_PARAM_INT = 1;
  public static int CARMEN_PARAM_DOUBLE = 2;
  public static int CARMEN_PARAM_ONOFF = 3;
  public static int CARMEN_PARAM_STRING = 4;
  public static int CARMEN_PARAM_FILE = 5;
  public static int CARMEN_PARAM_DIR = 6;
  
  /** inner class of Param */
  public static class Param_t {
	private int type;
	private String module;
	private String variable;
	private String value;
	
	public Param_t(String module, String variable, int type)
	{
		this.module = module;
		this.variable = variable;
		this.type = type;
	}
	
	public String getModule() {
		
		return module;
	}
	
	public String getVariable()
	{
		return variable;
	}
	
	public void setValue(String pvalue)
	{
		value = pvalue;
	}
	
	public String getValue()
	{
		return value;
	}
  }
  
  public static void installParams (Param_t param_list[])
  {
	  for(int i = 0; i < param_list.length; i++)
	  {
		  String module = param_list[i].module;
		  String variable = param_list[i].variable;
		  String result = Param.query(module, variable);
		  
		  param_list[i].value = result;
	  }
  }
  
  /** Class method for parameter queryring */
  public static String query(String moduleName, String variableName) {
    IPC.defineMsg(CARMEN_PARAM_QUERY_NAME, CARMEN_PARAM_QUERY_FMT);
    IPC.defineMsg(CARMEN_PARAM_RESPONSE_STRING_NAME, 
		  CARMEN_PARAM_RESPONSE_STRING_FMT);
    ParamQuery query = new ParamQuery(moduleName, variableName);
    ParamResponse response = (ParamResponse)IPC.queryResponseData
      (CARMEN_PARAM_QUERY_NAME, query, ParamResponse.class, 5000);
    if (response != null)
    	return response.value;
    else 
    	return "";
  }
  
  /** Class method to set a variable with a new value */
  public static boolean set(String moduleName, String variable, 
			    String newValue) {
    ParamSet msg = new ParamSet(moduleName, variable, newValue);
    ParamResponse response = (ParamResponse)IPC.queryResponseData
      (CARMEN_PARAM_SET_NAME, msg, ParamResponse.class, 5000);
    if (response.status == 0)
      return true;
    return false;
  }

  public static boolean parseBoolean(String value) {
    if (value.equalsIgnoreCase("1"))
      return true;
    return false;
  }

  public static boolean parseOnoff(String value) {
    if (value.equalsIgnoreCase("on"))
      return true;
    return false;
  }
}
