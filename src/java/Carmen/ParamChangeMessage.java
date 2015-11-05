package Carmen;
/** Carmen ParamChangeHandler's message */

public class ParamChangeMessage extends Message {
  public String moduleName;
  public String variableName;
  public String newValue;
  public int expert;
  public int status;

  private static final String CARMEN_PARAM_CHANGE_NAME = 
    "carmen_param_variable_change";
  private static final String CARMEN_PARAM_CHANGE_FMT =
    "{string, string, string, int, int, double, string}";

  public static void subscribe(ParamChangeHandler handler)
  {
    subscribe(CARMEN_PARAM_CHANGE_NAME, CARMEN_PARAM_CHANGE_FMT, handler, 
	      ParamChangeMessage.class, "handle");
  }
}

