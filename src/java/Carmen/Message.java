package Carmen;
import IPC.*;

import java.util.HashSet;
import java.lang.reflect.Method;
import java.lang.reflect.InvocationTargetException;

/** Carmen generic or base Message class */

public class Message {
	public double timestamp;
	public String host;  

	private static HashSet<String> defined_messages = new HashSet<String>(); 

	/** New message has timestamp from this host and host name */
	public Message() {
		timestamp = Util.getTime();
		host = Util.getHostName();
	}

	private static void verifyFormatString(String msgFormat, Class msgClass)
	{

	}

	private static void verifyFormatString(String msgFormat, Object msgInstance)
	{
		Class className = msgInstance.getClass();
		verifyFormatString(msgFormat, className);
	}

	private static class PrivateHandler implements IPC.HANDLER_TYPE {
		private Object userHandler;
		private Class handlerClass;
		private Class messageClass;
		private Method handleMethod;

		PrivateHandler(Object userHandler, Method handleMethod,
				Class handlerClass, Class messageClass) 
				{
			this.userHandler = userHandler;
			this.handleMethod = handleMethod;
			this.handlerClass = handlerClass;
			this.messageClass = messageClass;
				}

		public void handle(IPC.MSG_INSTANCE msgInstance, Object callData) 
		{
			try {
				handleMethod.invoke(userHandler, callData);
			}
			catch (IllegalAccessException e) {
				System.err.println(e.toString());
				System.exit(-1);
			}
			catch (IllegalArgumentException e) {
				System.err.println(e.toString());
				System.exit(-1);
			}
			catch (InvocationTargetException e) {
				System.err.println(e.toString());
				System.exit(-1);
			}
		}

	}  

	protected static void subscribe(String messageName, String messageFmt, 
			Object handler, Class messageClass, 
			String handlerFuncName) 
	{
		if (!defined_messages.contains(messageName)) {
			verifyFormatString(messageFmt, messageClass);
			IPC.defineMsg(messageName, messageFmt);
			defined_messages.add(messageName);
		}

		Class handlerClass = handler.getClass();

		System.out.println("Class: " + handlerClass.getCanonicalName());
		
		Method handleMethod;
		try {
			handleMethod = handlerClass.getMethod(handlerFuncName, messageClass);
			//System.out.println("handleMethod: " + handleMethod.getName());
		}
		catch (NoSuchMethodException e) {
			System.err.println("You subscribed to "+messageClass+" but you used a "+
					"handler that doesn't\nhave a method \n"+
					"handle("+messageClass+")\n");
			throw new Error(e.toString());
		}

		PrivateHandler pvtHandler = createHandler(handler, handleMethod, handlerClass, messageClass);

		IPC.subscribeData(messageName, pvtHandler, messageClass);
		IPC.setMsgQueueLength(messageName, 1);
	}

	private static PrivateHandler createHandler(Object handler, Method handleMethod, 
			Class handlerClass, Class messageClass) {
		return new PrivateHandler(handler, handleMethod,
				handlerClass, messageClass);
	}

	public void publish(String msgName, String msgFmt, Object message) 
	{
		if (!defined_messages.contains(msgName)) {
			verifyFormatString(msgFmt, message);
			IPC.defineMsg(msgName, msgFmt);
			defined_messages.add(msgName);
		}

		IPC.publishData(msgName, this);
	}
}

