/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Class responsible for logging messages. 
 * To log a message, import Log and call
 * Log.info("message here")
 */
import java.util.logging.Level;
import java.util.logging.Logger;

public class Log {
	private static Logger m_Logger = Logger.getLogger("Logger");
	
	public Log() {
	}
	
	public static <A> void info(A msg) {
		m_Logger.log(Level.INFO, msg.toString());
	}
}