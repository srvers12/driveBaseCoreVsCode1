package org.usfirst.frc.team2228.robot.util;

import java.util.Date;
import java.text.SimpleDateFormat;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;


public class DebugLogger {

	String logFileName = " ";

	public DebugLogger() {
		// Create a log file for this RobRio power on cycle
		try{
			SimpleDateFormat dateTimeFormat = new SimpleDateFormat("MM_dd.hh.mm");
			Date dateTimeNow = new Date();
			logFileName = "home/lvuser/log/log_" + dateTimeFormat.format(dateTimeNow) + ".txt";	
			System.out.println(logFileName);

			File logFile = new File(logFileName);
			if(!logFile.exists()){
				logFile.createNewFile();
			}
		}catch(IOException ioe){
    	   System.out.println("Exception occurred:");
    	   ioe.printStackTrace();
		}
		
	}
	
	public void write(String _logRecord) {
		try{
			// Determine time stamp
			SimpleDateFormat dateFormat = new SimpleDateFormat("hh.mm.ss.SSS");
			Date dateTimeNow = new Date();

			//Opening a file in append mode using FileWriter
			FileWriter fileWrter = new FileWriter(logFileName,true);
			PrintWriter prntWrter = new PrintWriter(fileWrter);

			// Line format: time stamp(dateFormat), program status or and for delimited data (_logRecord)
			prntWrter.println(dateFormat.format(dateTimeNow) + "," + _logRecord);
			prntWrter.close();
			
		}catch(IOException ioe){
    	   System.out.println("Exception occurred:");
    	   ioe.printStackTrace();
		}
	}
}