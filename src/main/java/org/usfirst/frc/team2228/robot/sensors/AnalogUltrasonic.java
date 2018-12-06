package org.usfirst.frc.team2228.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogUltrasonic {
	AnalogInput sensor = new AnalogInput(1);
	double voltage;
	double distance;

	public AnalogUltrasonic(AnalogInput sensor) {
		double voltage = sensor.getVoltage();
		
	}
	public double getDistance(){
		voltage = sensor.getVoltage();
		
		return voltage;
	}
}
