package org.usfirst.frc.team2228.robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AngleIF {
	
	// Define class instances
	private _navx;
	private roboAccel;
	
	// Constructor
	public AngleIF() {
		_navx = new navx();
		roboAccel = new RoboRioAccelerometers();
	}

	public void setZeroAngle(double gyro) {
		_navx.setAngleAdjustment(gyro);
	}

	public double getAngle() {
		SmartDashboard.putNumber("Navx angle", _navx.getAngle());
		return _navx.getAngle();
	}

	public double getYaw() {
		return _navx.getYaw();
	}
	
	public double getAccel() {
		return _navx.getRawAccelX();
	}
	
	public double getRoll() {
		return _navx.getRoll();
	}
	
	public void zeroYaw() {
		System.out.println("ZEROED THE YAW!");
		return _navx.zeroYaw();
	}

	}
	public double getAngleCorrectionFactor() {
		return _navx.correctionFactor();
	}
	
	// todo
	public boolean getIsTiltAtMax() {
		
	}
}
