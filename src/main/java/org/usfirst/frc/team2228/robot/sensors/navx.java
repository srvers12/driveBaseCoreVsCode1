// package org.usfirst.frc.team2228.robot.sensors;

// todo - review
// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.I2C.Port;
// import edu.wpi.first.wpilibj.PIDOutput;
// import edu.wpi.first.wpilibj.PIDSource;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.SPI;

//REVISION LEVEL:
//181218 - cleaning up code

// public class navx implements PIDOutput {
// 	private AHRS ahrs;
// //	private PIDController _PIDController;
// 	static final double AngleSP = 0.0;
// 	static double rate = 0;
// 	static double kP = 0.0003;
// 	static final double kI = 0.00;
// 	static final double kD = 0.00;
	
// 	static final double kF = 0.00;
// 	private PIDSource pidSource;
// 	private PIDOutput pidOutput;
// 	// how close the navX will get to the value
// 	static final double kToleranceDegrees = 2.0f;
// 	private boolean rotateToAngle = false;

// 	public AngleIF() {
// 		try {
// 			ahrs = new AHRS(Port.kOnboard);  // Mu
// //			ahrs = new AHRS(SPI.Port.kOnboardCS0);  // Lamda
// 		} catch (RuntimeException ex) {
// 			System.out.println("Error starting the navx");
// 		}
// 		ahrs.reset();
// //		PIDController _PIDController = new PIDController(kP, kI, kD, kF, pidSource, pidOutput);
// //        _PIDController.setInputRange(-180.0f,  180.0f);
// //        _PIDController.setOutputRange(-1.0, 1.0);
// //        _PIDController.setAbsoluteTolerance(kToleranceDegrees);
// ////        _PIDController.setContinuous(true);
// //        SmartDashboard.putNumber("Barometric Pressure", getBaroPressure());
// //        SmartDashboard.putNumber("kP", getBaroPressure());
// //        SmartDashboard.putNumber("Barometric Pressure", getBaroPressure()); 
// 	}

// 	public void setZeroAngle(double gyro) {
// 		ahrs.setAngleAdjustment(gyro);
// 	}

// 	public double getAngle() {
// 		SmartDashboard.putNumber("Navx angle", ahrs.getAngle());
// 		return ahrs.getAngle();

// 	}

// 	public double getYaw() {
// 		return ahrs.getYaw();
// 	}
	
// 	public double getAccel() {
// 		return ahrs.getRawAccelX();
// 	}
	
// 	public double getRoll() {
// 		return ahrs.getRoll();
// 	}
// 	public void zeroYaw() {
// 		ahrs.zeroYaw();
// 		System.out.println("ZEROED THE YAW!");
// 	}

// 	public double _PIDCorrection(double angle) {
// 		double error;
// 		error = getYaw() - angle;
// 		return kP * error;

// 	}
// 	public double getAngleCorrection() {
// 		double error;
// 		double Yaw = Math.floor(getYaw()*10000)/10000;
// 		error = AngleSP - Yaw;
// 		double correction = 1 *((kP * error) - (kD * rate));
// 		System.out.println("Yaw " + Yaw + "; correction " + correction);
// 		if (Math.abs(Yaw) < 0.07) {
// 			correction = 0.0;
// 		}
// 		return correction;
// 	}

// 	@Override
// 	public void pidWrite(double output) {
// 		// TODO Auto-generated method stub
// 	}
	
// 	public double round(double d, int places) {
// 		double a = Math.pow(10, places);
// 		return Math.round(d * a) / a;
// 	}
// }
