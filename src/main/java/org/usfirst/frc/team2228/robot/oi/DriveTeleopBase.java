package org.usfirst.frc.team2228.robot.oi;

import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBase;
import org.usfirst.frc.team2228.robot.util.DebugLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// REVISION LEVEL:
// 181218 - cleaning up code

public class DriveTeleopBase {
	
	// REVISION LEVEL:
	// 181218 -  cleaning up code
	// 181102 - renamed class, added wheel control, mecanum shiftSideways command
	// 181029 - added new accel filter, wheelAxis axis
	// 180831 -  updated normalize, 
	
	// ===============================
	// SET COMMANDS
	// public void init(boolean _isConsoleDataEnabled, boolean _isTestJoyStickEnabled)
	// public void Periodic() 
	// public void setMaxThrottlePower(double _kMaxThrottlePowerLimitLevel)
	// ===============================
	// OBJECT SET/GET COMMANDS
	// driveBase.setThrottleTurn(throttleAxis, turnAxis)
	// driveBase.setMecanumShiftSidewasysEnable(true/false)
	// driverIF.getMecanumShiftSidewasysBtn()
	// driverIF.getThrottleAxis()
	// driverIF.getTurnAxis()
	// driverIF.getWheelAxis()
	//================================
	// OBJECT MEHTODS
	// private void   setSmartDashBoardParmeters()
	// private double applyTurnProfileFilter(double _turnAxis)
	// private double applySineFunction(double fTurn)
	// private double applyThrottleProfileFilter(double _throttleAxis)
	// private double applyAccelFilter(double _accelFilterThrottleValue)
	// private double cap(double num)
	// private double conditionAxisSignal(double num)
	// private double ApplyJoyDeadBand(double num)
	// private void   msg(String _msgString)

	//================================
	// OBJECTS
	
	private DriverIF driverIF;
	private SRXDriveBase driveBase;
	private DebugLogger log;

	//================================
	// CONFIGURATION SWITCHES
	
	private static boolean isTeleopConsoleDataEnabled = true;
	private static boolean isTestJoyStickEnabled = false;
	private static boolean isThrotleReversed = true;
	
	//=================================
	// SET POINTS
	
	//  with joystick at 0 set value just above what value you get from joystick  
    private static double kJoyStickDeadBand = 0.13;

	// used to vary the affect when using throtle profile
	// 0 to 1; 0 is linear (output==input) 1 is cubed (output=input**3)
    private static double kLinearToCubedGain = 1.0; 
	
    // Turn profile gain: (0.1 to 1.0) larger value more response at lower turnAxis values
	private static double kTurnProfileGain = 0.3;

	// limits wheel turn axis to within +/- 90 degrees
	private static double kMaxTurnAxisAngle = 85;

	// determination of max throttleAxis delta values are determined by testing
	private static double kThrotleMaxDeltaChange = 0.1;

	private static double kMaxThrottlePowerLimit = .95;
	private static double kMaxTurnPowerLimit = 1;


	//======================================
	// VARIABLES

	private double origThrottleAxis = 0;
	private double origTurnAxis = 0;
	private double origWheelAxis =0;

	private double turnAxis = 0;
	private double throttleAxis = 0;
	private double wheelAxis = 0;
	private double signWheelAxis = 1;

	private double wheelAxisTrunOnStpt = .8;
	private double wheelAxisTurnOffStpt = .1;
	private double turnAxisAngle = 0;

	private double throttleChange = 0;
	private double signThrottleValue = 1;
	private double throttleNminus2 = 0;
	private double throttleNminus1 = 0;

	private boolean isWheelTurnActive = false;

	private String lastMsgString = " ";
	
	//==============================================
	// TELEOPCONTROLLER CONSTRUCTOR
	public DriveTeleopBase(DriverIF     _driverIF, 
						   SRXDriveBase _driveBase,
						   DebugLogger  _debuglogger) {
		driverIF = _driverIF;
		driveBase = _driveBase;
		log = _debuglogger;
	}
	
	//==========================================
	// TELEOP INIT
	public void init(boolean _isConsoleDataEnabled, boolean _isTestJoyStickEnabled){

		// Set class program switches
		isTeleopConsoleDataEnabled = _isConsoleDataEnabled? true : false;
		isTestJoyStickEnabled = _isTestJoyStickEnabled? true : false;
		isWheelTurnActive = false; 
	}
		
	//==========================================
	// TELEOP PERIODIC
	public void Periodic() {
		
		// Save the joystick values
		
		origThrottleAxis = isThrotleReversed? -driverIF.getThrottleAxis(): driverIF.getThrottleAxis();
		origTurnAxis = driverIF.getTurnAxis();
		origWheelAxis = driverIF.getWheelAxis();

		throttleAxis = origThrottleAxis;
		turnAxis = origTurnAxis;
		wheelAxis = origWheelAxis;

		throttleAxis = conditionAxisSignal(throttleAxis);
		turnAxis = conditionAxisSignal(turnAxis);
		wheelAxis = conditionAxisSignal(wheelAxis);

		
		// Condition throtle and turnAxis signals
		throttleAxis = applyThrottleProfileFilter(throttleAxis);
		throttleAxis = applyAccelFilter(throttleAxis);
		throttleAxis = cap(throttleAxis);
		throttleAxis *= kMaxThrottlePowerLimit;
		
		// This is leveraged from culver drive/Team 33
		// Integrate turn(x axis only) with a game controller to give car wheel action
		if(!isWheelTurnActive){
			// To turn game cntrller wheel function push left joystick to top of Y axis and use the round
			// edge at top of the joystick to simulate a wheel
			if((Math.abs(wheelAxis) > wheelAxisTrunOnStpt) && (Math.abs(turnAxis) < wheelAxisTurnOffStpt)){
				isWheelTurnActive = true;
				signWheelAxis = Math.signum(wheelAxis);
			} else {
			turnAxis = applyTurnProfileFilter(turnAxis);
			turnAxis = cap(turnAxis);

			// Limit turn power with respect to throttle power
			turnAxis *= kMaxTurnPowerLimit * (1 - (throttleAxis * throttleAxis));
			}
		} else{ 
			// Turn off wheel axis if at x-y zero position
			if((Math.abs(wheelAxis) < wheelAxisTurnOffStpt) && (Math.abs(turnAxis) < wheelAxisTurnOffStpt)){
				isWheelTurnActive = false;
			}
			// Find the gameController joystick angle of x,y
			turnAxisAngle = Math.toDegrees((double)Math.atan2(turnAxis,Math.abs(wheelAxis)));

			// Limit wheel angle
			turnAxisAngle = (turnAxisAngle > kMaxTurnAxisAngle)? kMaxTurnAxisAngle: turnAxisAngle;

			// map angle to (-1 to 1)
			turnAxis = turnAxisAngle / 90;

			// Limit turn power with respect to throttle power
			turnAxis *= kMaxTurnPowerLimit * throttleAxis * signWheelAxis;
		}
		
		// =======================================
		// DRIVE ROBOT
		
		if(!isTestJoyStickEnabled){
			if(driverIF.getMecanumShiftSidewaysBtn()){
				driveBase.setMecanumShiftSidewaysEnable(true);
				// public void setThrottleTurn(double _throttleValue, double _turnValue, double _headingCorrection) {	
				driveBase.setThrottleTurn(turnAxis, 0, 0);
			} else {
				driveBase.setMecanumShiftSidewaysEnable(false);
				// public void setThrottleTurn(double _throttleValue, double _turnValue, double _headingCorrection) {
				driveBase.setThrottleTurn(throttleAxis, turnAxis, 0);
			}
		}
		
		// ++++++++++++++++++++++++++++++++++++
		// Display
		if (isTeleopConsoleDataEnabled){
			System.out.printf("Thottle:%-4.2f==Turn:%-4.2f==Wheel:%-4.2f", 
								throttleAxis,
								turnAxis,
								wheelAxis);
		}
	}	
								
	
	//=======================================
	// SET METHODS
	public void setMaxThrottlePower(double _kMaxThrottlePowerLimitLevel) {
	kMaxThrottlePowerLimit = _kMaxThrottlePowerLimitLevel;
	}
	
	// ===========================================
	// DRIVERIF FILTERING FUNCTIONS

	private double applyTurnProfileFilter(double _turnAxis) {
		
		double fTurn = _turnAxis;
		
		//developed by team 254 for the turnAxis stick to provide a more realistic feel for turnAxising???
		// Filter(distorted sine) makes low turnAxis input more responsive and is flat at higher turnaxis input
		fTurn = applySineFunction(fTurn);
		fTurn = applySineFunction(fTurn);
		fTurn = applySineFunction(fTurn);
	
		return fTurn;
	}
	
	private double applySineFunction(double fTurn) {
		// kTurnProfileGain should be 0.1 to 1.0 (.1-linear response; 1-high response at low turnaxis input)
		double factor = (Math.PI / 2.0) * kTurnProfileGain;
		return Math.sin(factor * fTurn) / Math.sin(factor);
	}
	
	private double applyThrottleProfileFilter(double _throttleAxis) {

		// By changing the kLinearToCubedGain a linear to cubed output value. A squared or cubed filter  
		// changes slowly for low values of input throtle 
		
		// kLinearToCubedGain => from 0 to 1; 0 = linear, 1 = cubed
		return (kLinearToCubedGain * (Math.pow(_throttleAxis, 3)))	+ ((1 - kLinearToCubedGain) * _throttleAxis);
	}
	
	// ACCELERATION FILTER
	// The accel filter follows the actions of the driver. If the
	// driver exceeds the robot accel/decel capability, the throtle change is capped to prevent tipping.
	// Otherwise the robot follows the driver input command
	private double applyAccelFilter(double _accelFilterThrottleValue) {
		signThrottleValue = Math.signum(_accelFilterThrottleValue);
		// Determine throtle delta from last avg
		throttleChange = _accelFilterThrottleValue - ((throttleNminus1 + throttleNminus2) / 2);

		// Cap throtle change
		throttleChange = (Math.abs(throttleChange) > kThrotleMaxDeltaChange)? kThrotleMaxDeltaChange : throttleChange;

		_accelFilterThrottleValue += signThrottleValue * throttleChange;

		// Save throtle value for next average calculation
		throttleNminus2 = throttleNminus1;
		throttleNminus1 = _accelFilterThrottleValue;

		return  _accelFilterThrottleValue;
	}
	
	// ============================================
	// UTIL METHODS
	
	//helper function to keep inside of acceptable %power range
	private double cap(double num) {
		if(Math.abs(num) > 1){

			// Cap num to -1 or 1
			num = Math.signum(num)* 1.0;
		} 
		return num;
	}
	// This caps joystick axis, deadband's joystick axis and remaps axis to 0 -> 1 range
	private double conditionAxisSignal(double num) {
		if(Math.abs(num) > 1){
			// Cap num to -1 or 1
			num = Math.signum(num)* 1.0;

		// Check deabband	
		} else if (Math.abs(num) < kJoyStickDeadBand) {
			 num = 0;
		} else if(Math.abs(num) >= kJoyStickDeadBand) {

			// This remaps data from (kJoyStickDeadBand -> 1) to (0 -> 1)
			if(num >= 0){
				num = (num - kJoyStickDeadBand) / (1 - kJoyStickDeadBand);
			} else {
				num = (num + kJoyStickDeadBand) / (1 - kJoyStickDeadBand);
			}	
		}
		return num;
	}
	
	private double ApplyJoyDeadBand(double num){
		if (Math.abs(num) <= kJoyStickDeadBand) {
			num = 0;
		}
		return num;
	}
	
	private void msg(String _msgString){
		if (isTeleopConsoleDataEnabled){
			if (_msgString != lastMsgString){
				System.out.println(_msgString);
				lastMsgString = _msgString;
			}
		}
	}
}