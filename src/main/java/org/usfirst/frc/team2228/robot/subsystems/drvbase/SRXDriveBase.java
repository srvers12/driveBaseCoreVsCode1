package org.usfirst.frc.team2228.robot.subsystems.drvbase;
// 
//  Class SRXBaseDrive
//  RELEASE: 2019 
//  Team 2228
// REVISIONS:
// 181106 - removed test methods
// 181102 - updated header, added SRX motion profile
// 181024 - removed time/velocity move commands in 2018 version
// 181024 - added SRX turn, SRX motion profile, removed timed/velocity motion, renamed motion to move, rotate, turn
// 180831 - timer autonomous methods
// 180327 - added config init command to load config vaules specific to a robot chassis
// 180108 - original

// ===================================
// SET COMMANDS
// ===================================
// public void init(boolean _isConsoleEnabled, boolean _isLoggingEnabled)

// public void setMecanumShiftEnable(boolean _mecanumShiftState)

// public void setRightSensorPositionToZero()
// public void setLeftSensorPositionToZero()
// public void setRightEncPositionToZero()
// public void setLeftEncPositionToZero()
// public void driveStraightFwdCorrection(double _driveStraightCorrection){
// public void setBrakeMode(boolean _isBrakeEnabled)
// public void stopMotors()
// public void setDriveBaseRamp(double _SecToMaxPower)

// private void clearSRXDriveBasePrgFlgs()
		
// ===================================
// GET COMMANDS
// ===================================
// public double getRightSensorPosition()
// public double getRightSensorVelocity()
// public double getRightEncoderPosition()
// public double getRightEncoderVelocity()
// public double getRightMstrMtrCurrent()
// public double getRightFollowerMtrCurrent()
// public double getRightCloseLoopError()

// public double getLeftSensorPosition(
// public double getLeftSensorVelocity()
// public double getLeftEncoderPosition()
// public double getLeftEncoderVelocity()
// public double getLeftMstrMtrCurrent()
// public double getLeftFollowerMtrCurrent()
// public double getRobotTrackWidth(){
// public double getEncoderInchesPerCount(){
// public double getLeftCloseLoopError()

// public double getBusVoltage()
// public double getDriveStraightCorrection()
// public boolean getIsDriveMoving()

// ====================================
// DATA COMMANDS
//=====================================
// public void smartDashboardDriveBaseData()
// public void logSRXDriveData()

// ===================================
//	TELEOP MOTION COMMANDS
// ===================================
	
// *****
// public void SetDriveTrainCmdLevel(double _rightCMDLevel, 
//									 double _leftCMDLevel)
// *****

// *****
//	public void setThrottleTurn(double _throttleValue, 
//								double _turnValue'
//								double _headingCorrection) 
// *****

// =====================================
//	AUTONOMOUS MOTION COMMANDS
// =====================================

// ++++++++++
// public boolean move(double _MoveDistanceIn, 
//					   double _MovePwrlevel)
// 
// @parm  _MoveDistanceIn - move distance in inches
// @parm  _MovePwrlevel - power level 0 - 1, 
//						  program converts to VelocityNativeUnits as (0 to 1)* maxVelocityNativeUnits

// ++++++++++
// public boolean move(double  _MoveDistanceIn, 
//					   double  _MovePwrLevel,
//					   boolean _MoveSideways)
// +++++++++
//
// +++++++++
// public boolean rotate(double _RotateAngleDeg, 
//						 double _RotatePwrLevel)
// +++++++++
//
// +++++++++
// public boolean SRXBasemove(int    _rightCruiseVel, 
//					   		  int    _rightAccel, 
//					   		  double _rightDistance, 
//					   		  int    _leftCruiseVel,	
//					   		  int    _leftAccel, 
//					   		  double _leftDistance,
//					   		  double _indexFaultTimeSec)
// +++++++++
//
// ++++++++
// public boolean SRXProfileMove(double[][] ProfileRight, 
//								 double[][] ProfileLeft, 
//								 int totalPointNum)
// ++++++++


// ===================================


//Carrying over the classes from other libraries
import org.usfirst.frc.team2228.robot.RobotMap;
import org.usfirst.frc.team2228.robot.util.DebugLogger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRXDriveBase {
	
	
	// DifferentialDrive or tank motors
	private TalonSRX rightMasterMtr;
	private TalonSRX rightFollowerMtr;
	private TalonSRX leftMasterMtr;
	private TalonSRX leftFollowerMtr;
	private DebugLogger log;
	private RobotMap RobotMap;
	private Faults leftFaults;
	private Faults rightFaults;
	private Notifier pushAPI2SRXThread;
	
	// ===================================
	// SRX MOTION PROFILE DRIVE BASE 

	// The status of the motion profile executer and buffer inside the Talon.
	// Instead of creating a new one every time we call getMotionProfileStatus,
	//  keep one copy.	
	private MotionProfileStatus SRXProfileStatusRight = new MotionProfileStatus();
	private MotionProfileStatus SRXProfileStatusLeft = new MotionProfileStatus();

	private double[][] pointsRight;
	private double[][] pointsLeft;
	// How many trajectory points do we wait for before firing the motion
	// profile.
	private int kBufferCntMin = 50;
	private int kSRXBufferSize = 128;

	private int SRXProfileState = 0;
	private int bufferAccumCnt = 0;
	private int profileNumPoints = 0;
	private int profileIndexPointer = 0;
	private int profileCountAccumulator = 0;

	// additional cache for holding the active trajectory point	
	private double SRXRightTrajectoryPosition=0;
	private double SRXRightTrajectoryVelocity=0; 
	private double SRXRightTrajectoryHeading=0;
	private double SRXRLeftTrajectoryPosition=0;
	private double SRXLeftTrajectoryVelocity=0; 
	private double SRXLeftTrajectoryHeading=0;
	private double estProfileFaultTimeMs = 0;
	private double API2SRXThreadTimeSec = 0;
	private double kBaseTrajPeriodMs = 0;

	private boolean isSRXProfileMoveActive = false;

	// ====================================	
	// DRIVE BASE VARIABLES

	private int SRXTimeoutValueMs = 10;
	private int correctionSensorType = 1;	
	private int RightCruiseVelNativeUnits = 0;
	private int LeftMoveTimeSec = 0;
	private int RightAccelNativeUnits = 0;
	private int LeftCruiseVelNativeUnits = 0;
	private int LeftAccelNativeUnits = 0;
	private int LeftRotateTimeSec = 0;

	private double kDriveStraightFwdCorrection = SRXDriveBaseCfg.kDriveStraightFwdCorrection;
	private double kStdAccelTimeSegment = 3;

	private double RightDistanceCnts = 0;
	private double LeftDistanceCnts = 0;
	private double SRXMotionLeftPos =0;
	private double SRXMotioRightPos =0;
	private double SRXMotionLeftVel = 0;
	private double SRXMotioRightVel = 0;
	private double leftCmdLevel = 0;
	private double rightCmdLevel = 0;
	private double methodStartTime = 0;
	private double methodTime = 0;
	private double rightSensorPositionRead = 0;
	private double leftSensorPositionRead = 0;
	private double encoderHeadingDeg = 0;
	private double stepFunctionSpeed = 0;
		
	//  Program flow switches
	private boolean isConsoleEnabled = false;
	private boolean isLoggingEnabled = false;
	private boolean islogSRXDriveDataActive = false;
	private boolean isTrapezoidalTurnToAngleActive = false;
	private boolean isSRXMoveActive = false;
	private boolean isStdTrapezoidalRotateActive = false;
	private boolean isStdTrapezoidalMoveActive = false;
	private boolean isDriveTrainMoving = false;
	private boolean isMecanumShiftEnabled = false;
	private boolean isTestEnabled = false;
	private boolean isStepTestEnabled = false;
	private boolean isMotorEncoderTestEnabled = false;

	//private String logSRXDriveString = " ";
	private String lastMsgString = " ";
	
	// SRXDriveBase Class Constructor
	public SRXDriveBase(RobotMap _robotMap, DebugLogger _logger) {
	
		log = _logger;
		RobotMap = _robotMap;
		
		// Create CAN SRX motor controller objects
		rightMasterMtr = new TalonSRX(RobotMap.RIGHT_MSTR_MTR_CAN_ID);
		rightFollowerMtr = new TalonSRX(RobotMap.RIGHT_FOLLOWER_MTR_CAN_ID);
		leftMasterMtr = new TalonSRX(RobotMap.LEFT_MSTR_MTR_CAN_ID);
		leftFollowerMtr = new TalonSRX(RobotMap.LEFT_FOLLOWER_MTR_CAN_ID);

		rightFaults = new Faults();
		leftFaults = new Faults();

		
		// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// RIGHT MOTORS===========================================================
		// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Set min/max output
		rightMasterMtr.configNominalOutputForward(0.0, SRXTimeoutValueMs);
		rightMasterMtr.configNominalOutputReverse(0.0, SRXTimeoutValueMs);
		rightMasterMtr.configPeakOutputForward(1, SRXTimeoutValueMs);
		rightMasterMtr.configPeakOutputReverse(-1, SRXTimeoutValueMs);
		
		// Reverse motor if necessary
		rightMasterMtr.setInverted(SRXDriveBaseCfg.isDriveRightMasterMtrReversed);
		
		// Configure voltage compensation mode and set max voltage to 11 volts
		rightMasterMtr.configVoltageCompSaturation(11.0, SRXTimeoutValueMs);

		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		rightMasterMtr.configVoltageMeasurementFilter(32, SRXTimeoutValueMs);
		rightMasterMtr.enableVoltageCompensation(true);
		
		// set output zero (neutral) deadband at 4%
		rightMasterMtr.configNeutralDeadband(0.04, SRXTimeoutValueMs);

		// Set up stall conditions in SRX for the drive train
		rightMasterMtr.configPeakCurrentLimit(SRXDriveBaseCfg.kStallCurrentPeakAmps, SRXTimeoutValueMs);
		rightMasterMtr.configPeakCurrentDuration(SRXDriveBaseCfg.kStallTimeMs, SRXTimeoutValueMs);
		rightMasterMtr.configContinuousCurrentLimit(SRXDriveBaseCfg.kStallCurrentContinuousAmps, SRXTimeoutValueMs);
		rightMasterMtr.enableCurrentLimit(true);
		
		// Configure the velocity measurement period and sample window rolling average
		// Sample period in ms from supported sample periods-default 100ms period/64 sample window
		rightMasterMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms,SRXTimeoutValueMs);
		rightMasterMtr.configVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample,SRXTimeoutValueMs);
		
		// Set up encoder input
		rightMasterMtr.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, SRXDriveBaseCfg.kslotIDx, SRXTimeoutValueMs);
		rightMasterMtr.setSensorPhase(SRXDriveBaseCfg.isRightEncoderSensorReversed);
		
		// Clear quadrature position
		rightMasterMtr.clearStickyFaults(SRXTimeoutValueMs);
		rightMasterMtr.getSensorCollection().setQuadraturePosition(0, SRXTimeoutValueMs);
		
		
		// SET UP RIGHT FOLLOWER =======================
		// Invert SRX output to motors if necessary
		rightFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveRightFollowerMtrReversed);
		rightFollowerMtr.clearStickyFaults(SRXTimeoutValueMs);
		rightFollowerMtr.set(ControlMode.Follower, rightMasterMtr.getDeviceID());
		
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// LEFT MOTORS===============================================================================
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Set min/max output
		leftMasterMtr.configNominalOutputForward(0.0, SRXTimeoutValueMs);
		leftMasterMtr.configNominalOutputReverse(0.0, SRXTimeoutValueMs);
	    leftMasterMtr.configPeakOutputForward(1, SRXTimeoutValueMs);
		leftMasterMtr.configPeakOutputReverse(-1, SRXTimeoutValueMs);
		
		// Reverse direction if necessary
		leftMasterMtr.setInverted(SRXDriveBaseCfg.isDriveLeftMasterMtrReversed);
		
		// Configure voltage compensation mode and set max voltage to 11 volts
		leftMasterMtr.configVoltageCompSaturation(11.0, SRXTimeoutValueMs);
		
		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		leftMasterMtr.configVoltageMeasurementFilter(32, SRXTimeoutValueMs);
		leftMasterMtr.enableVoltageCompensation(true);
		
		// set output zero (neutral) deadband at 4%
		leftMasterMtr.configNeutralDeadband(0.04, SRXTimeoutValueMs);
		
		// Set up stall conditions in SRX for the drive train
		leftMasterMtr.configPeakCurrentLimit(SRXDriveBaseCfg.kStallCurrentPeakAmps, SRXTimeoutValueMs);
		leftMasterMtr.configPeakCurrentDuration(SRXDriveBaseCfg.kStallTimeMs, SRXTimeoutValueMs);
		leftMasterMtr.configContinuousCurrentLimit(SRXDriveBaseCfg.kStallCurrentContinuousAmps, SRXTimeoutValueMs);
		leftMasterMtr.enableCurrentLimit(true);
		
		// Configure the velocity measurement period and sample window rolling average
		// Sample period in ms from supported sample periods-default 100ms period/64 sample window
		leftMasterMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms,SRXTimeoutValueMs);
		leftMasterMtr.configVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample,SRXTimeoutValueMs);
		
		// Set up encoder input
		leftMasterMtr.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, SRXDriveBaseCfg.kslotIDx, SRXTimeoutValueMs);
		leftMasterMtr.setSensorPhase(SRXDriveBaseCfg.isLeftEncoderSensorReversed);
		
		// Clear quadrature position
		leftMasterMtr.clearStickyFaults(SRXTimeoutValueMs);
		leftMasterMtr.getSensorCollection().setQuadraturePosition(0, SRXTimeoutValueMs);
		
		
		// SET UP LEFT FOLLOWER =======================
		leftFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveLeftFollowerMtrReversed);
		leftFollowerMtr.clearStickyFaults(SRXTimeoutValueMs);
		leftFollowerMtr.set(ControlMode.Follower, leftMasterMtr.getDeviceID());
		
		// zero control output
		rightMasterMtr.setNeutralOutput();
		leftMasterMtr.setNeutralOutput();

		/*
		 * Set Brake-Coast mode to coast
		 */
		setBrakeMode(SRXDriveBaseCfg.isBrakeEnabled);

		
		
		// set timeout to zero to stop waiting for confirmations
		SRXTimeoutValueMs = 0;
	}
	// END OF CONSTRUCTOR
	
	// ======================================================================================
	// =======================================================================================
	// SRXBaseDrive SET/CONFIG METHODS
	// =======================================================================================
	// ======================================================================================
	public void setRightSensorPositionToZero() {
		// SRX API Commands are executed every 10ms
		rightMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, 25);
	}
	
	public void setLeftSensorPositionToZero() {
		// SRX API Commands are executed every 10ms
		leftMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, 25);
	}
	
	public void setRightEncPositionToZero() {
		// SRX API Commands are executed every 10ms
		rightMasterMtr.getSensorCollection().setQuadraturePosition(0, 15);
	}
	
	public void setLeftEncPositionToZero() {
		// SRX API Commands are executed every 10ms
		leftMasterMtr.getSensorCollection().setQuadraturePosition(0, 25);
	}

	public void driveStraightFwdCorrection(double _driveStraightCorrection){
		kDriveStraightFwdCorrection = _driveStraightCorrection;
	}
	
	public void setBrakeMode(boolean _isBrakeEnabled) {
		rightMasterMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		rightFollowerMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		leftMasterMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		leftFollowerMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
	}

	public void setDriveBaseRamp(double _SecToMaxPower){
		if(SRXDriveBaseCfg.isSRXClosedLoopEnabled){
			rightMasterMtr.configClosedloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
			leftMasterMtr.configClosedloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
		} else {
			rightMasterMtr.configOpenloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
			leftMasterMtr.configOpenloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
		}
	}

	public void stopMotors() {
		rightMasterMtr.setNeutralOutput();
		leftMasterMtr.setNeutralOutput();
	}

	public void setTestEnable(int _testNumber){
		switch(_testNumber){
			case 0:
				isMotorEncoderTestEnabled = false;
				isStepTestEnabled = false;
			break;
			case 1:
				isMotorEncoderTestEnabled = true;
			break;
			case 2:
				isStepTestEnabled = true;
			break;
			default:
			break;
		}
	}

	public void setMecanumShiftSidewaysEnable(boolean _mecanumShiftState){
		isMecanumShiftEnabled = _mecanumShiftState;
	}
	public void init(boolean _isConsoleEnabled, boolean _isLoggingEnabled) {

		// Set class program switches
		isConsoleEnabled = _isConsoleEnabled? true : false;
		isLoggingEnabled = _isLoggingEnabled? true : false;

		// Clear SRXDriveBase program control flags
		clearSRXDrvBasePrgFlgs();
		
		// Load smart dashboard and shuffle board parameters
		smartDashboardDriveBaseData();
		
		// Stop motors and clear position counters
		stopMotors();
		setDriveBaseRamp(0);
		setRightSensorPositionToZero();
		setLeftSensorPositionToZero();

		// Turn on motor safety - if no commands to SRX in 100ms, SRX's  will shut down
		rightMasterMtr.setSafetyEnabled(true);
		rightFollowerMtr.setSafetyEnabled(true);
		leftMasterMtr.setSafetyEnabled(true);
		leftFollowerMtr.setSafetyEnabled(true);

		
		// Load drive train PID values
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			rightMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			rightMasterMtr.configAllowableClosedloopError(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kClosedLoopErr, SRXTimeoutValueMs);
			rightMasterMtr.config_kF(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrFeedForwardGain, SRXTimeoutValueMs);
			rightMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrProportionalGain, SRXTimeoutValueMs);
			rightMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrIntegralGain, SRXTimeoutValueMs); 
			rightMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrDerivativeGain, SRXTimeoutValueMs);
			rightMasterMtr.config_IntegralZone(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrIzone, SRXTimeoutValueMs);
		
			leftMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			leftMasterMtr.configAllowableClosedloopError(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kClosedLoopErr, SRXTimeoutValueMs);
			leftMasterMtr.config_kF(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain, SRXTimeoutValueMs);
			leftMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrProportionalGain, SRXTimeoutValueMs);
			leftMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrIntegralGain, SRXTimeoutValueMs); 
			leftMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain, SRXTimeoutValueMs);
			leftMasterMtr.config_IntegralZone(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveleftMstrIzone, SRXTimeoutValueMs);
		}
		
	}
	
	// Clear all program control flags
	private void clearSRXDrvBasePrgFlgs() {
		isStdTrapezoidalMoveActive = false;
		isStdTrapezoidalRotateActive = false;
		isTrapezoidalTurnToAngleActive = false;
		isSRXMoveActive = false;
		isMecanumShiftEnabled = false;
		islogSRXDriveDataActive = false;
		isSRXProfileMoveActive = false;
		isTestEnabled = false;
	}
	
	
	
	// =======================================================================================
	// =======================================================================================
	// SRXBaseDrive GET METHODS
	// =======================================================================================
	//=======================================================================================
	
	// RIGHT MASTER MOTOR ==============
	
	public double getRightSensorPosition(){
		double sign = (SRXDriveBaseCfg.isRightEncoderSensorReadReversed)? -1 : 1;
		return (sign * rightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx));
	}
	
	public double getRightSensorVelocity() {
		return rightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	public double getRightEncoderPosition() {
		return -rightMasterMtr.getSensorCollection().getQuadraturePosition();
	}
	
	public double getRightEncoderVelocity(){
		return rightMasterMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	public double getRightMstrMtrCurrent() {
		return rightMasterMtr.getOutputCurrent();
	}

	public double getRightFollowerMtrCurrent() {
		return rightFollowerMtr.getOutputCurrent();
	}

	public double getRightCloseLoopError() {
		return rightMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	// LEFT MASTER MOTOR ==============================
	
	public double getLeftSensorPosition(){
		double sign = (SRXDriveBaseCfg.isLeftEncoderSensorReadReversed)? -1 : 1;
		return (sign * leftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx));	
	}
	
	
	public double getLeftSensorVelocity() {
		return leftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	public double getLeftEncoderPosition() {
		// This value is updated every 160ms
		return leftMasterMtr.getSensorCollection().getQuadraturePosition();
	}
	
	public double getLeftEncoderVelocity(){
		// This value is updated every 160ms
		return leftMasterMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	public double getLeftMstrMtrCurrent() {
		return leftMasterMtr.getOutputCurrent();
	}
	
	public double getLeftFollowerMtrCurrent() {
		return leftFollowerMtr.getOutputCurrent();
	}
	
	public double getLeftCloseLoopError() {
		return leftMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx);
	}

	public double getBusVoltage() {
		return leftMasterMtr.getBusVoltage();
	}

	public double getEncoderInchesPerCount(){
		return SRXDriveBaseCfg.kInchesPerCount;
	}
	public double getRobotTrackWidth(){
		return SRXDriveBaseCfg.kTrackWidthIn;
	}

	public boolean getIsDriveMoving() {
		if ((Math.abs(leftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx)) > 0.1) ||
			(Math.abs(rightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx)) > 0.1))
		{
			isDriveTrainMoving = true;
		} else {
			isDriveTrainMoving = false;
		}
		return isDriveTrainMoving;
	}
	
	// ======================================================================================
	// =======================================================================================
	// STATUS-DATA METHODS
	// =======================================================================================
	// ======================================================================================
	

	// Reads encoder, velocity, current, error, and displays on smartdashboard
	public void smartDashboardDriveBaseData() {

		// Display SRX module values
		SmartDashboard.putNumber("BaseDrive-Right Bus Voltage", rightMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive-Right Output Voltage", rightMasterMtr.getMotorOutputVoltage());
		SmartDashboard.putNumber("BaseDrive- Right Master Current", rightMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive- Right Follower Current", rightFollowerMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive-Left Bus Voltage", leftMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive-Left Output Voltage", leftMasterMtr.getMotorOutputVoltage());
		SmartDashboard.putNumber("BaseDrive- Left Master Current", leftMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive- Left Follower Current", rightFollowerMtr.getOutputCurrent());

		if (SRXDriveBaseCfg.isMasterEncodersPresent) {
			SmartDashboard.putNumber("BaseDrive-Right Position", rightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Right Velocity ", rightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Left Position", leftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Left Velocity ", leftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx));
		}

		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			SmartDashboard.putNumber("BaseDrive-Speed Right ClosedLoopErr",	rightMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Speed Left ClosedLoopErr", leftMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx));
		}

	}

	public void logSRXDriveData(){
		if (isLoggingEnabled){
			if(!islogSRXDriveDataActive){

			// log data header once
			islogSRXDriveDataActive = true;
			msg("Right Bus Voltage,Right Output Voltage,Right Master Current,Right Encoder Count,Right Follower Current," +
				 "Left Bus Voltage,Left Output Voltage,Left Master Current,Left Encoder Count,Left Follower Current");
			} else {
				msg(String.format(",%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f", 
									rightMasterMtr.getBusVoltage(), 
									rightMasterMtr.getMotorOutputVoltage(),
									rightMasterMtr.getOutputCurrent(),
									rightMasterMtr.getRightSensorVelocity(),
									rightFollowerMtr.getOutputCurrent(),
									leftMasterMtr.getBusVoltage(),
									leftMasterMtr.getMotorOutputVoltage(),
									leftMasterMtr.getOutputCurrent(),
									leftMasterMtr.getLeftSensorVelocity(),
									leftFollowerMtr.getOutputCurrent()));
			}
		}
	} 
	
	private void msg(String _msgString){
		if(isLoggingEnabled){
			log.write(_msgString);
		} else if (_msgString != lastMsgString){
			if(isConsoleEnabled){
				System.out.println(_msgString);
			}	
			lastMsgString = _msgString;}
		}
		
	// =======================================================================================
	// =======================================================================================
	// TELEOP METHODS
	// =======================================================================================
	// =======================================================================================
	
	
	//
	// NOTE: Motion command with open loop reflect power levels (-1 to 1) * (the motor bus voltage). 
	// Motion command with closed loop reflect speed level => (-1 to 1) * (top motor RPM)
	//
	
	public void SetDriveTrainCmdLevel(double _rightCMDLevel, double _leftCMDLevel) {
		rightCmdLevel = _rightCMDLevel;
		leftCmdLevel = _leftCMDLevel;
		
		if(isMotorEncoderTestEnabled) {
			// +++++++++++++++++++++++++++++++++
			// DATA DISPLAY FOR CHECKING ENCODERS AND ENCODER DIRECTION
			System.out.printf("REnc:%-4.0f =RESen:%-4.0f =RESenRd:%-4.0f =LEnc:%-4.0f =LESen:%-4.0f =LESenRd:%-4.0f%n", 
								rightMasterMtr.getSensorCollection().getQuadraturePosition(),
								rightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx),
								getRightSensorPosition(),
								
								leftMasterMtr.getSensorCollection().getQuadraturePosition(),
								leftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx),
								getLeftSensorPosition());
		}

		if(isStepTestEnabled) {
			if(rightCmdLevel>0){
				stepFunctionSpeed = rightCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits;
			} else {
				stepFunctionSpeed = leftCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits;
			}
			// +++++++++++++++++++++++++++++++++++++
			// Display data
			System.out.printf("StepVel:%-8.3f==RightVel:%-8.2f==RightErr:%-8.2f==LeftVel:%-8.2f==LeftErr:%-8.2f%n",
								stepFunctionSpeed,
								getRightSensorVelocity(),
								getRightCloseLoopError(),
								getLeftSensorVelocity(),
								getLeftCloseLoopError());
		}

		// Switch follower motors to move sideways
		if (isMecanumShiftEnabled) {
			rightFollowerMtr.set(ControlMode.Follower, driveLefttMasterMtr.getDeviceID());
			leftFollowerMtr.set(ControlMode.Follower, rightMasterMtr.getDeviceID());
		} else {
			rightFollowerMtr.set(ControlMode.Follower, rightMasterMtr.getDeviceID());
			leftFollowerMtr.set(ControlMode.Follower, leftMasterMtr.getDeviceID());
		}
		// Check for incorrect encoder input phase
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			rightMasterMtr.getFaults(rightFaults);
			leftMasterMtr.getFaults(leftFaults);
			If (rightFaults.SensorOutOfPhase || leftFaults.SensorOutOfPhase) {

				//Fault detected - change control mode to open loop percent output and stop motor
				stopMotors();
				rightMasterMtr.set(ControlMode.PercentOutput,0);
				leftMasterMtr.set(ControlMode.PercentOutput,0);
			} else {

				// Output commands to SRX modules set as [% from (-1 to 1)] * MaxVel_VelNativeUnits
				rightMasterMtr.set(ControlMode.Velocity, (rightCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
				leftMasterMtr.set(ControlMode.Velocity, (leftCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
			}	
		} else {
			rightMasterMtr.set(ControlMode.PercentOutput,rightCmdLevel);
			leftMasterMtr.set(ControlMode.PercentOutput,leftCmdLevel);
		}
	}

	

	// ======================================
	// SET THROTTLE-TURN
	// ======================================
	
	public void setThrottleTurn(double _throttleValue, double _turnValue, double _headingCorrection) {

			// Calculate cmd level in terms of PercentVbus; range (-1 to 1)
			leftCmdLevel = _throttleValue + (_turnValue/2) + (_headingCorrection/2);
			rightCmdLevel = ((_throttleValue * kDriveStraightFwdCorrection) - 
									(_turnValue/2)) - (_headingCorrection/2);
		
		// Output commands to SRX modules set as [% from (-1 to 1)]
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//++++++++++++++++++++++++++++++++++++++
		// Display data
		if (isConsoleEnabled || isLoggingEnabled){
			msg(String.format("LftCmd:,%-4.3f,=RgtCmd:,%-4.3f,=LftVel:,%-5.2f,=RgtVel:,%-5.2f,=LftCur:,%-5.2f,=RgtCur:,%-5.2f %n", 
									leftCmdLevel, 
									rightCmdLevel,
									getLeftSensorVelocity(),
									getRightSensorVelocity(),
									getLeftMstrMtrCurrent(),
									getRightMstrMtrCurrent()));
		}
	}

	

	// ======================================================================================
	// =======================================================================================
	// AUTONOMOUS METHODS
	// =======================================================================================
	// ======================================================================================
	
	public boolean move(double _MoveDistanceIn, double _MovePwrlevel) {

		rightSensorPositionRead = getRightSensorPosition();
		leftSensorPositionRead = getLeftSensorPosition();

		if(!isStdTrapezoidalMoveActive){
			msg("START MOTION CALCULATIONS ==================================");
			isStdTrapezoidalMoveActive = true;
			LeftDistanceCnts = (int)(_MoveDistanceIn / SRXDriveBaseCfg.kInchesPerCount);
			LeftCruiseVelNativeUnits = (int)(_MovePwrlevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits);
			LeftMoveTimeSec = (int)(1.5 * (LeftDistanceCnts / LeftCruiseVelNativeUnits)); 
			LeftAccelNativeUnits = (int)(LeftCruiseVelNativeUnits/(LeftMoveTimeSec/kStdAccelTimeSegment));
			
			RightDistanceCnts = LeftDistanceCnts;
			RightCruiseVelNativeUnits = LeftCruiseVelNativeUnits;
			RightAccelNativeUnits = LeftAccelNativeUnits;
			encoderHeadingDeg = (leftSensorPositionRead - rightSensorPositionRead) / SRXDriveBaseCfg.kTrackWidthIn;
			
			if (isConsoleEnabled || isLoggingEnabled){
				msg(String.format("RgtD:,%-8.2f, RgtV:,%-8d, RgtA:,%-8d, LftD:,%-8.2f, LftV:,%-8d, LftA:,%-8d %n", 
						RightDistanceCnts, 
						RightCruiseVelNativeUnits,
						RightAccelNativeUnits,
						LeftDistanceCnts,
						LeftCruiseVelNativeUnits,
						LeftAccelNativeUnits));
			}
			// todo - added else to print out encoders and heading
		} else if(!SRXBaseMove(RightCruiseVelNativeUnits, 
							   RightAccelNativeUnits, 
							   RightDistanceCnts, 
							   LeftCruiseVelNativeUnits, 
							   LeftAccelNativeUnits, 
							   LeftDistanceCnts,
							   (LeftMoveTimeSec + 1))){

			isStdTrapezoidalMoveActive = false;
			msg("END MOVE================");
		}
			
		return isStdTrapezoidalMoveActive;
		}
	}
	// todo - complete move overload for mecanum shift
	public boolean move(double _MoveDistanceIn, double _MovePwrLevel, boolean _MoveSideways){

	}
	public boolean rotate(double _RotateAngleDeg, double _RotatePwrLevel) {
		if(!isStdTrapezoidalRotateActive) {
			msg("START ROTATE CALCULATIONS ==================================");
			isStdTrapezoidalRotateActive = true;
			
			// rotationEncoderStopCount = C(=>PI*D) * (angle as a fraction of C)			                                
			LeftDistanceCnts = (int)(Math.PI * (SRXDriveBaseCfg.kTrackWidthIn) * SRXDriveBaseCfg.kEncoderCountsPerIn * (_RotateAngleDeg / 360));
			LeftCruiseVelNativeUnits = (int)(_RotatePwrLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits);
			LeftRotateTimeSec = (int)(1.5 * (LeftDistanceCnts / LeftCruiseVelNativeUnits)); 
			LeftAccelNativeUnits = (int)(LeftCruiseVelNativeUnits/(LeftRotateTimeSec/kStdAccelTimeSegment));
		
			RightDistanceCnts = -LeftDistanceCnts;
			RightCruiseVelNativeUnits = -LeftCruiseVelNativeUnits;
			RightAccelNativeUnits = -LeftAccelNativeUnits;
			
			if (isConsoleEnabled || isLoggingEnabled){
				msg(String.format("RgtD:,%-8.2f, ++RgtV:,%-8d, ++RgtA:,%-8d, ++LftD:,%-8.2f, ++LftV:,%-8d, ++LftA:,%-8d %n", 
						RightDistanceCnts, 
						RightCruiseVelNativeUnits,
						RightAccelNativeUnits,
						LeftDistanceCnts,
						LeftCruiseVelNativeUnits,
						LeftAccelNativeUnits));
			}
		// todo - add else to print out encoders and encoder heading
		} else if(!SRXBaseMove(RightCruiseVelNativeUnits, 
							   RightAccelNativeUnits, 
							   RightDistanceCnts, 
							   LeftCruiseVelNativeUnits,	
							   LeftAccelNativeUnits, 
							   LeftDistanceCnts,
							   (LeftRotateTimeSec + 1)){
			isStdTrapezoidalRotateActive = false;
			msg("END ROTATE MOVE================");
		}
			
		return isStdTrapezoidalRotateActive;
	}
	
	// This method performs a SRX magic motion command from user calculated values
	// All SRXBaseMove parms are in native units 
	
	public boolean SRXBaseMove(int    _rightCruiseVel, 
						   int    _rightAccel, 
						   double _rightDistance, 
						   int    _leftCruiseVel,	
						   int    _leftAccel, 
						   double _leftDistance,
						   double _indexFaultTimeSec) {
		
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		
		if (!isSRXMoveActive) {
			isSRXMoveActive = true;
			msg("START SRX MOTION ==================================");

			/* Set relevant frame periods to be at least as fast as periodic rate*/
			rightMasterMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, SRXTimeoutValueMs);
			rightMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			rightMasterMtr.setSelectedSensorPosition(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx, SRXTimeoutValueMs);
			rightMasterMtr.configMotionCruiseVelocity(_rightCruiseVel, SRXTimeoutValueMs);
			rightMasterMtr.configMotionAcceleration(_rightAccel, SRXTimeoutValueMs);

			leftMasterMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, SRXTimeoutValueMs);
			leftMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);	
			leftMasterMtr.setSelectedSensorPosition(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx, SRXTimeoutValueMs);
			leftMasterMtr.configMotionCruiseVelocity(_leftCruiseVel, SRXTimeoutValueMs);
			leftMasterMtr.configMotionAcceleration(_leftAccel, SRXTimeoutValueMs);

			
		} else {
			// todo - review collision dectection
			// Check for program errors or end of index
			if (((Math.abs(SRXMotionLeftPos) >= Math.abs(_leftDistance)) 
					&& (Math.abs(SRXMotioRightPos) >= Math.abs(_rightDistance)))
					|| distanceIF.collisionDetected()
			        || ((Timer.getFPGATimestamp() - methodStartTime) > _indexFaultTimeSec)) {
				rightMasterMtr.set(ControlMode.MotionMagic, 0); 
				leftMasterMtr.set(ControlMode.MotionMagic, 0);
				isSRXMoveActive = false;
				msg("END SRX MOTION  ========================");
			} else {
		
				// Output commands to SRX's
				rightFollowerMtr.set(ControlMode.Follower, rightMasterMtr.getDeviceID());
				leftFollowerMtr.set(ControlMode.Follower, leftMasterMtr.getDeviceID());
				
				rightMasterMtr.set(ControlMode.MotionMagic, _rightDistance); 
				leftMasterMtr.set(ControlMode.MotionMagic, _leftDistance);
				
				if (isConsoleEnabled || isLoggingEnabled){
					SRXMotionLeftPos = leftMasterMtr.getActiveTrajectoryPosition();
					SRXMotionLeftVel = leftMasterMtr.getActiveTrajectoryVelocity();
					SRXMotioRightPos = rightMasterMtr.getActiveTrajectoryPosition();
					SRXMotioRightVel = rightMasterMtr.getActiveTrajectoryVelocity();
					msg(String.format("LftEnc:,%-8.0f, ==RgtEnc:,%-8.0f, ==LftSRXVel:,%-8.2f, ==LftSRXPos:,%-8.2f, ==RgtSRXVel:,%-8.2f, ==RgtSRXPos:,%-8.2f %n", 
										leftSensorPositionRead, 
										rightSensorPositionRead,
										SRXMotionLeftVel,
										SRXMotionLeftPos,
										SRXMotioRightVel,
										SRXMotioRightPos));
				}
			}
		}
		return isSRXMoveActive;
	}
	

	// This code is leveraged from CTRE electronics, Team317, and Team3539
	public boolean SRXProfileMove(double[][] _profileRight, double[][] _profileLeft, int _totalPointNum) {
		pointsRight = _profileRight;
		pointsLeft = _profileLeft;
		profileNumPoints = _totalPointNum;

		if(!isSRXProfileMoveActive) {

			// Disable motion profile while setting up a profile move
			rightMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable);
			leftMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable);

			//Clear underrun fault 
			rightMasterMtr.clearMotionProfileHasUnderrun(0);
			leftMasterMtr.clearMotionProfileHasUnderrun(0);

			// todo - set sensor position to zero

			// Clear the buffer just in case profile was disabled in process
			rightMasterMtr.clearMotionProfileTrajectories();
			leftMasterMtr.clearMotionProfileTrajectories();

			// create an empty trajectory point			
			TrajectoryPoint trajectoryPointRight = new TrajectoryPoint();
			TrajectoryPoint trajectoryPointLeft = new TrajectoryPoint();

			// set the "base" trajectory period to zero, use the profile trajectory period in the profile file
			rightMasterMtr.configMotionProfileTrajectoryPeriod(0, SRXTimeoutValueMs);
			leftMasterMtr.configMotionProfileTrajectoryPeriod(0, SRXTimeoutValueMs);

			// set status frame update time-ms
			rightMasterMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, SRXTimeoutValueMs);
			leftMasterMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, SRXTimeoutValueMs);

			//When we do start running set state machine start at the beginning.
			SRXProfileState = 0;

			// Determine profile timeout time
			methodStartTime = Timer.getFPGATimestamp();
			estProfileFaultTimeMs = ((pointsRight[1][2] * profileNumPoints) + 500) + methodStartTime;

			// todo - next two lines ??
			rightMasterMtr.changeMotionControlFramePeriod(5);
			driveLefttMasterMtr.changeMotionControlFramePeriod(5);

			// Create a periodic task tread to funnel our Phoenix Framework API trajectory points into SRXtalon.
			// Generally speaking you want to call it at least twice as fast as the duration
			// of your trajectory points. 
			class PeriodicRunnable implements java.lang.Runnable {
				public void run() {  
					rightMasterMtr.processMotionProfileBuffer(); 
					leftMasterMtr.processMotionProfileBuffer();    
				}
			}
			Notifier pushAPI2SRXThread = new Notifier(new PeriodicRunnable());

		} else {

			//Get the motion profile status every loop
			rightMasterMtr.getMotionProfileStatus(SRXProfileStatusRight);
			driveLefttMasterMtr.getMotionProfileStatus(SRXProfileStatusLeft);

			SRXRightTrajectoryPosition = rightMasterMtr.getActiveTrajectoryPosition();
			SRXRightTrajectoryVelocity = rightMasterMtr.getActiveTrajectoryVelocity(); 
			SRXRightTrajectoryHeading = rightMasterMtr.getActiveTrajectoryHeading();
			SRXRLeftTrajectoryPosition = leftMasterMtr.getActiveTrajectoryPosition();
			SRXLeftTrajectoryVelocity = leftMasterMtr.getActiveTrajectoryVelocity(); 
			SRXLeftTrajectoryHeading = leftMasterMtr.getActiveTrajectoryHeading();

			// Check for errors - program hangup, change of mode, profile underrun	
			if ((Timer.getFPGATimestamp() > estProfileFaultTimeMs) || 
					(rightMasterMtr.getControlMode() != ControlMode.MotionProfile) ||
					SRXProfileStatusRight.hasUnderrun) {

				// Profile program error detected - stop everything
				rightMasterMtr.clearMotionProfileHasUnderrun(0);
				leftMasterMtr.clearMotionProfileHasUnderrun(0);

				rightMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable);
				leftMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable);

				pushAPI2SRXThread.stop();
				
				isSRXProfileMoveActive = false;
				
			} else {
				// State machine in MP control mode to load profile buffer and check for end of profile
				switch (SRXProfileState) {
					case 0: 
						// First load of the profile buffer
						bufferAccumCnt = (profileNumPoints < kSRXBufferSize)?  profileNumPoints-1: kBufferCntMin*2;
						profileIndexPointer = 0;
						fillProfileAPIBuffer(profileIndexPointer, bufferAccumCnt);

						// Want to load SRX buffer 2x faster than SRX profile point execute time(ms)
						API2SRXThreadTimeSec = ((pointsRight[1][2])/2)/1000;
						pushAPI2SRXThread.startPeriodic(API2SRXThreadTimeSec);

						SRXProfileState = 1;
						profileIndexPointer = bufferAccumCnt;
					break;
					case 1: 
						// Wait for MotionProfile API to stream to Talon motionProfile buffer
						if (SRXProfileStatusRight.TopBufferCnt == 0){

							// Start motion profile
							rightMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable);
							lefttMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable);

							// Wait til SRX profile buffer is a low content count then load more trajectory points
							SRXProfileState = 2;
						}
					break;
					case 2:
						// Continue loading SRX buffer if at min trajectory point count
						if ((SRXProfileStatusRight.TopBufferCnt == 0) &&
								(SRXProfileStatusRight.BottomBufferCnt < kBufferCntMin)){
				
							// check if last trajectory point is left to load
							if(bufferAccumCnt == profileNumPoints-1){
								fillProfileAPIBuffer(profileIndexPointer, profileNumPoints);

								// all the profile points have been loaded wait for profile end
								SRXProfileState = 3;

							// load another kBufferCntMin points or what is left less the last trajectory point
							} else if(profileNumPoints-bufferAccumCnt >= kBufferCntMin) {
								profileIndexPointer = bufferAccumCnt;
								bufferAccumCnt += kBufferCntMin;
								fillProfileAPIBuffer(profileIndexPointer, bufferAccumCnt);
								SRXProfileState = 2;	
							} else {
								profileIndexPointer = bufferAccumCnt;
								bufferAccumCnt = profileNumPoints-1;
								fillProfileAPIBuffer(profileIndexPointer, bufferAccumCnt);
								SRXProfileState = 2;	
							}
						}	
					break;
					case 3: 
		
						// Check if profile is done
						if (SRXProfileStatusRight.activePointValid && SRXProfileStatusRight.isLast) {
							
							rightMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold);
							lefttMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold);

							pushAPI2SRXThread.stop();
							
							isSRXProfileMoveActive = false;
						}
					break;
				}
			}	
		}
		return isSRXProfileMoveActive;
	}

	private void fillProfileAPIBuffer(int _profileIndexPtr, int _profileAccumCnt) {

		// for each point, fill trajectory point structure and pass it to API to load Top buffer
		// This will then be loaded into the SRX buffer via Phoenix Framework API "processMotionProfileBuffer()""
		for (int i = _profileIndexPtr; i < _profileAccumCnt; i++) {
			trajectoryPointRight.position = pointsRight[i][0];
			trajectoryPointLeft.position = pointsLeft[i][0];

			trajectoryPointRight.velocity = pointsRight[i][1];
			trajectoryPointLeftt.velocity = pointsLeft[i][1];

			trajectoryPointRight.profileSlotSelect0 = 0; // which set of gains to use
			trajectoryPointLeft.profileSlotSelect0 = 0; // which set of gains to use

			trajectoryPointRight.headingDeg = 0; // future feature - not used in this example
			trajectoryPointLeft.headingDeg = 0; // future feature - not used in this example

			trajectoryPointRight.profileSlotSelect1 = 0; // future feature  - not used in this example
			trajectoryPointLeft.profileSlotSelect1 = 0; // future feature  - not used in this example

			trajectoryPointRight.timeDur = pointsRight[i][2];
			trajectoryPointLeft.timeDur = pointsRight[i][2];

			// Set flag if zero trajectory position
			if ((SRXProfileState == 0) && (i == 0)){
				trajectoryPointRight.zeroPos = true; 
				trajectoryPointLeft.zeroPos = true; 
			} else {
				trajectoryPointRight.zeroPos = false;
				trajectoryPointLeft.zeroPos = false;
			}

			// Set flag if last trajectory position
			if ((SRXProfileState == 3) && (i == profileNumPoints+1)){
				trajectoryPointRight.isLastPoint = true; 
				trajectoryPointLeft.isLastPoint = true; 
			} else {
				trajectoryPointRight.isLastPoint = false;
				trajectoryPointLeft.isLastPoint = false;
			}
			// Push points to profile Phoenix Framework API (TopBuffer)
			rightMasterMtr.pushMotionProfileTrajectory(trajectoryPointRight);
			leftMasterMtr.pushMotionProfileTrajectory(trajectoryPointLeft);
		}
	}
	
}