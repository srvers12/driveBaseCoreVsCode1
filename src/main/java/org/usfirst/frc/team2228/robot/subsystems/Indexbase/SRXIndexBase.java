package org.usfirst.frc.team2228.robot.subsystems.indexbase;
// 
//  Class SRXIndexBase
//  RELEASE: 2019 
//  Team 2228

// REVISION LEVEL:
// 181221 - original

// ===================================
// SET COMMANDS
// ===================================
// public void init(boolean _isConsoleEnabled, boolean _isLoggingEnabled)


// public void setSensorPositionToZero()
// public void setEncPositionToZero()
// public void setBrakeMode(boolean _isBrakeEnabled)
// public void stopMotor()
// public void setIndexRamp(double _SecToMaxPower)

// private void clearSRXindexPrgFlgs()
		
// ===================================
// GET COMMANDS
// ===================================
// public double getSensorPosition()
// public double getSensorVelocity()
// public double getEncoderPosition()
// public double getEncoderVelocity()
// public double getIndexerCurrent()
// public double getCloseLoopError()



// public double getBusVoltage()
// public boolean getIsDriveMoving()

// ====================================
// DATA COMMANDS
//=====================================
// public void smartDashboardDriveBaseData()
// public void logSRXDriveData()



// =====================================
//	INDEX MOTION COMMANDS
// =====================================

// *****
// public void Jog(double _CMDLevel)
// *****
//
// ++++++++++
// public boolean move(double _MoveDistanceIn, 
//					   double _MovePwrlevel)
// 
// @parm  _MoveDistanceIn - move distance in inches
// @parm  _MovePwrlevel - power level 0 - 1, 
// program converts to VelocityNativeUnits as (0 to 1)* maxVelocityNativeUnits
//
// +++++++++
// public boolean SRXIndexmove(int    _CruiseVel, 
//					   		  int    _Accel, 
//					   		  double _Distance, 
//					   		  double _indexFaultTimeSec)
// +++++++++
//

// ++++++++
// public boolean SRXProfileMove(double[][] Profile, 
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

public class SRXIndexBase {
	
	
	// DifferentialDrive or tank motors
	private TalonSRX indexMtr;
	
	private DebugLogger log;
	private RobotMap RobotMap;
	private Faults leftFaults;
	private Notifier pushAPI2SRXThread;
	
	// ===================================
	// SRX MOTION PROFILE DRIVE BASE 

	// The status of the motion profile executer and buffer inside the Talon.
	// Instead of creating a new one every time we call getMotionProfileStatus,
	//  keep one copy.	
	private MotionProfileStatus SRXProfileStatus = new MotionProfileStatus();

	private double[][] points;
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
	private double SRXTrajectoryPosition=0;
	private double SRXTrajectoryVelocity=0; 
	private double SRXTrajectoryHeading=0;
	
	private double estProfileFaultTimeMs = 0;
	private double API2SRXThreadTimeSec = 0;
	private double kIndexTrajPeriodMs = 0;

	private boolean isSRXProfileMoveActive = false;

	// ====================================	
	// DRIVE BASE VARIABLES

	private int SRXTimeoutValueMs = 10;
	private int correctionSensorType = 1;	
	private int CruiseVelNativeUnits = 0;
	private int AccelNativeUnits = 0;
	

	private double kStdAccelTimeSegment = 3;

	private double DistanceCnts = 0;
	private double SRXIndexPos =0;
	private double SRXIndexVel = 0;
	private double indexerCmdLevel = 0;
	private double methodStartTime = 0;
	private double methodTime = 0;
	private double sensorPositionRead = 0;
	private double stepFunctionSpeed = 0;
		
	//  Program flow switches
	private boolean isConsoleEnabled = false;
	private boolean isLoggingEnabled = false;
	private boolean islogSRXIdexDataActive = false;
	private boolean isSRXIndexActive = false;
	private boolean isStdTrapezoidalIndexActive = false;
	private boolean isindexMoving = false;
	private boolean isTestEnabled = false;
	private boolean isStepTestEnabled = false;
	private boolean isMotorEncoderTestEnabled = false;

	//private String logSRXDriveString = " ";
	private String lastMsgString = " ";
	
	// SRXDriveBase Class Constructor
	public SRXIndexBase(RobotMap _robotMap, DebugLogger _logger) {
	
		log = _logger;
		RobotMap = _robotMap;
		
		// Create CAN SRX motor controller objects
		indexMtr = new TalonSRX(kindexMtrCANid);

		IndexerFaults = new Faults();

		
		// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// RIGHT MOTORS===========================================================
		// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Set min/max output
		indexMtr.configNominalOutputForward(0.0, SRXTimeoutValueMs);
		indexMtr.configNominalOutputReverse(0.0, SRXTimeoutValueMs);
		indexMtr.configPeakOutputForward(1, SRXTimeoutValueMs);
		indexMtr.configPeakOutputReverse(-1, SRXTimeoutValueMs);
		
		// Reverse motor if necessary
		indexMtr.setInverted(SRXIndexCfg.isDriveMasterMtrReversed);
		
		// Configure voltage compensation mode and set max voltage to 11 volts
		indexMtr.configVoltageCompSaturation(11.0, SRXTimeoutValueMs);

		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		indexMtr.configVoltageMeasurementFilter(32, SRXTimeoutValueMs);
		indexMtr.enableVoltageCompensation(true);
		
		// set output zero (neutral) deadband at 4%
		indexMtr.configNeutralDeadband(0.04, SRXTimeoutValueMs);

		// Set up stall conditions in SRX for the drive train
		indexMtr.configPeakCurrentLimit(SRXIndexCfg.kStallCurrentPeakAmps, SRXTimeoutValueMs);
		indexMtr.configPeakCurrentDuration(SRXIndexCfg.kStallTimeMs, SRXTimeoutValueMs);
		indexMtr.configContinuousCurrentLimit(SRXIndexCfg.kStallCurrentContinuousAmps, SRXTimeoutValueMs);
		indexMtr.enableCurrentLimit(true);
		
		// Configure the velocity measurement period and sample window rolling average
		// Sample period in ms from supported sample periods-default 100ms period/64 sample window
		indexMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms,SRXTimeoutValueMs);
		indexMtr.configVelocityMeasurementWindow(SRXIndexCfg.kSRXVelocitySample,SRXTimeoutValueMs);
		
		// Set up encoder input
		indexMtr.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, SRXIndexCfg.kslotIDx, SRXTimeoutValueMs);
		indexMtr.setSensorPhase(SRXIndexCfg.isEncoderSensorReversed);
		
		// Clear quadrature position
		indexMtr.clearStickyFaults(SRXTimeoutValueMs);
		indexMtr.getSensorCollection().setQuadraturePosition(0, SRXTimeoutValueMs);
		
	
		
		// zero control output
		indexMtr.setNeutralOutput();

		/*
		 * Set Brake-Coast mode to coast
		 */
		setBrakeMode(SRXIndexCfg.isBrakeEnabled);

		
		
		// set timeout to zero to stop waiting for confirmations
		SRXTimeoutValueMs = 0;
	}
	// END OF CONSTRUCTOR
	
	// ======================================================================================
	// =======================================================================================
	// SRXINDEX SET/CONFIG METHODS
	// =======================================================================================
	// ======================================================================================
	public void setSensorPositionToZero() {
		// SRX API Commands are executed every 10ms
		indexMtr.setSelectedSensorPosition(0, SRXIndexCfg.kPIDLoopIDx, 25);
	}
		
	public void setEncPositionToZero() {
		// SRX API Commands are executed every 10ms
		indexMtr.getSensorCollection().setQuadraturePosition(0, 15);
	}
		
	public void setBrakeMode(boolean _isBrakeEnabled) {
		indexMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		
	}

	public void setIndexRamp(double _SecToMaxPower){
		if(SRXIndexCfg.isSRXClosedLoopEnabled){
			indexMtr.configClosedloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
		} else {
			indexMtr.configOpenloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
		}
	}

	public void stopMotor() {
		indexMtr.setNeutralOutput();
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

	
	public void init(boolean _isConsoleEnabled, boolean _isLoggingEnabled) {

		// Set class program switches
		isConsoleEnabled = _isConsoleEnabled? true : false;
		isLoggingEnabled = _isLoggingEnabled? true : false;

		// Clear SRXDriveBase program control flags
		clearSRXIndexPrgFlgs();
		
		// Load smart dashboard and shuffle board parameters
		smartDashboardIndexData();
		
		// Stop motors and clear position counters
		stopMotor();
		setIndexRamp(0);
		setSensorPositionToZero();

		// Turn on motor safety - if no commands to SRX in 100ms, SRX's  will shut down
		indexMtr.setSafetyEnabled(true);
		
		
		// Load drive train PID values
		if (SRXIndexCfg.isSRXClosedLoopEnabled) {
			indexMtr.selectProfileSlot(SRXIndexCfg.kslotIDx, SRXIndexCfg.kPIDLoopIDx);
			indexMtr.configAllowableClosedloopError(SRXIndexCfg.kPIDLoopIDx, SRXIndexCfg.kClosedLoopErr, SRXTimeoutValueMs);
			indexMtr.config_kF(SRXIndexCfg.kPIDLoopIDx, SRXIndexCfg.kIndexFeedForwardGain, SRXTimeoutValueMs);
			indexMtr.config_kP(SRXIndexCfg.kPIDLoopIDx, SRXIndexCfg.kindexProportionalGain, SRXTimeoutValueMs);
			indexMtr.config_kI(SRXIndexCfg.kPIDLoopIDx, SRXIndexCfg.kindexIntegralGain, SRXTimeoutValueMs); 
			indexMtr.config_kD(SRXIndexCfg.kPIDLoopIDx, SRXIndexCfg.kIndexDerivativeGain, SRXTimeoutValueMs);
			indexMtr.config_IntegralZone(SRXIndexCfg.kPIDLoopIDx, SRXIndexCfg.kIndexIzone, SRXTimeoutValueMs);
		
			
		}
		
	}
	
	// Clear all program control flags
	private void clearIndexPrgFlgs() {
		isStdTrapezoidalIndexActive = false;
		isSRXindexActive = false;
		islogSRXIndexDataActive = false;
		isSRXProfileIndexActive = false;
		isTestEnabled = false;
	}
	
	
	
	// =======================================================================================
	// =======================================================================================
	// SRXINDEX GET METHODS
	// =======================================================================================
	//=======================================================================================
	
	// RIGHT MASTER MOTOR ==============
	
	public double getSensorPosition(){
		double sign = (SRXIndexCfg.isEncoderSensorReadReversed)? -1 : 1;
		return (sign * indexMtr.getSelectedSensorPosition(SRXIndexCfg.kPIDLoopIDx));
	}
	
	public double getSensorVelocity() {
		return indexMtr.getSelectedSensorVelocity(SRXIndexCfg.kPIDLoopIDx);
	}
	
	public double getEncoderPosition() {
		return -indexMtr.getSensorCollection().getQuadraturePosition();
	}
	
	public double getEncoderVelocity(){
		return indexMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	public double getIndexMtrCurrent() {
		return indexMtr.getOutputCurrent();
	}

	public double getCloseLoopError() {
		return indexMtr.getClosedLoopError(SRXIndexCfg.kPIDLoopIDx);
	}
	
	public double getBusVoltage() {
		return indexMtr.getBusVoltage();
	}

	public double getUintsPerCount(){
		return SRXIndexCfg.kInchesPerCount;
	}

	public boolean getIsIndexMoving() {
	if (Math.abs(indexMtr.getSelectedSensorVelocity(SRXIndexCfg.kPIDLoopIDx)) > 0.1) {
			isIndexMoving = true;
		} else {
			isIndexMoving = false;
		}
		return isIndexMoving;
	}
	
	// ======================================================================================
	// =======================================================================================
	// STATUS-DATA METHODS
	// =======================================================================================
	// ======================================================================================
	

	// Reads encoder, velocity, current, error, and displays on smartdashboard
	public void smartDashboardDriveBaseData() {

		// Display SRX module values
		SmartDashboard.putNumber("BaseDrive- Bus Voltage", indexMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive- Output Voltage", indexMtr.getMotorOutputVoltage());
		SmartDashboard.putNumber("BaseDrive-  Master Current", indexMtr.getOutputCurrent());
		

		if (SRXIndexCfg.isMasterEncodersPresent) {
			SmartDashboard.putNumber("BaseDrive- Position", indexMtr.getSelectedSensorPosition(SRXIndexCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive- Velocity ", indexMtr.getSelectedSensorVelocity(SRXIndexCfg.kPIDLoopIDx));
			
		}

		if (SRXIndexCfg.isSRXClosedLoopEnabled) {
			SmartDashboard.putNumber("BaseDrive-Speed  ClosedLoopErr",	indexMtr.getClosedLoopError(SRXIndexCfg.kPIDLoopIDx));
		}

	}

	public void logSRXDriveData(){
		if (isLoggingEnabled){
			if(!islogSRXIndexDataActive){

			// log data header once
			islogSRXIndexDataActive = true;
			msg(" Bus Voltage, Output Voltage, Master Current, Encoder Count, Follower Current,");
			} else {
				msg(String.format(",%8.2f,%8.2f,%8.2f,%8.2f", 
									indexMtr.getBusVoltage(), 
									indexMtr.getMotorOutputVoltage(),
									indexMtr.getOutputCurrent(),
									indexMtr.getSensorVelocity()
									));
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
	// INDEX METHODS
	// =======================================================================================
	// =======================================================================================
	//
	// NOTE: Motion command with open loop reflect power levels (-1 to 1) * (the motor bus voltage). 
	// Motion command with closed loop reflect speed level => (-1 to 1) * (top motor RPM)
	//
	
	public void jog(double _CMDLevel) {
		CmdLevel = _CMDLevel;
		
		if(isMotorEncoderTestEnabled) {
			// +++++++++++++++++++++++++++++++++
			// DATA DISPLAY FOR CHECKING ENCODERS AND ENCODER DIRECTION
			System.out.printf("Enc:%-4.0f =Sen:%-4.0f =SenRd:%-4.0f%n", 
								indexMtr.getSensorCollection().getQuadraturePosition(),
								indexMtr.getSelectedSensorPosition(SRXIndexCfg.kPIDLoopIDx),
								getSensorPosition());
								
								
		}

		if(isStepTestEnabled) {
			if(rightCmdLevel>0){
				stepFunctionSpeed = CmdLevel * SRXIndexCfg.MaxVel_VelNativeUnits;
			} else {
				stepFunctionSpeed = CmdLevel * SRXIndexCfg.MaxVel_VelNativeUnits;
			}
			// +++++++++++++++++++++++++++++++++++++
			// Display data
			System.out.printf("StepVel:%-8.3f==Vel:%-8.2f==Err:%-8.2fn",
								stepFunctionSpeed,
								getSensorVelocity(),
								getCloseLoopError());
								
		}

		
		// Check for incorrect encoder input phase
		if (SRXIndexCfg.isSRXClosedLoopEnabled) {
			indexMtr.getFaults(rightFaults);
			leftMasterMtr.getFaults(leftFaults);
			If (IndexFaults.SensorOutOfPhase) {

				//Fault detected - change control mode to open loop percent output and stop motor
				stopMotor();
				indexMtr.set(ControlMode.PercentOutput,0);
			} else {

				// Output commands to SRX modules set as [% from (-1 to 1)] * MaxVel_VelNativeUnits
				indexMtr.set(ControlMode.Velocity, (CmdLevel * SRXIndexCfg.MaxVel_VelNativeUnits ));
			}	
		} else {
			indexMtr.set(ControlMode.PercentOutput,CmdLevel);
		}
	}
	
	public boolean move(double _indexDistanceCnts, double _indexPwrlevel) {

		SensorPositionRead = getSensorPosition();

		if(!isStdTrapezoidalIndexActive){
			msg("START MOTION CALCULATIONS ==================================");
			isStdTrapezoidalIndexActive = true;
			DistanceCnts = (int)(_indexDistanceCnts);
			CruiseVelNativeUnits = (int)(_indexPwrlevel * SRXIndexCfg.MaxVel_VelNativeUnits);
			MoveTimeSec = (int)(1.5 * (DistanceCnts / CruiseVelNativeUnits)); 
			AccelNativeUnits = (int)(CruiseVelNativeUnits/(MoveTimeSec/kStdAccelTimeSegment));
			
			
			
			if (isConsoleEnabled || isLoggingEnabled){
				msg(String.format("D:,%-8.2f, V:,%-8d, A:,%-8d%n", 
						DistanceCnts, 
						CruiseVelNativeUnits,
						AccelNativeUnits));
			}
			// todo - added else to print out encoders and heading
		} else if(!SRXIndexMove(CruiseVelNativeUnits, 
							   AccelNativeUnits, 
							   DistanceCnts)){

			isStdTrapezoidalIndexActive = false;
			msg("END MOVE================");
		}
			
		return isStdTrapezoidalIndexActive;
		
	}
	
	
	// This method performs a SRX magic motion command from user calculated values
	// All SRXBaseMove parms are in native units 
	
	public boolean SRXIndexMove(int    _CruiseVel, 
						        int    _Accel, 
						        double _Distance, 
						        double _indexFaultTimeSec) {
		
		SensorPositionRead = getSensorPosition();
		
		if (!isSRXindexActive) {
			isSRXIndexActive = true;
			msg("START SRX MOTION ==================================");

			/* Set relevant frame periods to be at least as fast as periodic rate*/
			indexMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, SRXTimeoutValueMs);
			indexMtr.selectProfileSlot(SRXIndexCfg.kslotIDx, SRXIndexCfg.kPIDLoopIDx);
			indexMtr.setSelectedSensorPosition(SRXIndexCfg.kslotIDx, SRXIndexCfg.kPIDLoopIDx, SRXTimeoutValueMs);
			indexMtr.configMotionCruiseVelocity(_CruiseVel, SRXTimeoutValueMs);
			indexMtr.configMotionAcceleration(_Accel, SRXTimeoutValueMs);
		} else {
			// Check for program errors or end of index
			if (((Math.abs(SRXIndexPos) >= Math.abs(_DistanceCnts)) || ((Timer.getFPGATimestamp() - methodStartTime) > _indexFaultTimeSec) {
				indexMtr.set(ControlMode.MotionMagic, 0); 
				isSRXIndexActive = false;
				msg("END SRX MOTION  ========================");
			} else {
		
				indexMtr.set(ControlMode.MotionMagic, _Distance); 
				
				if (isConsoleEnabled || isLoggingEnabled){
					
					SRXIndexPos = indexMtr.getActiveTrajectoryPosition();
					SRXIndexVel = indexMtr.getActiveTrajectoryVelocity();
					msg(String.format("Enc:,%-8.0f, ==Vel:,%-8.0f, ==Pos:,%-8.2f%n", 
										SensorPositionRead,
										SRXIndexVel,
										SRXIndexPos));
				}
			}
		}
		return isSRXindexActive;
	}
	

}