package org.usfirst.frc.team2228.robot.test;

import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBase;
import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBaseCfg;
import org.usfirst.frc.team2228.robot.util.DebugLogger;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

// REVISITONS
// 
// 181204 - reorganized test selection process

// =========================
// SET METHODS
// public void init(boolean _isConsoleEnabled)

// =========================
// METHODS
//public void testMethodSelection()


public class SRXDriveTestBase {

	// objects
	private DebugLogger log;
	private SRXDriveBase driveBase;
	private SRXDriveBaseCfg SRXdriveBaseCfg; 

	private int testSelectionState = 0;

    private double Tst_kDriveStraightFwdCorrection = SRXDriveBaseCfg.kDriveStraightFwdCorrection;
	private double Tst_kDriveStraightRevCorrection = SRXDriveBaseCfg.kDriveStraightRevCorrection;
	private double Tst_RightDriveCmdLevel = 0;
	private double Tst_LeftDriveCmdLevel = 0;
	private double CAL_Throttle = 0;
	private double CAL_turn = 0;
	private double methodStartTime = 0;
	private double delayStartTime = 0;
	private double leftSensorPositionRead = 0;
	private double rightSensorPositionRead = 0;
	private double leftEncoderStopCount = 0;
	private double calCorrectionFactor = 0;
	private double headingDeg = 0;
	private double methodTime = 0;
	
	private boolean isMtrTestBtnActive = false;
	private boolean isStepFncTestBtnActive = false;
	private boolean isTestStepFunctionActive = false;
	private boolean isTestMoveStraightCalActive = false;
	private boolean isTestMethodSelectionActive = false;
	private boolean isConsoleEnabled = false;
	private boolean isDelayActive = false;

	private String lastMsgString = " ";

    // =====================
    // Constructor
    public SRXDriveTestBase(SRXDriveBase _driveBase, DebugLogger _logger){
		driveBase = _driveBase;
		log = _logger;
    }

    public void init(boolean _isConsoleEnabled){
		isConsoleEnabled = _isConsoleEnabled? true : false;

		isMtrTestBtnActive = false;
		isTestStepFunctionActive = false;
		isTestMoveStraightCalActive = false;
		isTestMethodSelectionActive = false;
		
        // Drivebase setup method calls
        SmartDashboard.putBoolean("drvB-TstBtn-StepFnc:", false);
		SmartDashboard.putBoolean("drvB-TstBtn-DrvStraightCal:", false);
		SmartDashboard.putBoolean("drvB-TstBtn-MotorEncoderTest:", false);

		// DriveBase autonomous motion method calls
		SmartDashboard.putBoolean("drvB-TstBtn-MoveToPos:", false);
		SmartDashboard.putBoolean("drvB-TstBtn-RotateToAngle:", false);
		SmartDashboard.putBoolean("drvB-TstBtn-ProfileMove:", false);

		SmartDashboard.putBoolean("drvB-TstBtn-EnableSRXDriveBaseConsoleDisplay:", isConsoleEnabled);
		
		SmartDashboard.putNumber("drvB-kDriveStraightFwdCorrection:", drvBtst-kDriveStraightFwdCorrection);
		SmartDashboard.putNumber("drvB-kDriveStraightRevCorrection:", drvBtst-kDriveStraightRevCorrection);
		
		SmartDashboard.putNumber("drvB-Tst_RightDriveCmdLevel:", Tst_RightDriveCmdLevel);
		SmartDashboard.putNumber("drvB-Tst_LeftDriveCmdLevel:", Tst_LeftDriveCmdLevel);
		
    }

	
	//==============================
	// TEST METHOD SELECTION
	//==============================
	public void testMethodSelection(){
		switch(testSelectionState) {
			case 0:
				if (SmartDashboard.getBoolean("drvB-TstBtn-MotorEncoderTest:", false)){
					motorEncoderTest();
				} else {
					testSelectionState = 1;
				}
				break;
			case 1:
				if(SmartDashboard.getBoolean("drvB-TstBtn-StepFnc:", true)){
					// public void testStepFunction(double _stepFunctionPower, double _stepFnctHighTimeSec, boolean _isTestForRightDrive)
					stepFunctionTest(.3,3,true);
				} else {
					testSelectionState = 2;
				}
			break;
			case 2:
				if(SmartDashboard.getBoolean("drvB-TstBtn-DrvStraightCal:", true)){
					// public boolean testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel){
					driveStraightCorrectiontest(50,.3);
				} else {
					testSelectionState = 3;
				}
			break;
			case 3:
				if(SmartDashboard.getBoolean("drvB-TstBtn-MoveToPos:", true)){
					// public boolean move(double _MoveDistanceIn, double _MovePwrlevel) {
						driveBase.move(30, .3);
				} else {
					testSelectionState = 4;
				}
			break;
			case 4:
				if(SmartDashboard.getBoolean("drvB-TstBtn-MoveSideways:", true)){
					// public boolean move(double _MoveDistanceIn, double _MovePwrLevel, boolean _MoveSideways){
					driveBase.move(30, .3, true);
				} else {
					testSelectionState = 5;
				}
			break;

			case 5:
				if(SmartDashboard.getBoolean("TstBtn-RotateToAngle:", false)){
					// public boolean rotate(double _RotateAngleDeg, double _RotatePwrLevel) {
					driveBase.rotate(45, .3);
				} else {
					testSelectionState = 0;
				}
			break;
			default:
				testSelectionState = 0;
			break;
		}
	}
		
	//=====================
	// MOTOR ENCODER TEST
	// ====================
	// Run motors to in open loop to test encoder directions
	private void motorEncoderTest(){
		if(!isMtrTestBtnActive){
			msg("START MOTOR ENCODER TEST=============");
			isMtrTestBtnActive = true;
			
			// clear encoder registers
			driveBase.setRightEncPositionToZero();
			driveBase.setRightSensorPositionToZero();
			driveBase.setLeftEncPositionToZero();
			driveBase.setLeftSensorPositionToZero();
			driveBase.setTestEnable(1);
		} else  {
			driveBase.SetDriveTrainCmdLevel(SmartDashboard.getNumber("drvB-Tst_RightDriveCmdLevel:", Tst_RightDriveCmdLevel), 
											SmartDashboard.getNumber("drvB-Tst_LeftDriveCmdLevel:", Tst_LeftDriveCmdLevel));

			if(!SmartDashboard.getBoolean("drvB-TstBtn-MotorEncoderTest:", false)) {
				msg("END MOTOR ENCODER TEST=============");
				isMtrTestBtnActive = false;
				driveBase.stopMotors();
				driveBase.setTestEnable(0);
			}
		}
	}	

	//===============================
	// TEST STEP FUNCTION
	//===============================
	// This provides a pulse(low-High-low) and stays in lowpower. Need to stop motors or call constantly to produce a square wave
	private void stepFunctionTest(double _stepFunctionPower, double _stepFnctHighTimeSec, boolean _isTestForRightDrive) {
		if(!isStepFncTestBtnActive &&  SRXDriveBaseCfg.isSRXClosedLoopEnabled){
			msg("SHUFFLE START TEST STEP FUNCTION=============");
			isStepFncTestBtnActive = true;

			driveBase.setRightSensorPositionToZero();
			driveBase.setLeftSensorPositionToZero();
			Timer.delay(0.2);

			if(_isTestForRightDrive){
				Tst_RightDriveCmdLevel = _stepFunctionPower;
				Tst_LeftDriveCmdLevel = 0;
			} else {
				Tst_RightDriveCmdLevel = 0;
				Tst_LeftDriveCmdLevel = _stepFunctionPower;
			}
			
			driveBase.SetDriveTrainCmdLevel( Tst_RightDriveCmdLevel, Tst_LeftDriveCmdLevel);
					
		} else if(!delay(_stepFnctHighTimeSec)) {
			driveBase.stopMotors();
			msg("TEST STEP FUNCTION DONE=======================");
			SmartDashboard.putBoolean("drvB-TstBtn-StepFnc:", false);
			isStepFncTestBtnActive = false;
		}
		return ;
	}
	
	//===================================
	// TEST DRIVE STRAIGHT CALIBRATION
	//===================================
	public boolean testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel){
		
		leftSensorPositionRead = driveBase.getLeftSensorPosition();
		rightSensorPositionRead = driveBase.getRightSensorPosition();
		
		if (!isTestMoveStraightCalActive){
			msg("SHUFFLE START DRIVE STRAIGHT CAL=============");
			methodStartTime = Timer.getFPGATimestamp();
			isTestMoveStraightCalActive = true;
			Tst_kDriveStraightFwdCorrection = SmartDashboard.getNumber("CAL_kDriveStraightFwdCorrection:", Tst_kDriveStraightFwdCorrection);
			
			Tst_RightDriveCmdLevel = _pwrLevel;
			Tst_LeftDriveCmdLevel = _pwrLevel;

			Tst_RightDriveCmdLevel = (_pwrLevel * Tst_kDriveStraightFwdCorrection); 
			leftEncoderStopCount = (_testDistanceIn / driveBase.getEncoderInchesPerCount());

			driveBase.setRightSensorPositionToZero();
			driveBase.setLeftSensorPositionToZero();
			Timer.delay(0.2);
		
			System.out.printf("StopCnt:%-8.0f+++LftEnc:%-8.0f +++RgtEnc:%-8.0f+++LftCmd:%-8.4f+++RgtCmd:%-8.4f%n", 
								leftEncoderStopCount, 
								leftSensorPositionRead, 
								rightSensorPositionRead,
								Tst_LeftDriveCmdLevel,
								Tst_RightDriveCmdLevel);
		
		} else if(leftSensorPositionRead <= leftEncoderStopCount)	{
			msg("CALIBRATION AT STOP ===========================================");
			
			driveBase.setDriveTrainRamp(0);
			// Apply power level in opposite direction for 1 second to brake
			rightCmdLevel = -kAutoRightMoveStopBrakeValue;
			leftCmdLevel = -kAutoRightMoveStopBrakeValue;
			if (!delay(1)) {
				msg("CALIBRATION END ==================================");
				isTestMoveStraightCalActive = false;
				isCalAtStop = false;
				rightCmdLevel = 0;
				leftCmdLevel = 0;
				methodTime = Timer.getFPGATimestamp() - methodStartTime;
				msg("Drive Straight Calibration Time(Sec) = " + methodTime);
			}	
		}
		
		calCorrectionFactor = leftSensorPositionRead / rightSensorPositionRead;
		headingDeg = (leftSensorPositionRead - rightSensorPositionRead) / driveBase.getRobotTrackWidth();
		
		// Output to SRX drive modules
		driveBase.SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		
		//Print on console data
		System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f===Correction:%-8.4f==Heading:%-8.2f%n", 
							leftEncoderStopCount, 
							leftSensorPositionRead, 
							rightSensorPositionRead,
							calCorrectionFactor,
							headingDeg);

		return isTestMoveStraightCalActive;
	}		


	private void msg(String _msgString){
		if (isConsoleEnabled){
			if (_msgString != lastMsgString){
				System.out.println(_msgString);
				lastMsgString = _msgString;
			}
		}
	}	
	//===================
	// DELAY
	//===================
	// This delay is looked at each scan so delay = seconds + scan(~20ms)
	public boolean delay(double _seconds){
		if (!isDelayActive) {
			isDelayActive = true;
			delayStartTime = Timer.getFPGATimestamp();
		} else if (Timer.getFPGATimestamp() >= (delayStartTime + _seconds)){
			isDelayActive = false;
		}
		return isDelayActive;
	}
}