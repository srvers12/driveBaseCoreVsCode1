package org.usfirst.frc.team2228.robot.test;

import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBase;
import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBaseCfg;
import org.usfirst.frc.team2228.robot.util.DebugLogger;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

// REVISITON LEVEL:
// 181218 - cleaning up code
// 181204 - reorganized test selection process

// =========================
// SET METHODS
// public void init(boolean _isConsoleEnabled)

// =========================
// METHODS
//public void testMethodSelection()


public class SRXDriveBaseTest {

	// objects
	private DebugLogger log;
	private SRXDriveBase driveBase;
	//private SRXDriveBaseCfg SRXdriveBaseCfg; 

	private int testSelectionState = 0;

    private double drvBTst_kDriveStraightFwdCorrection = 1;
	private double kRightStopBrakeValue = .2;
	private double kLeftStopBrakeValue = .2;

	private double drvBTst_RightDriveCmdLevel = 0;
	private double drvBTst_LeftDriveCmdLevel = 0;
	//private double CAL_Throttle = 0;
	//private double CAL_turn = 0;
	//private double methodStartTime = 0;
	private double tstDelayStartTime = 0;
	private double leftSensorPositionRead = 0;
	private double rightSensorPositionRead = 0;
	private double leftEncoderStopCount = 0;
	private double calCorrectionFactor = 0;
	private double headingDeg = 0;
	//private double methodTime = 0;
	
	private boolean isMtrTestBtnActive = false;
	private boolean isStepFncTestBtnActive = false;
	//private boolean isTestStepFunctionActive = false;
	private boolean isTestMoveStraightCalActive = false;
	//private boolean isTestMethodSelectionActive = false;
	private boolean isConsoleEnabled = false;
	private boolean isTstDelayActive = false;

	private String lastMsgString = " ";

    // =====================
    // Constructor
    public SRXDriveBaseTest(SRXDriveBase _driveBase, DebugLogger _logger){
		driveBase = _driveBase;
		log = _logger;
    }

    public void init(boolean _isConsoleEnabled){
		isConsoleEnabled = _isConsoleEnabled? true : false;

		isMtrTestBtnActive = false;
		//isTestStepFunctionActive = false;
		isTestMoveStraightCalActive = false;
		//isTestMethodSelectionActive = false;
		
        // Drivebase setup method calls
        SmartDashboard.putBoolean("drvBTstBtn_StepFnc:", false);
		SmartDashboard.putBoolean("drvBTstBtn_DrvStraightCal:", false);
		SmartDashboard.putBoolean("drvBTstBtn_MotorEncoderTest:", false);

		// DriveBase autonomous motion method calls
		SmartDashboard.putBoolean("drvBTstBtn_MoveToPos:", false);
		SmartDashboard.putBoolean("drvBTstBtn_RotateToAngle:", false);
		SmartDashboard.putBoolean("drvBTstBtn_ProfileMove:", false);

		//SmartDashboard.putBoolean("drvBTstBtn-EnableSRXDriveBaseConsoleDisplay:", isConsoleEnabled);
		
		SmartDashboard.putNumber("drvBTst_kDriveStraightFwdCorrection:", drvBTst_kDriveStraightFwdCorrection);
		//SmartDashboard.putNumber("drvBkDriveStraightRevCorrection:", drvBkDriveStraightRevCorrection);
		
		SmartDashboard.putNumber("drvBTst_RightDriveCmdLevel:", drvBTst_RightDriveCmdLevel);
		SmartDashboard.putNumber("drvBTst_LeftDriveCmdLevel:", drvBTst_LeftDriveCmdLevel);
		
    }

	
	//==============================
	// TEST METHOD SELECTION
	//==============================
	public void testMethodSelection(){
		switch(testSelectionState) {
			case 0:
				if (SmartDashboard.getBoolean("drvBTstBtn_MotorEncoderTest:", false)){
					motorEncoderTest();
				} else {
					testSelectionState = 1;
				}
				break;
			case 1:
				if(SmartDashboard.getBoolean("drvBTstBtn_StepFnc:", false)){
					// public void testStepFunction(double _stepFunctionPower, double _stepFnctHighTimeSec, boolean _isTestForRightDrive)
					stepFunctionTest(.3,3,true);
				} else {
					testSelectionState = 2;
				}
			break;
			case 2:
				if(SmartDashboard.getBoolean("drvBTstBtn_DrvStraightCal:", false)){
					// public boolean driveStraightCalibrationTest(double _testDistanceIn, double _pwrLevel){
						driveStraightCalibrationTest(50,.3);
				} else {
					testSelectionState = 3;
				}
			break;
			case 3:
				if(SmartDashboard.getBoolean("drvBTstBtn_MoveToPos:", false)){
					// public boolean move(double _MoveDistanceIn, double _MovePwrlevel) {
						driveBase.move(30, .3);
				} else {
					testSelectionState = 4;
				}
			break;
			case 4:
				if(SmartDashboard.getBoolean("drvBTstBtn_MoveSideways:",false)){
					// public boolean move(double _MoveDistanceIn, double _MovePwrLevel, boolean _MoveSideways){
					driveBase.move(30, .3, true);
				} else {
					testSelectionState = 5;
				}
			break;

			case 5:
				if(SmartDashboard.getBoolean("TstBtn_RotateToAngle:", false)){
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
			driveBase.SetDriveTrainCmdLevel(SmartDashboard.getNumber("drvBTst_RightDriveCmdLevel:", drvBTst_RightDriveCmdLevel), 
											SmartDashboard.getNumber("drvBTst_LeftDriveCmdLevel:", drvBTst_LeftDriveCmdLevel));

			if(!SmartDashboard.getBoolean("drvBTstBtn_MotorEncoderTest:", false)) {
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
				drvBTst_RightDriveCmdLevel = _stepFunctionPower;
				drvBTst_LeftDriveCmdLevel = 0;
			} else {
				drvBTst_RightDriveCmdLevel = 0;
				drvBTst_LeftDriveCmdLevel = _stepFunctionPower;
			}
			
			driveBase.SetDriveTrainCmdLevel( drvBTst_RightDriveCmdLevel, drvBTst_LeftDriveCmdLevel);
					
		} else if(!tstDelay(_stepFnctHighTimeSec)) {
			driveBase.stopMotors();
			msg("TEST STEP FUNCTION DONE=======================");
			SmartDashboard.putBoolean("drvBTstBtn_StepFnc:", false);
			isStepFncTestBtnActive = false;
		}
		return ;
	}
	
	//===================================
	// TEST DRIVE STRAIGHT CALIBRATION
	//===================================
	public boolean driveStraightCalibrationTest(double _testDistanceIn, double _pwrLevel){
		
		leftSensorPositionRead = driveBase.getLeftSensorPosition();
		rightSensorPositionRead = driveBase.getRightSensorPosition();
		
		if (!isTestMoveStraightCalActive){
			msg("SHUFFLE START DRIVE STRAIGHT CAL=============");
			isTestMoveStraightCalActive = true;
			drvBTst_kDriveStraightFwdCorrection = SmartDashboard.getNumber("drvBTst_kDriveStraightFwdCorrection:", drvBTst_kDriveStraightFwdCorrection);
			
			drvBTst_RightDriveCmdLevel = _pwrLevel;
			drvBTst_LeftDriveCmdLevel = _pwrLevel;

			drvBTst_RightDriveCmdLevel = (_pwrLevel * drvBTst_kDriveStraightFwdCorrection); 
			leftEncoderStopCount = (_testDistanceIn / driveBase.getEncoderInchesPerCount());

			driveBase.setRightSensorPositionToZero();
			driveBase.setLeftSensorPositionToZero();
			Timer.delay(0.2);
		
			System.out.printf("StopCnt:%-8.0f+++LftEnc:%-8.0f +++RgtEnc:%-8.0f+++LftCmd:%-8.4f+++RgtCmd:%-8.4f%n", 
								leftEncoderStopCount, 
								leftSensorPositionRead, 
								rightSensorPositionRead,
								drvBTst_LeftDriveCmdLevel,
								drvBTst_RightDriveCmdLevel);
		
		} else if(leftSensorPositionRead <= leftEncoderStopCount)	{
			msg("CALIBRATION AT STOP ===========================================");
			
			driveBase.setDriveBaseRamp(0);
			// Apply power level in opposite direction for 1 second to brake
			drvBTst_RightDriveCmdLevel = -kRightStopBrakeValue;
			drvBTst_LeftDriveCmdLevel = -kLeftStopBrakeValue;
			if (!tstDelay(1)) {
				msg("CALIBRATION END ==================================");
				isTestMoveStraightCalActive = false;
				drvBTst_RightDriveCmdLevel = 0;
				drvBTst_LeftDriveCmdLevel = 0;
				
			}	
		}
		
		calCorrectionFactor = leftSensorPositionRead / rightSensorPositionRead;
		headingDeg = (leftSensorPositionRead - rightSensorPositionRead) / driveBase.getRobotTrackWidth();
		
		// Output to SRX drive modules
		driveBase.SetDriveTrainCmdLevel(drvBTst_RightDriveCmdLevel, drvBTst_LeftDriveCmdLevel);
		
		
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
	// This tstDelay is looked at each scan so tstDelay = seconds + scan(~20ms)
	public boolean tstDelay(double _seconds){
		if (!isTstDelayActive) {
			isTstDelayActive = true;
			tstDelayStartTime = Timer.getFPGATimestamp();
		} else if (Timer.getFPGATimestamp() >= (tstDelayStartTime + _seconds)){
			isTstDelayActive = false;
		}
		return isTstDelayActive;
	}
}