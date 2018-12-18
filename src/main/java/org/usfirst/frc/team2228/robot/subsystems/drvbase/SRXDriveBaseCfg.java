package org.usfirst.frc.team2228.robot.subsystems.drvbase;

public class SRXDriveBaseCfg {
		// REVISION LEVEL:
		// 181218 - cleaning up code
		// 181106 - removed drivebase choose switch
		// 180327 - added switch to config a specific robot chassis drive base  
		
		// The following is the default configuration values. The function SRXDriveBaseCfgInit(int _robotChassisSelect)
		// overwrites the configuration values for a specific robot chassis 
		
		//=======================================
		// SRXDRIVEBASE CONTROL FLAGS
		public static boolean isSRXClosedLoopEnabled = true;
		public static boolean isMasterEncodersPresent = true;
		public static boolean isDriveStraightAssistEnabled = false;
		public static boolean isMotionMagicEnabled = false;
		public static boolean isMecanumEnabled = false;
		
		//!!!!!!!!!!!!!!!!!!!- MOTOR FWD -> ENCODER INCREASES COUNT AND POSITIVE -> SENSOR READ IS POSITIVE
		// =======================================
		// SET MOTOR DIRECTION
		public static boolean isDriveRightMasterMtrReversed = false;
		public static boolean isDriveRightFollowerMtrReversed = false;
		public static boolean isDriveLeftMasterMtrReversed = true;
		public static boolean isDriveLeftFollowerMtrReversed = true;
				
		// ===================================
		// FEEDBACK SENSOR - ENCODER DIRECTION
		
		// The following changes the encoder sign internal to the SRX only
		// If direct read of encoder is negative in fwd dir => is----EncoderSensorReversed = true
		public static boolean isRightEncoderSensorReversed = true;
		public static boolean isLeftEncoderSensorReversed = true;
		
		//=================================
		// PROGRAM ENCODER SENSOR DIRECTION
		
		public static boolean isRightEncoderSensorReadReversed = false;
		public static boolean isLeftEncoderSensorReadReversed = false;
		
		// ========================
		// ROBOT MEASUREMENTS/DATA:
		
		// AndyMark tough box mini (50.0/14.0)*(48.0/16.0)
		// CIMCoder and 2018 drive train gear ratio 72/11
		public static double kGearRatio =8.459;
		
		// CIMcode magnetic quadrature 20 cycles per revolution
		public static int EncoderCyclesPerRev = 20;
		
		// !!!!!!!!!!!!!!!!!!!!!! This is measured with a tape measure
		public static double kTrackWidthIn = 21.5;
				
		// !!!!!!!!!!!!!!!!!!!!!! This is measured with a thin tape measure - use mm and convert to in
		public static double kMeasuredRgtWheelCircum = 18.937;
				
		public static double kMeasuredLftWheelCircum = 18.937;
				
		
		// =============================================================
		// DRIVE TRAIN CALCULATIONS
		
		// kDriveEncoderCyclesPerRev = (cycles/rev) * (gearRatio)
				public static double kDriveEncoderCyclesPerRev = 169.18;
				
		// kCountsPerRevolution = quadrature(4) * kDriveEncoderCyclesPerRev
		public static double kCountsPerRevolution = 676.72;
				
		
		// kWheelCircumIn = (kMeasuredRgtWheelCircum + kMeasuredLftWheelCircum)/2
		public static double kWheelCircumIn = 18.678;
		
		// kInchesPerCount = kWheelCircumIn / kCountsPerRevolution; 1/kInchesPerCount
		public static double kInchesPerCount = 0.0276;
				
		public static double kEncoderCountsPerIn = 36.231;
				
		
		//==========
		//INFO CALCULATIONS
		// Diameter = WheelCircum / Pi
		public static double kRgtWheelDiameter = 4.035;
		public static double kLftWheelDiameter = 4.035;
		
		// kRightInchesPerCount = kMeasuredRgtWheelCircum / kCountsPerRevolution; 1/kRightInchesPerCount
		public static double kRightInchesPerCount = 0.0242; 
		public static double kRightEncoderCountsPerIn = 41.302;
		
		// kLeftInchesPerCount = kMeasuredLftWheelCircum / kCountsPerRevolution; 1/kLeftInchesPerCount
		public static double kLeftInchesPerCount = 0.0242; 
		public static double kLeftEncoderCountsPerIn = 41.302;

		//===============================================
		// SRX CLOSED LOOP VELOCITY CALCULATIONS
		
		// Use RoboRio web dashboard: Run at high speed and read Output % and nativeUnit velocity - calculate max
		// MaxVel_VelNativeUnits = RPM * 1/60sec * 1/[10 => 100ms samples/sec] * kCountsPerRevolution = counts/100ms
		//From RoboRio WebDashBoard:
		public static double MaxVel_VelNativeUnits = 662.164;
	
		// kTopRPM = (vel[measureVelPeriod->100ms] * 600) / kCountsPerRevolution
		public static double kTopRPM = 587.72;
		
		// ===============================================
		// SRX ESC MODULE
		//timeoutMS is recommended to be 10 milliseconds for bootup sequence according to the manual (3.1.2.1)
		public static int kslotIDx = 0;
		public static int kPIDLoopIDx = 0;
		
		// =======
		// SRX CLOSE LOOP SETUP PARAMETERS
		public static double kdriveRightMstrFeedForwardGain = 0; 
		public static double kdriveRightMstrProportionalGain = 0; 
		public static double kdriveRightMstrIntegralGain = 0;
		public static double kdriveRightMstrDerivativeGain = 0; 
		public static int    kdriveRightMstrIzone = 0;
		public static int    kdriveRightMstrRampRate = 0;
		public static int    kdriveRightMstrProfile = 0;
		
		public static double kdriveLeftMstrFeedForwardGain = 0; 
		public static double kdriveLeftMstrProportionalGain = 0; 
		public static double kdriveLeftMstrIntegralGain = 0;
		public static double kdriveLeftMstrDerivativeGain = 0; 
		public static int    kdriveleftMstrIzone = 0;
		public static int    kdriveLeftMstrRampRate = 0;
		public static int    kdriveLeftMstrProfile = 0;
		
		// =============================================
		// DEADBANDS
		public static int kClosedLoopErr = 0;
		
		// 0.001 represents 0.1% - default value is 0.04 or 4%previousEMAAccelFltrThrottleValue;
        public static double kSpeedDeadBand = 0.1;
		public static double kNeutralDeadBand = 0.08;
		
		// This sets the velocity calculation time sample
		public static int kSRXVelocitySample = 64;
		
		//Sample period in ms from supported sample periods-default 100ms period/64 sample window
		public static double kVelMeasurePeriodSec =.1; 
		
		//===============================================
		// SRX DRIVE BASE BRAKE, POWER BRAKE, AND COAST PARAMETERS
		
		// sets SRX zero speed brake mode to brake(true) and coast(false)
		public static boolean isBrakeEnabled = true;
		
		public static double kAutoRightMoveStopBrakeValue = 0.0;
		public static double kAutoLeftMoveStopBrakeValue = 0.0;
		
		public static double kAutoRightRotateStopBrakeValue = 0.0;
		public static double kAutoLeftRotateStopBrakeValue = 0.0;
		
		public static double kAutoRightTurnStopBrakeValue = 0.0;
		public static double kAutoLeftTurnStopBrakeValue = 0.0;
		
		
		public static double kAutoMoveCoastToStopCounts = 0;
		public static double kAutoRotateCoastToStopCounts = 0;
		public static double kAutoTurnCoastToStopCounts = 0;	
		
		//================================================
		// DRIVING STRAIGHT
		
		// This value is determined by testDriveStraightCalibration method
		public static double kDriveStraightFwdCorrection = 1;
		public static double kDriveStraightRevCorrection = 1;
		
		public static double kRotateCWDriveStraightCorrection = 1;
		public static double kRotateCCWDriveStraightCorrection = 1;
		
		public static double kTurnRightDriveStraightCorrection = 1;
		public static double kTurnLeftDriveStraightCorrection = 1;
		
		//================================================
		// DRIVE TRAIN STALL PARAMETERS
		public static int kStallCurrentContinuousAmps = 10;
		public static int kStallCurrentPeakAmps = 100;
        public static int kStallTimeMs = 6000;
	

}
