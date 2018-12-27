package org.usfirst.frc.team2228.robot.subsystems.drvbase;

// Profiles are stored in directory robot\profiles
import org.usfirst.frc.team2228.robot.subsystems.drvbase.profile.DriveStraightLine;

// REVISION LEVEL:
// 181218 - cleaning up code

public class MotionProfileIF {
	private SRXDriveBase driveBase;
	private boolean isSRXMotionProfileActive = false;
	private DriveStraightLine profileData;
	
	// Constructor
	public MotionProfileIF(SRXDriveBase _driveBase){
		driveBase = _driveBase;
	} 

	public void init(){
		isSRXMotionProfileActive = false;
	}
	
	//NEED THE FOLLOWING METHOD FOR EACH PROFILE ADDED TO THIS CLASS IN robot\profiles
	public boolean driveStraight() {
		if(!isSRXMotionProfileActive){
			// Create and instance of the DriveStraightLine class 
			profileData = new DriveStraightLine();
			// This profile example uses the same profile file
			// xxxx profileDataLeft = new xxxx();

			isSRXMotionProfileActive = true;
		} else {
			//public boolean SRXProfileMove(double[][] ProfileRight, double[][] ProfileLelft, int totalPointNum)
			if(!driveBase.SRXProfileMove(profileData.PointsR, 
										 profileData.PointsL, 
										 profileData.kNumPoints)){
				isSRXMotionProfileActive = false;
				// Remove object from memory
				profileData = null;
			};
		}
		return isSRXMotionProfileActive;
	}
}


