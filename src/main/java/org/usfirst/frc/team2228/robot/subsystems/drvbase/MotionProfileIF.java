package org.usfirst.frc.team2228.robot.subsystems.drvbase;

// Profiles are stored in directory robot\profiles
import org.usfirst.frc.team2228.robot.profiles.DriveStraightLine;

public class MotionProfileIF {
	private SRXDriveBase driveBase;
	private boolean isSRXMotionProfileActive = false;
	
	// Constructor
	public MotionProfileIF(SRXDriveBase _driveBase){
		driveBase = _driveBase;
	} 

	public void MotionProfileIFinit(){
		isSRXMotionProfileActive = false;
	}
	
	//NEED THE FOLLOWING METHOD FOR EACH PROFILE ADDED TO THIS CLASS IN robot\profiles
	public boolean DriveStraight() {
		if(!isSRXMotionProfileActive){
			// Create and instance of the DriveStraightLine class 
			DriveStraightLine profileDataRight = new DriveStraightLine();
			isSRXMotionProfileActive = true;
		} else {
			//public boolean SRXProfileMove(double[][] ProfileRight, double[][] ProfileLelft, int totalPointNum)
			if(!drivebase.SRXProfileMove(profileDataRight.PointsR, 
										profileDataRight.PointsR, 
										profileDataRight.kNumPoints)){
				isSRXMotionProfileActive = false;
			};
		}
		return isSRXMotionProfileActive;
	}
}


