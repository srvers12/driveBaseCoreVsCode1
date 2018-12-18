// package org.usfirst.frc.team2228.robot.commands;

// todo - this is just an example - if not used - delete file
// import org.usfirst.frc.team2228.robot.DebugLogger;
// import org.usfirst.frc.team2228.robot.PneumaticController;
// import org.usfirst.frc.team2228.robot.RobotMap;
// import org.usfirst.frc.team2228.robot.SRXDriveBase;

// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.command.Command;

//REVISION LEVEL:
// 181218 - cleaning up code

// public class PneumaticGrabberExample extends Command {
// 		private PneumaticController pneu;
// 		private boolean isgrabbing;
	
	
// 	public PneumaticGrabber(PneumaticController _pneu, boolean _isgrabbing, double timeWait) {
// 		super(timeWait);
// 		pneu = _pneu;
// 		isgrabbing = _isgrabbing;
// 	}
	
// 	protected void initialize() {
// 		if(isgrabbing == true){
// 			System.out.println("Grabbing Started At " + Timer.getFPGATimestamp());
			
// 		}
// 		else{
// 			System.out.println("Not Grabbing At " + Timer.getFPGATimestamp());
// 		}
// 	}

// 	// Called repeatedly when this Command is scheduled to run
// 	public void execute() {
// 		pneu.squeeze(isgrabbing);
// 	}

// 	// Make this return true when this Command no longer needs to run execute()
// 	@Override
// 	protected boolean isFinished() {
// 		return isTimedOut();
// 	}

// 	// Called once after isFinished returns true
// 	@Override
// 	protected void end() {
		
// 	}

// 	// Called when another command which requires one or more of the same
// 	// subsystems is scheduled to run
// 	@Override
// 	protected void interrupted() {
// 	}

// }
