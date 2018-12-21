package org.usfirst.frc.team2228.robot;


import org.usfirst.frc.team2228.robot.oi.DriveTeleopBase;
import org.usfirst.frc.team2228.robot.oi.DriverIF;
import org.usfirst.frc.team2228.robot.sensors.AngleIF;
import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBase;
import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBaseCfg;
import org.usfirst.frc.team2228.robot.test.SRXDriveBaseTest;
import org.usfirst.frc.team2228.robot.util.DebugLogger;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	// define object instances
	private SRXDriveBase driveBase;
	private SRXDriveBaseTest testDriveBase;
	private DriverIF driverIF;
	private AngleIF angleIF;
	private DriveTeleopBase driveTeleopBase;
	private SRXDriveBaseCfg driveBaseCfg;
	private DebugLogger logger;
	private RobotMap robotMap;

	private boolean isConsoleDataEnabled = false;
	private String lastMsgString = " ";

	
	// This function is run when the robot is first started up: 
	// 1) create object instances
	@Override
	public void robotInit() {

		// Create object instances
		logger = new DebugLogger();
		driverIF = new DriverIF();
		angleIF = new AngleIF();
		robotMap = new RobotMap();
		
		driveBase = new SRXDriveBase(robotMap, logger);
		driveBaseCfg = new SRXDriveBaseCfg();

		driveTelopBase = new DriveTeleopBase(driverIF, driveBase, logger);
		testDriveBase = new SRXDriveBaseTest(driveBase, logger);
		
	}

	
	@Override
	public void autonomousInit() {

		// public void init(boolean _isConsoleEnabled, boolean _isDataLoggingEnabled)
		driveBase.init(false, false);
	}

	
	// This function is called periodically during autonomous
	@Override
	public void autonomousPeriodic() {
		
	}

	@Override
	public void teleopInit() {
		// public void init(boolean _isConsoleEnabled, boolean _isLoggingEnabled)
		driveBase.init(false, false);

		// public void init(boolean _isConsoleDataEnabled, boolean _isTestJoyStickEnabled)
		driveTelopBase.init(false, false);

	}
	
	// This function is called periodically during operator control
	@Override
	public void teleopPeriodic() {
		driveTelopBase.Periodic();	
	}
	
	
	// This function is called once during test mode
	@Override
	public void testInit() {

		//public void init(boolean _isConsoleEnabled)
		testDriveBase.init(false);
	}
	
	
	// This function is called periodically during test mode
	@Override
	public void testPeriodic() {
		testDriveBase.testMethodSelection();
	}
	
	private void msg(String _msgString){
		if (isConsoleDataEnabled){
			if (_msgString != lastMsgString){
				System.out.println(_msgString);
				lastMsgString = _msgString;
			}
		}
	}			
}

