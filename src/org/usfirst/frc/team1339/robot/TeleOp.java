package org.usfirst.frc.team1339.robot;

import org.usfirst.frc.team1339.utils.Looper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is the class for teleoperated robot control. In this
 * instance, we use Arcade Drive, consisting of two joysticks,
 * one being assigned to throttle and the other being assigned
 * to turn.
 * <br><br>
 * $
 * @author Sam Schwartz
 * @author Nate Howard
 * @author Sam Korman
 * <br><br>
 * @see Autonomous
 * @see Disabled
 * @see Robot
 */
public class TeleOp {
	
	TeleOp(){
		
	}
	/** This method runs TeleOp at the speed of 20 milliseconds.*/
    public void teleOpPeriodic(){
    	Looper.getInstance().update();
    	Robot.HardwareAdapter.checkTriggers();
		SmartDashboard.putNumber("left drive encoder", Robot.HardwareAdapter.getLeftDriveEnc());
		SmartDashboard.putNumber("right drive encoder", Robot.HardwareAdapter.getRightDriveEnc());
		SmartDashboard.putNumber("Gyro",  Robot.HardwareAdapter.kSpartanGyro.getAngle());
		SmartDashboard.putNumber("encoderConversion", getClicksInches(26));
		SmartDashboard.putNumber("Spline Angle", Robot.chassis.chassisSP.getAngle());
		SmartDashboard.putNumber("inner dist", Robot.chassis.chassisSP.getInnerDistance());
		SmartDashboard.putNumber("Gyro rate change", Robot.HardwareAdapter.kSpartanGyro.getRate());
    }
    /** This method is called before TeleOp has run.*/
	public void init(){	
		Looper.getInstance().setInitDefaults();
		Robot.HardwareAdapter.kLeftDriveEncoder.reset();
		Robot.HardwareAdapter.kRightDriveEncoder.reset();
		Robot.HardwareAdapter.kSpartanGyro.reset();
	}
	/** This method is called after TeleOp has run.*/

	public static int getClicksInches(double distance){
		double clicks = distance * 32.2554018;
		return (int)clicks;
	}
	
}
