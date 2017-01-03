package org.usfirst.frc.team1339.commands;

import org.usfirst.frc.team1339.base.CommandBase;
import org.usfirst.frc.team1339.robot.Robot;

/**
 *
 */
public class DriveShooterTimeout extends CommandBase {
	
	private double m_speed;
	
    public DriveShooterTimeout(double speed, double timeout) {
    	setTimeout(timeout);
    	setRunSpeed(0.05);
    	m_speed = speed;
    }

	protected void init() {
		
	}
    
    // Called repeatedly when this Command is scheduled to run
    public void execute() {
    	Robot.shooter.shoot(m_speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end(){
    	Robot.shooter.shoot(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.shooter.shoot(0);
    }

}
