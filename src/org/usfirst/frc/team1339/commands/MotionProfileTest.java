package org.usfirst.frc.team1339.commands;

import org.usfirst.frc.team1339.base.CommandBase;
import org.usfirst.frc.team1339.robot.Robot;

public class MotionProfileTest extends CommandBase {
	
	private double m_goal, tolerance, initialLeft, initialRight, m_decelerateVel;
	private int counter;
	
	public MotionProfileTest(double goal, double input_tolerance, double decelerateVel){
		requires(Robot.chassis);
		m_goal = goal;
		tolerance = input_tolerance;
		m_decelerateVel = decelerateVel;
	}

	protected void init() {
		// TODO Auto-generated method stub
		Robot.HardwareAdapter.GyroPID.setSetpoint(Robot.HardwareAdapter.kSpartanGyro.getAngle());
		Robot.HardwareAdapter.ChassisMP.configureNewProfile(m_goal, m_decelerateVel);
		initialLeft = Robot.HardwareAdapter.getLeftDriveEnc();
		initialRight = Robot.HardwareAdapter.getRightDriveEnc();
		Robot.HardwareAdapter.ChassisMP.initializeProfile(initialLeft, initialRight);
	}

	public void execute() {
		// TODO Auto-generated method stub
		//System.out.println("Running");
		Robot.chassis.motionProfile();
		if((Math.abs(Robot.HardwareAdapter.getLeftDriveEnc() - m_goal - initialLeft) < tolerance)
				&& (Math.abs(Robot.HardwareAdapter.getRightDriveEnc() - m_goal - initialRight) < tolerance)){
			counter++;
		}
	}

	public boolean isFinished() {
		// TODO Auto-generated method stub
		return Robot.HardwareAdapter.ChassisMP.isFinishedTrajectory() && counter >= 5;
	}

	protected void end() {
		// TODO Auto-generated method stub
		//Robot.chassis.setMotorValues(0, 0);
	}

	protected void interrupted() {
		// TODO Auto-generated method stub
		//Robot.chassis.setMotorValues(0, 0);
	}

}
