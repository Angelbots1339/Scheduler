package org.usfirst.frc.team1339.commands;

import org.usfirst.frc.team1339.base.CommandBase;
import org.usfirst.frc.team1339.robot.Robot;

public class SplineTest extends CommandBase{

	double m_radius, m_angle;
	boolean m_direction;
	
	public SplineTest(double radius, double angle, boolean direction) {
		// TODO Auto-generated constructor stub
		requires(Robot.chassis);
		m_radius = radius;
		m_angle = Math.toRadians(angle);
		m_direction = direction;
	}

	@Override
	protected void init() {
		// TODO Auto-generated method stub
		Robot.chassis.chassisSP.configureSplineProfile(m_radius, m_angle, m_direction);
		Robot.chassis.chassisSP.initializeProfile(Robot.HardwareAdapter.getLeftDriveEnc(), Robot.HardwareAdapter.getRightDriveEnc());
	}

	@Override
	public void execute() {
		// TODO Auto-generated method stub
		Robot.chassis.splineProfile();
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return Robot.chassis.chassisSP.isFinishedTrajectory();
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		Robot.chassis.setMotorValues(0, 0);
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		Robot.chassis.setMotorValues(0, 0);
	}

}
