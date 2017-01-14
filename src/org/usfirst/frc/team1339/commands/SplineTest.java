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
		// TODO Auto-generated method stub\
		//Robot.HardwareAdapter.GyroPID.setSetpoint(Math.toDegrees(m_angle));
		Robot.chassis.chassisSP.configureSplineProfile(m_radius, m_angle, m_direction);
		Robot.chassis.chassisSP.initializeProfile(Robot.HardwareAdapter.getLeftDriveEnc(), Robot.HardwareAdapter.getRightDriveEnc());
	}

	@Override
	public void execute() {
		// TODO Auto-generated method stub
		if(!Robot.chassis.chassisSP.isFinishedTrajectory()){
			Robot.chassis.splineProfile();
		}
		/*
		if(Robot.HardwareAdapter.GyroPID.onTarget(90)){
			Robot.HardwareAdapter.GyroPID.setPID(0.09, 0.0, 0.4);
			Robot.HardwareAdapter.GyroPID.setSetpoint(Math.toDegrees(m_angle));
			Robot.chassis.gyroPID();
		}*/
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return Robot.chassis.chassisSP.isFinishedTrajectory();
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		Robot.chassis.gyroPID();
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		Robot.chassis.setMotorValues(0, 0);
	}

}
