package org.usfirst.frc.team1339.commands;

import org.usfirst.frc.team1339.base.CommandBase;
import org.usfirst.frc.team1339.robot.Robot;

public class GyroPID extends CommandBase{

	double m_goal;
	
	public GyroPID(double goal){
		requires(Robot.chassis);
		m_goal = goal;
	}
	
	@Override
	protected void init() {
		// TODO Auto-generated method stub
		Robot.HardwareAdapter.GyroPID.setSetpoint(Robot.HardwareAdapter.kSpartanGyro.getAngle() + m_goal);
	}

	@Override
	public void execute() {
		// TODO Auto-generated method stub
		Robot.chassis.gyroPID();
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return Robot.HardwareAdapter.GyroPID.onTarget(10);
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
