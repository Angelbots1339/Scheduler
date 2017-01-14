package org.usfirst.frc.team1339.commands;

import org.usfirst.frc.team1339.base.CommandBase;
import org.usfirst.frc.team1339.robot.Robot;
import org.usfirst.frc.team1339.utils.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionPID extends CommandBase{
	
	double throttle, turn;
	Joystick stick;
	
	public VisionPID(){
		requires(Robot.chassis);
	}
	
	protected void init(){
		double center = Robot.centerX;
		Robot.HardwareAdapter.GyroPID.setSetpoint(0);
	}
	
	public void execute(){
		double center = Robot.centerX;
		SmartDashboard.putNumber("center X", center);
	}
	
	public boolean isFinished(){
		return false;
	}
	
	protected void end(){
		
	}
	
	protected void interrupted(){
		
	}
}
