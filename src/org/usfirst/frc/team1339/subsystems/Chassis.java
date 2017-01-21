package org.usfirst.frc.team1339.subsystems;

import java.util.ArrayList;

import org.usfirst.frc.team1339.base.SubsystemBase;
import org.usfirst.frc.team1339.commands.ArcadeDrive;
import org.usfirst.frc.team1339.robot.Robot;
import org.usfirst.frc.team1339.utils.Constants;
import org.usfirst.frc.team1339.utils.SplineProfile;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Chassis extends SubsystemBase{
	
	public SplineProfile chassisSP = new SplineProfile(
			Constants.splineMPKp, Constants.splineMPKi, Constants.splineMPKd,
			Constants.splineMPKa, Constants.splineMPKv);

	public static CANTalon leftMotorOne = new CANTalon(Constants.kLeftMotorOne);
	public static CANTalon leftMotorTwo = new CANTalon(Constants.kLeftMotorTwo);
	public static CANTalon rightMotorOne = new CANTalon(Constants.kRightMotorOne);
	public static CANTalon rightMotorTwo = new CANTalon(Constants.kRightMotorTwo);
	
	double leftLastSpeed, rightLastSpeed;
	double rightSpeed, leftSpeed;
	double rate = 0.085;
	
	private ArrayList<Double> accel = new ArrayList<Double>();
	
	private double lastTime = 0, lastRightSpeed = 0, lastLeftSpeed = 0;
	
	public Chassis(){
	}
	
	public void initDefaultCommand(){
		setDefaultCommand(new ArcadeDrive());
	}
	
	public void driveWithJoystick(double throttle, double turn){
       	double right = throttle;
    	double left = throttle;
    	double turningThrottleScale;
    	
    	if (java.lang.Math.abs(throttle) > 0.1) {
            turningThrottleScale = java.lang.Math.abs(throttle);
        }
        else{
        	turningThrottleScale = 0.75;
        }
    	turningThrottleScale *= 1.5;
    	
    	right += turn * turningThrottleScale;  
        left -= turn * turningThrottleScale;
        
        if(Math.abs(right) <= 0.05)
        	right = 0;
        if(Math.abs(left) <= 0.05)
        	left = 0;
        
        if(left >= leftLastSpeed + rate)
        	leftSpeed = leftLastSpeed + rate;
        else if (left <= leftLastSpeed - rate)
        	leftSpeed = leftLastSpeed - rate;
        else
        	leftSpeed = left;
        
        if(right >= rightLastSpeed + rate)
        	rightSpeed = rightLastSpeed + rate;
        else if (right <= rightLastSpeed - rate)
        	rightSpeed = rightLastSpeed - rate;
        else
        	rightSpeed = right;
        
        
        setMotorValues(leftSpeed, rightSpeed);
        
        leftLastSpeed = leftSpeed;
        rightLastSpeed = rightSpeed;
    }
    
    public void setMotorValues(double left, double right){
    	leftMotorOne.set(left);
    	leftMotorTwo.set(-left);
    	rightMotorOne.set(right);
    	rightMotorTwo.set(-right);
    }
    public void tankDrive(double left, double right){
    	setMotorValues(left, right);
    }
    
    public void PIDDriveEncoder(){
    	double rightSpeed = Robot.HardwareAdapter.RightDriveEncoderPID.calculate(Robot.HardwareAdapter.getRightDriveEnc());
    	double leftSpeed = Robot.HardwareAdapter.LeftDriveEncoderPID.calculate(Robot.HardwareAdapter.getLeftDriveEnc());
    	double gyroOutput = Robot.HardwareAdapter.GyroPID.calculate(Robot.HardwareAdapter.kSpartanGyro.getAngle());
    	rightSpeed -= gyroOutput;
    	leftSpeed += gyroOutput;
    	rightSpeed *= 0.5;
    	leftSpeed *= 0.5;
    	if(leftSpeed > 0.5){
    		leftSpeed = 0.5;
    	}
    	if(rightSpeed > 0.5){
    		rightSpeed = 0.5;
    	}    	if(leftSpeed < -0.5){
    		leftSpeed = -0.5;
    	}
    	if(rightSpeed < -0.5){
    		rightSpeed = -0.5;
    	}
    	
    	SmartDashboard.putNumber("Right PID Output", rightSpeed);
    	SmartDashboard.putNumber("Left PID Output", -leftSpeed);
    	setMotorValues(-leftSpeed, rightSpeed);
    }
    
    public void motionProfile(){
    	Robot.HardwareAdapter.ChassisMP.calculate(Robot.HardwareAdapter.getRightDriveEnc(), 
    			Robot.HardwareAdapter.getLeftDriveEnc());
    	double gyroOutput = Robot.HardwareAdapter.GyroPID.calculate(Robot.HardwareAdapter.kSpartanGyro.getAngle());
    	double rightSpeed = Robot.HardwareAdapter.ChassisMP.getRightOutput();
    	double leftSpeed = Robot.HardwareAdapter.ChassisMP.getLeftOutput();
    	rightSpeed -= gyroOutput;
    	leftSpeed += gyroOutput;
    	//System.out.println(speed);
    	SmartDashboard.putNumber("MP output", rightSpeed);
    	setMotorValues(-leftSpeed, -rightSpeed);
    }
    
    public void splineProfile(){
    	chassisSP.calculate(Robot.HardwareAdapter.getLeftDriveEnc(), Robot.HardwareAdapter.getRightDriveEnc());
    	if(!chassisSP.isFinishedTrajectory()){
    		double leftSpeed = chassisSP.getLeftOutput();
    		double rightSpeed = chassisSP.getRightOutput();
    		SmartDashboard.putNumber("left speed", leftSpeed);
    		SmartDashboard.putNumber("right Speed", rightSpeed);
    		setMotorValues(-leftSpeed, -rightSpeed);
    	}
    }
    
    public void calculate(){
    	double rightEncSpeed = Robot.HardwareAdapter.getRightDriveEncSpeed();
    	double rightSpeed = rightEncSpeed - lastRightSpeed;
    	lastRightSpeed = rightEncSpeed;
    	
    	double leftEncSpeed = Robot.HardwareAdapter.getLeftDriveEncSpeed();
    	double leftSpeed = leftEncSpeed - lastLeftSpeed;
    	lastLeftSpeed = leftEncSpeed;

    	double currentTime = Timer.getFPGATimestamp();
    	double time = currentTime - lastTime;
    	lastTime = currentTime;
    	
    	double rightAcc = rightSpeed / time;
    	double leftAcc = leftSpeed / time;

    	double avg = (rightAcc + leftAcc) / 2;
    	accel.add(avg);
    	
    	double speedAvg = (leftEncSpeed + rightEncSpeed) / 2;

    	SmartDashboard.putNumber("MP speed", speedAvg);
		  SmartDashboard.putNumber("Accel Array", avg);
    }
    
    public void gyroPID(){
    	double speed = Robot.HardwareAdapter.GyroPID.calculate(Robot.HardwareAdapter.kSpartanGyro.getAngle()) * 4;
    	double leftSpeed = -speed;
    	double rightSpeed = speed;
    	setMotorValues(leftSpeed, rightSpeed);
    }
	
    public ArrayList<Double> getAvgAcc(){
    	return accel;
    }
    
}
