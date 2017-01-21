package org.usfirst.frc.team1339.utils;

import org.usfirst.frc.team1339.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SplineProfile {

	private double Kp, Ki, Kd, Ka, Kv, cruiseVel, 
	splineCruiseVel, maxAcc, cruiseVelScaleFactor, splineRadius,
	splineAngle, totalOuterDistance, lastInnerPos = 0, sumAngle = 0,
	leftStartPos, rightStartPos, lastRightError = 0, startVel,
	lastLeftError = 0, totalInnerDistance,
	lastGyro = 0, startGyroAngle, sumGyroAngle, decelerateVel;
	private double lastTime;
	public Segment currentOuterSegment = new Segment(0, 0, 0);
	public Segment currentInnerSegment = new Segment(0, 0, 0);
	public Segment nextOuterSegment = new Segment(0, 0, 0);
	public Segment nextInnerSegment = new Segment(0, 0, 0);
	private double maxSplineVel;
	private double width = Constants.robotWidth;
	private double rightOutput = 0, leftOutput = 0;
	private boolean angleInverted = false, splineBackwards = false, rightOuter, trajectoryFinished = false;
	
	public SplineProfile(double Kp, double Ki, double Kd, double Ka,
			double Kv){
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		this.Ka = Ka;
		this.Kv = Kv;
		currentOuterSegment = new Segment(0, 0, 0);
		currentInnerSegment = new Segment(0, 0, 0);
		nextOuterSegment = new Segment(0, 0, 0);
		nextInnerSegment = new Segment(0, 0, 0);
	}

	private static class Segment{
		public double pos, vel, acc;
		
		public Segment(double pos, double vel, double acc){
			this.pos = pos;
			this.vel = vel;
			this.acc = acc;
		}
	}
	
	private enum MotionState{
		ACCELERATING, CRUISING, DECELERATING, END
	}
	
	private MotionState state = MotionState.END;
	
	private void setState(MotionState newState){
		state = newState;
	}
	
	private MotionState getState(){
		return state;
	}
	
	public void configureSplineProfile(double radius, double angle, boolean direction) {
		this.trajectoryFinished = false;
		this.maxAcc = Constants.maxAcceleration;
		this.cruiseVelScaleFactor = Constants.motionProfileSlowScaleFactor;
		this.maxSplineVel = Constants.maxSplineVel;
		this.splineAngle = Math.abs(angle);
		this.angleInverted = angle < 0;
		this.splineRadius = Math.abs(radius);
		this.totalOuterDistance = this.splineRadius * this.splineAngle;
		this.totalInnerDistance = (this.splineRadius - this.width) * this.splineAngle;
		this.cruiseVel = getCruiseVel(this.totalOuterDistance);
		this.splineCruiseVel = getSplineCruiseVel(this.totalOuterDistance);
		this.splineBackwards = !direction;
		if(!direction) {
			this.splineCruiseVel *= -1;
		}
		if(this.splineBackwards) {
			this.totalOuterDistance *= -1;
			this.maxAcc *= -1;
		}
		if (this.angleInverted == this.splineBackwards){
			rightOuter = false;
		}
		else rightOuter = true;
		this.startVel = 0;
		setState(MotionState.ACCELERATING);
	}
	
	public void configureSplineProfile(double radius, double angle, double startVel, double decelerateVel, boolean direction){
		this.trajectoryFinished = false;
		this.angleInverted = angle < 0;
		this.maxAcc = Constants.maxAcceleration;
		this.cruiseVelScaleFactor = Constants.motionProfileSlowScaleFactor;
		this.maxSplineVel = Constants.maxSplineVel;
		this.splineAngle = Math.abs(angle);
		this.splineRadius = Math.abs(radius);
		this.totalOuterDistance = this.splineRadius * this.splineAngle;
		this.totalInnerDistance = (this.splineRadius - this.width) * this.splineAngle;
		this.cruiseVel = getCruiseVel(this.totalOuterDistance);
		this.splineCruiseVel = getSplineCruiseVel(this.totalOuterDistance);
		this.splineBackwards = !direction;
		if(!direction) {
			this.splineCruiseVel *= -1;
		}
		if(this.splineBackwards) {
			this.totalOuterDistance *= -1;
			this.maxAcc *= -1;
		}
		if (this.angleInverted == this.splineBackwards){
			rightOuter = false;
		}
		else rightOuter = true;
		this.startVel = startVel;
		this.decelerateVel = decelerateVel;
		setState(MotionState.ACCELERATING);
	}
	
	private double getCruiseVel(double distance){
		double halfDist = distance / 2;
		double maxVelOverHalfDistance = Math.sqrt(2 * halfDist * maxAcc);
		return Math.min(maxVelOverHalfDistance * cruiseVelScaleFactor, Constants.maxCruiseSpeed);
	}
	
	private double getSplineCruiseVel(double distance){
		if(distance > 0) return Math.min(this.cruiseVel, maxSplineVel);
		return Math.max(this.cruiseVel, -maxSplineVel);
	}
	
	public double getRightOutput(){
		return rightOutput;
	}
	
	public double getLeftOutput(){
		return leftOutput;
	}
	
	public double getAngle(){
		return sumAngle;
	}
	
	public double getInnerDistance(){
		return this.totalInnerDistance;
	}
	
	public double getOuterDistance(){
		return this.totalOuterDistance;
	}
	
	public void initializeProfile(double leftCurrentPos, double rightCurrentPos, double currentGyroAngle){
		sumAngle = 0;
		lastGyro = 0;
		this.leftStartPos = leftCurrentPos;
		this.rightStartPos = rightCurrentPos;
		this.startGyroAngle = currentGyroAngle;
		currentOuterSegment = new Segment(0, this.startVel, 0);
		currentInnerSegment = new Segment(0, this.startVel, 0);
		nextOuterSegment = new Segment(0, 0, 0);
		nextInnerSegment = new Segment(0, 0, 0);
		lastTime = Timer.getFPGATimestamp();
	}
	
	public void calculate(double leftDistance, double rightDistance){
		double dt;
		double currentTime = Timer.getFPGATimestamp();
		dt = currentTime - lastTime;
		lastTime = currentTime;
		
		double currentInnerVel = currentInnerSegment.vel;
		double currentOuterVel = currentOuterSegment.vel;
		double outerDistanceToGo = this.totalOuterDistance - currentOuterSegment.pos;
		
		double t_to_cruise = Math.abs((this.splineCruiseVel - currentOuterVel) / maxAcc); //time to accelerate to cruise speed
		double x_to_cruise = currentOuterVel * t_to_cruise + .5 * maxAcc * t_to_cruise * t_to_cruise; //distance to get to cruise speed
		
		double t_to_zero = Math.abs((currentOuterVel) / maxAcc); //time to get to zero speed from cruise speed
		double x_to_zero = currentOuterVel * t_to_zero - .5 * maxAcc * t_to_zero * t_to_zero; //distance to get to zero speed
				
		double cruiseX;
		if(this.totalOuterDistance > 0){
			cruiseX  = Math.max(0, outerDistanceToGo - x_to_cruise - x_to_zero);
		}
		else{
			cruiseX  = Math.min(0, outerDistanceToGo - x_to_cruise - x_to_zero);
		}
		double cruiseT = Math.abs(cruiseX / this.splineCruiseVel);
		
		if (getState() == MotionState.ACCELERATING){
			if (t_to_cruise < dt){
				setState(MotionState.CRUISING);
			}
		}
		
		if(getState() == MotionState.CRUISING){
			if(cruiseT < dt){
				setState(MotionState.DECELERATING);
			}
		}
		
		if(getState() == MotionState.DECELERATING){
			if(t_to_zero < dt){
				setState(MotionState.END);
			}
		}
		
		double currentAngle = Math.abs(Robot.HardwareAdapter.kSpartanGyro.getAngle() - this.startGyroAngle);
		if(this.splineBackwards){
			currentAngle *= -1;
		}
		/*double leftRadius = (leftDistance - this.leftStartPos) / Math.toRadians(currentAngle);
		double rightRadius = (rightDistance - this.rightStartPos) / Math.toRadians(currentAngle);
		this.width = Math.abs(leftRadius - rightRadius);*/
		
		if(getState() == MotionState.ACCELERATING){
			nextOuterSegment.pos = currentOuterVel * dt + 0.5 * maxAcc * dt * dt;
			nextOuterSegment.vel = currentOuterVel + dt * maxAcc;
			nextOuterSegment.acc = maxAcc;
		}
		else if(getState() == MotionState.CRUISING){
			nextOuterSegment.pos = splineCruiseVel * dt;
			nextOuterSegment.vel = splineCruiseVel;
			nextOuterSegment.acc = 0;
		}
		else if(getState() == MotionState.DECELERATING){
			nextOuterSegment.pos = currentOuterVel * dt - 0.5 * maxAcc * dt * dt;
			nextOuterSegment.vel = currentOuterVel - maxAcc * dt;
			nextOuterSegment.acc = -maxAcc;
		}
		else{
			nextOuterSegment.pos = 0;
			currentOuterSegment.pos = this.totalOuterDistance;
			nextOuterSegment.vel = this.decelerateVel;
			nextOuterSegment.acc = 0;
		}
		/*
		nextOuterSegment.pos = splineCruiseVel * dt;
		nextOuterSegment.vel = splineCruiseVel;
		nextOuterSegment.acc = 0;
		*/
		double innerRadius = splineRadius - width;
		double angle = nextOuterSegment.pos / splineRadius;
		if(this.angleInverted != this.splineBackwards) sumAngle -= Math.toDegrees(angle);
		else sumAngle += Math.toDegrees(angle);

		nextInnerSegment.pos = angle * innerRadius;
		double displacement = nextInnerSegment.pos;
		nextInnerSegment.vel = displacement / dt;
		lastInnerPos = displacement;
		double finalVel = nextInnerSegment.vel;
		nextInnerSegment.acc = (finalVel - currentInnerVel) / dt;
				
		currentOuterSegment.pos += nextOuterSegment.pos;
		currentOuterSegment.vel = nextOuterSegment.vel;
		currentOuterSegment.acc = nextOuterSegment.acc;
		
		currentInnerSegment.pos += nextInnerSegment.pos;
		currentInnerSegment.vel = nextInnerSegment.vel;
		currentInnerSegment.acc = nextInnerSegment.acc;
		
		double delta_gyro = Math.toRadians(currentAngle - this.lastGyro);
		this.lastGyro = currentAngle;
		
		double gyroAngleError = angle - delta_gyro;
		
		double gyroChangeDist = gyroAngleError * this.splineRadius;
		
		currentOuterSegment.pos += gyroChangeDist;
		this.totalOuterDistance += gyroChangeDist;
		
		double outerOutput = Kv * currentOuterSegment.vel + Ka * currentOuterSegment.acc;
		double innerOutput = Kv * currentInnerSegment.vel + Ka * currentInnerSegment.acc;
		
		if(rightOuter){
			double rightError = currentOuterSegment.pos + this.rightStartPos - rightDistance;
			
			rightOutput = rightError * Kp + ((rightError - lastRightError) / dt) * Kd + outerOutput;
			lastRightError = rightError;
			
			double leftError = currentInnerSegment.pos + this.leftStartPos - leftDistance;
			
			leftOutput = leftError * Kp + ((leftError - lastLeftError) / dt) * Kd + innerOutput;
			lastLeftError = leftError;
		}
		else{
			double rightError = currentInnerSegment.pos + this.rightStartPos - rightDistance;
			
			rightOutput = rightError * Kp + ((rightError - lastRightError) / dt) * Kd + innerOutput;
			lastRightError = rightError;
			
			double leftError = currentOuterSegment.pos + this.leftStartPos - leftDistance;
			
			leftOutput = leftError * Kp + ((leftError - lastLeftError) / dt) * Kd + outerOutput;
			lastLeftError = leftError;
		}
		
		double x_to_goal = this.totalOuterDistance - currentOuterSegment.pos;
		double t_to_goal = Math.abs(x_to_goal / splineCruiseVel);
	
		SmartDashboard.putNumber("Spline Goal", this.totalOuterDistance);
		
		if(t_to_goal < dt){
			trajectoryFinished = true;
		}
	}
	
	public boolean isFinishedTrajectory() {
        return this.trajectoryFinished;
    }
}
