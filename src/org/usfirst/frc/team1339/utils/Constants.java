package org.usfirst.frc.team1339.utils;

/**
 * A class where we define all the constant joystick ports, joystick axis, sensor ports, talon ports, etc.
 * @author Nate Howard
 * @author Sam Schwartz
 * @author Sam Korman
 *
 */
public class Constants {
	//Spline Constants
	public static double robotWidth = 1050;
	public static double maxSplineVel = 2500;
	
	//Motion Profile Constants
	public static double maxAcceleration = 3500;
	public static double maxCruiseSpeed = 3000;
	public static double motionProfileSlowScaleFactor = 0.25;
	public static double motionProfileMediumScaleFactor = 0.65;
	public static double motionProfileFastScaleFactor = 0.9;
	
	public static final double splineMPKp = 0.02;//0.02;
	public static final double splineMPKi = 0.0;
	public static final double splineMPKd = 0.0007;
	public static final double splineMPKa = 0.00000005;
	public static final double splineMPKv = 0.000005;
	
	public static final double chassisMPKp = 0.0008;
	public static final double chassisMPKi = 0.0;
	public static final double chassisMPKd = 0.0;
	public static final double chassisMPKa = 0.00000005;
	public static final double chassisMPKv = 0.000005;
	
	public static final double kDriveKp = 0.07;
	public static final double kDriveKi = 0;
	public static final double kDriveKd = 0.05;
	//Short Drive PID values

	public static final double kShortDriveKp = 0.008;
	public static final double kShortDriveKi = 0;
	public static final double kShortDriveKd = 0.04;
	
	//Turn PID Values
	//Tuned for making a stationary turn (eg a 180)
	public static final double kTurnKp = 0.113; // 0.05, 0.1
	public static final double kTurnKi = 0.;
	public static final double kTurnKd = 0.04;
	//Shooter PID Values
	public static final double kShooterKp = 0.0004;//0.0003;
	public static final double kShooterKi = 0;
	public static final double kShooterKd = 0.0000;//.01;
	//Intake PID Values
	public static final double kArmKp = 0.0008;
	public static final double kArmKi = 0.000003;
	public static final double kArmKd = 0.005;
	//Vision PID Values
	public static final double kVisionKp = 0.05;
	public static final double kVisionKi = 0;
	public static final double kVisionKd = 0.4;
	//Gyro PID Values
	//Tuned for use while driving straight in auto
	public static final double kAutoGyroKp = 0.22;
	public static final double kAutoGyroKi = .0;
	public static final double kAutoGyroKd = .1;
	//Tuned for turning and driving at the same time
	public static final double kGyroKp = 0.035;
	public static final double kGyroKi = .0;
	public static final double kGyroKd = .4;
	//Acceleration
	public static final double kAccelKp = 0.00000035; //0.00000035
	public static final double kAccelKi = 0;
	public static final double kAccelKd = 0.;
	//snap
	public static final double kJerkKp = 0.0000005;
	public static final double kJerkKi = 0;
	public static final double kJerkKd = 0;
	
	public static final double k_mp_Kp = 0;
	public static final double k_mp_Ki = 0;
	public static final double k_mp_Kd = 0;
	
	//Joysticks + Axis
	public static int xboxPort = 0;
	public static int xboxLeftXAxis = 0;
	public static int xboxLeftYAxis = 1;
	public static int xboxRightXAxis = 4;
	public static int xboxRightYAxis = 5;
	public static int xboxRightTrigger = 3;
	public static int xboxLeftTrigger = 2;
	public static int xboxAbutton = 1;
	public static int xboxBbutton = 2;
	public static int xboxXbutton = 3;
	public static int xboxYbutton = 4;
	public static int xboxLeftBumper = 5;
	public static int xboxRightBumper = 6;
	public static int xboxViewButton = 7;
	public static int xboxMenuButton = 8;
	public static int xboxRightStickButton = 9;
	public static int xboxLeftStickButton = 10;

	//Motor CAN ID's
	public static int kLeverMotor = 2; //savage
	public static int kAxleMotor = 9;
	public static int kLeftMotorOne = 4;
	public static int kLeftMotorTwo = 5;
	public static int kRightMotorOne = 6;
	public static int kRightMotorTwo = 7;
	public static int kShooterMotorOne = 3;
	public static int kShooterMotorTwo = 8;
	public static int kRollerMotor = 1;
	public static int kLiftMotor = 10;
	//Encoders
	public static int kRightDriveAEncoder = 12;
	public static int kRightDriveBEncoder = 13;
	public static int kLeftDriveAEncoder = 10;//12
	public static int kLeftDriveBEncoder = 11;//13
	//Halleffects
	public static int kLeftHallEffect = 5;
	public static int kRightHallEffect = 4;

	//Gyro
	public static int kGyro = 0;
}
