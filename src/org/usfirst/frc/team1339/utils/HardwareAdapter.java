package org.usfirst.frc.team1339.utils;

import org.usfirst.frc.team1339.auto.commandgroups.CommandGroupTest;
import org.usfirst.frc.team1339.commands.ArcadeDrive;
import org.usfirst.frc.team1339.commands.GyroPID;
import org.usfirst.frc.team1339.commands.MotionProfileTest;
import org.usfirst.frc.team1339.commands.SplineTest;
import org.usfirst.frc.team1339.commands.TankDrive;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

/**
 * A class where we define all the variables for talons, solenoids, sensors, etc.
 * We also define joysticks and make get methods for the joysticks.
 * Finally, we define the triggers for certain commands.
 * @author Nate Howard
 * @author Sam Schwartz
 * @author Sam Korman
 *
 */
		
public class HardwareAdapter{
	
	//Encoders
	public Encoder kRightDriveEncoder = new Encoder(
			Constants.kRightDriveAEncoder , Constants.kRightDriveBEncoder, true);
	public Encoder kLeftDriveEncoder = new Encoder(
			Constants.kLeftDriveAEncoder , Constants.kLeftDriveBEncoder);
	public ADXRS450_Gyro kSpartanGyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	
	//Joysticks
	private Joystick xboxStick = new Joystick(Constants.xboxPort);
	
	//Razer Joystick Buttons
	private AngelButton AButton = new AngelButton(xboxStick , Constants.xboxAbutton);
	private AngelButton BButton = new AngelButton(xboxStick , Constants.xboxBbutton);
	private AngelButton XButton = new AngelButton(xboxStick , Constants.xboxXbutton);
	private AngelButton YButton = new AngelButton(xboxStick , Constants.xboxYbutton);
	private AngelButton leftBumper = new AngelButton(xboxStick , Constants.xboxLeftBumper);
	private AngelButton rightBumper = new AngelButton(xboxStick , Constants.xboxRightBumper);
	private AngelButton leftStickButton = new AngelButton(xboxStick , Constants.xboxLeftStickButton);
	private AngelButton rightStickButton = new AngelButton(xboxStick , Constants.xboxRightStickButton);
	
	//PID Loops
	public SynchronousPID LeftDriveEncoderPID = new SynchronousPID(
			Constants.kDriveKp , Constants.kDriveKi , Constants.kDriveKd);
	public SynchronousPID ShortLeftDriveEncoderPID = new SynchronousPID(
			Constants.kShortDriveKp , Constants.kShortDriveKi , Constants.kShortDriveKd);
	public SynchronousPID RightDriveEncoderPID = new SynchronousPID(
			Constants.kDriveKp , Constants.kDriveKi , Constants.kDriveKd);
	public SynchronousPID ShortRightDriveEncoderPID = new SynchronousPID(
			Constants.kShortDriveKp , Constants.kShortDriveKi , Constants.kShortDriveKd);
	public SynchronousPID TurnGyroPID = new SynchronousPID(
			Constants.kTurnKp , Constants.kTurnKi , Constants.kTurnKd);
	public SynchronousPID ArmPID = new SynchronousPID(
			Constants.kArmKp , Constants.kArmKi , Constants.kArmKd);
	public SynchronousPID ShooterPID = new SynchronousPID(
			Constants.kShooterKp , Constants.kShooterKi , Constants.kShooterKd);
	public SynchronousPID AccelPID = new SynchronousPID(
			Constants.kAccelKp, Constants.kAccelKi, Constants.kAccelKd);
	public SynchronousPID JerkPID = new SynchronousPID(
			Constants.kJerkKp, Constants.kJerkKi, Constants.kJerkKd);
	public SynchronousPID GyroPID = new SynchronousPID(
			Constants.kGyroKp , Constants.kGyroKi , Constants.kGyroKd);
	public SynchronousPID MotionProfile = new SynchronousPID(
			Constants.k_mp_Kp , Constants.k_mp_Ki , Constants.k_mp_Kd);
	
	//Motion Profiles
	public MotionProfile ChassisMP = new MotionProfile(
			Constants.chassisMPKp, Constants.chassisMPKi, Constants.chassisMPKd, 
			Constants.chassisMPKa, Constants.chassisMPKv);
	

	public HardwareAdapter(){
		kRightDriveEncoder.setDistancePerPulse(1);
		kLeftDriveEncoder.setDistancePerPulse(1);
	}
	
	public void checkTriggers(){
		XButton.whenPressed(new SplineTest(2000, 90, 0, 0, true));
		BButton.whenPressed(new CommandGroupTest());
		//BButton.whenPressed(new SplineTest(2421, -180, true));
	}
	
	//Joystick get methods
	public Joystick getXboxStick(){
		return xboxStick;
	}
	public AngelButton getAButton(){
		return AButton;
	}
	public AngelButton getBButton(){
		return BButton;
	}
	public AngelButton getXButton(){
		return XButton;
	}
	public AngelButton getYButton(){
		return YButton;
	}
	public AngelButton getLeftBumper(){
		return leftBumper;
	}
	public AngelButton getRightBumper(){
		return rightBumper;
	}
	public AngelButton getLeftStickButton(){
		return leftStickButton;
	}
	public AngelButton getRightStickButton(){
		return rightStickButton;
	}
	public double getLeftDriveEnc(){
		return kLeftDriveEncoder.get();
	}
	public double getRightDriveEnc(){
		return kRightDriveEncoder.get();
	}
	public double getRightDriveEncSpeed(){
		return kRightDriveEncoder.getRate();
	}
	public double getLeftDriveEncSpeed(){
		return kLeftDriveEncoder.getRate();
	}
}
