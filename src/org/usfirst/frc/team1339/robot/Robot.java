
package org.usfirst.frc.team1339.robot;


import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1339.subsystems.Chassis;
import org.usfirst.frc.team1339.subsystems.Intake;
import org.usfirst.frc.team1339.subsystems.Shooter;
import org.usfirst.frc.team1339.utils.HardwareAdapter;
import org.usfirst.frc.team1339.utils.Pipeline;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	public Autonomous a;
	public TeleOp t;
	public static Chassis chassis;
	public static Intake intake;
	public static Shooter shooter;
	public static HardwareAdapter HardwareAdapter;

	AxisCamera camera;
	public static double centerX = 0;
	private final Object imgLock = new Object();

	public Robot() {
		a = new Autonomous();
		t = new TeleOp();
	}

	public void robotInit() {
		HardwareAdapter = new HardwareAdapter();
		chassis = new Chassis();
		intake = new Intake();
		shooter = new Shooter();

		//WORKS
		new Thread(() -> {
			camera = CameraServer.getInstance().addAxisCamera("10.13.39.11");
			camera.setResolution(640, 480);
			CvSink cvSink = CameraServer.getInstance().getVideo();

			Mat source = new Mat();

			Pipeline pl = new Pipeline();

			while(true) {
				try {
					Thread.sleep(50);
				} catch (InterruptedException e1) {
					e1.printStackTrace();
				}
				cvSink.grabFrame(source);
				pl.setsource0(source);
				try{
					pl.process();
					Rect r = Imgproc.boundingRect(pl.filterContoursOutput().get(0));
					synchronized (imgLock) {
						centerX = r.x + (r.width / 2);
						SmartDashboard.putNumber("center X", centerX);
					}
				}
				catch(Exception e){
					System.out.println(e);
				}

			}
		}).start();
	}

	@Override
	public void disabledInit(){

	}

	@Override
	public void disabledPeriodic(){

	}

	@Override
	public void teleopInit(){
		t.init();
	}

	@Override
	public void teleopPeriodic(){
		while (isEnabled()) {
			t.teleOpPeriodic();
		}
	}

	@Override
	public void autonomousInit(){
		a.init();
	}

	@Override
	public void autonomousPeriodic(){
		while(isAutonomous())
			a.autonomousPeriodic();
	}
}
