package org.usfirst.frc.team1339.auto.commandgroups;

import org.usfirst.frc.team1339.base.CommandGroupBase;
import org.usfirst.frc.team1339.commands.*;
import org.usfirst.frc.team1339.robot.Robot;
import org.usfirst.frc.team1339.utils.SplineProfile;

/**
 * A test commandgroup.
 * @author Sam Schwartz
 * @author Nate Howard
 * @author Sam Korman
 * @see CommandGroupBase
 * @see Robot
 */

public class CommandGroupTest extends CommandGroupBase{

	public CommandGroupTest(){
		addSequential(new MotionProfileTest(1211, 50, 1500));
		addSequential(new SplineTest(2000, 90, 1500, 0, true));
		addSequential(new SplineTest(2000, -90, 0, 0, true));
	}
}
