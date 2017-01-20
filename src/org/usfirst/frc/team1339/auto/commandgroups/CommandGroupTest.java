package org.usfirst.frc.team1339.auto.commandgroups;

import org.usfirst.frc.team1339.base.CommandGroupBase;
import org.usfirst.frc.team1339.commands.*;
import org.usfirst.frc.team1339.robot.Robot;

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
		addSequential(new MotionProfileTest(1211, 50, 250));
		//addSequential(new DriveStraight(-.5, 2));
	}
}
