package org.usfirst.frc5124.WestTorranceSwagbotics2017.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousGearBasicStraight extends CommandGroup {

    public AutonomousGearBasicStraight() {
    	addSequential(new AutoDriveByPID(75));
    	//go to lift
    	addSequential(new AutoKamikazeGear());
    	addSequential(new GearHolderSafelyOpen());
    }
}
