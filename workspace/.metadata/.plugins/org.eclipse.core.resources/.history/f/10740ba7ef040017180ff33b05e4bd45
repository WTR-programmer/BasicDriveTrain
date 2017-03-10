package org.usfirst.frc5124.WestTorranceSwagbotics2017.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class GearHolderSafelyClose extends CommandGroup {

    public GearHolderSafelyClose() {
    	
        addSequential(new GearHolderRectractPusher());
        addSequential(new Wait(0.5));
        addSequential(new GearHolderClose());
    }
}
