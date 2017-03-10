package org.usfirst.frc5124.WestTorranceSwagbotics2017.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GearHolderSafelyOpen extends CommandGroup {

    public GearHolderSafelyOpen() {
    	
    	addSequential(new GearHolderOpen());
        addSequential(new Wait(0.125));
        addSequential(new GearHolderExtendPusher());
    }
}
