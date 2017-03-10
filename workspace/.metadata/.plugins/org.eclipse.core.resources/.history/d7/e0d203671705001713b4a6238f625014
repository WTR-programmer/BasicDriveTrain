package org.usfirst.frc5124.WestTorranceSwagbotics2017.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousFuelForBlue extends CommandGroup {

    public AutonomousFuelForBlue() {
    	//move up ~2/3 or so the distance it takes to reach the base line starting from right next to the boiler
    	addSequential(new AutoDriveByPID(60));
    	//Turn towards boiler
    	addSequential(new AutoTurnWithGyro(20));
    	//move towards boiler
    	addSequential(new AutoDriveByPID(63));
    	//addSequential(new AutoTrashDrive(4, -1));
    	//turn on conveyor and shooters for rest of auto
    	addParallel(new FuelInjectorConveyorToShooters());
    	addSequential(new ShooterStartShooting());
    }
}
