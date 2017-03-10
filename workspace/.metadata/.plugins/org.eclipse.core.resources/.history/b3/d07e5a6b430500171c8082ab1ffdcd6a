package org.usfirst.frc5124.WestTorranceSwagbotics2017.commands;

import org.usfirst.frc5124.WestTorranceSwagbotics2017.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FuelInjectorExhaust extends Command {

    public FuelInjectorExhaust() {
    	requires(Robot.fuelInjector);
    }

    protected void initialize() {
    	Robot.fuelInjector.exhaust();
    	Robot.fuelInjector.extract();
    }

    protected void execute() {
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    }

    protected void interrupted() {
    }
}
