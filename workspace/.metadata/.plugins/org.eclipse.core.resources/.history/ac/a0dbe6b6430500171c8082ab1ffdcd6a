package org.usfirst.frc5124.WestTorranceSwagbotics2017.commands;

import org.usfirst.frc5124.WestTorranceSwagbotics2017.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FuelInjectorIntake extends Command {

    public FuelInjectorIntake() {
    	requires(Robot.fuelInjector);
    }

    protected void initialize() {
    	Robot.fuelInjector.intake();
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
