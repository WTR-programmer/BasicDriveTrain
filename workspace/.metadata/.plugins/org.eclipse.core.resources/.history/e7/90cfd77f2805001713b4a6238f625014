package org.usfirst.frc5124.WestTorranceSwagbotics2017.commands;

import org.usfirst.frc5124.WestTorranceSwagbotics2017.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class AutoTurnWithGyro extends Command {
	
	double degrees;
	
    public AutoTurnWithGyro(double degrees) {
    	requires(Robot.drivetrain);
    	this.degrees = degrees;
    }

    protected void initialize() {
    	Robot.drivetrain.resetAllOutputs();
    	Robot.gyroPIDHandler.setSetpoint(degrees + Robot.gyroPIDHandler.getGyro());
    	Robot.gyroPIDHandler.enable();
    }
    
    protected void execute() {
    	Robot.drivetrain.setPIDOutputs();
    }

    protected boolean isFinished() {
    	return Robot.gyroPIDHandler.onTarget();
    }

    protected void end() {
    	Robot.gyroPIDHandler.disable();
    	Robot.drivetrain.stop();
    	Robot.drivetrain.resetAllOutputs();
    }

    protected void interrupted() {
    	end();
    }
}
