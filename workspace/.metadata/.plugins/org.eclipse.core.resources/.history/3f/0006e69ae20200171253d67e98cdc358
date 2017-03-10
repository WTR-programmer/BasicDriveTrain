package org.usfirst.frc5124.WestTorranceSadbotics2017.commands;

import org.usfirst.frc5124.WestTorranceSadbotics2017.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DrivetrainTurnWithGyro extends Command {
	
	public double degrees;

    public DrivetrainTurnWithGyro(double degrees) {
        requires(Robot.drivetrain);
        this.degrees = degrees;
    }

    protected void initialize() {
    	Robot.drivetrain.setSetpoint(degrees + Robot.drivetrain.getGyro());
    	Robot.drivetrain.enable();
    }

    protected void execute() {
    }

    protected boolean isFinished() {
        return Robot.drivetrain.onTarget();
    }

    protected void end() {
    	Robot.drivetrain.disable();
    	Robot.drivetrain.driveStop();
    }

    protected void interrupted() {
    	end();
    }
}
