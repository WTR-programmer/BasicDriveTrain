package org.usfirst.frc5124.WestTorranceSadbotics2017.subsystems;

import org.usfirst.frc5124.WestTorranceSadbotics2017.RobotMap;
import org.usfirst.frc5124.WestTorranceSadbotics2017.commands.*;

import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends PIDSubsystem {

    private final SpeedController left1 = RobotMap.drivetrainLeft1;
    private final SpeedController left2 = RobotMap.drivetrainLeft2;
    private final SpeedController right1 = RobotMap.drivetrainRight1;
    private final SpeedController right2 = RobotMap.drivetrainRight2;
    private final RobotDrive robotDrive = RobotMap.drivetrainRobotDrive;
    private final ADXRS450_Gyro gyro = RobotMap.drivetrainGyro;
    
    public double direction = 1;
    public double turnSpeed = 0.75;
    
    public double ticksPerInch = 27143.360527;
    
    public Drivetrain() {
    	super(0.1 , 0, 0);
    	getPIDController().setContinuous(false);
    	getPIDController().setAbsoluteTolerance(1);
    	getPIDController().setOutputRange(-0.7, 0.7);
    }

    public void initDefaultCommand() {
    	setDefaultCommand(new JoystickPuppetry());
    }
    
    public void setSpeed(double speed) {
    	robotDrive.setMaxOutput(speed);
    }
    
    public void driveStop() {
    	robotDrive.arcadeDrive(0, 0);
    }
    
    public void drive(double drivePower, double driveTurn) {
    	robotDrive.arcadeDrive(drivePower, driveTurn);
    }

	@Override
	protected double returnPIDInput() {
		return gyro.getAngle();
	}

	@Override
	protected void usePIDOutput(double output) {
		robotDrive.arcadeDrive(0, output);
	}
	
	public double getGyro() {
		return gyro.getAngle();
	}
	
	public double getDirection() {
		return direction;
	}
	
	public void reverseFront() {
		direction = Math.copySign(1, -direction);
	}
	
	public void frontAndCenter() {
		direction = 1;
	}
    
	public void slowTurn() {
		turnSpeed = 0.75;
	}
	
	public void fastTurn() {
		turnSpeed = 0.8;
	}
	
	public double getTurnSpeed() {
		return turnSpeed;
	}
}

