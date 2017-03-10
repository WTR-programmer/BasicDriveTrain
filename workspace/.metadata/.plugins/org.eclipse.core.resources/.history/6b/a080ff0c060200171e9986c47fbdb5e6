package org.usfirst.frc5124.WestTorranceSadbotics2017.subsystems;

import org.usfirst.frc5124.WestTorranceSadbotics2017.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooters extends Subsystem {

	private final CANTalon shooterLeftMotor = RobotMap.shooterLeftShooterMotor;
	private final CANTalon shooterCenterMotor = RobotMap.shooterCenterShooterMotor;
	private final CANTalon shooterRightMotor = RobotMap.shooterRightShooterMotor;

	public static final int shootingSpeedLeft = -15800;
	public static final int shootingSpeedCenter = -16200;
	public static final int shootingSpeedRight = -16200;
	 

	 	public void initDefaultCommand() {
	    }
	    
	 	public void setLeftShooter(double speed) {
	    	shooterLeftMotor.set(speed);
	    }
	    
	 	public void setCenterShooter(double speed) {
	    	shooterCenterMotor.set(speed);
	    }
	    
	    public void setRightShooter(double speed) {
	    	shooterRightMotor.set(speed);
	    }
	    
	    public void setAllShooters(double speed) {
	    	setLeftShooter(speed);
	    	setCenterShooter(speed);
	    	setRightShooter(speed);
	    }
	    
	    public void setControlMode(int mode) {
	    	shooterLeftMotor.setControlMode(mode);
	    	shooterCenterMotor.setControlMode(mode);
	    	shooterRightMotor.setControlMode(mode);
	    }
	    
	    public void enableControl() {
	    	
	    }
	    
	    public double getLeftVelocity() {
	    	return shooterLeftMotor.getEncVelocity();
	    }
	    
	    public double getCenterVelocity() {
	    	return shooterCenterMotor.getEncVelocity();
	    }
	    
	    public double getRightVelocity() {
	    	return shooterRightMotor.getEncVelocity();
	    }
	    
	    public int getShootingSpeedLeft() {
	    	return shootingSpeedLeft;
	    }
	    
	    public int getShootingSpeedCenter() {
	    	return shootingSpeedCenter;
	    }
	    
	    public int getShootingSpeedRight() {
	    	return shootingSpeedRight;
	    }
	    
	    public double getLeftCurrent() {
	    	return shooterLeftMotor.getOutputCurrent();
	    }
	    
	    public double getCenterCurrent() {
	    	return shooterCenterMotor.getOutputCurrent();
	    }
	    
	    public double getRightCurrent() {
	    	return shooterRightMotor.getOutputCurrent();
	    }
	    
	    public void displayDiagnostics() {
	    	SmartDashboard.putNumber("Left Velocity", shooterLeftMotor.getEncVelocity());
	    	SmartDashboard.putNumber("Left Target", getShootingSpeedLeft());
	    	SmartDashboard.putNumber("Left Delta", getShootingSpeedLeft() - shooterLeftMotor.getEncVelocity());
	        SmartDashboard.putNumber("Center Velocity", shooterCenterMotor.getEncVelocity());
	        SmartDashboard.putNumber("Center Target", getShootingSpeedCenter());
	        SmartDashboard.putNumber("Center Delta", getShootingSpeedCenter() - shooterCenterMotor.getEncVelocity());
	        SmartDashboard.putNumber("Right Velocity", shooterRightMotor.getEncVelocity());
	        SmartDashboard.putNumber("Right Target", getShootingSpeedRight());
	        SmartDashboard.putNumber("Right Delta", getShootingSpeedRight() - shooterRightMotor.getEncVelocity());
	    }
	    
}

