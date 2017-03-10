// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5124.JohnsonReboot.subsystems;

import org.usfirst.frc5124.JohnsonReboot.Robot;
import org.usfirst.frc5124.JohnsonReboot.RobotMap;
import org.usfirst.frc5124.JohnsonReboot.commands.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Drivetrain extends Subsystem {

   
    private final SpeedController left1 = RobotMap.drivetrainLeft1;
    private final SpeedController left2 = RobotMap.drivetrainLeft2;
    private final SpeedController right1 = RobotMap.drivetrainRight1;
    private final SpeedController right2 = RobotMap.drivetrainRight2;
    private final RobotDrive robotDrive = RobotMap.drivetrainRobotDrive;
    private final Compressor compressor = RobotMap.drivetrainCompressor;

    
    public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithJoystick());
    }
    
    public void arcade() {
    	robotDrive.arcadeDrive(Robot.oi.getDriver());
    }
    
    public void tank() {
    	robotDrive.tankDrive(Robot.oi.getDriver().getRawAxis(1), Robot.oi.getDriver().getRawAxis(5));
    }
    
    public void stop() {
    	robotDrive.arcadeDrive(0, 0);
    }
}

