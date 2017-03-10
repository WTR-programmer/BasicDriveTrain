package org.usfirst.frc5124.WestTorranceSadbotics2017;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5124.WestTorranceSadbotics2017.commands.*;
import org.usfirst.frc5124.WestTorranceSadbotics2017.subsystems.*;

public class Robot extends IterativeRobot {

    Command autonomousCommand;

    public static OI oi;
    public static Drivetrain drivetrain;
    public static GearHolder gearHolder; 
    public static Shooters shooters; 
   

    public void robotInit() {
    	RobotMap.init();
        drivetrain = new Drivetrain();
        gearHolder = new GearHolder();
        shooters = new Shooters();
        
        oi = new OI();
        
        RobotMap.drivetrainGyro.calibrate();
        RobotMap.pdp.clearStickyFaults();
    }

    public void disabledInit(){
    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    public void autonomousInit() {
        if (autonomousCommand != null) autonomousCommand.start();
    }

    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
        if (autonomousCommand != null) autonomousCommand.cancel();
        drivetrain.disable();
        Robot.drivetrain.frontAndCenter();
        Robot.drivetrain.slowTurn();
    }

    public void teleopPeriodic() {
    	SmartDashboard.putNumber("Velocity", Robot.shooters.getCenterVelocity());
        Scheduler.getInstance().run();
        
        Robot.shooters.displayDiagnostics();
        
        SmartDashboard.putData("gyro", RobotMap.drivetrainGyro);
        
        if(Robot.oi.getDriver().getRawButton(5)) {
        	Robot.drivetrain.setSpeed(1);
        	Robot.drivetrain.fastTurn();
        } else {
        	Robot.drivetrain.setSpeed(0.65);
        	Robot.drivetrain.slowTurn();
        }
    }

    public void testPeriodic() {
        LiveWindow.run();
    }
}
