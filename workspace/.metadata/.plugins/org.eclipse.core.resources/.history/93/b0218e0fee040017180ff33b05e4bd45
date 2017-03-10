package org.usfirst.frc5124.WestTorranceSwagbotics2017;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.usfirst.frc5124.WestTorranceSwagbotics2017.subsystems.*;

public class Robot extends IterativeRobot {

    Command autonomousCommand;

    public static OI oi;
    public static Agitator agitator;
    public static GearHolder gearHolder;
    public static FuelInjector fuelInjector;
    public static Shooter shooter;
    public static Hanger hanger;
    public static Drivetrain drivetrain;
    public static GyroPIDHandler gyroPIDHandler;
    public static EncoderPIDHandler encoderPIDHandler;
    public static boolean button6IsPressed;
    public static boolean button8IsPressed;

    public void robotInit() {
    	
    	RobotMap.init();
        
    	agitator = new Agitator();
        gearHolder = new GearHolder();
        fuelInjector = new FuelInjector();
        shooter = new Shooter();
        hanger = new Hanger();
        drivetrain = new Drivetrain();
        gyroPIDHandler = new GyroPIDHandler();
        encoderPIDHandler = new EncoderPIDHandler();
        
        oi = new OI();
        
        CameraServer.getInstance().startAutomaticCapture();
    }

    public void disabledInit(){
    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    public void autonomousInit() {
    	Robot.drivetrain.setDrivetrainSpeed(1);
    	
    	Robot.drivetrain.frontAndCenter();
        
    	if(oi.getAuto4()) {
    		if(oi.getAuto3() && oi.getAuto1() && oi.getAuto2()) {
    			autonomousCommand = null;
    		} else if(oi.getAuto3() && oi.getAuto1() && !oi.getAuto2()) {
    			autonomousCommand = null;
    		} else if(oi.getAuto3() && !oi.getAuto1() && oi.getAuto2()) {
    			autonomousCommand = null;
    		} else if(oi.getAuto3() && !oi.getAuto1() && !oi.getAuto2()) {
    			autonomousCommand = null;
    		} else if(!oi.getAuto3() && oi.getAuto1() && oi.getAuto2()) {
    			autonomousCommand = null;
    		} else if(!oi.getAuto3() && oi.getAuto1() && !oi.getAuto2()) {
    			autonomousCommand = null;
    		} else if(!oi.getAuto3() && !oi.getAuto1() && oi.getAuto2()) {
    			autonomousCommand = null;
    		} else if(!oi.getAuto3() && !oi.getAuto1() && !oi.getAuto2()) {
    			autonomousCommand = null;
    		}
    	} else {
    		autonomousCommand = null;
    	}
    	
    	if (autonomousCommand != null) autonomousCommand.start();
        
   
    }

    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        Timer.delay(0.005);
    }

    public void teleopInit() {
    	Robot.gyroPIDHandler.disable();
    	Robot.encoderPIDHandler.disable();
    	Robot.drivetrain.resetAllOutputs();
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        
        if(oi.getDriver().getRawButton(5)) {
        	drivetrain.setSpeed(1);
        	drivetrain.fastTurn();
        } else {
        	drivetrain.setSpeed(0.65);
        	drivetrain.slowTurn();
        }
        
        if(oi.getDriver().getRawButton(6) && !button6IsPressed) {
        	button6IsPressed = true;
        	shooter.shootingSpeedLeft -= 200;
        	shooter.shootingSpeedCenter -= 200;
        	shooter.shootingSpeedLeft -= 200;
        } else if(oi.getDriver().getRawButton(6)) {
        	button6IsPressed = false;
        }
        
        if(oi.getDriver().getRawButton(8) && !button8IsPressed) {
        	button8IsPressed = true;
        	shooter.shootingSpeedLeft += 200;
        	shooter.shootingSpeedCenter += 200;
        	shooter.shootingSpeedLeft += 200;
        } else if(oi.getDriver().getRawButton(8)) {
        	button8IsPressed = false;
        }
        
        Timer.delay(0.005);
        
        Robot.drivetrain.frontAndCenter();
        
        Robot.drivetrain.slowTurn();       
    }
    
    public void testPeriodic() {
        LiveWindow.run();
    }
}
