package org.usfirst.frc5124.WestTorranceSwagbotics2017;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Encoder;

public class RobotMap {
	
	//Left PIDF Values
	public static double leftKp = 0.05;
	public static double leftKi = 0;
	public static double leftKd = 0;
	public static double leftKf = 0.0365;
		
	//Center PIDF Values
	public static double centerKp = 0.04;
	public static double centerKi = 0;
	public static double centerKd = 0;
	public static double centerKf = 0.035;

	//Right PIDF Values
	public static double rightKp = 0.05;
	public static double rightKi = 0;
	public static double rightKd = 0;
	public static double rightKf = 0.0375;
	
	//Agitator
    public static VictorSP agitatorAgitatorMotor;
	
	//Gearage
	public static DoubleSolenoid gearHolderHolderSolenoid;
	public static DoubleSolenoid gearHolderPusherSolenoid;
	public static DoubleSolenoid gearHolderFunnelSolenoid;
	public static DoubleSolenoid gearHolderBackboardSolenoid;
	public static DigitalInput gearHolderLimitSwitch;
	
	//Fuel Injector
	public static VictorSP fuelInjectorFuelInjectorMotor;

	//Intake
	public static VictorSP intakeIntakeMotor;
	
	//Shooter
	public static CANTalon shooterLeftShooterMotor;
	public static CANTalon shooterCenterShooterMotor;
	public static CANTalon shooterRightShooterMotor;
	
	//Hanger
	public static CANTalon hangerHangerMotor;
	
	//DriveTrain
	public static Compressor drivetrainCompressor;
	public static Encoder drivetrainLeftEncoder;
	public static Encoder drivetrainRightEncoder;
	public static ADIS16448_IMU drivetrainIMU;
	public static ADXL362 drivetrainAccelerometer; 
	public static ADXRS450_Gyro drivetrainGyro;
    public static VictorSP drivetrainLeft1;
    public static VictorSP drivetrainLeft2;
    public static VictorSP drivetrainRight1;
    public static VictorSP drivetrainRight2;
    public static RobotDrive drivetrainRobotDrive;

    public static void init() {
    	
    	///////////////////////
    	///AGITATOR HARDWARE///
    	///////////////////////
    	
    	agitatorAgitatorMotor = new VictorSP(6);
    	LiveWindow.addActuator("Agitator", "Agitator Motor", agitatorAgitatorMotor);
    	
    	
    	//////////////////////////
    	///GEAR HOLDER HARDWARE///
    	//////////////////////////
    	
    	gearHolderHolderSolenoid = new DoubleSolenoid(2, 7);
    	LiveWindow.addActuator("Gear Holder", "Holder Solenoid", gearHolderHolderSolenoid);
    	
    	gearHolderPusherSolenoid = new DoubleSolenoid(3, 6);
    	LiveWindow.addActuator("Gear Holder", "Pusher Solenoid", gearHolderPusherSolenoid);
    	
    	gearHolderFunnelSolenoid = new DoubleSolenoid(4, 5);
    	LiveWindow.addActuator("Gear Holder", "Funnel Solenoid", gearHolderFunnelSolenoid);
    	
    	gearHolderBackboardSolenoid = new DoubleSolenoid(0, 1);
    	LiveWindow.addActuator("Gear Holder", "Backboard Solenoid", gearHolderBackboardSolenoid);
    	
    	gearHolderLimitSwitch = new DigitalInput(5);
    	
    	
    	////////////////////////////
    	///FUEL INJECTOR HARDWARE///
    	////////////////////////////
    	
    	fuelInjectorFuelInjectorMotor = new VictorSP(4);
    	LiveWindow.addActuator("Fuel Injector", "Fuel Injector Motor", fuelInjectorFuelInjectorMotor);
    	
    	
    	/////////////////////
    	///INATKE HARDWARE///
    	/////////////////////
    	
    	intakeIntakeMotor = new VictorSP(5);
    	LiveWindow.addActuator("Intake", "Intake Motor", intakeIntakeMotor);
    	
    	
    	//////////////////////
    	///SHOOTER HARDWARE///
    	//////////////////////
    	
    	shooterLeftShooterMotor = new CANTalon(3);
    	shooterLeftShooterMotor.enableBrakeMode(false);
    	shooterLeftShooterMotor.setProfile(0);
    	shooterLeftShooterMotor.setP(leftKp);
    	shooterLeftShooterMotor.setI(leftKi);
    	shooterLeftShooterMotor.setD(leftKd);
    	shooterLeftShooterMotor.setF(leftKf);
    	shooterLeftShooterMotor.setAllowableClosedLoopErr(50);
    	shooterLeftShooterMotor.setControlMode(0);
    	shooterLeftShooterMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	LiveWindow.addActuator("Shooter", "Left Shooter", shooterLeftShooterMotor);
    	
    	shooterCenterShooterMotor = new CANTalon(4);
    	shooterCenterShooterMotor.enableBrakeMode(false);
    	shooterCenterShooterMotor.setProfile(0);
    	shooterCenterShooterMotor.setP(centerKp);
    	shooterCenterShooterMotor.setI(centerKi);
    	shooterCenterShooterMotor.setD(centerKd);
    	shooterCenterShooterMotor.setF(centerKf);
    	shooterCenterShooterMotor.setAllowableClosedLoopErr(50);
    	shooterCenterShooterMotor.setControlMode(0);
    	shooterCenterShooterMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	LiveWindow.addActuator("Shooter", "Center Shooter", shooterCenterShooterMotor);
    	
    	shooterRightShooterMotor = new CANTalon(5);
    	shooterRightShooterMotor.enableBrakeMode(false);
    	shooterRightShooterMotor.setProfile(0);
    	shooterRightShooterMotor.setP(rightKp);
    	shooterRightShooterMotor.setI(rightKi);
    	shooterRightShooterMotor.setD(rightKd);
    	shooterRightShooterMotor.setF(rightKf);
    	shooterRightShooterMotor.setAllowableClosedLoopErr(50);
    	shooterRightShooterMotor.setControlMode(0);
    	shooterRightShooterMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	LiveWindow.addActuator("Shooter", "Right Shooter", shooterRightShooterMotor);
    	
    	
    	/////////////////////
    	///HANGER HARDWARE///
    	/////////////////////
    	
    	hangerHangerMotor = new CANTalon(6);
    	LiveWindow.addActuator("Hanger", "Hanger Motor", hangerHangerMotor);
    	
    	
    	/////////////////////////
    	///DRIVETRAIN HARDWARE///
    	/////////////////////////
    	
    	drivetrainCompressor = new Compressor(0);
    	
    	drivetrainLeftEncoder = new Encoder(8, 9);
    	drivetrainRightEncoder = new Encoder(6, 7);
    	
    	drivetrainIMU = new ADIS16448_IMU();
    	
    	drivetrainGyro = new ADXRS450_Gyro();
        LiveWindow.addSensor("Gyro", "Gyro", drivetrainGyro);
        
        drivetrainAccelerometer = new ADXL362(Range.k2G);
    
        drivetrainLeft1 = new VictorSP(0);
        LiveWindow.addActuator("Drivetrain", "Left 1", drivetrainLeft1);
        
        drivetrainLeft2 = new VictorSP(1);
        LiveWindow.addActuator("Drivetrain", "Left 2", drivetrainLeft2);
        
        drivetrainRight1 = new VictorSP(3);
        LiveWindow.addActuator("Drivetrain", "Right 1", drivetrainRight1);
        
        drivetrainRight2 = new VictorSP(2);
        LiveWindow.addActuator("Drivetrain", "Right 2", drivetrainRight2);
        
        drivetrainRobotDrive = new RobotDrive(drivetrainLeft1, drivetrainLeft2,
              drivetrainRight1, drivetrainRight2);
        
        drivetrainRobotDrive.setSafetyEnabled(false);
        drivetrainRobotDrive.setExpiration(0.1);
        drivetrainRobotDrive.setSensitivity(1);
        drivetrainRobotDrive.setMaxOutput(1);
        drivetrainRobotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        drivetrainRobotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        drivetrainRobotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        drivetrainRobotDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);

    }
}
