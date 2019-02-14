package org.usfirst.frc5124.VisionTesting;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

/**
 * Entry-point class of the Raspberry pi java program for Deep Space.
 * 
 * This program takes the stress off the roboRio with two functionalities, both
 * implemented through network tables.
 * 
 * @author wtrob_000
 */
public class Main {
    
    public static final int IMG_WIDTH = 640;
    public static final int IMG_HEIGHT = 480;
    public static final int IMG_FOV_H = (int) Math.toRadians(45);
    
    public static void main(String[] args) {
        VideoCapture camera = Util.makeCamera(0, 640, 480, 0);
        Mat image = new Mat();
        GripPipeline pipe = new GripPipeline();
        PipeProcessor brain = new PipeProcessor();
        InstructionPipeline spliner = new InstructionPipeline();
        NetworkTable gripTable = NetworkTableInstance.getDefault().getTable("GRIP");
        NetworkTableEntry anglePost = gripTable.getEntry("Angle");
        NetworkTableEntry powerPost = gripTable.getEntry("Power");
        NetworkTable splineTable = NetworkTableInstance.getDefault().getTable("LocationCalculation");
        NetworkTableEntry requestedX = splineTable.getEntry("TargetX");
        NetworkTableEntry requestedY = splineTable.getEntry("TargetY");
        NetworkTableEntry requestedAngle = splineTable.getEntry("TargetAngle");
        NetworkTableEntry targetSpeed = splineTable.getEntry("TargetSpeed");
        NetworkTableEntry splineAngle = splineTable.getEntry("DriveAngle");
        NetworkTableEntry splinePower = splineTable.getEntry("DrivePower");
        while (true) {
            camera.read(image);
            pipe.process(image);
            brain.process(pipe);
            spliner.process(
                    brain.getCameraTargetX(),
                    brain.getCameraTargetY(),
                    brain.getCameraTargetDirection(),
                    0.3
            );
            anglePost.setDouble(spliner.getAngle());
            powerPost.setDouble(spliner.getPower());
            double reqX = requestedX.getDouble(0);
            double reqY = requestedY.getDouble(0);
            double reqAngle = 90 - Math.toDegrees(requestedAngle.getDouble(0));
            if (reqX != 0 || reqY != 0 || reqAngle != 0) {
                spliner.process(reqX, reqY, reqAngle, targetSpeed.getDouble(1));
                splineAngle.setDouble(spliner.getAngle());
                splinePower.setDouble(spliner.getPower());
            }
        }
    }
    
    private static final NetworkTable ROBOT_INFO = NetworkTableInstance.getDefault()
            .getTable("RobotInformation");
    private static final NetworkTableEntry GYRO_Y = ROBOT_INFO.getEntry("GyroY");
    private static final NetworkTableEntry GYRO_RATE = ROBOT_INFO.getEntry("GyroRate");
    private static final NetworkTableEntry LEFT_ENC = ROBOT_INFO.getEntry("LeftEncoder");
    private static final NetworkTableEntry RIGHT_ENC = ROBOT_INFO.getEntry("RightEncoder");
    private static final NetworkTableEntry LEFT_VOLT = ROBOT_INFO.getEntry("LeftVolt");
    private static final NetworkTableEntry RIGHT_VOLT = ROBOT_INFO.getEntry("RightVolt");
    
    public static class DriveTrain {
        public double gyroY;
        public double gyroRate;
        public double leftEnc;
        public double rightEnc;
        public double leftVolt;
        public double rightVolt;
    }
    
    public static DriveTrain getDriveInfo () {
        DriveTrain currentInfo = new DriveTrain();
        currentInfo.gyroY = GYRO_Y.getDouble(0);
        currentInfo.gyroRate = GYRO_RATE.getDouble(0);
        currentInfo.leftEnc = LEFT_ENC.getDouble(0);
        currentInfo.rightEnc = RIGHT_ENC.getDouble(0);
        currentInfo.leftVolt = LEFT_VOLT.getDouble(0);
        currentInfo.rightVolt = RIGHT_VOLT.getDouble(0);
        return currentInfo;
    }
    
}