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
        NetworkTable gripTable = NetworkTableInstance.getDefault().getTable("GRIP");
        NetworkTableEntry anglePost = gripTable.getEntry("Angle");
        NetworkTableEntry powerPost = gripTable.getEntry("Power");
        NetworkTable splineTable = NetworkTableInstance.getDefault().getTable("LocationCalculation");
        NetworkTableEntry requestedX = splineTable.getEntry("TargetX");
        NetworkTableEntry requestedY = splineTable.getEntry("TargetY");
        NetworkTableEntry requestedAngle = splineTable.getEntry("TargetAngle");
        NetworkTableEntry splineAngle = splineTable.getEntry("DriveAngle");
        NetworkTableEntry splinePower = splineTable.getEntry("DrivePower");
        while (true) {
            camera.read(image);
            pipe.process(image);
            brain.process(pipe);
            anglePost.setDouble(brain.getTurnAngle());
            powerPost.setDouble(brain.getPower());
            double reqX = requestedX.getDouble(0);
            double reqY = requestedY.getDouble(0);
            double reqAngle = requestedAngle.getDouble(0);
            if (reqX != 0 || reqY != 0 || reqAngle != 0) {
                InstructionPipeline spliner = new InstructionPipeline();
                spliner.process(reqX, reqY, reqAngle);
                splineAngle.setDouble(spliner.getAngle());
                splinePower.setDouble(spliner.getPower());
            }
        }
    }

}
