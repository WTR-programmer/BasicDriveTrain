package org.usfirst.frc5124.VisionTesting;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

@SuppressWarnings("deprecation")
public class Main {

    public static void main(String[] args) {
        VideoCapture camera = Util.makeCamera(0, 640, 480, 0);
        Mat image = new Mat();
        GripPipeline pipe = new GripPipeline();
        PipeProcessor brain = new PipeProcessor();
        NetworkTable table = NetworkTableInstance.getDefault().getTable("GRIP");
        NetworkTableEntry anglePost = table.getEntry("Angle");
        NetworkTableEntry powerPost = table.getEntry("Power");
        while (true) {
            camera.read(image);
            pipe.process(image);
            brain.process(pipe);
            anglePost.setDouble(brain.getTurnAngle());
            powerPost.setDouble(brain.getPower());
        }
    }

}
