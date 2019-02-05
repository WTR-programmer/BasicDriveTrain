package org.usfirst.frc5124.VisionTesting;

import java.util.ArrayList;
import org.opencv.core.MatOfPoint;
import org.usfirst.frc5124.VisionTesting.GripPipeline.Line;

public class PipeProcessor {
    
    private double angle;
    private double power;

    public void process(GripPipeline pipe) {
        ArrayList<MatOfPoint> contours = pipe.filterContoursOutput();
        ArrayList<Line> lines = pipe.filterLinesOutput();
//        angle = useForThinking;//new value
        power = 0;//new value
    }

    public double getTurnAngle () {
        return angle;
    }

    public double getPower () {
        return power;
    }

}
