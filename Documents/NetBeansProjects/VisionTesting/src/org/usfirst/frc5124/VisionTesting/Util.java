/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc5124.VisionTesting;

import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

/**
 *
 * @author wtrob_000
 */
public class Util {
    
    public static VideoCapture makeCamera(int device, int width, int height, double exposure) {
        VideoCapture camera = new VideoCapture(device);
        camera.set(Videoio.CAP_PROP_FRAME_WIDTH, width);
        camera.set(Videoio.CAP_PROP_FRAME_HEIGHT, height);
        if (exposure > -1.0) {
            camera.set(Videoio.CAP_PROP_AUTO_EXPOSURE, 0);
            camera.set(Videoio.CAP_PROP_EXPOSURE, exposure);
        }
        if (!camera.isOpened()) {
            throw new RuntimeException("Camera will not open");
        }
        return camera;
    }
    
}
