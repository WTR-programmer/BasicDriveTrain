/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc5124.VisionTesting;

public class InstructionPipeline {
    
    private double power;
    private double angle;
    
    private double lastL;
    private double deltaL;
    private double lastR;
    private double deltaR;
    
    private double x;
    private double y;
    
    private static final double TURN_P = 1.0;
    private static final double TURN_I = 0.1;
    private static final double TURN_D = 0.0;
    private static final double DRIVE_P = 1.0;
    private static final double DRIVE_I = 0.05;
    private static final double DRIVE_D = 0.0;
    
    private double iAccum = 0;
    private Double dLast = null;
    
    private TargetingState state = null;
    
    private enum TargetingState {
        NEAR_DRIVE,
        FAR_DRIVE,
        NEAR_TURN,
        FAR_TURN
    }
    
    /**
     * Do the math so you can get the answer later.
     * @param targetX current is 0
     * @param targetY current is 0
     * @param targetAngle in radians, current is (pi/2)
     * @param targetSpeed for speed output determination
     */
    public void process (double targetX, double targetY, double targetAngle, double targetSpeed) {
        Main.DriveTrain driveInfo = Main.getDriveInfo();
        double dstError = Math.hypot(targetX, targetY);
        if (dstError < 0.8) {
            // NEAR
            double error = Math.atan2(targetY, targetX) - Math.PI / 2;
            if (Math.abs(error) < 0.03 * dstError) {
                state = TargetingState.NEAR_DRIVE;
                power = targetSpeed;
                angle = 0;
                return;
            }
            if (state != TargetingState.NEAR_TURN) {
                state = TargetingState.NEAR_TURN;
                iAccum = 0;
                dLast = null;
            }
            iAccum += error * TURN_I;
            power = 0;
            angle = TURN_P * error + iAccum + TURN_D * (error - dLast);
            dLast = error;
            return;
        }
        // FAR
        targetX -= 0.6 * Math.cos(targetAngle);
        targetY -= 0.6 * Math.sin(targetAngle);
        double angError = Math.atan2(targetY, targetX) - Math.PI / 2;
        dstError = Math.hypot(targetX, targetY);
        if (Math.abs(angError) < 0.1 * dstError) {
            if (state != TargetingState.FAR_TURN) {
                state = TargetingState.FAR_TURN;
                iAccum = 0;
                dLast = null;
            }
            iAccum += angError * TURN_I;
            power = 0;
            angle = TURN_P * angError + iAccum + TURN_D * (angError - dLast);
            dLast = angError;
            return;
        }
        if (state != TargetingState.FAR_DRIVE) {
            state = TargetingState.FAR_DRIVE;
            iAccum = 0;
            dLast = null;
        }
        iAccum += dstError * DRIVE_I;
        power = DRIVE_P * dstError + iAccum + DRIVE_D * (dstError - dLast);
        angle = 0;
        dLast = dstError;
    }
    
    public void process (double targetX, double targetY, double targetAngle) {
        process(targetX, targetY, targetAngle, 1.0);
    }
    
    public void process (java.awt.geom.Point2D.Double position, double targetAngle, double targetSpeed) {
        process(position.x, position.y, targetAngle, targetSpeed);
    }
    
    public void process (java.awt.geom.Point2D.Double position, double targetAngle) {
        process(position.x, position.y, targetAngle);
    }
    
    public double getPower () {
        return power;
    }
    
    /**
     * Angle to drive
     * @return between -1 and 1
     */
    public double getAngle () {
        return angle;
    }
    
}