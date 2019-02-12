/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc5124.VisionTesting;

/**
 * Splines, hell yeah.
 */
public class InstructionPipeline {
    
    private double power;
    private double angle;
    
    public void process (double targetX, double targetY, double targetAngle, double targetSpeed) {
        power = 0;
        angle = 0;
        // TODO make this do actual stuff
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
    
    public double getAngle () {
        return angle;
    }
    
}