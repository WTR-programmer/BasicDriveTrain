package org.usfirst.frc5124.VisionTesting;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.TreeSet;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc5124.VisionTesting.GripPipeline.Line;

public class PipeProcessor {
    
    private double angle;
    private double power;

    private static final double LINE_ENDPOINT_COMBINE_DISTANCE = 1;
    private static final double LINE_SLOPE_COMBINE_ANGLE_DIFFERENCE = 15;
    
    public void process(GripPipeline pipe) {
        ArrayList<MatOfPoint> contours = pipe.filterContoursOutput();
        ArrayList<Line> lines = pipe.filterLinesOutput();
        lines = combineSameLines(lines);
        TreeSet<TapeRectangle> tapes = new TreeSet<>();
        for (MatOfPoint contour : contours) {
            RotatedRect boundingRect = Imgproc.fitEllipseDirect(contour);
            Point2D.Double center = new Point2D.Double(
                    boundingRect.center.x, boundingRect.center.y);
            Line l1 = null;
            double l1dst = Double.MAX_VALUE;
            Line l2 = null;
            double l2dst = Double.MAX_VALUE;
            for (Line l : lines) {
                Point2D.Double centerOfLine = new Point2D.Double((l.x1+l.x2)/2, (l.y1+l.y2)/2);
                double dst = centerOfLine.distance(center);
                if (dst < l1dst) {
                    l2 = l1;
                    l2dst = l1dst;
                    l1 = l;
                    l1dst = dst;
                } else if (dst < l2dst) {
                    l2 = l;
                    l2dst = dst;
                }
            }
            TapeRectangle tape = new TapeRectangle(contour, l1, l2);
            if (Math.abs(tape.location.yRotation) > 0.25) {
                continue;
            }
            tapes.add(tape);
        }
        ArrayList<Target> targets = new ArrayList<>();
        TapeRectangle.SideOfTarget lastSide = null;
        TapeRectangle last = null;
        for (TapeRectangle rect : tapes) {
            if (lastSide == TapeRectangle.SideOfTarget.LEFT && last != null &&
                    rect.getSideOfTarget() == TapeRectangle.SideOfTarget.RIGHT) {
                targets.add(new Target(rect, last));
            }
            last = rect;
            lastSide = rect.getSideOfTarget();
        }
        double closestTargX = Double.MAX_VALUE;
        Point3D target = null;
        Double targAngle = null;
        for (Target pt : targets) {
            double targX = Math.abs(new Vector3D(pt.getCenter()).xRotation);
            if (targX < closestTargX) {
                target = pt.getCenter();
                targAngle = pt.getAngle();
                closestTargX = targX;
            }
        }
        if (target == null) {
            angle = 0;
            power = 0;
            return;
        }
        InstructionPipeline spliner = new InstructionPipeline();
        spliner.process(target.x, target.z, targAngle);
        angle = spliner.getAngle();
        power = spliner.getPower();
    }
    
    private static ArrayList<Line> combineSameLines(ArrayList<Line> lines) {
        ArrayList<Line> newLines = new ArrayList<>();
        for (Line possNewLine : lines) {
            Point2D.Double pt1 = new Point2D.Double(possNewLine.x1, possNewLine.y1);
            Point2D.Double pt2 = new Point2D.Double(possNewLine.x2, possNewLine.y2);
            for (Line oldLine : newLines) {
                if (Math.abs(oldLine.angle() - possNewLine.angle()) >
                        LINE_SLOPE_COMBINE_ANGLE_DIFFERENCE) {
                    continue;
                }
                if (pt1.distance(oldLine.x1, oldLine.y1) <= LINE_ENDPOINT_COMBINE_DISTANCE) {
                    newLines.remove(oldLine);
                    possNewLine = new Line(oldLine.x2, oldLine.y2, possNewLine.x2, possNewLine.y2);
                }
                if (pt2.distance(oldLine.x1, oldLine.y1) <= LINE_ENDPOINT_COMBINE_DISTANCE) {
                    newLines.remove(oldLine);
                    possNewLine = new Line(oldLine.x2, oldLine.y2, possNewLine.x1, possNewLine.y1);
                }
                if (pt1.distance(oldLine.x2, oldLine.y2) <= LINE_ENDPOINT_COMBINE_DISTANCE) {
                    newLines.remove(oldLine);
                    possNewLine = new Line(oldLine.x1, oldLine.y1, possNewLine.x2, possNewLine.y2);
                }
                if (pt2.distance(oldLine.x2, oldLine.y2) <= LINE_ENDPOINT_COMBINE_DISTANCE) {
                    newLines.remove(oldLine);
                    possNewLine = new Line(oldLine.x1, oldLine.y1, possNewLine.x1, possNewLine.y1);
                }
            }
            newLines.add(possNewLine);
        }
        return newLines;
    }

    public double getTurnAngle () {
        return angle;
    }

    public double getPower () {
        return power;
    }
    
    public static class TapeRectangle implements Comparable<TapeRectangle> {
        
        private static final double DISTANCE_CONSTANT = 1;
        
        private final Vector3D location;
        private final RotatedRect contourRect;
        private final RotatedRect linesRect;
        private final RotatedRect combinedRect;
        
        public TapeRectangle (MatOfPoint contour, Line line1, Line line2) {
            Point2D.Double pt11 = new Point2D.Double(line1.x1, line1.y1);
            Point2D.Double pt12 = new Point2D.Double(line1.x2, line1.y2);
            Point2D.Double pt21;
            Point2D.Double pt22;
            if (pt11.distance(line2.x1, line2.y1) < pt11.distance(line2.x2, line2.y2)) {
                pt21 = new Point2D.Double(line2.x1, line2.y1);
                pt22 = new Point2D.Double(line2.x2, line2.y2);
            } else {
                pt21 = new Point2D.Double(line2.x2, line2.y2);
                pt22 = new Point2D.Double(line2.x1, line2.y1);
            }
            Line line3 = new Line(pt11.x, pt11.y, pt21.x, pt21.y);
            Line line4 = new Line(pt12.x, pt12.y, pt22.x, pt22.y);
            contourRect = Imgproc.fitEllipseDirect(contour);
            Point2D.Double centerOfLine1 = new Point2D.Double(
                    (line1.x1+line1.x2)/2, (line1.y1+line1.y2)/2);
            Point2D.Double centerOfLine2 = new Point2D.Double(
                    (line2.x1+line2.x2)/2, (line2.y1+line2.y2)/2);
            Point2D.Double centerOfLines = new Point2D.Double(
                    (centerOfLine1.x+centerOfLine2.x)/2, (centerOfLine1.y+centerOfLine2.y)/2);
            linesRect = new RotatedRect(
                    new Point(centerOfLines.x, centerOfLines.y),
                    new Size((line1.length()+line2.length())/2, (line3.length()+line4.length())/2),
                    (line1.angle()+line2.angle())/2);
            combinedRect = new RotatedRect(
                    new Point(
                            (contourRect.center.x+linesRect.center.x)/2,
                            (contourRect.center.x+linesRect.center.x)/2),
                    new Size(
                            (contourRect.size.width+linesRect.size.width)/2,
                            (contourRect.size.height+linesRect.size.height)/2),
                    (contourRect.angle+linesRect.angle)/2);
            double u = combinedRect.center.x;
            double v = combinedRect.center.y;
            double cx = Main.IMG_WIDTH / 2 - 0.5;
            double cy = Main.IMG_HEIGHT / 2 - 0.5;
            double f = Main.IMG_WIDTH / (2 * Math.tan(Main.IMG_FOV_H / 2));
            location = new Vector3D(
                    Math.atan((u-cx)/f),
                    Math.atan((v-cy)/f),
                    DISTANCE_CONSTANT/Math.sqrt(combinedRect.size.area()));
        }
        
        public Vector3D getLocation () {
            return location;
        }
        
        public SideOfTarget getSideOfTarget () {
            return linesRect.angle > 0 ? SideOfTarget.LEFT : SideOfTarget.RIGHT;
        }

        @Override
        public int compareTo(TapeRectangle o) {
            return this.location.xRotation - o.location.xRotation > 0 ? 1 : -1;
        }
        
        public static enum SideOfTarget {
            LEFT,
            RIGHT
        }
        
    }
    
    public static class Target {
        private TapeRectangle left;
        private TapeRectangle right;

        public Target(TapeRectangle left, TapeRectangle right) {
            this.left = left;
            this.right = right;
        }
        
        public Point3D getCenter () {
            return Point3D.getAverage(new Point3D(left.location), new Point3D(right.location));
        }
        
        public double getAngle () {
            Point3D r = new Point3D(right.location);
            Point3D l = new Point3D(left.location);
            return Math.atan2(r.x-l.x, r.y-l.y);
        }
    }
    
    public static class Vector3D {
        public double xRotation;
        public double yRotation;
        public double zDistance;

        public Vector3D(double xRotation, double yRotation, double zDistance) {
            this.xRotation = xRotation;
            this.yRotation = yRotation;
            this.zDistance = zDistance;
        }
        
        public Vector3D(Point3D rectangularPosition) {
            double x = rectangularPosition.x;
            double y = rectangularPosition.y;
            double z = rectangularPosition.z;
            this.zDistance = Math.hypot(Math.hypot(x, y), z);
            this.yRotation = Math.atan2(y, z);
            this.xRotation = Math.atan2(x, z);
        }
    }
    
    /**
     * Functions assume z to be depth and y to be vertical height.
     */
    public static class Point3D {
        public double x;
        public double y;
        public double z;

        public Point3D(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        
        public Point3D(Vector3D polarPos) {
            double v = polarPos.yRotation;
            double h = polarPos.xRotation;
            double r = polarPos.zDistance;
            this.x = r * Math.sin(h);
            this.y = r * Math.sin(v);
            this.z = Math.sqrt(r*r - x*x - y*y);
        }
        
        public static Point3D getAverage (Point3D point1, Point3D point2) {
            return new Point3D(
                    (point1.x + point2.x) / 2,
                    (point1.y + point2.y) / 2,
                    (point1.z + point2.z) / 2
            );
        }
    }

}
