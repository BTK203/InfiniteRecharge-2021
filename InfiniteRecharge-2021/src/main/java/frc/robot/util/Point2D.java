// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * Simple class structure that holds a point in the XY plane.
 */
public class Point2D {
    private double
        x,
        y,
        heading;

    /**
     * Creates a new Point2D.
     * @param x X-coordinate of the point.
     * @param y Y-coordinate of the point.
     * @param heading Heading of the point.
     */
    public Point2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Returns the X-coordinate of the point.
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the Y-coordinate of the point.
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the heading of the point.
     */
    public double getHeading() {
        return heading;
    }

    /**
     * Returns the distance from the given point.
     * @param point The point to measure distance to.
     * @return The distance between this point and the passed point.
     */
    public double getDistanceFrom(Point2D point) {
        //pythagorean theorem moment
        double xDist = point.getX() - this.x;
        double yDist = point.getY() - this.y;
        return Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
    }

    /**
     * Returns the heading needed to point at the passed point.
     * @param point The point to calculate the heading to.
     * @return The heading between this point and the passed point.
     */
    public double getHeadingTo(Point2D point) {
        double displacementX = point.getX() - getX();
        double displacementY = point.getY() - getY();
        double targetHeading = Math.toDegrees(Math.atan2(displacementY, displacementX));
        return targetHeading;
    }

    /**
     * Returns a representation of this point in String format.
     * Format: [x],[y],[heading]
     * The output from this method can be used in the Point2D.fromString() method.
     */
    public String toString() {
        double roundedX = Util.roundTo(this.getX(), 2);
        double roundedY = Util.roundTo(this.getY(), 2);
        return Double.valueOf(roundedX).toString() + "," + Double.valueOf(roundedY).toString() + "," + Double.valueOf(heading).toString();
    }

    /**
     * Returns a Point2D from a given String input
     * - Expected Format: [x],[y],[heading]
     */
    public static Point2D fromString(String input) {
        String[] parts = input.split(",");
        double x = Double.valueOf(parts[0]).doubleValue();
        double y = Double.valueOf(parts[1]).doubleValue();
        double heading = Double.valueOf(parts[2]).doubleValue();
        return new Point2D(x, y, heading);
    }
}
