// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Simple class structure that holds a point in the XY plane.
 */
public class Point2D {
    private double
        x,
        y,
        heading;

    public Point2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public double getDistanceFrom(Point2D point) {
        //pythagorean theorem moment
        double xDist = point.getX() - this.x;
        double yDist = point.getY() - this.y;
        return Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
    }

    public String toString() {
        double roundedX = Util.roundTo(this.getX(), 2);
        double roundedY = Util.roundTo(this.getY(), 2);
        return "(" + Double.valueOf(roundedX).toString() + ", " + Double.valueOf(roundedY).toString() + "): " + Double.valueOf(heading).toString();
    }
}
