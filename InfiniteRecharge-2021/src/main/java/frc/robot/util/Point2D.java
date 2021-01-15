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

    public String toString() {
        return "(" + Double.valueOf(x).toString() + ", " + Double.valueOf(y).toString() + "): " + Double.valueOf(heading).toString();
    }
}
