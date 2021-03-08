// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOException;
import java.nio.file.Files;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Represents a Path that can be rendered on the screen.
 */
public class Path {
    private Point2D[] points;
    private boolean valid;

    /**
     * Creates a new path from the given file path.
     * @param file The path to the file to read from.
     * @param color The color of the path.
     */
    public Path(String file) {
        this.valid = false;
        try {
            String fileContents = Files.readString(java.nio.file.Path.of(file));
            String[] pointStrings = fileContents.split("\n");
            points = new Point2D[pointStrings.length];
            for(int i=0; i<pointStrings.length; i++) {
                points[i] = Point2D.fromString(pointStrings[i]);
            }

            valid = true;
        } catch (IOException ex) {
            ex.printStackTrace();
        } catch (NumberFormatException ex) {
            DriverStation.reportError("Path: Invalid File!", true);
        }
    }

    /**
     * Creates a new Path.
     * @param points An array of points describing the path.
     * @param color The color of the path.
     */
    public Path(Point2D[] points) {
        this.points = points;
        this.valid = true;
    }

    /**
     * Returns the Path's points.
     */
    public Point2D[] getPoints() {
        return points;
    }

    /**
     * Returns true if this Path was initalized correctly, false otherwise.
     */
    public boolean isValid() {
        return valid;
    }

    /**
     * Converts the Path into a user (and computer) readable String.
     * @return The string representation of the Path.
     */
    public String toString() {
        String pathString = "";
        for(Point2D point : points) {
            pathString += point.toString() + "\n";
        }

        return pathString;
    }
}
