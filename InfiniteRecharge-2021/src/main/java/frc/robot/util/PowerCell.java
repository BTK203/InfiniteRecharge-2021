// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** 
 * Represents a power cell spotted by the Jevois.
 */
public class PowerCell {
    private int
        x,
        y,
        radius;

    /**
     * Creates a new PowerCell.
     * @param x The x-coordinate of the center of the power cell in pixels, where 0 is the center of the image.
     * @param y The y-coordinate of the center of the power cell in pixels, where 0 is the top of the image.
     * @param radius The radius of the power cell in pixels.
     */
    public PowerCell(int x, int y, int radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
    }

    /**
     * Returns the x-coordinate of the power cell in pixels.
     * @return x-coordinate of center of power cell. 0 is center of image.
     */
    public int getX() {
        return x;
    }

    /**
     * Returns the y-coordinate of the power cell in pixels.
     * @return The y-coordinate of the power cell. 0 is top of image.
     */
    public int getY() {
        return y;
    }

    /**
     * Returns the radius of the power cell in pixels.
     * @return The radius of the power cell.
     */
    public int getRadius() {
        return radius;
    }

    /**
     * Returns a string representation of the PowerCell.
     * @return String formatted as [x, y, radius]
     */
    public String toString() {
        return "[" + Integer.valueOf(x).toString() + ", " + Integer.valueOf(y).toString() + ", " + Integer.valueOf(radius).toString() + "]";
    }
}
