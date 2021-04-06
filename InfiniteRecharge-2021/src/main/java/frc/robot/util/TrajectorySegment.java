// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * Represents a segment of a trajectory taken by the robot during path emulation.
 */
public class TrajectorySegment {
    private double
        leftVelocity,
        rightVelocity,
        distance,
        turn;

    private boolean isForwards;

    /**
     * Creates a new TrajectorySegment.
     * @param leftVelocity The velocity of the left drivetrain wheels.
     * @param rightVelocity The velocity of the right drivetrain wheels.
     * @param distance The distance that the segment drivetrain covers.
     */    
    public TrajectorySegment(double leftVelocity, double rightVelocity, double distance, double turn, boolean isForwards) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.distance = distance;
        this.turn = turn;
        this.isForwards = isForwards;
    }

    /**
     * Returns the velocity of the left wheels.
     */
    public double getLeftVelocity() {
        return leftVelocity;
    }

    /**
     * Returns the velocity of the right wheels.
     */
    public double getRightVelocity() {
        return rightVelocity;
    }

    /**
     * Returns the distance that this TrajectorySegment covers.
     */
    public double getDistance() {
        return distance;
    }

    /**
     * Returns the angle that the robot will turn through during the trajectory.
     */
    public double getTurn() {
        return turn;
    }

    /**
     * Returns true if the robot will drive forwards through the path, false otherwise.
     */
    public boolean getIsForwards() {
        return isForwards;
    }
}
