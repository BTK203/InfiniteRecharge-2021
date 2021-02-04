// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;

/** Add your docs here. */
public class PositionTracker {
    private SubsystemDrive drivetrain;
    private double
        lastLeftDistance,
        lastRightDistance,
        x,
        y,
        heading;

    Thread updater;

    boolean
        zero,
        waitForZeroDrive;

    /**
     * Creates a new PositionTracker.
     * @param x The starting X-coordinate of the robot.
     * @param y The starting Y-coordinate of the robot.
     * @param angle The starting heading angle of the robot in degrees. (0 = towards positive X. Positive = CCW)
     */
    public PositionTracker(SubsystemDrive drivetrain, double x, double y, double heading) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.waitForZeroDrive = false;
        this.zero = false;

        //track robot position in new thread
        this.updater = new Thread(
            () -> {
                while(true) {
                    //update using drivetrain values.
                    update();
                }
            }
        );

        this.updater.start();
    }

    /**
     * Creates a new Position tracker, with starting position and rotation at 0.
     */
    public PositionTracker(SubsystemDrive drivetrain) {
        this(drivetrain, 0, 0, 0);
    }

    /**
     * Sets the position and heading of the robot.
     * @param x The new X-coordinate of the robot.
     * @param y The new Y-coordinate of the robot.
     * @param angle The new heading angle of the robot.
     */
    public void setPositionAndHeading(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;

        //set lastLeft and lastRight so that coordinates don't jump
        lastLeftDistance = drivetrain.getLeftPosition();
        lastRightDistance = drivetrain.getRightPosition();
    }

    /**
     * This is a test
     */
    public void zeroPositionAndHeading(boolean waitForZeroDrivetrain) {
        zero = true;
        waitForZeroDrive = waitForZeroDrivetrain;
    }

    public void zeroPositionAndHeading() {
        zeroPositionAndHeading(true);
    }

    /**
     * Updates the position of the robot using a distance travelled and heading travelled in.
     * @param driveDistance The average of the drive distance of the two sides of the drivetrain.
     * @param rotation The current rotation of the robot.
     */
    public void update(double driveDistance, double rotation) {
        rotation %= 360;
        double averageHeading = (rotation + this.heading) / 2;

        //break vector into components
        double driveX = driveDistance * Math.cos(Math.toRadians(averageHeading));
        double driveY = driveDistance * Math.sin(Math.toRadians(averageHeading));

        driveX /= Constants.DRIVE_ROTATIONS_PER_INCH;
        driveY /= Constants.DRIVE_ROTATIONS_PER_INCH;

        SmartDashboard.putNumber("PT DriveX", driveX);
        SmartDashboard.putNumber("PT DriveY", driveY);

        this.x += driveX;
        this.y += driveY;

        this.heading = rotation;

        if(zero) {
            this.x = 0;
            this.y = 0;
            this.heading = 0;
        }
    }

    /**
     * Updates the position of the robot using values from the drivetrain.
     */
    public void update() {
        double currentLeftDistance = drivetrain.getLeftPosition();
        double currentRightDistance = drivetrain.getRightPosition();
        double leftChange = currentLeftDistance - lastLeftDistance;
        double rightChange = currentRightDistance - lastRightDistance;
        
        //take average to get average distance travelled by the center of the bot
        double netDistanceTravelled = (leftChange + rightChange) / 2;
        double currentHeading = drivetrain.getGyroAngle();

        //update using new values
        update(netDistanceTravelled, currentHeading);

        lastLeftDistance = currentLeftDistance;
        lastRightDistance = currentRightDistance;

        if(zero) {
            if(waitForZeroDrive && drivetrainAtZero()) {
                lastLeftDistance = drivetrain.getLeftPosition();
                lastRightDistance = drivetrain.getRightPosition();

                zero = false;
            }
        } else {
            waitForZeroDrive = false;
        }
    }

    /**
     * Returns the current position and heading of the robot.
     */
    public Point2D getPositionAndHeading() {
        return new Point2D(x, y, heading);
    }

    private boolean drivetrainAtZero() {
        return 
            Math.abs(drivetrain.getLeftPosition()) < 0.25 &&
            Math.abs(drivetrain.getRightPosition()) < 0.25;
    }
}
