// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;

/**
 * The new 2021 driver. Yee haw.
 */
public class CourseAdjuster {
    private SubsystemDrive drivetrain;
    private PIDController headingController;
    private double
        targetHeading,
        headingCorrectionInhibitor,
        leftVelocity,
        rightVelocity;
        
    /**
     * Creates a new CourseAdjuster
     * @param drivetrain Drivetrain of the robot.
     * @param headingkP The P value of the heading loop.
     * @param headingkI The I value of the heading loop.
     * @param headingkD The D value of the heading loop.
     * 
     * NOTE: This class does not set the drivetrain PID values in this constructor.
     * Drivetrain PID values can be set using setDrivetrainPIDF() or setting the values on the drivetrain.
     * Also, the init() method must be called before running. Call this in the initialize() method of
     * the command.
     */
    public CourseAdjuster(SubsystemDrive drivetrain, double headingkP, double headingkI, double headingkD) {
        this.drivetrain = drivetrain;
        this.headingController = new PIDController(headingkP, headingkI, headingkD);
        this.headingCorrectionInhibitor = 1;
    }

    /**
     * Sets the PID constants on the drivetrain.
     */
    public void setDrivetrainPIDF(double kP, double kI, double kD, double kF, double iZone, double lowOutLimit, double highOutLimit) {
        drivetrain.setPIDConstants(kP, kI, kD, kF, iZone, lowOutLimit, highOutLimit);
    }

    /**
     * Initializes / resets the object.
     * This should be called in the initialize() method of the command invoking this object.
     */
    public void init() {
        this.headingController.setSetpoint(0); //set default heading to current heading
        targetHeading = drivetrain.getGyroAngle();

        leftVelocity = 0;
        rightVelocity = 0;
    }

    /**
     * Updates the object.
     * This should be called in the update() method of the command invoking this object.
     */
    public void update() {
        double leftVelocitySetpoint = curveVelocity(leftVelocity);
        double rightVelocitySetpoint = curveVelocity(rightVelocity);

        //correct heading
        double angleToHeading = Util.getAngleToHeading(drivetrain.getGyroAngle(), targetHeading);
        double headingCorrection = headingController.calculate(angleToHeading); //just a reminder here that headingController's setpoint is 0
        double currentVelocity = drivetrain.getOverallVelocity();
        headingCorrection *= (currentVelocity) / 2;
        headingCorrection *= headingCorrectionInhibitor;
        double leftVelocity  = leftVelocitySetpoint - headingCorrection;
        double rightVelocity = rightVelocitySetpoint + headingCorrection;

        drivetrain.setLeftVelocity(leftVelocity);
        drivetrain.setRightVelocity(rightVelocity);
    }

    /**
     * Sets the target heading of the robot.
     */
    public void setHeading(double heading) {
        targetHeading = heading;
    }

    /**
     * Sets the inhibitor for heading correction.
     * @param correctionInhibitor The value of the new inhibitor. A value of less than 1 will make heading correction less effective. A value of greater than 1 will make it more effective.
     */
    public void setHeadingCorrectionInhibitor(double correctionInhibitor) {
        this.headingCorrectionInhibitor = correctionInhibitor;
    }

    /**
     * Sets the left wheel velocity of the robot.
     */
    public void setBaseLeftVelocity(double velocity) {
        this.leftVelocity = IPStoRPM(velocity);
    }

    /**
     * Sets the right wheel velocity of the robot.
     */
    public void setBaseRightVelocity(double velocity) {
        this.rightVelocity = IPStoRPM(velocity);
    }

    /**
     * Updates the PID Constants for heading correction.
     * @param kP The P (proportional) value.
     * @param kI The I (integral) value.
     * @param kD The D (derivative) value.
     */
    public void setHeadingPID(double kP, double kI, double kD) {
        headingController.setP(kP);
        headingController.setI(kI);
        headingController.setD(kD);
    }

    /**
     * Converts a velocity in inches/sec to RPM.
     * @param ips A velocity in inches/sec
     * @return A velocity in RPM that corresponds to the velocity in ips.
     */
    private double IPStoRPM(double ips) {
        double newVelocity = ips * Constants.DRIVE_ROTATIONS_PER_INCH; //convert to rotations per second
        newVelocity *= 60; //convert to rotations per minute
        return newVelocity;
    }

    /**
     * This odd method is here so that the robot can achieve almost any velocity using only one set of PID constants.
     * It works by increasing the target velocity so that the PID is forced to work harder than it would work otherwise.
     * @param velocitySetpoint The original velocity setpoint in RPM
     * @return The curved velocity setpoint in RPM
     */
    private double curveVelocity(double velocitySetpoint) {
        return (velocitySetpoint > 1132 ? velocitySetpoint += (velocitySetpoint - 40) * 0.4 : velocitySetpoint); //1132 RPM ~= 45 in/sec
    }
}
