// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.Util;

/**
 * Tests the drivetrain velocity PID.
 * Safe fastest speed: 148 in/sec
 */
public class CyborgCommandTestVelocity extends CommandBase {
  private SubsystemDrive drivetrain;
  private double
    targetDistance,
    currentDistance,
    lastLeftPosition,
    lastRightPosition;

  private long
    elapsedTime,
    lastExecute;

  private PIDController headingController;

  /** Creates a new CyborgCommandTestVelocity. */
  public CyborgCommandTestVelocity(SubsystemDrive drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.targetDistance = distance;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //grab pid constants
    double 
      kP           = Util.getAndSetDouble("Drive Velocity kP", 0),
      kI           = Util.getAndSetDouble("Drive Velocity kI", 0),
      kD           = Util.getAndSetDouble("Drive Velocity kD", 0),
      kF           = Util.getAndSetDouble("Drive Velocity kF", 0),
      izone        = Util.getAndSetDouble("Drive Velocity IZone", 0),
      outLimitLow  = Util.getAndSetDouble("Drive Velocity Out Limit Low", -1),
      outLimitHigh = Util.getAndSetDouble("Drive Velocity Out Limit High", 1);

    double
      headingkP = Util.getAndSetDouble("Drive Heading kP", 0),
      headingkI = Util.getAndSetDouble("Drive Heading kI", 0),
      headingkD = Util.getAndSetDouble("Drive Heading kD", 0),
      headingTarget = Util.getAndSetDouble("Drive Heading Target", 0);

    this.headingController = new PIDController(headingkP, headingkI, headingkD);
    this.headingController.setSetpoint(headingTarget);    

    drivetrain.setPIDConstants(kP, kI, kD, kF, izone, outLimitLow, outLimitHigh);

    this.currentDistance = 0;
    this.lastLeftPosition = drivetrain.getLeftPosition();
    this.lastRightPosition = drivetrain.getRightPosition();

    this.elapsedTime = 0;
    this.lastExecute = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocitySetpoint = Util.getAndSetDouble("Drive Velocity Setpoint", 12);
    velocitySetpoint *= Constants.DRIVE_ROTATIONS_PER_INCH; //convert to rotations per second
    velocitySetpoint *= 60; //convert to rotations per minute

    //ramp setpoint so that the start of the command is easier on the robot.
    long currentTime = System.currentTimeMillis();
    // elapsedTime += (currentTime - lastExecute);
    // double ramp = Util.getAndSetDouble("Drive Velocity Ramp", 500);
    // double rampMultiplier = elapsedTime / ramp;
    // rampMultiplier = (rampMultiplier > 1 ? 1 : rampMultiplier);
    // velocitySetpoint *= rampMultiplier;

    //correct heading
    double headingCorrection = headingController.calculate(drivetrain.getGyroAngle());
    double currentVelocity = drivetrain.getOverallVelocity();
    headingCorrection *= (currentVelocity) / 2;
    double leftVelocity  = velocitySetpoint - headingCorrection;
    double rightVelocity = velocitySetpoint + headingCorrection;

    drivetrain.setLeftVelocity(leftVelocity);
    drivetrain.setRightVelocity(rightVelocity);

    SmartDashboard.putNumber("Test Velocity Setpoint", velocitySetpoint);

    //update the distance we have driven
    double newLeftPosition = drivetrain.getLeftPosition();
    double newRightPosition = drivetrain.getRightPosition();
    double leftChange = newLeftPosition - lastLeftPosition;
    double rightChange = newRightPosition - lastRightPosition;

    //average those to get the distance travelled since last call
    double revolutions = (leftChange + rightChange) / 2;
    this.currentDistance += revolutions / Constants.DRIVE_ROTATIONS_PER_INCH;

    SmartDashboard.putNumber("Test Velocity Distance Travelled", this.currentDistance);

    //set history
    lastLeftPosition = newLeftPosition;
    lastRightPosition = newRightPosition;
    lastExecute = currentTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setRightPercentOutput(0);
    drivetrain.setLeftPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.currentDistance >= targetDistance;
  }
}
