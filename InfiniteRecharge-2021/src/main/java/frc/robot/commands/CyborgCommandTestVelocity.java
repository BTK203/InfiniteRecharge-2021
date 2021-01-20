// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.Util;

public class CyborgCommandTestVelocity extends CommandBase {
  private SubsystemDrive drivetrain;
  private double
    targetDistance,
    currentDistance,
    lastLeftPosition,
    lastRightPosition;

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

    drivetrain.setPIDConstants(kP, kI, kD, kF, izone, outLimitLow, outLimitHigh);

    this.currentDistance = 0;
    this.lastLeftPosition = drivetrain.getLeftPosition();
    this.lastRightPosition = drivetrain.getRightPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocitySetpoint = Util.getAndSetDouble("Drive Velocity Setpoint", 12);
    //convert velocitySetpoint from inches / second to revs / min

    velocitySetpoint *= Constants.DRIVE_ROTATIONS_PER_INCH; //convert to rotations per second
    velocitySetpoint *= 60; //convert to rotations per minute

    drivetrain.setLeftVelocity(velocitySetpoint);
    drivetrain.setRightVelocity(velocitySetpoint);

    //update the distance we have driven
    double newLeftPosition = drivetrain.getLeftPosition();
    double newRightPosition = drivetrain.getRightPosition();
    double leftChange = newLeftPosition - lastLeftPosition;
    double rightChange = newRightPosition - lastRightPosition;

    //average those to get the distance travelled since last call
    this.currentDistance += (leftChange + rightChange) / 2;

    //set history
    lastLeftPosition = newLeftPosition;
    lastRightPosition = newRightPosition;
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
