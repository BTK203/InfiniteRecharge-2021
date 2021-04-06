/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.Util;

public class CyborgCommandDriveDistance extends CommandBase {
  private SubsystemDrive drivetrain;
  private double 
    distance,
    leftDestination,
    rightDestination,
    inhibitor;

  /**
   * Creates a new CyborgCommandDriveDistance.
   * @param drivetrain the drivetrain to use.
   * @param distance the distance to drive in inches.
   */
  public CyborgCommandDriveDistance(SubsystemDrive drivetrain, double distance, double inhibitor) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.inhibitor = inhibitor;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.reportWarning("CyborgCommandDriveDistance enters", false);

    //figure out how many rotations to turn
    double rotations = distance * Constants.DRIVE_ROTATIONS_PER_INCH; 

    this.leftDestination = drivetrain.getLeftPosition() + rotations;
    this.rightDestination = drivetrain.getRightPosition() + rotations;

    SmartDashboard.putNumber("Drive Target Left", leftDestination);
    SmartDashboard.putNumber("Drive Target Right", rightDestination);

    DriverStation.reportWarning("Drive Target Left: " + Double.valueOf(leftDestination).toString(), false);
    DriverStation.reportWarning("Drive Target Right: " + Double.valueOf(rightDestination).toString(), false);

    double p = Util.getAndSetDouble("Dumb Drive Distance kP", 0.03);
    double i = Util.getAndSetDouble("Dumb Drive Distance kI", 0);
    double d = Util.getAndSetDouble("Dumb Drive Distance kD", 0);
    double f = Util.getAndSetDouble("Dumb Drive Distance kF", 0);
    double iZone = Util.getAndSetDouble("Dumb Drive Distance IZone", 0);
    double out = inhibitor;

    drivetrain.setPIDConstants(p, i, d, f, iZone, out);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setLeftPosition(leftDestination);
    drivetrain.setRightPosition(rightDestination);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //override and stop the PID
    drivetrain.setRightPercentOutput(0);
    drivetrain.setLeftPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double rightError = Math.abs(rightDestination - drivetrain.getRightPosition());
    double leftError = Math.abs(leftDestination - drivetrain.getLeftPosition());

    double allowableErrorRotations = Constants.DRIVETRAIN_ALLOWABLE_ERROR * Constants.DRIVE_ROTATIONS_PER_INCH;

    boolean rightWithinRange = rightError < allowableErrorRotations;
    boolean leftWithinRange  = leftError  < allowableErrorRotations;

    return rightWithinRange && leftWithinRange;
  }
}
