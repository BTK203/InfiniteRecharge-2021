/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.Util;

public class CyborgCommandDriveDistance extends CommandBase {
  private SubsystemDrive drivetrain;
  private double 
    leftDestination,
    rightDestination;

  /**
   * Creates a new CyborgCommandDriveDistance.
   * @param drivetrain the drivetrain to use.
   * @param distance the distance to drive in inches.
   */
  public CyborgCommandDriveDistance(SubsystemDrive drivetrain, double distance) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);

    //figure out how many rotations to turn
    double wheelCircumference = Math.PI * Constants.DRIVETRAIN_WHEEL_DIAMETER;
    double rotations = distance / wheelCircumference;

    this.leftDestination = drivetrain.getLeftPosition() + rotations;
    this.rightDestination = drivetrain.getRightPosition() + rotations;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double p = Util.getAndSetDouble("Drivetrain kP", 0);
    double i = Util.getAndSetDouble("Drivetrain kI", 0);
    double d = Util.getAndSetDouble("Drivetrain kD", 0);
    double f = Util.getAndSetDouble("Drivetrain kF", 0);
    double iZone = Util.getAndSetDouble("Drivetrain IZone", 0);
    double out = Util.getAndSetDouble("Drivetrain Out Limit", 1);

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
    drivetrain.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double rightError = Math.abs(rightDestination - drivetrain.getRightPosition());
    double leftError = Math.abs(leftDestination - drivetrain.getLeftPosition());

    boolean rightWithinRange = rightError < Constants.DRIVETRAIN_ALLOWABLE_ERROR;
    boolean leftWithinRange  = leftError  < Constants.DRIVETRAIN_ALLOWABLE_ERROR;

    return rightWithinRange && leftWithinRange;
  }
}
