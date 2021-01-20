// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.Util;

public class CyborgCommandTestVelocity extends CommandBase {
  private SubsystemDrive drivetrain;
  private PIDController controller;
  private double
    distance;

  private double
    distanceDriven,
    lastLeftPosition,
    lastRightPosition;

  /** Creates a new CyborgCommandTestVelocity. */
  public CyborgCommandTestVelocity(SubsystemDrive drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //grab pid constants
    double 
      kP = Util.getAndSetDouble("Drive Velocity kP", 0),
      kI = Util.getAndSetDouble("Drive Velocity kI", 0),
      kD = Util.getAndSetDouble("Drive Velocity kD", 0);
    
    this.controller = new PIDController(kP, kI, kD);
    
    this.controller.setSetpoint(0); //set to 0 for now, setpoint will be set in execute()

    this.distanceDriven = 0;
    this.lastLeftPosition = drivetrain.getLeftPosition();
    this.lastRightPosition = drivetrain.getRightPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentVelocitySetpoint = Util.getAndSetDouble("Drive Velocity Setpoint", 12);
    currentVelocitySetpoint *= Constants.DRIVE_ROTATIONS_PER_INCH;

    controller.setSetpoint(currentVelocitySetpoint);

    double averageVelocity = (drivetrain.getLeftVelocity() + drivetrain.getRightVelocity()) / 2;
    averageVelocity *= Constants.DRIVE_ROTATIONS_PER_INCH;
    double output = controller.calculate(averageVelocity);
    double inhibitor = Util.getAndSetDouble("Drive Inhibitor", 1);
    output = (output > inhibitor ? inhibitor : (output < -1 * inhibitor ? -1 * inhibitor : output)); // ensure that inhibitor < output < -inhibitor

    drivetrain.setLeftPercentOutput(output);
    drivetrain.setRightPercentOutput(output);
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
    return this.distanceDriven >= distance;
  }
}
