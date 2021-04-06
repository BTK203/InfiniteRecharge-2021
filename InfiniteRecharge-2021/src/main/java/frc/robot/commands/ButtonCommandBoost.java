// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.Util;

public class ButtonCommandBoost extends CommandBase {
  private SubsystemDrive drive;

  /** Creates a new ButtonCommandBoost. */
  public ButtonCommandBoost(SubsystemDrive drivetrain) {
    this.drive = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setBoosting(Util.getAndSetDouble("Boost Inhibitor", 1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopBoosting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
