// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemFlywheel;

public class CyborgCommandWaitForFlywheel extends CommandBase {
  private SubsystemFlywheel flywheel;

  /** Creates a new CyborgCommandWaitForFlywheel. */
  public CyborgCommandWaitForFlywheel(SubsystemFlywheel flywheel) {
    this.flywheel = flywheel;
    //this command does not require the flywheel because we do not actually change its behavior.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flywheel.getVelocity() > Constants.FLYWHEEL_STABLE_RPM;
  }
}
