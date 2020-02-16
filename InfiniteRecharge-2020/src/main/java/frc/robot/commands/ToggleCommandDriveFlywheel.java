/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.util.Util;

public class ToggleCommandDriveFlywheel extends CommandBase {
  private SubsystemFlywheel flywheel;

  /**
   * Creates a new ToggleCommandDriveFlywheel.
   */
  public ToggleCommandDriveFlywheel(SubsystemFlywheel flywheel) {
    this.flywheel = flywheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive = Util.getAndSetDouble("Flywheel Drive", 0.5);
    flywheel.setFlywheelPercentOutput(drive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.setFlywheelPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
