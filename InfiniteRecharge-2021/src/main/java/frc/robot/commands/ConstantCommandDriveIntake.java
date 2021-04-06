/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.util.Util;

public class ConstantCommandDriveIntake extends CommandBase {
  private SubsystemIntake intake;
  private SubsystemFeeder feeder;

  /**
   * Creates a new ConstantCommandDriveIntake.
   */
  public ConstantCommandDriveIntake(SubsystemIntake intake, SubsystemFeeder feeder) {
    this.intake = intake;
    this.feeder = feeder;

    addRequirements(intake);
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.driveEater(Util.getAndSetDouble("Eat Speed", 1));
    intake.driveSlapper(Util.getAndSetDouble("Slap Speed", 0.5));
    feeder.driveBeater(Util.getAndSetDouble("Beat Speed", 1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.driveEater(0);
    intake.driveSlapper(0);
    feeder.driveBeater(0);
    feeder.driveFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
