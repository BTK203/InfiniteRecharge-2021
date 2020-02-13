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

public class ButtonCommandSpit extends CommandBase {
  private SubsystemIntake intake;
  private SubsystemFeeder feeder;

  /**
   * Creates a new ButtonCommandSpit.
   */
  public ButtonCommandSpit(SubsystemIntake intake, SubsystemFeeder feeder) {
    this.intake = intake;
    this.feeder = feeder;
    addRequirements(this.intake);
    addRequirements(this.feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double eatSpeed  = Util.getAndSetDouble("Eat Speed", 0.5) * -1;
    double slapSpeed = Util.getAndSetDouble("Slap Speed", 0.33) * -1;
    double beatSpeed = Util.getAndSetDouble("Beat Speed", 0.5) * -1;
    double feedSpeed = Util.getAndSetDouble("Feed Speed", 0.5) * -1;

    intake.driveEater(eatSpeed);
    intake.driveSlapper(slapSpeed);
    feeder.driveBeater(beatSpeed);
    feeder.driveFeeder(feedSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotors();
    feeder.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
