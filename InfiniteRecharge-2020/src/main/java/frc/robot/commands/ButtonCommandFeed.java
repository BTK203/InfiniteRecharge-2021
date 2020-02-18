/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.util.Util;
import frc.robot.util.Xbox;

public class ButtonCommandFeed extends CommandBase {
  private SubsystemIntake intake;
  private SubsystemFeeder feeder;
  private Joystick operator;

  /**
   * Creates a new ButtonCommandFeed.
   */
  public ButtonCommandFeed(SubsystemIntake intake, SubsystemFeeder feeder, Joystick operator) {
    this.intake = intake;
    this.feeder = feeder;
    this.operator = operator;
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
    double slapSpeed = Util.getAndSetDouble("Slap Speed", 0.33);
    double beatSpeed = Util.getAndSetDouble("Beat Speed", 0.5);
    double feedSpeed = Util.getAndSetDouble("Feed Speed", 0.5);

    intake.driveSlapper(slapSpeed);
    feeder.driveBeater(beatSpeed);
    feeder.driveFeeder(feedSpeed);

    if(operator.getRawButton(Xbox.A)) {
      double eatSpeed = Util.getAndSetDouble("Eat Speed", 0.5);
      intake.driveEater(eatSpeed);
    }
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
