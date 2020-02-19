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

public class ButtonCommandGroupRunIntakeFeeder extends CommandBase {
  private SubsystemIntake intake;
  private SubsystemFeeder feeder;
  private Joystick controller;

  /**
   * Creates a new ButtonCommandGroupRunIntakeFeeder.
   */
  public ButtonCommandGroupRunIntakeFeeder(SubsystemIntake intake, SubsystemFeeder feeder, Joystick controller) {
    this.intake = intake;
    this.feeder = feeder;
    this.controller = controller;
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
    double
      eatSpeed = 0,
      slapSpeed = 0,
      beatSpeed = 0,
      feedSpeed = 0;

    if(controller.getRawButton(Xbox.A)) {
      eatSpeed = Util.getAndSetDouble("Eat Speed", 0.5);
      slapSpeed = Util.getAndSetDouble("Slap Speed", 0.33);
      beatSpeed = Util.getAndSetDouble("Beat Speed", 0.5);
    }

    if(controller.getRawButton(Xbox.X)) {
      slapSpeed = Util.getAndSetDouble("Slap Speed", 0.33);
      beatSpeed = Util.getAndSetDouble("Beat Speed", 0.5);
      feedSpeed = Util.getAndSetDouble("Feed Speed", 0.5);
    }

    if(controller.getRawButton(Xbox.B)) {
      eatSpeed = Util.getAndSetDouble("Eat Speed", 0.5) * -1;
      slapSpeed = Util.getAndSetDouble("Slap Speed", 0.33) * -1;
      beatSpeed = Util.getAndSetDouble("Beat Speed", 0.5) * -1;
      feedSpeed = Util.getAndSetDouble("Feed Speed", 0.5) * -1;
    }

    intake.driveEater(eatSpeed);
    intake.driveSlapper(slapSpeed);
    feeder.driveBeater(beatSpeed);
    feeder.driveFeeder(feedSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
