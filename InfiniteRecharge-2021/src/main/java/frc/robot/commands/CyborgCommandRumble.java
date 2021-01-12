/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CyborgCommandRumble extends CommandBase {
  private Joystick controller;
  private double
    startTime,
    length;

  private RumbleType type;

  /**
   * Creates a new CyborgCommandRumble.
   */
  public CyborgCommandRumble(Joystick controller, double length, RumbleType type) {
    this.controller = controller;
    this.length = length;
    this.type = type;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.setRumble(type, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(type, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elapsedTime = System.currentTimeMillis() - startTime;
    return elapsedTime > length;
  }
}
