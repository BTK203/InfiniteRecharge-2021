/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemClimb;
import frc.robot.util.Util;


public class CyborgCommandTestScissorPositition extends CommandBase {
  private SubsystemClimb scissors;
  private Joystick controller;
  /**
   * Creates a new CyborgCommandTestScissorPositition.
   */
  public CyborgCommandTestScissorPositition(SubsystemClimb scissors, Joystick controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.scissors = scissors;
    this.controller = controller;
    addRequirements(this.scissors);
    SmartDashboard.putBoolean("Test Scissor Climb", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double p = Util.getAndSetDouble("Scissor Position P", 0);
    double i = Util.getAndSetDouble("Scissor Position I", 0);
    double d = Util.getAndSetDouble("Scissor Position D", 0);
    double f = Util.getAndSetDouble("Scissor Position F", 0);
    double IZone = Util.getAndSetDouble("Scissor Position IZone", 0);

    double upperOutLimit = Util.getAndSetDouble("Scissor Position Max Out", 1);
    double lowerOutLimit = Util.getAndSetDouble("Scissor Position Min Out", -1);

    scissors.setScissorPIDF(p, i, d, f, IZone, lowerOutLimit, upperOutLimit);

    SmartDashboard.putBoolean("Test Scissor Climb", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double scissorsTargetPosition = Util.getAndSetDouble("Scissors Target Position", 0);
    scissors.setScissorsPosition(scissorsTargetPosition);
    scissors.decendByController(controller);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    scissors.setScissorsPercentOutput(0);
    SmartDashboard.putBoolean("Test Scissor Climb", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
