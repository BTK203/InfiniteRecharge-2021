/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemClimb;

public class ToggleCommandRunWinch extends CommandBase {
  private SubsystemClimb climber;
  private Joystick controller;

  /**
   * Creates a new ToggleCommandRunWinch.
   */
  public ToggleCommandRunWinch(SubsystemClimb climber, Joystick controller) {
    this.climber = climber;
    this.controller = controller;
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Winch", true);

    new CyborgCommandRumble(controller, 1000, RumbleType.kLeftRumble).schedule();
    new CyborgCommandRumble(controller, 1000, RumbleType.kRightRumble).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setScissorBraking(IdleMode.kCoast);
    climber.decendByController(controller);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Winch", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
