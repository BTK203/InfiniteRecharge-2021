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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemClimb;
import frc.robot.subsystems.SubsystemTurret;

public class ToggleCommandDriveClimber extends CommandBase {
  private SubsystemClimb climber;
  private Joystick controller;

  /**
   * Creates a new ToggleCommandDriveClimber.
   */
  public ToggleCommandDriveClimber(SubsystemClimb climber, SubsystemTurret turret, Joystick controller) {
    this.climber = climber;
    this.controller = controller;

    addRequirements(this.climber);
    addRequirements(turret); //require the turret simply to override its controls, we dont need it at this point anyway.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new CyborgCommandRumble(controller, 1000, RumbleType.kLeftRumble).schedule();
    new CyborgCommandRumble(controller, 1000, RumbleType.kRightRumble).schedule();
    climber.setScissorBraking(IdleMode.kBrake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.moveScissorsByController(controller);
    climber.moveWinchByController(controller);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setScissorsPercentOutput(0);
    climber.setWinchPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
