/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemSpinner;

public class CyborgCommandPositionControl extends CommandBase {
  /**
   * Creates a new CyborgCommandPositionControl.
   */
  private SubsystemSpinner spinner;
  private boolean finished;
  
  public CyborgCommandPositionControl(SubsystemSpinner spinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.spinner = spinner;
    addRequirements(this.spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = spinner.spinColor('Y');
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinner.stopSpinner();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
