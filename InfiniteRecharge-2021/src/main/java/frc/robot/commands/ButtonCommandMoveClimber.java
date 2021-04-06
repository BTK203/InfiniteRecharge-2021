/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.enumeration.ClimbPosition;
import frc.robot.subsystems.SubsystemClimb;
import frc.robot.util.Util;

public class ButtonCommandMoveClimber extends CommandBase {
  Boolean isFinished;

  int displacement;
  int intPosition;

  private SubsystemClimb climber;

  /**
   * Creates a new ButtonCommandMoveClimber.
   */
  public ButtonCommandMoveClimber(SubsystemClimb climber, int displacement) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.displacement = displacement;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    intPosition = climber.getStoredPosition().toInt() + displacement;
    intPosition = Util.truncateInt(intPosition, 1, 4);
    if (intPosition == climber.getStoredPosition().toInt()) { isFinished = true; }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isFinished()) {
      ClimbPosition position;
      switch(intPosition) {
        case 1:
          position = ClimbPosition.LOWEST;
          break;
        
          case 2:
          position = ClimbPosition.ON_WHEEL;
          break;

          case 3:
          position = ClimbPosition.ABOVE_WHEEL;
          break;

          case 4:
          position = ClimbPosition.HIGHEST;
          break;

          default:
          position = null;
          break;
      }
      climber.setStoredPosition(position);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
