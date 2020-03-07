/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.enumeration.ClimbPosition;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemClimb;
import frc.robot.util.Util;


public class IterativeCommandMoveClimber extends CommandBase {
  private SubsystemClimb climber;
  private ClimbPosition position;
  private double winchHeight;

  /**
   * Creates a new IterativeCommandMoveClimber.
 * 
   */
  public IterativeCommandMoveClimber(SubsystemClimb climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    addRequirements(this.climber);
    SmartDashboard.putBoolean("Move Climber", false);
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

    climber.setScissorPIDF(p, i, d, f, IZone, lowerOutLimit, upperOutLimit);
    climber.zeroEncoders(); //this line is VERY important, DO NOT remove it! The climber might break without it.
    climber.setScissorBraking(IdleMode.kBrake);
    SmartDashboard.putBoolean("Move Climber", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = climber.getStoredPosition();

    switch(position) {
      case LOWEST:
        winchHeight = Constants.LOWEST_HEIGHT;
        break;

      case ON_WHEEL:
        winchHeight = Constants.ON_WHEEL_HEIGHT;
        break;

      case ABOVE_WHEEL:
        winchHeight = Constants.ABOVE_WHEEL_HEIGHT;
        break;

      case HIGHEST:
        winchHeight = Constants.HIGHEST_HEIGHT;
        break;

      default:
        winchHeight = (Double) null;
        break;
    }

    if (!Double.isNaN(winchHeight))
    {
      climber.setWinchPosition(winchHeight);
      double winchPosition = climber.getWinchPosition();
      double winchInches = (Math.pow(Math.E, -0.001504 * winchPosition) * -56.96) + 57.07;
      double targetScissorPosition = (-.0547 * winchPosition) - .1042;
      climber.setScissorsPosition(targetScissorPosition);
      SmartDashboard.putNumber("Target Scissor Position", targetScissorPosition);
      SmartDashboard.putNumber("Winch Inches", winchInches);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setWinchPercentOutput(0);
    climber.setScissorsPercentOutput(0);
    SmartDashboard.putBoolean("Move Climber", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
