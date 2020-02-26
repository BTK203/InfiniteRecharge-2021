/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemClimb;
import frc.robot.util.Util;

public class CyborgCommandTestScissorPositition extends CommandBase {
  private SubsystemClimb scissor;
  /**
   * Creates a new CyborgCommandTestScissorPositition.
   */
  public CyborgCommandTestScissorPositition(SubsystemClimb scissor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.scissor = scissor;
    addRequirements(this.scissor);
    SmartDashboard.putBoolean("Test Scissor Climb", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double p = Util.getAndSetDouble("Scissor Position P", 0);
    double i = Util.getAndSetDouble("Scissor Position I", 0);
    double d = Util.getAndSetDouble("Scissor Position D", 0);
    double f = Util.getAndSetDouble("Scissor Position F", 0);

    double upperOutLimit = Util.getAndSetDouble("Scissor Position Max Out", 1);
    double lowerOutLimit = Util.getAndSetDouble("Scissor Position Min Out", -1);

    scissor.setScissorPIDF(p, i, d, f, lowerOutLimit, upperOutLimit);

    SmartDashboard.putBoolean("Test Scissor Climb", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Util.getAndSetDouble("", backup);
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
