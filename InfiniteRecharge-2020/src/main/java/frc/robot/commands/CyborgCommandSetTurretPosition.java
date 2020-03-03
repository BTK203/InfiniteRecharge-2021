/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

public class CyborgCommandSetTurretPosition extends CommandBase {
  private SubsystemTurret turret;

  private int
    yawPosition,
    pitchPosition;

  /**
   * Creates a new CyborgCommandSetTurretPosition.
   */
  public CyborgCommandSetTurretPosition(SubsystemTurret turret, int yawTarget, int pitchTarget) {
    this.turret = turret;
    this.yawPosition = yawTarget;
    this.pitchPosition = pitchTarget;
    addRequirements(this.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set yaw pid
    double yawkP = Util.getAndSetDouble("Yaw Position kP", 0.004);
    double yawkI = Util.getAndSetDouble("Yaw Position kI", 0.001);
    double yawIZone = Util.getAndSetDouble("Yaw Position IZone", 100000);
    double yawkD = Util.getAndSetDouble("Yaw Position KD", 0);
    double yawkF = Util.getAndSetDouble("Yaw Position KF", 0);
    double yawhighOutLimit = Util.getAndSetDouble("Yaw High Output", 1);

    turret.setYawPIDF(yawkP, yawkI, yawkD, yawkF, yawhighOutLimit, (int) yawIZone);

    //pitch pid
    double pitchkP = Util.getAndSetDouble("Pitch Position kP", 0);
    double pitchkI = Util.getAndSetDouble("Pitch Position kI", 0);
    double pitchIZone = Util.getAndSetDouble("Pitch Position IZone", 75);
    double pitchkD = Util.getAndSetDouble("Pitch Position kD", 0);
    double pitchkF = Util.getAndSetDouble("Pitch Position kF", 0);
    double pitchhighOutLimit = Util.getAndSetDouble("Pitch High Output", 1);

    turret.setPitchPIDF(pitchkP, pitchkI, pitchkD, pitchkF, pitchhighOutLimit, (int) pitchIZone);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setYawPosition(yawPosition);
    turret.setPitchPosition(pitchPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setYawPercentOutput(0);
    turret.setPitchPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double yawError = Math.abs(yawPosition - turret.getYawPosition());
    double pitchError = Math.abs(pitchPosition - turret.getPitchPosition());

    boolean yawStable = yawError <= Constants.TURRET_YAW_ALLOWABLE_ERROR;
    boolean pitchStable = pitchError <= Constants.TURRET_PITCH_ALLOWABLE_ERROR;

    return yawStable && pitchStable;
  }
}
