/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

public class CyborgCommandAlignTurret extends CommandBase {
  private SubsystemTurret turret;
  private SubsystemReceiver kiwilight;

  /**
   * Creates a new CyborgCommandAlignTurret.
   */
  public CyborgCommandAlignTurret(SubsystemTurret turret, SubsystemReceiver kiwilight) {
    this.turret = turret;
    this.kiwilight = kiwilight;
    addRequirements(this.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set yaw pid
    double yawkP = Util.getAndSetDouble("Yaw Position kP", 0);
    double yawkI = Util.getAndSetDouble("Yaw Position kI", 0);
    double yawkD = Util.getAndSetDouble("Yaw Position KD", 0);
    double yawkF = Util.getAndSetDouble("Yaw Position KF", 0);
    double yawhighOutLimit = Util.getAndSetDouble("Yaw High Output", 1);

    turret.setYawPIDF(yawkP, yawkI, yawkD, yawkF, yawhighOutLimit);

    //pitch pid
    double pitchkP = Util.getAndSetDouble("Pitch Position kP", 0);
    double pitchkI = Util.getAndSetDouble("Pitch Position kI", 0);
    double pitchkD = Util.getAndSetDouble("Pitch Position kD", 0);
    double pitchkF = Util.getAndSetDouble("Pitch Position kF", 0);
    double pitchhighOutLimit = Util.getAndSetDouble("Pitch High Output", 1);

    turret.setPitchPIDF(pitchkP, pitchkI, pitchkD, pitchkF, pitchhighOutLimit);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double horizontalAngle = kiwilight.getHorizontalAngleToTarget();
    double verticalAngle = kiwilight.getVerticalAngleToTarget();

    double horizontalTicks = Util.getAndSetDouble("Horizontal Ticks", 5000);
    double verticalTicks = Util.getAndSetDouble("Vertical Ticks", 1000);

    //horizontal angle
    if(Math.abs(horizontalAngle) != 180) {
      double horizontalTicksPerInch = horizontalTicks / (double) Constants.TURRET_YAW_DEGREES;
      double horizontalTicksToTurn = horizontalAngle * horizontalTicksPerInch;

      double newTargetPosition = turret.getYawPosition() + horizontalTicksToTurn;
    }

    //vertical angle
    if(Math.abs(verticalAngle) != 180) {
      
    } else {

    }
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
