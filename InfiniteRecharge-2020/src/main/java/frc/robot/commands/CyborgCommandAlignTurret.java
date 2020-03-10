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

    SmartDashboard.putBoolean("Aligning", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double horizontalAngle = kiwilight.getHorizontalAngleToTarget() * -1;
    horizontalAngle *= Util.getAndSetDouble("Vision multiplier", 1);

    double horizontalTicks = turret.getTotalYawTicks();

    double targetDistance = kiwilight.getDistanceToTarget();

    //horizontal angle
    if(kiwilight.targetSpotted()) {
      double horizontalTicksPerDegree = horizontalTicks / (double) Constants.TURRET_YAW_DEGREES;
      double horizontalTicksToTurn = horizontalAngle * horizontalTicksPerDegree;

      SmartDashboard.putNumber("H Ticks To Turn", horizontalTicksToTurn);

      SmartDashboard.putNumber("Yaw Ticks To Turn", horizontalTicksToTurn);

      double newTargetPosition = turret.getYawPosition() + horizontalTicksToTurn;
      turret.setYawPosition(newTargetPosition);
    } else {
      //disable motors
      turret.setYawPercentOutput(0);
    }

    //vertical angle
    if(kiwilight.targetSpotted()) {
      double newPitchPosition = turret.getPitchPosition();
      if(targetDistance > 5) {
        //use the cool parabola equation to calculate the pitch position
        //equation: f(x) = 0.006851x^2 - 2.654x - 447.8 | where: x is the distance kiwilight reports and f returns the pitch position.
        double ax2 = 0.006851 * Math.pow(targetDistance, 2);
        double bx  = -2.654 * targetDistance;
        double c   = -446.8;

        newPitchPosition = ax2 + bx + c;

        SmartDashboard.putNumber("Pitch Error", turret.getPitchPosition() - newPitchPosition);
      } else {
        //use the slightly less cool linear equation to calculate the pitch position
        //equation: f(x) = -9.512x - 65.85 | where: x is the distance kiwilight reports and f returns the pitch position.

        newPitchPosition = (-9.512 * targetDistance) -65.85;
      }

      turret.setPitchPosition(newPitchPosition);
    } else {
      //disable motors
      turret.setPitchPercentOutput(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setYawPercentOutput(0);
    turret.setPitchPercentOutput(0);

    SmartDashboard.putBoolean("Aligning", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
