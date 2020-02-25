/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

public class CyborgCommandAlignTurret extends CommandBase {
  private SubsystemTurret turret;
  private SubsystemReceiver kiwilight;

  private Joystick
    driver,
    operator;

  private boolean targetPreviouslySeen;

  /**
   * Creates a new CyborgCommandAlignTurret.
   */
  public CyborgCommandAlignTurret(SubsystemTurret turret, SubsystemReceiver kiwilight, Joystick driver, Joystick operator) {
    this.turret = turret;
    this.kiwilight = kiwilight;
    this.driver = driver;
    this.operator = operator;
    addRequirements(this.turret);

    targetPreviouslySeen = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set yaw pid
    double yawkP = Util.getAndSetDouble("Yaw Position kP", 0.05);
    double yawkI = Util.getAndSetDouble("Yaw Position kI", 0);
    double yawIZone = Util.getAndSetDouble("Yaw Position IZone", 75);
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
    double horizontalAngle = kiwilight.getHorizontalAngleToTarget() * -1;
    double verticalAngle = kiwilight.getVerticalAngleToTarget();

    double horizontalTicks = turret.getTotalYawTicks();
    double verticalTicks = Util.getAndSetDouble("Vertical Ticks", 1000);

    //horizontal angle
    if(Math.abs(horizontalAngle) != 180) {
      double horizontalTicksPerDegree = horizontalTicks / (double) Constants.TURRET_YAW_DEGREES;
      double horizontalTicksToTurn = horizontalAngle * horizontalTicksPerDegree;

      double newTargetPosition = turret.getYawPosition() + horizontalTicksToTurn;
      turret.setYawPosition(newTargetPosition);
    } else {
      //disable PID
      turret.setYawPercentOutput(0);
    }

    //vertical angle
    if(Math.abs(verticalAngle) != 180) {
      double verticalTicksPerInch = verticalTicks / (double) Constants.TURRET_PITCH_DEGREES;
      double verticalTicksToTurn = verticalAngle * verticalTicksPerInch;

      double newTargetPosition = turret.getPitchPosition() + verticalTicksToTurn;

      turret.setPitchPosition(newTargetPosition);
    } else {
      turret.setPitchPercentOutput(0);
    }

    //set controller rumbles
    if(targetPreviouslySeen && !kiwilight.targetSpotted()) {
      new CyborgCommandRumble(driver, 500, RumbleType.kRightRumble).schedule();
    }

    if(!targetPreviouslySeen && kiwilight.targetSpotted()) {
      new CyborgCommandRumble(operator, 500, RumbleType.kLeftRumble).schedule();
    }

    targetPreviouslySeen = kiwilight.targetSpotted();
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
