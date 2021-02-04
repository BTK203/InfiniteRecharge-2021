/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;
import frc.robot.util.Xbox;

public class CyborgCommandSmartMoveTurret extends CommandBase {
  private SubsystemTurret turret;
  private long lastTime;
  private double desiredPitchPosition;

  /**
   * Creates a new CyborgCommandSmartMoveTurret.
   * Uses a PID loop to control the pitch in order to overcome the tension caused by the springs
   */
  public CyborgCommandSmartMoveTurret(SubsystemTurret turret) {
    this.turret = turret;
    this.lastTime = 0;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //init lastTime so it is usable
    this.lastTime = System.currentTimeMillis();
    this.desiredPitchPosition = turret.getPitchPosition();

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
    //SMART pitch control
    Joystick operator = Robot.getRobotContainer().getOperator();
    double pitchDemand = Xbox.RIGHT_Y(operator);
    int timeElapsed = (int) (System.currentTimeMillis() - lastTime);
    pitchDemand *= timeElapsed;
    pitchDemand *= Util.getAndSetDouble("Turret Spin Inhibitor Pitch", 1);

    double newPitchPosition = desiredPitchPosition + pitchDemand;
    // conditionally freeze pitch if at limits
    if(
      (turret.getPitchLowerLimit() && pitchDemand > 0) ||
      (turret.getPitchUpperLimit() && pitchDemand < 0)
    ) {
      newPitchPosition = turret.getPitchPosition();
    }

    SmartDashboard.putNumber("Desired Pitch Position", newPitchPosition);
    turret.setPitchPosition(newPitchPosition);

    desiredPitchPosition = newPitchPosition;
    lastTime = System.currentTimeMillis();

    //DUMB yaw control
    turret.setYawPercentOutput(Xbox.LEFT_X(operator));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setPitchPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
