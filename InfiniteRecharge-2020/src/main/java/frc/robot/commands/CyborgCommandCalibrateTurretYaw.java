/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

public class CyborgCommandCalibrateTurretYaw extends CommandBase {
  private SubsystemTurret turret;
  private boolean
    zeroing,
    finished;

  /**
   * Creates a new CyborgCommandCalibrateTurretYaw.
   */
  public CyborgCommandCalibrateTurretYaw(SubsystemTurret turret) {
    this.turret = turret;
    addRequirements(this.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    zeroing = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    if(zeroing) {
      turret.setYawPercentOutput(Util.getAndSetDouble("Calibrate Speed", 0.3));

      if(turret.getYawRightlimit()) {
        turret.setCurrentYawEncoderPosition(0);
        zeroing = false;
      }
    } else {
      DriverStation.reportWarning("Looking For Max", false);
      turret.setYawPercentOutput(Util.getAndSetDouble("Calibrate Speed", 0.3) * -1);
      if(turret.attemptToSetTotalYawTicks()) {
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setYawPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
