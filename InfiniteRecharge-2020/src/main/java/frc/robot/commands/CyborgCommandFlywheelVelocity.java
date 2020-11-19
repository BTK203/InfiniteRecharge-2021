/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.util.Util;

public class CyborgCommandFlywheelVelocity extends CommandBase {
  private SubsystemFlywheel flywheel;
  private boolean rpmOverridden;
  private double overrideRPM;

  /**
   * Creates a new CyborgCommandFlywheelVelocity.
   */
  public CyborgCommandFlywheelVelocity(SubsystemFlywheel flywheel) {
    this.flywheel = flywheel;
    this.rpmOverridden = false;
    this.overrideRPM = 0;
    addRequirements(this.flywheel);
    SmartDashboard.putBoolean("Drive FW Velocity", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double p = Util.getAndSetDouble("FW Velocity kP", 0.0014);
    double i = Util.getAndSetDouble("FW Velocity kI", 0.000005);
    double d = Util.getAndSetDouble("FW Velocity kD", 0);
    double f = Util.getAndSetDouble("FW Velocity kF", 0.000185);

    double upperOutLimit = Util.getAndSetDouble("FW Velocity Max Out", 1);
    double lowerOutLimit = Util.getAndSetDouble("FW Velocity Min Out", -1);

    double izone = Util.getAndSetDouble("FW Velocity IZone", 100);

    flywheel.setPIDF(p, i, d, f, lowerOutLimit, upperOutLimit, izone);

    SmartDashboard.putBoolean("Drive FW Velocity", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Util.getAndSetDouble("FW Velocity Target", 3000) / Constants.FLYWHEEL_GEAR_RATIO;
    if(rpmOverridden) {
      speed = overrideRPM;
    }
    
    flywheel.setVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.setVelocity(0);
    flywheel.setFlywheelPercentOutput(0);
    SmartDashboard.putBoolean("Drive FW Velocity", false);

    if(interrupted) {
      DriverStation.reportError("FW VELOCITY PID WAS INTERRUPTED", false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Set whether or not this command should override the target RPM set in Preferences.
   * @param rpm The RPM to override to.
   */
  public void overrideRPM(double rpm) {
    this.rpmOverridden = true;
    this.overrideRPM = rpm / Constants.FLYWHEEL_GEAR_RATIO;
  }

  /**
   * Marks that the flywheel RPM should no longer be overridden and instead use the value from Prefs.
   */
  public void stopOverriding() {
    this.rpmOverridden = false;
    this.overrideRPM = 0;
  }
}
