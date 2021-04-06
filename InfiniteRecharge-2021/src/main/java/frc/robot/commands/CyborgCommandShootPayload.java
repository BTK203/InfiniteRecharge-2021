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
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

/**
 * Autonomously shoots a specified number of power cells, or ends if no balls were shot in a specified time period.
 */
public class CyborgCommandShootPayload extends CommandBase {
  private SubsystemIntake intake;
  private SubsystemFeeder feeder;
  private SubsystemFlywheel flywheel;
  private SubsystemTurret turret;

  private int
    ballsToShoot,
    timeToWait,
    ballsShot;

  private long
    timeSinceLastShot;

  private boolean 
    lastFrameStable,
    runIntake;

  /**
   * Creates a new CyborgCommandShootPayload.
   * @param turret the Turret to use.
   * @param ballsToShoot the number of power cells to shoot.
   * @param timeToWait how many wait to shoot a ball before ending the command.
   */
  public CyborgCommandShootPayload(
    SubsystemIntake intake, 
    SubsystemFeeder feeder, 
    SubsystemFlywheel flywheel, 
    SubsystemTurret turret,
    int ballsToShoot, 
    int timeToWait,
    boolean runIntake
  ) {
    this.intake = intake;
    this.feeder = feeder;
    this.flywheel = flywheel;
    this.turret = turret;
    this.ballsToShoot = ballsToShoot;
    this.timeToWait = timeToWait;
    this.runIntake = runIntake;

    addRequirements(this.intake);
    addRequirements(this.feeder);
    //we do not require flywheel because CyborgCommandFlywheelVelocity takes care of rpms; we just read out velocity
  }

  public CyborgCommandShootPayload(
    SubsystemIntake intake,
    SubsystemFeeder feeder,
    SubsystemFlywheel flywheel,
    SubsystemTurret turret,
    int ballsToShoot,
    boolean runIntake
  ) {
    this(intake, feeder, flywheel, turret, ballsToShoot, Constants.SHOOT_PAYLOAD_DEFAULT_WAIT_TIME, runIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.ballsShot = 0;
    this.timeSinceLastShot = System.currentTimeMillis();
    this.lastFrameStable = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentFlywheelRPM = this.flywheel.getVelocity();
    // boolean flywheelStable = currentFlywheelRPM >= Constants.FLYWHEEL_STABLE_RPM;

    double fwStableRPM = Util.getAndSetDouble("FW Velocity Target", 6000) - 250;
    if(Constants.AUTO_OVERREV_TURRET) {
      fwStableRPM += Constants.AUTO_OVERREV_EXTRA_RPM;
    }

    boolean flywheelStable = currentFlywheelRPM >= fwStableRPM;

    DriverStation.reportWarning("flywheel stable: " + (flywheelStable ? "Yes" : "No") + " | current Flywheel RPM: " + Double.valueOf(currentFlywheelRPM).toString() + " | FW stable RPM: " + Double.valueOf(fwStableRPM).toString(), false);

    //decide whether or not to drive the feeder
    if(flywheelStable) { 
      intake.driveSlapper(Util.getAndSetDouble("Slap Speed", 0.5));
      feeder.driveBeater(Util.getAndSetDouble("Beat Speed", 1));
      feeder.driveFeeder(Util.getAndSetDouble("Feed Speed", 1));
      lastFrameStable = true;
      turret.setPitchPositioningDisabled(true);
    } else if(!Constants.AUTO_OVERREV_TURRET) {
      intake.driveSlapper(0);
      feeder.driveBeater(0);
      feeder.driveFeeder(0);
      turret.setPitchPositioningDisabled(false);
    }

    if(lastFrameStable && !flywheelStable) { //bro, rpm was stable last time, so we just shot a ball
      if(timeSinceLastShot >= Util.getAndSetDouble("Ball Shot Timeout", 100)) {
        ballsShot++;
      }

      lastFrameStable = false;
      timeSinceLastShot = System.currentTimeMillis();
    }

    if(runIntake) {
      intake.driveEater(Util.getAndSetDouble("Eat Speed", 1));
    }

    SmartDashboard.putBoolean("Auto Flywheel Stable", flywheelStable);
    // SmartDashboard.putBoolean("KiwiLight Aligned", kiwilightStable());
    SmartDashboard.putNumber("Auto Balls Shot", ballsShot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.driveEater(0);
    intake.driveSlapper(0);
    feeder.driveBeater(0);
    feeder.driveFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ballsShot == ballsToShoot) {
      //payload has been shot, we have succeeded!
      DriverStation.reportWarning("PAYLOAD SHOT, CyborgCommandShootPayload exits victoriously.", false);
      return true;
    }

    long elapsedTime = System.currentTimeMillis() - timeSinceLastShot;
    if(elapsedTime >= timeToWait) {
      //payload not shot, but command has waited too long.
      DriverStation.reportWarning("TOO MUCH TIME ELAPSED, CyborgCommandShootPayload is defeated.", false);
      return true;
    }

    return false;
  }
}
