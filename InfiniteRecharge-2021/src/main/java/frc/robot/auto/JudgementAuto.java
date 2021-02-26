// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ConstantCommandDriveIntake;
import frc.robot.commands.CyborgCommandAlignTurret;
import frc.robot.commands.CyborgCommandEmulatePath;
import frc.robot.commands.CyborgCommandFlywheelVelocity;
import frc.robot.commands.CyborgCommandSetTurretPosition;
import frc.robot.commands.CyborgCommandShootPayload;
import frc.robot.commands.CyborgCommandSmartDriveDistance;
import frc.robot.commands.CyborgCommandWait;
import frc.robot.commands.CyborgCommandZeroTurret;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

/**
 * This auto is written for the optional autonomous video for judging. 
 * Simply to flex
 */
public class JudgementAuto implements IAuto {
  //commands listed in the approximate order in which they are used.
  private CyborgCommandZeroTurret
    zeroTurret,
    zeroTurretAgain; //extra zero turret needed as I cannot run a command twice during a single auto.
    
  private InstantCommand zeroDrivetrain;
  private CyborgCommandEmulatePath
    driveToPowerCells,
    driveToSite,
    driveBackToStart;

  private ConstantCommandDriveIntake driveIntake;
  private CyborgCommandWait 
    finishCollecting,
    waitToAlign;
  
  private CyborgCommandSetTurretPosition positionTurret;
  private CyborgCommandSmartDriveDistance driveForward;
  private CyborgCommandFlywheelVelocity driveFlywheel;
  private CyborgCommandAlignTurret align;
  private CyborgCommandShootPayload shootPowerCells;

  /**
   * Creates a new JudgementAuto.
   */
  public JudgementAuto(
    SubsystemDrive drivetrain,
    SubsystemIntake intake,
    SubsystemFeeder feeder,
    SubsystemTurret turret,
    SubsystemFlywheel flywheel,
    SubsystemReceiver kiwilight
  ) {
    zeroTurret              = new CyborgCommandZeroTurret(turret);
    zeroTurretAgain         = new CyborgCommandZeroTurret(turret);
    zeroDrivetrain          = new InstantCommand( () -> { Robot.getRobotContainer().zeroAllDrivetrain(); } );
    driveToPowerCells       = new CyborgCommandEmulatePath(drivetrain, Constants.JUDGEMENT_AUTO_DRIVE_TO_POWER_CELLS_PATH_FILE);
    driveToSite             = new CyborgCommandEmulatePath(drivetrain, Constants.JUDGEMENT_AUTO_DRIVE_TO_SITE_PATH_FILE);
    driveBackToStart        = new CyborgCommandEmulatePath(drivetrain, Constants.JUDGEMENT_AUTO_DRIVE_BACK_TO_START_PATH_FILE);
    driveIntake             = new ConstantCommandDriveIntake(intake, feeder);
    finishCollecting        = new CyborgCommandWait(750);
    waitToAlign             = new CyborgCommandWait(1000);
    positionTurret          = new CyborgCommandSetTurretPosition(turret, Constants.JUDGEMENT_AUTO_YAW_TARGET, Constants.JUDGEMENT_AUTO_PITCH_TARGET);
    driveForward            = new CyborgCommandSmartDriveDistance(drivetrain, Constants.JUDGEMENT_AUTO_SHOOT_DRIVE_DISTANCE, Constants.JUDGEMENT_AUTO_SHOOT_DRIVE_POWER, -90, 0.40);
    driveFlywheel           = new CyborgCommandFlywheelVelocity(flywheel);
    align                   = new CyborgCommandAlignTurret(turret, kiwilight, false, (int) Util.getAndSetDouble("Judgement Auto Turret Yaw Offset", 0));
    shootPowerCells         = new CyborgCommandShootPayload(intake, feeder, flywheel, turret, Constants.JUDGEMENT_AUTO_BALLS_TO_SHOOT, false);
  
    if(Constants.AUTO_OVERREV_TURRET) {
      driveFlywheel.overrideRPM(Util.getAndSetDouble("FW Velocity Target", 6000) + Constants.AUTO_OVERREV_EXTRA_RPM);
    }
  }

  /**
   * Returns the command for the entire auto.
   * Order of events:
   * -Zero Drivetrain
   * -Zero Turret
   * -Start flywheel
   *   -Run Intake
   *     -Drive to power cells
   *     -Finish collecting power cells
   *   -Drive path "ja_driveToSite.txt"
   *   -Position Turret
   *   -Align Turret while driving and shooting
   * -Drive path "ja_driveBackToStart"
   * -Zero turret again
   */
  public Command getCommand() {
    //join any commands that need to be joined
    //the next two commands deal with driving to and collecting the power cells.
    Command driveAndCollect = driveToPowerCells.andThen(finishCollecting);
    Command driveAndCollectWithIntake = driveAndCollect.raceWith(driveIntake);
    
    Command waitAndThenShoot = waitToAlign.andThen(shootPowerCells);
    Command driveAndShoot = align.raceWith(driveForward, waitAndThenShoot);

    //form it all together
    Command collectAndShootPowerCells = driveAndCollectWithIntake.andThen(driveToSite, positionTurret, driveAndShoot);

    //full command that executes everything with the flywheel. All above commands can be run with this command
    Command collectAndShootPowerCellsWithFlywheel = collectAndShootPowerCells.raceWith(driveFlywheel);

    Command finalCommand = zeroDrivetrain.andThen(zeroTurret, collectAndShootPowerCellsWithFlywheel, driveBackToStart, zeroTurretAgain);
    return finalCommand;
  }

  /**
   * Returns true if this auto requires the flywheel to be spinning at all times, false otherwise.
   */
  public boolean requiresFlywheel() {
    return false;
  }
}
