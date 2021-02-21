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
import frc.robot.commands.CyborgCommandZeroTurret;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemTurret;

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

  private ConstantCommandDriveIntake collectPowerCells;
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
    collectPowerCells       = new ConstantCommandDriveIntake(intake, feeder);
    positionTurret          = new CyborgCommandSetTurretPosition(turret, Constants.AUTO_INIT_YAW_TARGET, Constants.AUTO_INIT_PITCH_TARGET, true, kiwilight);
    driveForward            = new CyborgCommandSmartDriveDistance(drivetrain, Constants.JUDGEMENT_AUTO_SHOOT_DRIVE_DISTANCE, Constants.JUDGEMENT_AUTO_SHOOT_DRIVE_POWER);
    driveFlywheel           = new CyborgCommandFlywheelVelocity(flywheel);
    align                   = new CyborgCommandAlignTurret(turret, kiwilight);
    shootPowerCells         = new CyborgCommandShootPayload(intake, feeder, flywheel, turret, Constants.JUDGEMENT_AUTO_BALLS_TO_SHOOT, false);
  }

  /**
   * Returns the command for the entire auto.
   * Order of events:
   * - Zero turret
   * - Drive path "ja_driveToCells.txt" WHILE running intake (collecting power cells) UNTIL start of 3-pt turn
   * - Drive path "ja_driveToSite.txt" to completion.
   * - Start flywheel (will run until cancelation)
   *   - Drive forwards 132 inches (11 ft)
   *      - Position turret
   *      - Align turret
   *      - Shoot payload
   * - Cancel flywheel
   * - Drive path "ja_driveBackToStart.txt"
   */
  public Command getCommand() {
    //join any commands that need to be joinec
    Command driveAndCollectPowerCells  = collectPowerCells.raceWith(driveToPowerCells); //will drive intake until path is completed

    //next three commands are all one group. To run the group, schedule doShootingThings.
    Command driveAndShootPowerCells    = align.raceWith(driveForward, shootPowerCells); //align WHILE driving forward and shooting power cells.
    Command positionAndShootPowerCells = positionTurret.andThen(driveAndShootPowerCells); //position turret than do ^
    Command doShootingThings           = driveFlywheel.raceWith(positionAndShootPowerCells); //do all of ^ while spinning flywheel
    
    Command finalCommand = zeroDrivetrain.andThen(zeroTurret, driveAndCollectPowerCells, driveToSite, doShootingThings, driveBackToStart, zeroTurretAgain);
    return finalCommand;
  }

  /**
   * Returns true if this auto requires the flywheel to be spinning at all times, false otherwise.
   */
  public boolean requiresFlywheel() {
    return false;
  }
}
