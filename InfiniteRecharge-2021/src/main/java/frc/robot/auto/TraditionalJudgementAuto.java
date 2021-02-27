// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.ConstantCommandDriveIntake;
import frc.robot.commands.CyborgCommandAlignTurret;
import frc.robot.commands.CyborgCommandEmulatePath;
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
 * A more "traditional" auto used for judging.
 */
public class TraditionalJudgementAuto implements IAuto {
    private InstantCommand zeroDrivetrain;
    private CyborgCommandZeroTurret zeroTurret;
    private CyborgCommandSetTurretPosition 
        positionTurretForDriveBack, //position: yaw: -510901, pitch: -7584
        positionTurretForFinalShoot; //position: yaw: -614919, pitch: -6342

    private CyborgCommandAlignTurret
        alignPt1,
        alignPt2;

    private CyborgCommandShootPayload
        shoot6Cells,
        shoot5Cells;

    private CyborgCommandEmulatePath
        collectCellsPt1,
        collectCellsPt2,
        driveToSite;

    private CyborgCommandSmartDriveDistance
        driveToCollect,
        driveToAvoidPost;

    private ConstantCommandDriveIntake
        driveIntakePt1,
        driveIntakePt2;

    public TraditionalJudgementAuto(
        SubsystemDrive drivetrain,
        SubsystemTurret turret,
        SubsystemIntake intake,
        SubsystemFeeder feeder,
        SubsystemFlywheel flywheel,
        SubsystemReceiver kiwilight
    ) {
        this.zeroDrivetrain = new InstantCommand( () -> { Robot.getRobotContainer().zeroAllDrivetrain(); } );
        this.zeroTurret = new CyborgCommandZeroTurret(turret);
        this.positionTurretForDriveBack = new CyborgCommandSetTurretPosition(turret, 510901, -7584);
        this.positionTurretForFinalShoot = new CyborgCommandSetTurretPosition(turret, 614919, -6342);
        this.alignPt1 = new CyborgCommandAlignTurret(turret, kiwilight);
        this.alignPt2 = new CyborgCommandAlignTurret(turret, kiwilight);
        this.shoot6Cells = new CyborgCommandShootPayload(intake, feeder, flywheel, turret, 6, true);
        this.shoot5Cells = new CyborgCommandShootPayload(intake, feeder, flywheel, turret, 5, false);
        this.collectCellsPt1 = new CyborgCommandEmulatePath(drivetrain, "/home/lvuser/ja2_collectCells_pt1.txt");
        this.collectCellsPt2 = new CyborgCommandEmulatePath(drivetrain, "/home/lvuser/ja2_collectCells_pt2.txt");
        this.driveToSite     = new CyborgCommandEmulatePath(drivetrain, "/home/lvuser/ja2_driveToSite.txt");
        this.driveToCollect = new CyborgCommandSmartDriveDistance(drivetrain, -168, 0.25);
        this.driveToAvoidPost = new CyborgCommandSmartDriveDistance(drivetrain, 36, 0.25);
        this.driveIntakePt1 = new ConstantCommandDriveIntake(intake, feeder);
        this.driveIntakePt2 = new ConstantCommandDriveIntake(intake, feeder);
    }

    public Command getCommand() {
        //drive back to collect balls while shooting the ones we have
        Command collectWhileShooting = shoot6Cells.raceWith(alignPt1);
        Command driveBackWhileShooting = driveToCollect.alongWith(collectWhileShooting);

        //collect other cells by structure
        Command driveToOtherCells = collectCellsPt1.andThen(driveToAvoidPost, collectCellsPt2);
        Command collectOtherCells = driveToOtherCells.raceWith(driveIntakePt2);

        //shoot remaining cells
        Command alignAndShootRemainingCells = alignPt2.raceWith(shoot5Cells);

        return zeroDrivetrain.andThen(
            zeroTurret,
            positionTurretForDriveBack,
            driveBackWhileShooting
            // collectOtherCells,
            // driveToSite,
            // positionTurretForFinalShoot,
            // alignAndShootRemainingCells
        );
    }

    public boolean requiresFlywheel() {
        return true;
    }
}
