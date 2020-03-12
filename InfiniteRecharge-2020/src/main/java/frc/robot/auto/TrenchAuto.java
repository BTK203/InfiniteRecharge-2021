/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ConstantCommandDriveIntake;
import frc.robot.commands.CyborgCommandAlignTurret;
import frc.robot.commands.CyborgCommandSetTurretPosition;
import frc.robot.commands.CyborgCommandShootPayload;
import frc.robot.commands.CyborgCommandSmartDriveDistance;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemFlywheel;


/**
 * Possible 8 ball auto. Backs off line, shoots the 3 preloaded balls, then
 * backs to collect the 5 balls in the trench run. Drives to the front of the
 * trench run and shoots those 5 balls.
 */
public class TrenchAuto implements IAuto {
    private Command
        init,
        positionTurret,
        shootStartingPayload,
        setLowerTurretPosition,
        collectBalls,
        backIntoTrench,
        driveForward,
        alignTurret,
        shootRemainingPayload;

    public TrenchAuto(
        SubsystemDrive drivetrain,
        SubsystemTurret turret,
        SubsystemReceiver kiwilight,
        SubsystemIntake intake,
        SubsystemFeeder feeder,
        SubsystemFlywheel flywheel
    ) {
        this.init = new InitAuto(drivetrain, turret).getCommand();
        
        //position turret to get target in vision view
        int yawTarget = Auto.getYawTicksToTarget();
        int pitchTarget = Constants.AUTO_INIT_PITCH_TARGET;
        this.positionTurret = new CyborgCommandSetTurretPosition(turret, yawTarget, pitchTarget);

        //align
        this.alignTurret = new CyborgCommandAlignTurret(turret, kiwilight);
        this.shootStartingPayload = new CyborgCommandShootPayload(intake, feeder, flywheel, kiwilight, turret, 3, 15000, false);

        //set the lower turret position so that the turret doesn't get destroyed
        this.setLowerTurretPosition = new CyborgCommandSetTurretPosition(turret, yawTarget, 0);

        this.backIntoTrench = new CyborgCommandSmartDriveDistance(drivetrain, Constants.AUTO_DEEP_TRENCH_DISTANCE, 0.75);
        this.collectBalls = new ConstantCommandDriveIntake(intake, feeder);
        this.driveForward = new CyborgCommandSmartDriveDistance(drivetrain, Constants.AUTO_DEEP_TRENCH_DISTANCE * -1, 0.75);

        //shoot balls
        this.shootRemainingPayload = new CyborgCommandShootPayload(intake, feeder, flywheel, kiwilight, turret, 1000, 15000, false);
    }

    public Command getCommand() {
        Command initAndShoot = init.andThen(positionTurret, shootStartingPayload.raceWith(alignTurret));
        Command lowerTurretAndDrive = setLowerTurretPosition.andThen(backIntoTrench.raceWith(collectBalls), driveForward);
        Command shootForRemainingTime = positionTurret.andThen(shootRemainingPayload.raceWith(alignTurret));

        return initAndShoot.andThen(lowerTurretAndDrive, shootForRemainingTime);
    }

    public boolean requiresFlywheel() {
        return true;
    }
}
