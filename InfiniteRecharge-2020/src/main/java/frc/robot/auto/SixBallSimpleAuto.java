/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ConstantCommandDriveIntake;
import frc.robot.commands.CyborgCommandAlignTurret;
import frc.robot.commands.CyborgCommandDriveDistance;
import frc.robot.commands.CyborgCommandShootPayload;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

/**
 * A slightly cooler auto than it's other simpler counterparts.
 * This auto will back off the line while zeroing the turret, and 
 * then slowly drive backwards, collecting power cells from the trench while shooting power
 * cells already loaded in.
 */
public class SixBallSimpleAuto implements IAuto {
    private Command
        initAndShoot,
        driveBack,
        collectBalls,
        driveForward,
        alignTurret,
        shootPayload;

    /**
     * Creates a new SixBallSimpleAuto, initalizing commands
     */
    public SixBallSimpleAuto (
        SubsystemDrive drivetrain,
        SubsystemTurret turret,
        SubsystemReceiver kiwilight,
        SubsystemIntake intake,
        SubsystemFeeder feeder,
        SubsystemFlywheel flywheel
    ) {
        this.initAndShoot = new BareMinimumAuto(drivetrain, turret, kiwilight, intake, feeder, flywheel).getCommand();
        
        double trenchDistance = Util.getAndSetDouble("Trench Distance", -132);
        this.driveBack = new CyborgCommandDriveDistance(drivetrain, trenchDistance, 0.75);
        this.collectBalls = new ConstantCommandDriveIntake(intake, feeder);
        this.driveForward = new CyborgCommandDriveDistance(drivetrain, trenchDistance * -1, 0.75);

        //align
        this.alignTurret = new CyborgCommandAlignTurret(turret, kiwilight);

        //shoot balls
        int ballsToShoot = (int) Util.getAndSetDouble("Init Auto Payload", 3);
        int timeToWait = (int) Util.getAndSetDouble("Auto Payload Timeout", 3000);
        this.shootPayload = new CyborgCommandShootPayload(intake, feeder, flywheel, kiwilight, ballsToShoot, timeToWait, false);
    }

    public Command getCommand() {
        return initAndShoot.andThen(
            driveBack.raceWith(collectBalls).andThen(
                driveForward.andThen(shootPayload.raceWith(alignTurret))
            )
        );
    }

    public boolean requiresFlywheel() {
        return true;
    }
}
