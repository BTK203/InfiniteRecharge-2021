/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CyborgCommandAlignTurret;
import frc.robot.commands.CyborgCommandDriveDistance;
import frc.robot.commands.CyborgCommandFlywheelVelocity;
import frc.robot.commands.CyborgCommandSetTurretPosition;
import frc.robot.commands.CyborgCommandShootPayload;
import frc.robot.commands.CyborgCommandZeroTurret;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Point3D;
import frc.robot.util.Util;

/**
 * Class that links together everything auto.
 */
public class Auto {
    public static Command autoInitCommand(SubsystemDrive drivetrain, SubsystemTurret turret) {
        //numbers we will use
        double startDriveDistance = Util.getAndSetDouble("Initiation Drive", -20);

        //instantiate commands we will use
        InstantCommand                zeroDrivetrain = new InstantCommand(() -> drivetrain.zeroEncoders(), drivetrain);
        CyborgCommandZeroTurret       zeroTurret     = new CyborgCommandZeroTurret(turret);
        CyborgCommandDriveDistance    driveOffLine   = new CyborgCommandDriveDistance(drivetrain, startDriveDistance);

        ParallelCommandGroup zeroTurretWhileDriving = zeroTurret.alongWith(driveOffLine);
        return zeroDrivetrain.andThen(zeroTurretWhileDriving);
    }

    public static Command theBareMinimum (
        SubsystemDrive drivetrain,
        SubsystemTurret turret,
        SubsystemFlywheel flywheel,
        SubsystemIntake intake,
        SubsystemFeeder feeder,
        SubsystemReceiver kiwilight
    ) {
        int yawTarget = getYawTicksToTarget(Util.getAndSetDouble("Auto Start Offset", 0));
        SmartDashboard.putNumber("Auto Yaw Target", yawTarget);

        //zero turret, drive off init line, and set the turret position
        Command init = autoInitCommand(drivetrain, turret);
        CyborgCommandSetTurretPosition setPosition = new CyborgCommandSetTurretPosition(turret, yawTarget, Constants.AUTO_INIT_YAW_TARGET);
        SequentialCommandGroup initAndSetPosition = init.andThen(setPosition);

        //align turret and shoot payload
        CyborgCommandAlignTurret align = new CyborgCommandAlignTurret(turret, kiwilight);
        CyborgCommandShootPayload shootPayload = new CyborgCommandShootPayload(
            intake, 
            feeder, 
            flywheel, 
            kiwilight, 
            (int) Util.getAndSetDouble("Init Auto Payload", 3), 
            (int) Util.getAndSetDouble("Auto Payload Timeout", 3000)
        );
        ParallelCommandGroup alignAndShoot = align.alongWith(shootPayload);

        //bring above two sequences together sequentially
        SequentialCommandGroup initAndShootPayload = initAndSetPosition.andThen(alignAndShoot);
        return initAndShootPayload;
    }

    /**
     * Gets the position of the turret after the init auto drive is finished.
     * @return position of the turret after first drive is done.    
     */
    private static Point3D getStartingPositionAfterInit() {
        double hDistanceFromWall = Util.getAndSetDouble("Starting Horizontal Position", 48);
        double vDistanceFromWall = Constants.DISTANCE_INIT_LINE_TO_ALLIANCE_WALL + Util.getAndSetDouble("Start Drive", -20);
        return new Point3D(hDistanceFromWall, vDistanceFromWall, Constants.TURRET_HEIGHT);
    }

    /**
     * gets the approximate yaw ticks to look at the target
     * @param offsetY inches away from the side wall closest to the power port.
     * @return ticks to set the turret to
     */
    private static int getYawTicksToTarget(double offsetY) {
        double offsetYawTicks = (offsetY * Constants.TURRET_TARGET_TICKS_PER_INCH) + Constants.TURRET_APPROX_TARGET_TICKS_CLOSE;
        offsetYawTicks *= -1;
        return (int) offsetYawTicks;
    }
}
