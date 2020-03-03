/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

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
    public static ParallelCommandGroup autoInitCommand(SubsystemDrive drivetrain, SubsystemTurret turret) {
        //numbers we will use
        double startDriveDistance = Util.getAndSetDouble("Initiation Drive", -20);

        //instantiate commands we will use
        CyborgCommandZeroTurret       zeroTurret    = new CyborgCommandZeroTurret(turret);
        CyborgCommandDriveDistance    driveOffLine  = new CyborgCommandDriveDistance(drivetrain, startDriveDistance);

        return zeroTurret
               .alongWith(driveOffLine);
    }

    public static ParallelCommandGroup theBareMinimum (
        SubsystemDrive drivetrain,
        SubsystemTurret turret,
        SubsystemFlywheel flywheel,
        SubsystemIntake intake,
        SubsystemFeeder feeder,
        SubsystemReceiver kiwilight
    ) {
        Point3D initPos = getStartingPositionAfterInit();
        int yawTarget = getYawTicksToPowerPort(initPos.x(), initPos.y());
        int pitchTarget = getPitchTicksToPowerPort(initPos.y(), initPos.z());

        //zero turret, drive off init line, and set the turret position
        ParallelCommandGroup init = autoInitCommand(drivetrain, turret);
        CyborgCommandSetTurretPosition setPosition = new CyborgCommandSetTurretPosition(turret, yawTarget, pitchTarget);
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

        //run everything along with driving the flywheel
        CyborgCommandFlywheelVelocity driveFlywheel = new CyborgCommandFlywheelVelocity(flywheel);
        ParallelCommandGroup defaultAuto = driveFlywheel.alongWith(initAndShootPayload);
        return defaultAuto;
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
     * Approximately calculates the yaw ticks to align with the power port.
     * @param x x coordinate of robot
     * @param y y coordinate of robot
     * @return the number of encoder yaw ticks for turret to align with powerport.
     */
    private static int getYawTicksToPowerPort(double x, double y) {
        double hDistanceToPort = x - Constants.POWERPORT_LOCATION.x();
        double vDistanceToPort = y - Constants.POWERPORT_LOCATION.y();

        double tangent = hDistanceToPort / vDistanceToPort;
        double angleToPort = Math.atan(tangent);
        double angleFromZero = Constants.TURRET_YAW_DEGREES_AT_ZERO + angleToPort;

        //calculate number of ticks needed to achieve angle
        double ticksPerDegree = Constants.DEFAULT_TURRET_YAW_TICKS / (double) Constants.TURRET_YAW_DEGREES;
        double targetTicks = angleFromZero * ticksPerDegree;
        return (int) targetTicks;
    }

    /**
     * Approximately calculates the pitch ticks to align with the power port.
     * @param y y coordinate of robot
     * @param z z coordinate of robot
     * @return the number of encoder ticks for pitch to align with the powerport.
     */
    private static int getPitchTicksToPowerPort(double y, double z) {
        double distanceToPort = y - Constants.POWERPORT_LOCATION.y();
        double heightFromTurret = Constants.POWERPORT_LOCATION.z() - z;

        //calculate angle
        double tangent = heightFromTurret / distanceToPort;
        double angleToPort = Math.atan(tangent);
        double angleFromZero = Constants.TURRET_PITCH_DEGREES_AT_ZERO + angleToPort;

        //calcualte number of ticks to acheive angle
        double ticksPerDegree = Constants.DEFAULT_TURRET_PITCH_TICKS / (double) Constants.TURRET_PITCH_DEGREES;
        double targetTicks = angleFromZero * ticksPerDegree;
        return (int) targetTicks;
    }
}
