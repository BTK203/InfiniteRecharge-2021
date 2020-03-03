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
import frc.robot.util.Util;

/**
 * Class that links together everything auto.
 */
public class Auto {
    public static ParallelCommandGroup autoInitCommand(SubsystemDrive drivetrain, SubsystemTurret turret, SubsystemFlywheel flywheel) {
        //numbers we will use
        double startDriveDistance = Util.getAndSetDouble("Initiation Drive", -20);

        //instantiate commands we will use
        CyborgCommandFlywheelVelocity driveFlywheel = new CyborgCommandFlywheelVelocity(flywheel);
        CyborgCommandZeroTurret       zeroTurret    = new CyborgCommandZeroTurret(turret);
        CyborgCommandDriveDistance    driveOffLine  = new CyborgCommandDriveDistance(drivetrain, startDriveDistance);

        return driveFlywheel
                .alongWith(zeroTurret)
                .alongWith(driveOffLine);
    }

    public static SequentialCommandGroup theBareMinimum (
        SubsystemDrive drivetrain,
        SubsystemTurret turret,
        SubsystemFlywheel flywheel,
        SubsystemIntake intake,
        SubsystemFeeder feeder,
        SubsystemReceiver kiwilight
    ) {
        //grab some numbers and make some calculations
        double startOffset = Util.getAndSetDouble("Start Offset", 48);
        double startDriveDistance = Util.getAndSetDouble("Initiation Drive", -20);

        //horizontal angle to power port
        double xDistanceToPowerPort = startOffset - Constants.DISTANCE_POWERPORT_TO_SIDE;
        double yDistanceToPowerPort = Constants.DISTANCE_INIT_LINE_TO_ALLIANCE_WALL + startDriveDistance;
        double horizontalAngleToPowerPort = Math.atan(yDistanceToPowerPort / xDistanceToPowerPort);
        
        //vertical angle to power port
        double verticalAngleToPowerPort = Math.atan(Constants.POWER_PORT_HEIGHT / yDistanceToPowerPort);

        //calculate yaw encoder targets
        double yawTicksPerDegree = Constants.DEFAULT_TURRET_YAW_TICKS / Constants.TURRET_YAW_DEGREES;
        double yawTarget = (horizontalAngleToPowerPort * yawTicksPerDegree) + (Constants.TURRET_CENTER_ANGLE_OFFSET_YAW * yawTicksPerDegree);

        //calculate pitch encoder targets
        double pitchTicksPerDegree = Constants.DEFAULT_TURRET_PITCH_TICKS / Constants.TURRET_PITCH_DEGREES;
        double pitchTarget = (verticalAngleToPowerPort * pitchTicksPerDegree) - (Constants.TURRET_PITCH_CAMERA_OFFSET * yawTicksPerDegree);

        //instantiate commands
        ParallelCommandGroup initCommand = autoInitCommand(drivetrain, turret, flywheel);
        CyborgCommandSetTurretPosition setTurretPosition = new CyborgCommandSetTurretPosition(turret, (int) yawTarget, (int) pitchTarget);

        CyborgCommandAlignTurret alignTurret = new CyborgCommandAlignTurret(turret, kiwilight);
        CyborgCommandShootPayload shootPayload = new CyborgCommandShootPayload(
            intake,
            feeder,
            flywheel,
            kiwilight,
            Constants.AUTO_INIT_BALL_COUNT,
            Constants.AUTO_PAYLOAD_TIMEOUT
        );

        ParallelCommandGroup alignAndShoot = alignTurret.alongWith(shootPayload);

        return initCommand.andThen(setTurretPosition).andThen(alignAndShoot);
    }

    /**
     * 
     * @param x x coordinate of robot
     * @param y y coordinate of robot
     * @param z z coordinate of turret
     * @return the number of encoder ticks for turret
     */
    private static int getTicksToPowerPort(double x, double y, double z) {
        return 0;
    }
}
