/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.CyborgCommandAlignTurret;
import frc.robot.commands.CyborgCommandDriveDistance;
import frc.robot.commands.CyborgCommandSetTurretPosition;
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
        init,
        positionTurret,
        alignTurret,
        shootOneBall,
        backIntoTrench,
        shootWhileCollecting;

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
        //drive off line and zero turret
        this.init = new InitAuto(drivetrain, turret).getCommand();

        //set turret position and align
        int yawTarget = Auto.getYawTicksToTarget(Util.getAndSetDouble("Auto Start Offset", 0));
        SmartDashboard.putNumber("Auto Yaw Target", yawTarget);
        this.positionTurret = new CyborgCommandSetTurretPosition(turret, yawTarget, Constants.AUTO_INIT_YAW_TARGET);
        this.alignTurret = new CyborgCommandAlignTurret(turret, kiwilight);
        
        //shoot one ball before backing off to make sure we dont break the rules
        int timeToWait = (int) Util.getAndSetDouble("Auto Payload Timeout", 3000);
        this.shootOneBall = new CyborgCommandShootPayload(intake, feeder, flywheel, kiwilight, 1, timeToWait, false);

        //back into trench
        this.backIntoTrench = new CyborgCommandDriveDistance(drivetrain, Util.getAndSetDouble("Trench Distance", 190));

        //shoot remaining payload
        this.shootWhileCollecting = new CyborgCommandShootPayload(intake, feeder, flywheel, kiwilight, 5, timeToWait, true);
    }

    public Command getCommand() {
        //as an experiment
        return init.andThen(
            shootOneBall.andThen(
                shootWhileCollecting.alongWith(backIntoTrench)
            ).alongWith(alignTurret)
        );
    }

    public boolean requiresFlywheel() {
        return true;
    }
}
