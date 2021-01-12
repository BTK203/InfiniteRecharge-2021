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
 * "Bare Minimum" autonomous command, driving off the line and shooting all
 * preloaded power cells.
 */
public class BareMinimumAuto implements IAuto {
    private Command 
        init,
        positionTurret,
        alignTurret,
        shootPayload;

    public BareMinimumAuto(
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

        this.positionTurret = new CyborgCommandSetTurretPosition(turret, yawTarget, Constants.AUTO_INIT_PITCH_TARGET);
        this.alignTurret = new CyborgCommandAlignTurret(turret, kiwilight, true);
        
        //shoot preloaded power cells
        this.shootPayload = new CyborgCommandShootPayload(intake, feeder, flywheel, kiwilight, turret, 3, 15000, false);
    }

    public Command getCommand() {
        //init then position
        Command initAndPosition = init.andThen(positionTurret);

        //align and shoot
        Command alignAndShoot = this.alignTurret.andThen(this.shootPayload);

        //init then position, then align and shoot
        return initAndPosition.andThen(alignAndShoot);
    }

    public boolean requiresFlywheel() {
        return true;
    }
}
