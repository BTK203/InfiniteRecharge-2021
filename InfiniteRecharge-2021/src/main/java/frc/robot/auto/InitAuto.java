/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.CyborgCommandDriveDistance;
import frc.robot.commands.CyborgCommandSmartDriveDistance;
import frc.robot.commands.CyborgCommandZeroTurret;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

/**
 * Auto command run at the beginning of every auto.
 * Zeros drive encoders, zeros turret, and drives off of the line.
 */
public class InitAuto implements IAuto {
    private Command
        zeroDriveEncoders,
        zeroTurret,
        driveOffLine;
    
    public InitAuto(SubsystemDrive drivetrain, SubsystemTurret turret) {
        this.zeroDriveEncoders = new InstantCommand(() -> drivetrain.zeroEncoders(), drivetrain);
        this.zeroTurret = new CyborgCommandZeroTurret(turret);
        //drive, could be smart drive or dumb drive
        if(drivetrain.getNavXConnected() && Util.getAndSetBoolean("Use SmartDistance", true)) {
            this.driveOffLine = new CyborgCommandSmartDriveDistance(drivetrain, Util.getAndSetDouble("Initiation Drive", -36), Constants.DRIVE_AUTO_INHIBITOR);
        } else {
            this.driveOffLine = new CyborgCommandDriveDistance(drivetrain, Util.getAndSetDouble("Initiation Drive", -36), Constants.DRIVE_AUTO_INHIBITOR);
        }
    }

    public Command getCommand() {
        Command driveWhileZeroing = zeroTurret.alongWith(driveOffLine);
        return zeroDriveEncoders.andThen(driveWhileZeroing);
    }

    public boolean requiresFlywheel() {
        return false;
    }
}
