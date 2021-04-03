// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CyborgCommandEmulatePath;
import frc.robot.subsystems.SubsystemDrive;

/** 
 * The auto that does the auto-nav challenge. Legally.
 */
public class AutoNavAuto implements IAuto {
    private CyborgCommandEmulatePath emulatePath;

    public AutoNavAuto(SubsystemDrive drivetrain, String path) {
        this.emulatePath = new CyborgCommandEmulatePath(drivetrain, path);
    }

    @Override
    public Command getCommand() {
        return emulatePath;
    }

    @Override
    public boolean requiresFlywheel() {
        return false;
    }
    
}
