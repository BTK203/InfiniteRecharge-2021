/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Basic interface for autonomous classes.
 */
public interface IAuto {
    /**
     * Returns the command that should be run during auto.
     */
    public Command getCommand();

    /**
     * Returns true if the current auto requires the flywheel to spin up, false otherwise.
     */
    public boolean requiresFlywheel();

}
