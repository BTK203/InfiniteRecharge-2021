// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ConstantCommandDriveIntake;
import frc.robot.commands.CyborgCommandEmulatePath;
import frc.robot.enumeration.GalacticSearchMode;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemJevois;

/** 
 * Does The Galactic Search Challenge. 
 */
public class GalacticSearchAuto implements IAuto {
    private SubsystemDrive drivetrain;
    private SubsystemJevois jevois;
    private CyborgCommandEmulatePath drivePath;
    private ConstantCommandDriveIntake driveIntake;

    /**
     * Create a new GalacticSearchAuto.
     * @param drivetrain The drivetrain of the robot.
     */
    public GalacticSearchAuto(SubsystemDrive drivetrain, SubsystemJevois jevois, SubsystemIntake intake, SubsystemFeeder feeder) {
        this.drivetrain = drivetrain;
        this.jevois = jevois;
        this.driveIntake = new ConstantCommandDriveIntake(intake, feeder);
    }

    /**
     * Returns the command to schedule to run the auto.
     */
    public Command getCommand() {
        int horizontalPosition = jevois.getHorizontalPosition();
        if(horizontalPosition > 105) { // path A
            if(horizontalPosition < 200) {
                DriverStation.reportWarning("Set A Path 1", false);
                drivePath = new CyborgCommandEmulatePath(drivetrain, Constants.GALACTIC_SEARCH_SET_A_PATH_1);
            } else {
                DriverStation.reportWarning("Set A Path 2", false);
                drivePath = new CyborgCommandEmulatePath(drivetrain, Constants.GALACTIC_SEARCH_SET_A_PATH_2);
            }
        } else { //path B
            if(horizontalPosition < 0) { //closest ball to robot is on the left, run path 1
                DriverStation.reportWarning("Set B Path 1", false);
                drivePath = new CyborgCommandEmulatePath(drivetrain, Constants.GALACTIC_SEARCH_SET_B_PATH_1);
            } else { //closest ball to the robot is on the right, run path 
                DriverStation.reportWarning("Set B Path 2", false);
                drivePath = new CyborgCommandEmulatePath(drivetrain, Constants.GALACTIC_SEARCH_SET_B_PATH_2);
            }
        }

        return drivePath.raceWith(driveIntake);
    }

    /**
     * This auto does not require the flywheel.
     */
    public boolean requiresFlywheel() {
        return false;
    }
}
