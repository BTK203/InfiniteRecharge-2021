// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ConstantCommandDriveIntake;
import frc.robot.commands.CyborgCommandEmulatePath;
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
        int ballsSeen = jevois.getPowerCells().size();
        if(ballsSeen < 2 || ballsSeen > 3) { //not valid read, error out driver station, thus not starting auto.
            DriverStation.reportError("Invalid read!", true);
            return null;
        }

        int closeBalls = 0;
        for(int i=0; i<jevois.getPowerCells().size(); i++) {
            if(jevois.getPowerCells().get(i).getY() > 300) {
                closeBalls++;
            }
        }

        if(closeBalls >= 1) { // one of the front patterns
            //for front patterns, look at the placement of the closest power cell to determine which path to run
            int closestBallX = jevois.getPowerCells().get(0).getX();
            if(closestBallX > -100) {
                DriverStation.reportWarning("Set A Path 1", false);
                drivePath = new CyborgCommandEmulatePath(drivetrain, Constants.GALACTIC_SEARCH_SET_A_PATH_1);
            } else {
                DriverStation.reportWarning("Set B Path 1", false);
                drivePath = new CyborgCommandEmulatePath(drivetrain, Constants.GALACTIC_SEARCH_SET_B_PATH_1);
            }
        } else { //one of the back patterns. The JeVois cannot see the farthest ball because it is too small.
            //for back patterns, look at the space between the power cells to determine which path to run
            int distanceBetweenBalls = Math.abs(jevois.getPowerCells().get(0).getX() - jevois.getPowerCells().get(1).getX());
            DriverStation.reportWarning("distance between balls: " + Integer.valueOf(distanceBetweenBalls).toString(), false);
            if(distanceBetweenBalls > 250) {
                DriverStation.reportWarning("Set A Path 2", false);
                drivePath = new CyborgCommandEmulatePath(drivetrain, Constants.GALACTIC_SEARCH_SET_A_PATH_2);
            } else {
                DriverStation.reportWarning("Set B Path 2", false);
                drivePath = new CyborgCommandEmulatePath(drivetrain, Constants.GALACTIC_SEARCH_SET_B_PATH_2);
            }
        }

        return drivePath.raceWith(driveIntake);
        // return null;
    }

    /**
     * This auto does not require the flywheel.
     */
    public boolean requiresFlywheel() {
        return false;
    }
}
