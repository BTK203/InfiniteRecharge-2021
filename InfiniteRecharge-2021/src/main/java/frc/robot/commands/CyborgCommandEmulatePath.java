// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Point2D;

public class CyborgCommandEmulatePath extends CommandBase {
  private Point2D[] points;
  private int destinationPointIndex;

  /** Creates a new CyborgCommandEmulatePath. */
  public CyborgCommandEmulatePath() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      // String fileContents = Files.readString(Path.of("D:\\_Users\\Brach\\projects\\Test\\FRC\\out.txt")); //TODO: DELETE
      String fileContents = Files.readString(Path.of("/home/lvuser/points.txt"));
      DriverStation.reportWarning("file contents: " + fileContents, false);

      //create array of points based on fileContents
      String pointStrings[] = fileContents.split("\n");
      points = new Point2D[pointStrings.length];
      for(int i=0; i<pointStrings.length; i++) {
        points[i] = Point2D.fromString(pointStrings[i]);
      }

      destinationPointIndex = 0;

      //tmp, delete when we know file parsing is good
      String confirmString = "";
      for(int i=0; i<points.length; i++) {
        confirmString += points[i].toString() + "\n";
      }

      boolean good = confirmString.equals(fileContents);
      if(good) {
        DriverStation.reportError("YAY", false);
      } else {
        DriverStation.reportError("NAH", false);
      }
    } catch (IOException ex) {
      DriverStation.reportError("IO EXCEPTION", true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Point2D currentLocation = Robot.getRobotContainer().getRobotPositionAndHeading();
    //can we focus on the next point?
    if(currentLocation.getDistanceFrom(points[destinationPointIndex]) > Constants.EMULATE_PATH_MAX_POINT_DISTANCE) {
      destinationPointIndex++;
    }

    Point2D destinationPoint = points[destinationPointIndex];
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
