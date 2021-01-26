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
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.CourseAdjuster;
import frc.robot.util.Point2D;
import frc.robot.util.Util;

public class CyborgCommandEmulatePath extends CommandBase {
  private CourseAdjuster courseAdjuster;
  private SubsystemDrive drivetrain;
  private Point2D[] points;
  private int currentPointIndex;

  /** Creates a new CyborgCommandEmulatePath. */
  public CyborgCommandEmulatePath(SubsystemDrive drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.courseAdjuster = new CourseAdjuster(drivetrain, 0, 0, 0);
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

      currentPointIndex = 0;
    } catch (IOException ex) {
      DriverStation.reportError("IO EXCEPTION", true);
    } catch (NumberFormatException ex) {
      DriverStation.reportError("NUMBER FORMAT EXCEPTION", true);
    }

    //update the PID Constants for heading.
    double 
      kP           = Util.getAndSetDouble("Drive Velocity kP", 0),
      kI           = Util.getAndSetDouble("Drive Velocity kI", 0),
      kD           = Util.getAndSetDouble("Drive Velocity kD", 0),
      kF           = Util.getAndSetDouble("Drive Velocity kF", 0),
      izone        = Util.getAndSetDouble("Drive Velocity IZone", 0),
      outLimitLow  = Util.getAndSetDouble("Drive Velocity Out Limit Low", -1),
      outLimitHigh = Util.getAndSetDouble("Drive Velocity Out Limit High", 1);

    // //drivetrain closed loop ramp
    drivetrain.setPIDRamp(Util.getAndSetDouble("Drive PID Ramp", 0.5));
    drivetrain.setPIDConstants(kP, kI, kD, kF, izone, outLimitLow, outLimitHigh);
    
    //update the PID Constansts for velocity.
    double
      headingkP = Util.getAndSetDouble("Drive Heading kP", 0),
      headingkI = Util.getAndSetDouble("Drive Heading kI", 0),
      headingkD = Util.getAndSetDouble("Drive Heading kD", 0),
      headingInhibitor = Util.getAndSetDouble("Course Heading Correction Inhibitor", 0.25);

    courseAdjuster.setHeadingPID(headingkP, headingkI, headingkD);
    courseAdjuster.setHeadingCorrectionInhibitor(headingInhibitor);
    courseAdjuster.setTurn(0);
    courseAdjuster.init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Point2D currentLocation = Robot.getRobotContainer().getRobotPositionAndHeading();
    double baseSpeed = Util.getAndSetDouble("Emulation Base Speed", 75);

    //resolve the point that the robot is currently at and where we want to aim
    if(currentPointIndex < points.length - 1) {
      double distanceToBasePoint = currentLocation.getDistanceFrom(points[currentPointIndex]);
      double distanceToNextPoint = currentLocation.getDistanceFrom(points[currentPointIndex + 1]);

      if(distanceToNextPoint < distanceToBasePoint) { //robot closer to aim point than current point.
        currentPointIndex++;
      }
    }

    Point2D currentDestination = points[currentPointIndex + 1];
    
    //figure out heading needed to be on top of currentDestination
    double headingToCurrentDestination = currentLocation.getHeadingTo(currentDestination);
    double requiredTurn = currentLocation.getHeading() - headingToCurrentDestination;
    courseAdjuster.setTurn(requiredTurn);

    //figure out the average turn for the next points ahead to help smooth the path.
    Point2D[] immediatePath = getNextNPoints(points, currentPointIndex, Constants.EMULATE_IMMEDIATE_PATH_SIZE);
    double immediateDistance = getDistanceOfPath(immediatePath); //unit: in
    double immediateTurn = Util.getAngleToHeading(immediatePath[0].getHeading(), immediatePath[immediatePath.length - 1].getHeading()); //unit: degrees
    immediateTurn = Math.toRadians(immediateTurn); //we need radians for arc length

    //use immediateDistance and immediateTurn to calculate the left and right base velocities of the wheels.
    //TODO: ADD ACTUAL WHEEL BASE TO CONSTANTS
    double radius            = immediateDistance / immediateTurn; //unit: in
    double leftDisplacement  = immediateTurn * (radius + (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2)); //unit: in
    double rightDisplacement = immediateTurn * (radius - (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2));

    //convert displacments to velocities
    double timeInterval  = immediateDistance / baseSpeed; // unit: sec
    double leftVelocity  = leftDisplacement / timeInterval; //unit: in/sec
    double rightVelocity = rightDisplacement / timeInterval;

    courseAdjuster.setBaseLeftVelocity(leftVelocity);
    courseAdjuster.setBaseRightVelocity(rightVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setLeftPercentOutput(0);
    drivetrain.setRightPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentPointIndex >= points.length - 1; //command will finish when the last point is acheived.
  }

  /**
   * Returns an "n" long array of points, starting at start.
   * @param baseArray The array to create a sub-array from.
   * @param start     The index to start the sub-array from.
   * @param n         The length of the sub-array.
   * @return An "n" long array of Point2D objects. May be shorter if forbidden indices exist (start + n > length).
   */
  private Point2D[] getNextNPoints(Point2D[] baseArray, int start, int n) {
    int end = start + n;
    end = (end > baseArray.length ? baseArray.length : end);

    Point2D[] points = new Point2D[end - start];
    for(int i=start; i<end; i++) {
      points[i - start] = baseArray[i];
    }

    return points;
  }

  private double getDistanceOfPath(Point2D[] path) {
    double distance = 0;
    for(int i=0; i<path.length - 1; i++) {
      distance += path[i].getDistanceFrom(path[i + 1]);
    }

    return distance;
  }
}
