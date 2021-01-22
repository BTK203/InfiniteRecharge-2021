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
  private int destinationPointIndex;

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

      destinationPointIndex = 0;
    } catch (IOException ex) {
      DriverStation.reportError("IO EXCEPTION", true);
    }

    //update the PID Constants for heading.
    double 
      kP           = Util.getAndSetDouble("Drive Velocity kP", 0),
      kI           = Util.getAndSetDouble("Drive Velocity kI", 0),
      kD           = Util.getAndSetDouble("Drive Velocity kD", 0),
      kF           = Util.getAndSetDouble("Drive Velocity kF", 0),
      izone        = Util.getAndSetDouble("Drive Velocity IZone", 0),
      outLimitLow  = Util.getAndSetDouble("Drive Velocity Out Limit Low", -1),
      outLimitHigh = Util.getAndSetDouble("Drive Velocity Out Limit High", 1),
      velocitySetpoint = Util.getAndSetDouble("Drive Velocity Setpoint", 12);
    
    drivetrain.setPIDConstants(kP, kI, kD, kF, izone, outLimitLow, outLimitHigh);
    courseAdjuster.setVelocity(velocitySetpoint);

    //update the PID Constansts for velocity.
    double
      headingkP = Util.getAndSetDouble("Drive Heading kP", 0),
      headingkI = Util.getAndSetDouble("Drive Heading kI", 0),
      headingkD = Util.getAndSetDouble("Drive Heading kD", 0);

    courseAdjuster.setHeadingPID(headingkP, headingkI, headingkD);
    courseAdjuster.setHeading(drivetrain.getGyroAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Point2D currentLocation = Robot.getRobotContainer().getRobotPositionAndHeading();
    // //can we focus on the next point?
    // if(currentLocation.getDistanceFrom(points[destinationPointIndex]) > Constants.EMULATE_PATH_MAX_POINT_DISTANCE) {
    //   destinationPointIndex++;
    // }

    // Point2D destinationPoint = points[destinationPointIndex];



    //TEMPORARY TESTING:
    courseAdjuster.setHeading(90);
    courseAdjuster.setVelocity(60);
    courseAdjuster.update();
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
    return false;
  }
}
