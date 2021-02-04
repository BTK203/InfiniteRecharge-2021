// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.Point2D;
import frc.robot.util.TrajectorySegment;
import frc.robot.util.Util;

public class CyborgCommandEmulatePath extends CommandBase {
  private SubsystemDrive drivetrain;
  private Point2D[] points;
  private int currentPointIndex;
  private boolean isForwards;

  /** Creates a new CyborgCommandEmulatePath. */
  public CyborgCommandEmulatePath(SubsystemDrive drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPointIndex = 1;
    isForwards = true;

    try {
      String fileContents = Files.readString(Path.of("/home/lvuser/points.txt"));

      //create array of points based on fileContents
      String pointStrings[] = fileContents.split("\n");
      points = new Point2D[pointStrings.length];
      for(int i=0; i<pointStrings.length; i++) {
        points[i] = Point2D.fromString(pointStrings[i]);
      }
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
      outLimitHigh = Util.getAndSetDouble("Drive Velocity Out Limit High", 1);

    // //drivetrain closed loop ramp
    drivetrain.setPIDRamp(Util.getAndSetDouble("Drive PID Ramp", 0.5));
    drivetrain.setPIDConstants(kP, kI, kD, kF, izone, outLimitLow, outLimitHigh);
    Robot.getRobotContainer().zeroAllDrivetrain();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Point2D currentLocation = Robot.getRobotContainer().getRobotPositionAndHeading();
    double baseSpeed = Util.getAndSetDouble("Emulation Base Speed", 75);

    //resolve the point that the robot is currently at and where we want to aim
    if(currentPointIndex < points.length - 1) {
      // double headingToBasePoint = currentLocation.getHeadingTo(points[currentPointIndex]);
      // double headingDifference = Math.abs(Util.getAngleToHeading(currentLocation.getHeading(), headingToBasePoint));
      // SmartDashboard.putNumber("Emulate Heading Difference", headingDifference);
      // SmartDashboard.putNumber("Emulate Last Head Diff", lastHeadingDifference);
      
      // if((headingDifference >= 90 && lastHeadingDifference < 90) || (headingDifference <= 90 && lastHeadingDifference > 90)) {
      //   SmartDashboard.putBoolean("Emulate Advancing", true);
      //   currentPointIndex++;
      // } else {
      //   SmartDashboard.putBoolean("Emulate Advancing", false);
      // }

      // lastHeadingDifference = headingDifference;

      double currentDirection = forwardsify(currentLocation.getHeading());
      for(int limit=0; limit<10; limit++) {
        if(Math.abs(Util.getAngleToHeading(currentDirection, currentLocation.getHeadingTo(points[currentPointIndex]))) >= 90) {
          currentPointIndex++;
        } else {
          break;
        }
      }
    }
    
    Point2D currentDestination = points[currentPointIndex + 1];

    //figure out if we need to drive forwards or backwards to acheive the point
    double headingToNextPoint = currentLocation.getHeadingTo(currentDestination);
    double headingDifference = Util.getAngleToHeading(currentLocation.getHeading(), headingToNextPoint);
    this.isForwards = Math.abs(headingDifference) < 90;

    //Resolve the path of points that are immediately ahead of the robot. This array will include the robot's location as the first point.
    Point2D[] nextPoints = getNextNPoints(points, currentPointIndex, Constants.EMULATE_IMMEDIATE_PATH_SIZE);
    Point2D[] immediatePath = new Point2D[Constants.EMULATE_IMMEDIATE_PATH_SIZE + 1];

    //set first point to robot location, but the heading must be forwards trajectory.
    immediatePath[0] = new Point2D(currentLocation.getX(), currentLocation.getY(), forwardsify(currentLocation.getHeading()));
    for(int i=1; i<immediatePath.length; i++) {
      immediatePath[i] = nextPoints[i - 1];
    }

    //draw an "arc" that closely fits the path. The arc will be used to calculate the left and right velocities.
    double immediateDistance = getDistanceOfPath(immediatePath); //unit: in
    double immediateTurn = getTurnOfPath(immediatePath); //unit: degrees. Old code: double immediateTurn = Util.getAngleToHeading(forwardsify(currentLocation.getHeading()), currentDesination.getHeading());
    immediateTurn = Math.toRadians(immediateTurn); //we need radians for arc length

    if(immediateTurn != 0) {
      //use immediateDistance and immediateTurn to calculate the left and right base velocities of the wheels.
      double radius = immediateDistance / immediateTurn; //unit: in
      double leftDisplacement = 0;
      double rightDisplacement = 0;

      if(isForwards) {
        leftDisplacement  = immediateTurn * (radius - (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2)); //unit: in
        rightDisplacement = immediateTurn * (radius + (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2));
      } else {
        leftDisplacement  = -1 * immediateTurn * (radius + (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2)); //unit: in
        rightDisplacement = -1 * immediateTurn * (radius - (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2));
      }

      //convert displacments to velocities
      double timeInterval  = immediateDistance / baseSpeed; // unit: sec
      double leftVelocity  = leftDisplacement / timeInterval; //unit: in/sec
      double rightVelocity = rightDisplacement / timeInterval;

      drivetrain.setLeftVelocity(leftVelocity);
      drivetrain.setRightVelocity(rightVelocity);
    } else {
      drivetrain.setLeftVelocity(baseSpeed);
      drivetrain.setRightVelocity(baseSpeed);
    }
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
    return currentPointIndex >= points.length - 2; //command will finish when the last point is acheived.
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

  /**
   * Returns the sum of the distance between all points of a path.
   * @param path An array of points representing the path.
   * @return The approximate distance of the path.
   */
  private double getDistanceOfPath(Point2D[] path) {
    double distance = 0;
    for(int i=0; i<path.length - 1; i++) {
      distance += path[i].getDistanceFrom(path[i + 1]);
    }

    SmartDashboard.putNumber("Emulate Mini Path length", path.length);
    SmartDashboard.putNumber("Emulate Path Distance", distance);

    return distance;
  }

  /**
   * Returns the average turn of a path.
   * @param path An array of points representing the path.
   * @return The average turn of the path in degrees.
   */
  private double getTurnOfPath(Point2D[] path) {
    double turn = 0;
    double lastHeading = path[0].getHeading();
    for(int i=1; i<path.length; i++) {
      double headingToPoint = path[i - 1].getHeadingTo(path[i]);
      turn += Util.getAngleToHeading(lastHeading, headingToPoint);
      lastHeading = headingToPoint;
    }

    return turn;
  }

  /**
   * Returns an angle corresponding to the direction that the robot is travelling in
   * @param angle Original angle.
   * @param isForwards True if robot is driving forwards, false otherwise
   */
  private double forwardsify(double angle) {
    return (isForwards ? angle : (angle + 180) % 180);
  }

  /**
   * Calculates the left and right velocities required for the robot to acheive a destination and heading, and returns
   * it along with a distance that the robot needs to drive with those velocities.
   * @param currentPosition The current position and heading of the robot's center.
   * @param destination The new desination (with heading) of the robot's center.
   * @return A TrajectorySegment, which contains the left velocty, right velocity, and required distance to acheive the desination.
   */
  @Deprecated //will likely remove soon because other algorithm works just fine
  private static TrajectorySegment calculateTrajectory(Point2D currentPosition, Point2D destination, double baseSpeed) {
    //to calculate the required velocities, first derive the radius of the required arc from the two points.
    //to derive the radius, we need the distance between the points and the heading to the desination.
    boolean isForwards = Math.abs(currentPosition.getHeadingTo(destination)) < 90;

    double turn = Util.getAngleToHeading(currentPosition.getHeading(), destination.getHeading());
    double distanceToDestination = currentPosition.getDistanceFrom(destination); //not what will be returned. This is a birds-eye value. Unit: in

    if(turn == 0) {
      return new TrajectorySegment(baseSpeed, baseSpeed, distanceToDestination, 0, isForwards);
    }

    double originalHeadingToDestination = currentPosition.getHeadingTo(destination);
    double headingToDestination  = Util.getAcuteSuppliment(originalHeadingToDestination); //this heading should be the shortest angle to the horizontal

    double a = 90 - Util.getAcuteSuppliment(destination.getHeading()) - headingToDestination; //this value represents the angle between the line through both points (current and dest.) and the radius. It is named "a" because that description doesn't make a good name.
    double c = 180 - (2 * a); //represents the angle between the radii of the endpoints of the arc we are making.

    //for the remainder of calcuations, the angles must be in radians.
    a = Math.toRadians(a);
    c = Math.toRadians(c);

    //use the Law of Sines to derive the radius of the arc.
    double radius = (distanceToDestination * Math.sin(a)) / Math.sin(c); //unit: in

    if(radius < 0) {
      double a2 = Util.getAcuteSuppliment(currentPosition.getHeading() + 90) - Util.getAcuteSuppliment(originalHeadingToDestination);
      radius = distanceToDestination * Math.sin(a2);
    }

    //now use angular kinematics to determine the velocities of both sides of the drivetrain.
    double arcDistance = radius * c; //unit: in

    double leftDisplacement = 0;
    double rightDisplacement = 0;

    if(isForwards) {
      leftDisplacement  = turn * (radius - (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2)); //unit: in
      rightDisplacement = turn * (radius + (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2));
    } else {
      leftDisplacement  = -1 * turn * (radius + (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2)); //unit: in
      rightDisplacement = -1 * turn * (radius - (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2));
    }

    //convert displacments to velocities
    double timeInterval  = arcDistance / baseSpeed; // unit: sec
    double leftVelocity  = leftDisplacement / timeInterval; //unit: in/sec
    double rightVelocity = rightDisplacement / timeInterval;

    return new TrajectorySegment(leftVelocity, rightVelocity, arcDistance, turn, isForwards);
  }
}
