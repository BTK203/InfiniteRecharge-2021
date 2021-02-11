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
import frc.robot.util.CourseAdjuster;
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

    isForwards = Robot.getRobotContainer().getRobotPositionAndHeading().getHeadingTo(points[1]) > 90;
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
      SmartDashboard.putNumber("Emulate Current Direction", currentDirection);
      for(int limit=0; limit<10; limit++) {
        double headingToNext = Math.abs(Util.getAngleToHeading(currentDirection, currentLocation.getHeadingTo(points[currentPointIndex])));
        SmartDashboard.putNumber("Emulate Heading to next", headingToNext);
        if(currentPointIndex < points.length - 1 && headingToNext >= 90) {
          currentPointIndex++;
        } else {
          break;
        }
      }
    }

    currentPointIndex = (currentPointIndex > points.length - 2 ? points.length - 2 : currentPointIndex);

    SmartDashboard.putNumber("Emulate current point", currentPointIndex);
    SmartDashboard.putNumber("Emulate points", points.length);
    SmartDashboard.putString("Emulate target point", points[currentPointIndex].toString());

    Point2D currentDestination = points[currentPointIndex + 1];

    //figure out if we need to drive forwards or backwards to acheive the point
    double headingToNextPoint = currentLocation.getHeadingTo(currentDestination);
    double headingDifference = Util.getAngleToHeading(currentLocation.getHeading(), headingToNextPoint);
    this.isForwards = Math.abs(headingDifference) < 90;

    SmartDashboard.putBoolean("Emulate Forward", isForwards);

    //Resolve the path of points that are immediately ahead of the robot. This array will include the robot's location as the first point.
    Point2D[] nextPoints = getNextNPoints(points, currentPointIndex + 1, Constants.EMULATE_IMMEDIATE_PATH_SIZE);
    Point2D[] immediatePath = new Point2D[nextPoints.length + 1];

    //set first point to robot location, but the heading must be forwards trajectory.
    immediatePath[0] = new Point2D(currentLocation.getX(), currentLocation.getY(), forwardsify(currentLocation.getHeading()));
    for(int i=1; i<immediatePath.length; i++) {
      immediatePath[i] = nextPoints[i - 1];
    }

    //draw an "arc" that closely fits the path. The arc will be used to calculate the left and right velocities.
    double immediateDistance = getDistanceOfPath(immediatePath); //unit: in
    double immediateTurn = getTurnOfPath(immediatePath); //unit: degrees. Old code: double immediateTurn = Util.getAngleToHeading(forwardsify(currentLocation.getHeading()), currentDesination.getHeading());
    immediateTurn *= Util.getAndSetDouble("Emulate Overturn", 1.2);

    //EXPERIMENTAL AND NOT PERM.
    if(!isForwards) {
      immediateTurn *= 2;
    }

    SmartDashboard.putNumber("Emulate immediate turn", immediateTurn);
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

      leftVelocity = IPStoRPM(leftVelocity);
      rightVelocity = IPStoRPM(rightVelocity);
      
      leftVelocity = curveVelocity(leftVelocity);
      rightVelocity = curveVelocity(rightVelocity);

      drivetrain.setLeftVelocity(leftVelocity);
      drivetrain.setRightVelocity(rightVelocity);
    } else {
      double curvedBaseSpeed = curveVelocity(IPStoRPM(baseSpeed));
      drivetrain.setLeftVelocity(curvedBaseSpeed);
      drivetrain.setRightVelocity(curvedBaseSpeed);
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
   * Converts a velocity in inches/sec to RPM.
   * @param ips A velocity in inches/sec
   * @return A velocity in RPM that corresponds to the velocity in ips.
   */
  private double IPStoRPM(double ips) {
    double newVelocity = ips * Constants.DRIVE_ROTATIONS_PER_INCH; //convert to rotations per second
    newVelocity *= 60; //convert to rotations per minute
    return newVelocity;
  }

  /**
   * This odd method is here so that the robot can achieve almost any velocity using only one set of PID constants.
   * It works by increasing the target velocity so that the PID is forced to work harder than it would work otherwise.
   * @param velocitySetpoint The original velocity setpoint in RPM
   * @return The curved velocity setpoint in RPM
   */
  private double curveVelocity(double velocitySetpoint) {
    return (velocitySetpoint > 1132 ? velocitySetpoint += (velocitySetpoint - 40) * 0.4 : velocitySetpoint); //1132 RPM ~= 45 in/sec
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
      double correctionToPoint = Util.getAngleToHeading(lastHeading, headingToPoint);
      if(Math.abs(correctionToPoint) < 90) {
        turn += correctionToPoint;
      }
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
    return (isForwards ? angle : (angle + 180) % 360);
  }

  /**
   * Calculates the best speed that the robot should drive through an arc at.
   * @param turnRadius The radius of the turn that the robot will take.
   * @return The best speed for the turn in in/sec
   */
  private static double calculateBestTangentialSpeed(double turnRadius) {
    //gather needed variables (coefficient of friction, normal force, and mass)
    double coefficientOfFriction = Util.getAndSetDouble("Emulate Coefficient of Friction", 1); //defaults to the approximate CoE of rubber on concrete
    double normalForce = Constants.ROBOT_WEIGHT_POUND_FORCE;
    double robotMass = Util.poundForceToMass(Constants.ROBOT_WEIGHT_POUND_FORCE);

    return 0;
  }
}
