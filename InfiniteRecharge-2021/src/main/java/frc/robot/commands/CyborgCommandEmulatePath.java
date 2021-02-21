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
import frc.robot.util.PathRecorder;
import frc.robot.util.Point2D;
import frc.robot.util.Util;

public class CyborgCommandEmulatePath extends CommandBase {
  private SubsystemDrive drivetrain;
  private Point2D[] points;
  private int currentPointIndex;
  private boolean isForwards;
  private String pointsFilePath;
  private PathRecorder recorder;

  /** Creates a new CyborgCommandEmulatePath. */
  public CyborgCommandEmulatePath(SubsystemDrive drivetrain, String filePath) {
    this.drivetrain = drivetrain;
    this.pointsFilePath = filePath;
    recorder = new PathRecorder(Constants.EMULATE_RESULTS_FILE_PATH);

    addRequirements(drivetrain);
  }

  public CyborgCommandEmulatePath(SubsystemDrive drivetrain) {
    this(drivetrain, Constants.EMULATE_DEFAULT_POINTS_FILE_PATH);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPointIndex = 1;
    recorder.init();

    try {
      String fileContents = Files.readString(Path.of(pointsFilePath));

      //create array of points based on fileContents
      String pointStrings[] = fileContents.split("\n");
      points = new Point2D[pointStrings.length];
      for(int i=0; i<pointStrings.length; i++) {
        points[i] = Point2D.fromString(pointStrings[i]);
      }
    } catch (IOException ex) {
      DriverStation.reportError("IO EXCEPTION", true);
      return;
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
    // Robot.getRobotContainer().zeroAllDrivetrain();

    isForwards = new Point2D(0, 0, 0).getHeadingTo(points[1]) < 90;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Point2D currentLocation = Robot.getRobotContainer().getRobotPositionAndHeading();
    recorder.recordPoint(currentLocation);

    //resolve the point that the robot is currently at and where we want to aim
    if(currentPointIndex < points.length - 1) {
      double currentDirection = forwardsify(currentLocation.getHeading());
      for(int limit=0; limit<Constants.EMULATE_POINT_SKIP_LIMIT; limit++) {
        //get the angle that the root needs to turn to acheive the point
        SmartDashboard.putNumber("Emulate heading current to next", currentLocation.getHeadingTo(points[currentPointIndex]));
        double headingToNext = Math.abs(Util.getAngleToHeading(currentDirection, currentLocation.getHeadingTo(points[currentPointIndex])));
        SmartDashboard.putNumber("Emulate Heading to next", headingToNext);

        //get a path that consists of future points. If they are straight, 
        if(currentPointIndex < points.length - 1 && headingToNext >= 75) {
          currentPointIndex++;

          DriverStation.reportError("current heading: " + Double.valueOf(currentDirection).toString() + ", heading to: " + Double.valueOf(currentLocation.getHeadingTo(points[currentPointIndex])).toString() + ", heading to next: " + Double.valueOf(headingToNext).toString(), false);
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
    double headingDifference = Util.getAngleToHeading(currentLocation.getHeading(), headingToNextPoint); //add this back in if the new code breaks
    SmartDashboard.putNumber("Emulate heading diff", headingDifference);
    // double headingDifference = Util.getAngleToHeading(currentLocation.getHeading(), currentDestination.getHeading());
    this.isForwards = Math.abs(headingDifference) < 90;

    SmartDashboard.putBoolean("Emulate Forward", isForwards);

    //Resolve the path of points that are immediately ahead of the robot. This array will include the robot's location as the first point.
    int immediatePathSize = (int) Util.getAndSetDouble("Emulate Immediate Path Size", 5);
    int pointsToSkip = (int) Util.getAndSetDouble("Emulate Points to skip", 2);
    Point2D[] nextPoints = getNextNPoints(points, currentPointIndex + pointsToSkip, immediatePathSize);
    Point2D[] immediatePath = new Point2D[nextPoints.length + 1];

    //set first point to robot location, but the heading must be forwards trajectory.
    immediatePath[0] = new Point2D(currentLocation.getX(), currentLocation.getY(), forwardsify(currentLocation.getHeading()));
    for(int i=1; i<immediatePath.length; i++) {
      immediatePath[i] = nextPoints[i - 1];
    }

    //get an "arc" that closely fits the path. The arc will be used to calculate the left and right velocities.
    double immediateDistance = getDistanceOfPath(immediatePath); //unit: in
    double immediateTurn = getTurnOfPath(immediatePath); //unit: degrees
    double headingChange = Util.getAngleToHeading(immediatePath[1].getHeading(), immediatePath[immediatePath.length - 1].getHeading());
    SmartDashboard.putNumber("Emulate heading change", headingChange);

    //figure out if the robot should switch directions (forward to backward or vice versa) without changing heading.
    double turnToHeadingDifference = Math.abs(Util.getAngleToHeading(headingChange, immediateTurn));
    SmartDashboard.putNumber("Emulate Turn to heading difference", turnToHeadingDifference);
    boolean shouldZeroTurn = turnToHeadingDifference > Constants.EMULATE_MAX_HEADING_TO_TURN_DIFFERENCE;
    // boolean shouldZeroTurn = Math.abs(immediateTurn) > 165;
    

    SmartDashboard.putNumber("Emulate Immediate Turn Before Overturn", immediateTurn);

    if(immediateTurn < Util.getAndSetDouble("Positional Correction Threshold", 20)) {
      //add positional correction to heading by aiming for 2 points ahead of us
      Point2D targetPoint = points[currentPointIndex + 2];
      double positionalCorrection = Util.getAngleToHeading(forwardsify(currentLocation.getHeading()), currentLocation.getHeadingTo(targetPoint));
      positionalCorrection *= currentLocation.getDistanceFrom(targetPoint) * Util.getAndSetDouble("Emulate Positional Correction Inhibitor", 1);
      immediateTurn += positionalCorrection;
      immediateTurn *= Util.getAndSetDouble("Emulate Overturn", 1.2);
    }

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

      double baseVelocity = calculateBestTangentialSpeed(radius);

      SmartDashboard.putNumber("Calc. Emulate Base Velocity", baseVelocity);

      if(isForwards) {
        leftDisplacement  = immediateTurn * (radius - (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2)); //unit: in
        rightDisplacement = immediateTurn * (radius + (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2));
      } else {
        leftDisplacement  = -1 * immediateTurn * (radius + (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2)); //unit: in
        rightDisplacement = -1 * immediateTurn * (radius - (Constants.DRIVETRAIN_WHEEL_BASE_WIDTH / 2));
      }

      SmartDashboard.putNumber("Emulate left displacement", leftDisplacement);
      SmartDashboard.putNumber("Emulate right displacement", rightDisplacement);

      //convert displacments to velocities
      double timeInterval  = immediateDistance / baseVelocity; // unit: sec
      double leftVelocity  = leftDisplacement / timeInterval; //unit: in/sec
      double rightVelocity = rightDisplacement / timeInterval;

      SmartDashboard.putNumber("Emulate left vel.", leftVelocity);
      SmartDashboard.putNumber("Emulate right vel.", rightVelocity);

      if(shouldZeroTurn) {
        double vel = (isForwards ? baseVelocity : -1 * baseVelocity);
        leftVelocity = vel;
        rightVelocity = vel;
      }

      leftVelocity = IPStoRPM(leftVelocity);
      rightVelocity = IPStoRPM(rightVelocity);
      
      leftVelocity = curveVelocity(leftVelocity);
      rightVelocity = curveVelocity(rightVelocity);

      drivetrain.setLeftVelocity(leftVelocity);
      drivetrain.setRightVelocity(rightVelocity);
    } else {
      double baseVelocity = Util.getAndSetDouble("Emulate Max Speed", 40);
      if(!isForwards) {
        baseVelocity *= -1;
      }

      double curvedBaseSpeed = curveVelocity(IPStoRPM(baseVelocity));
      drivetrain.setLeftVelocity(curvedBaseSpeed);
      drivetrain.setRightVelocity(curvedBaseSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setLeftPercentOutput(0);
    drivetrain.setRightPercentOutput(0);
    recorder.closeFile();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentPointIndex >= points.length - Util.getAndSetDouble("Emulate Points to skip", 2) - 2; //command will finish when the last point is acheived.
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

      turn += correctionToPoint;
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
   * @param turnRadius The radius of the turn that the robot will take in inches.
   * @return The best speed for the turn in in/sec
   */
  private static double calculateBestTangentialSpeed(double turnRadius) {
    double maxSpeed = Util.getAndSetDouble("Emulate Max Speed", 60);
    double minSpeed = Util.getAndSetDouble("Emulate Min Speed", 50);
    if(Double.isNaN(turnRadius)) {
      return maxSpeed;
    }

    //gather needed variables (coefficient of friction, normal force, and mass) and convert to SI units.
    double coefficientOfFriction = Util.getAndSetDouble("Emulate Coefficient of Friction", 1); //defaults to the approximate CoE of rubber on concrete. No Unit.
    double normalForce = Util.poundForceToNewtons(Constants.ROBOT_WEIGHT_POUND_FORCE); //unit: N. There is no extra downwards force on the robot so Fn == Fg
    double robotMass   = Util.weightLBFToMassKG(Constants.ROBOT_WEIGHT_POUND_FORCE); //unit: kg
    double radius      = Math.abs(Util.inchesToMeters(turnRadius)); //unit: m. We can absolute value it because we dont care about the direction of the arc.

    //formula: v = sqrt( (r * CoE * Fn) / m )
    double bestSpeed = Math.sqrt( ( radius * coefficientOfFriction * normalForce ) / robotMass ); //unit: m/s

    //convert best speed to in/s
    bestSpeed = Util.metersToInches(bestSpeed); //unit: in/s
    bestSpeed = (bestSpeed > maxSpeed ? maxSpeed : (bestSpeed < minSpeed ? minSpeed : bestSpeed));

    SmartDashboard.putNumber("Emulate bs radius", radius);
    SmartDashboard.putNumber("Emulate turn radius", turnRadius);
    SmartDashboard.putNumber("Emulate best speed", bestSpeed);
    SmartDashboard.putNumber("Emulate Normal Force", normalForce);
    SmartDashboard.putNumber("Emulate Robot Mass", robotMass);
    SmartDashboard.putNumber("Emulate CoE", coefficientOfFriction);

    return bestSpeed;
  }

  //TESTING
  /**
   * Prints test values. This method is temporary.
   */
  public boolean test() {
    return true;
  }
}
