/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.Util;

/**
 * Drives a specified distance while maintaining a heading.
 */
public class CyborgCommandSmartDriveDistance extends CommandBase {
  private SubsystemDrive drivetrain;

  private double
    distance,
    heading,
    speed,
    distanceTraveled,
    lastLeftPosition,
    lastRightPosition;

  private PIDController
    distanceController,
    headingController;

  /**
   * Creates a new CyborgCommandSmartDriveDistance.
   * @param drivetrain the drivetrain to drive.
   * @param distance the distance to drive, in inches.
   */
  public CyborgCommandSmartDriveDistance(SubsystemDrive drivetrain, double distance, double speed) {
    this.drivetrain = drivetrain;
    this.distance = distance * Constants.DRIVE_ROTATIONS_PER_INCH;
    this.speed = speed;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set up vars
    DriverStation.reportWarning("CyborgCommandSmartDriveDistance enters", false);
    this.lastLeftPosition = drivetrain.getLeftPosition();
    this.lastRightPosition = drivetrain.getRightPosition();

    //set up distance controller 
    this.distanceTraveled = 0;
    double distanceP = Util.getAndSetDouble("Drive Distance kP", 0);
    double distanceI = Util.getAndSetDouble("Drive Distance kI", 0);
    double distanceD = Util.getAndSetDouble("Drive Distance kD", 0);
    distanceController = new PIDController(distanceP, distanceI, distanceD);
    distanceController.setSetpoint(distance);

    //set up heading controller
    this.heading = drivetrain.getGyroAngle();
    // this.heading = Util.getAndSetDouble("Test Drivetrain Heading", 0);
    double headingP = Util.getAndSetDouble("Drive Heading kP", 0);
    double headingI = Util.getAndSetDouble("Drive Heading kI", 0);
    double headingD = Util.getAndSetDouble("Drive Heading kD", 0);
    headingController = new PIDController(headingP, headingI, headingD);
    headingController.setSetpoint(heading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //calculate distance travelled
    double currentLeftPosition = drivetrain.getLeftPosition();
    double currentRightPosition = drivetrain.getRightPosition();

    double leftTravel = currentLeftPosition - lastLeftPosition;
    double rightTravel = currentRightPosition - lastRightPosition;

    lastLeftPosition = currentLeftPosition;
    lastRightPosition = currentRightPosition;

    double totalTravel = (leftTravel + rightTravel) / 2;
    distanceTraveled += totalTravel;

    //get distance PID output
    double outputForDistance = distanceController.calculate(distanceTraveled);
    outputForDistance = (outputForDistance > speed ? speed : (outputForDistance < speed * -1 ? speed * -1 : outputForDistance));

    //get output for heading
    double outputForHeading = headingController.calculate(drivetrain.getGyroAngle());
    double maxHeadingOutput = Util.getAndSetDouble("Drive Distance Heading Inhibitor", 0.3);
    outputForHeading = (outputForHeading > maxHeadingOutput ? maxHeadingOutput : (maxHeadingOutput < maxHeadingOutput * -1 ? maxHeadingOutput * -1 : outputForHeading));

    double leftOutput = outputForDistance - outputForHeading;
    double rightOutput = outputForDistance + outputForHeading;

    drivetrain.setLeftPercentOutput(leftOutput);
    drivetrain.setRightPercentOutput(rightOutput);

    SmartDashboard.putNumber("Drive Distance Travelled", distanceTraveled);
    SmartDashboard.putNumber("Drive Heading",  drivetrain.getGyroAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setRightPercentOutput(0);
    drivetrain.setLeftPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distError = Math.abs(distance - distanceTraveled);
    return distError < Constants.DRIVETRAIN_ALLOWABLE_ERROR;
  }
}
