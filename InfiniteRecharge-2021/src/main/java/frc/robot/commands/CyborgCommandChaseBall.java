// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemJevois;
import frc.robot.util.Util;

public class CyborgCommandChaseBall extends CommandBase {
  private SubsystemDrive drivetrain;
  private SubsystemJevois jevois;
  private SubsystemIntake intake;
  private SubsystemFeeder feeder;
  private PIDController headingController;
  private boolean ballSpottedBefore;

  /** Creates a new CyborgCommandChaseBall. */
  public CyborgCommandChaseBall(SubsystemDrive drivetrain, SubsystemJevois jevois, SubsystemIntake intake, SubsystemFeeder feeder) {
    this.drivetrain = drivetrain;
    this.jevois = jevois;
    this.intake = intake;
    this.feeder = feeder;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double headingP = Util.getAndSetDouble("Chase Heading kP", 0.001);
    double headingI = Util.getAndSetDouble("Chase Heading kI", 0);
    double headingD = Util.getAndSetDouble("Chase Heading kD", 0);
    headingController = new PIDController(headingP, headingI, headingD);
    headingController.setSetpoint(0);

    ballSpottedBefore = false;

    drivetrain.setRamps(Util.getAndSetDouble("Chase Ramp", 1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double
      basePower = Util.getAndSetDouble("Chase Base Power", 0),
      leftPower = basePower,
      rightPower = basePower;

    boolean spotted = jevois.ballSpotted();
    if(spotted) {
      //calcuate power needed for heading correction
      int headingChange = getAngleFromImagePosition(jevois.getHorizontalPosition());
      headingChange += Util.getAndSetDouble("Chase Offset", 10);

      double headingCorrection = headingController.calculate(headingChange);
      headingCorrection = (headingCorrection > 1 ? 1 : (headingCorrection < -1 ? -1 : headingCorrection));

      //calculate power required to drive
      leftPower = basePower + headingCorrection;
      rightPower = basePower - headingCorrection;

      SmartDashboard.putNumber("Chase Heading Correction", headingCorrection);
    } else if(ballSpottedBefore) {
      ConstantCommandDriveIntake driveIntake = new ConstantCommandDriveIntake(intake, feeder);
      CyborgCommandWait wait = new CyborgCommandWait(Constants.JEVOIS_SUCK_TIME);
      Command suck = driveIntake.raceWith(wait);
      suck.schedule();
    }

    drivetrain.setLeftPercentOutput(leftPower);
    drivetrain.setRightPercentOutput(rightPower);

    ballSpottedBefore = spotted;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setLeftPercentOutput(0);
    drivetrain.setRightPercentOutput(0);
    drivetrain.setRamps();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Converts a horizontal position in pixels to a heading change in degrees.
   * @param imagePosition X position in image space.
   * @return Heading change in degrees.
   */
  private int getAngleFromImagePosition(int imagePosition) {
    return (int) (imagePosition * (90 / (double) Constants.JEVOIS_RESOLUTION_X));
  }
}
