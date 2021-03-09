// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemJevois;
import frc.robot.util.Util;

public class CyborgCommandChaseBall extends CommandBase {
  private SubsystemDrive drivetrain;
  private SubsystemJevois jevois;
  private PIDController headingController;

  /** Creates a new CyborgCommandChaseBall. */
  public CyborgCommandChaseBall(SubsystemDrive drivetrain, SubsystemJevois jevois) {
    this.drivetrain = drivetrain;
    this.jevois = jevois;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(jevois.ballSpotted()) {
      //calcuate power needed for heading correction
      int headingChange = getAngleFromImagePosition(jevois.getHorizontalPosition());
      double headingCorrection = headingController.calculate(headingChange);
      headingCorrection = (headingCorrection > 1 ? 1 : (headingCorrection < -1 ? -1 : headingCorrection));

      //calculate power required to drive
      double basePower = Util.getAndSetDouble("Chase Base Power", 0);
      double leftPower = basePower + headingCorrection;
      double rightPower = basePower - headingCorrection;

      drivetrain.setLeftPercentOutput(leftPower);
      drivetrain.setRightPercentOutput(rightPower);

      SmartDashboard.putNumber("Chase Heading Correction", headingCorrection);
    } else {
      drivetrain.setLeftPercentOutput(0);
      drivetrain.setRightPercentOutput(0);
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
