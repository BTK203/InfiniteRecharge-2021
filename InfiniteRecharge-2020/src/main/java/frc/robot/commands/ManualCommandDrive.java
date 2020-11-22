/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.SubsystemDrive;

public class ManualCommandDrive extends CommandBase {
  private SubsystemDrive drivetrain;

  /**
   * Creates a new ManualCommandDrive.
   */
  public ManualCommandDrive(SubsystemDrive drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Joystick driver = Robot.getRobotContainer().getDriver();
    Joystick driver2 = Robot.getRobotContainer().getDriver2();

    switch(Robot.getRobotContainer().getDriveScheme()) {
      case RL:
        drivetrain.DriveTankByController(driver);
        break;
      case TRUE_TANK:
        drivetrain.driveTankTrue(driver, driver2);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
