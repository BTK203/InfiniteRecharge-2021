// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantCommandSetHeading extends InstantCommand {
  private SubsystemDrive drivetrain;
  private SubsystemReceiver kiwilight;
  private SubsystemTurret turret;

  public InstantCommandSetHeading(SubsystemDrive drivetrain, SubsystemReceiver kiwilight, SubsystemTurret turret) {
    this.drivetrain = drivetrain;
    this.kiwilight = kiwilight;
    this.turret = turret;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double yawTicksPerDegree = Constants.DEFAULT_TURRET_YAW_TICKS / Constants.TURRET_YAW_DEGREES;
    double yawAngularPosition = turret.getYawPosition() / yawTicksPerDegree;

    double heading = yawAngularPosition - Constants.YAW_FACE_FORWARD_DEGREES;
    
  }
}
