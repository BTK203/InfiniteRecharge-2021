/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;

/**
 * Picks up the balls and feeds them to the turret.
 */
public class SubsystemFeeder extends SubsystemBase {
  private TalonSRX 
    intake,
    flaps,
    feeder;

  /**
   * Creates a new SubsystemFeeder.
   */
  public SubsystemFeeder() {
    intake = new TalonSRX(Constants.INTAKE_ID);
    flaps  = new TalonSRX(Constants.FLAP_ID);
    feeder = new TalonSRX(Constants.FEEDER_ID);

    setBraking();
    setInverts();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void eat() {
    double speed = Util.getAndSetDouble("Eat Speed", 1);
    intake.set(ControlMode.PercentOutput, speed);
    flaps.set(ControlMode.PercentOutput, speed);
  }

  public void feed() {
    double speed = Util.getAndSetDouble("Eat Speed", 1);
    feeder.set(ControlMode.PercentOutput, speed);
  }

  public void spit() {
    double speed = Util.getAndSetDouble("Eat Speed", 1) * -1;
    intake.set(ControlMode.PercentOutput, speed);
    flaps.set(ControlMode.PercentOutput, speed);
    feeder.set(ControlMode.PercentOutput, speed);
  }

  public void stopMotors() {
    intake.set(ControlMode.PercentOutput, 0);
    flaps.set(ControlMode.PercentOutput, 0);
    feeder.set(ControlMode.PercentOutput, 0);
  }

  private void setBraking() {
    NeutralMode mode = (Constants.FEEDER_BRAKING ? NeutralMode.Brake : NeutralMode.Coast);
    intake.setNeutralMode(mode);
    flaps.setNeutralMode(mode);
    feeder.setNeutralMode(mode);
  }

  private void setInverts() {
    intake.setInverted(Constants.INTAKE_INVERT);
    flaps.setInverted(Constants.FLAP_INVERT);
    feeder.setInverted(Constants.FEEDER_INVERT);
  }
}
