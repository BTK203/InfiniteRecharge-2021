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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Digestive System
 */
public class SubsystemFeeder extends SubsystemBase {
  
  private TalonSRX 
    beater, //orange spiral motor that pushes ball into turret
    feeder; //accepts ball from beater and gives it to flywheel

  /**
   * Creates a new SubsystemFeeder.
   */
  public SubsystemFeeder() {
    beater = new TalonSRX(Constants.BEATER_ID);
    feeder = new TalonSRX(Constants.FEEDER_ID);

    configureMotors();
  }

  /**
   * Runs with every robot frame.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Beater Amps", beater.getStatorCurrent());
    SmartDashboard.putNumber("Feeder Amps", feeder.getStatorCurrent());
  }

  /**
   * Prints dashboard indicators indicating whether the subsystem is ready for a match.
   * Indicators are to be used for pre-match only. They do not provide an accurite indication
   * of the state of a subsystem in mid match.
   * @return true if the system is ready for a match, false otherwise.
   */
  public boolean getSystemIsGo() {
    boolean beaterConnected = beater.getBusVoltage() > Constants.SPARK_MINIMUM_VOLTAGE;
    boolean feederConnected = feeder.getBusVoltage() > Constants.SPARK_MINIMUM_VOLTAGE;

    SmartDashboard.putBoolean("Beater Connected", beaterConnected);
    SmartDashboard.putBoolean("Feeder Connected", feederConnected);


    return beaterConnected && feederConnected;
  }

  /**
   * Sets the percent output of the beater motor.
   * @param percent desired percent output of beater
   */
  public void driveBeater(double percent) {
    beater.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Sets the percent output of the feeder motor
   * @param percent desired percent output of feeder
   */
  public void driveFeeder(double percent) {
    feeder.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Stops all motors
   */
  public void stopMotors() {
    beater.set(ControlMode.PercentOutput, 0);
    feeder.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Configures neutral modes and inverts of the motors.
   */
  private void configureMotors() {
    NeutralMode mode = (Constants.FEEDER_BRAKING ? NeutralMode.Brake : NeutralMode.Coast);
    beater.setNeutralMode(mode);
    feeder.setNeutralMode(mode);

    beater.setInverted(Constants.BEATER_INVERT);
    feeder.setInverted(Constants.FEEDER_INVERT);
  }
}
