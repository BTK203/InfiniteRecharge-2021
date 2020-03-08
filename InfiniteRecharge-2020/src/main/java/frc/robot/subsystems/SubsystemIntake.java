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
 * Nom nom
 */
public class SubsystemIntake extends SubsystemBase {
  
  private TalonSRX 
    eater,
    slapper;

  /**
   * Creates a new SubsystemIntake.
   */
  public SubsystemIntake() {
    eater = new TalonSRX(Constants.EATER_ID);
    slapper = new TalonSRX(Constants.SLAPPER_ID);

    configureMotors();
  }

  /**
   * Runs with every robot frame.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Eater Amps", eater.getStatorCurrent());
    SmartDashboard.putNumber("Slapper Amps", slapper.getStatorCurrent());
  }

  /**
   * Sets the percent output of the eater motor
   * @param percent desired percent output
   */
  public void driveEater(double percent) {
    eater.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Sets the percent output of the slapper motor
   * @param percent desired percent output
   */
  public void driveSlapper(double percent) {
    slapper.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Stops the motors.
   */
  public void stopMotors() {
    eater.set(ControlMode.PercentOutput, 0);
    slapper.set(ControlMode.PercentOutput, 0);
  }
  
  /**
   * Sets the NeutralModes and inverts of the motors.
   */
  private void configureMotors() {
    NeutralMode mode = (Constants.INTAKE_BRAKING ? NeutralMode.Brake : NeutralMode.Coast);
    eater.setNeutralMode(mode);
    slapper.setNeutralMode(mode);

    eater.setInverted(Constants.EATER_INVERT);
    slapper.setInverted(Constants.SLAPPER_INVERT);
  }
}
