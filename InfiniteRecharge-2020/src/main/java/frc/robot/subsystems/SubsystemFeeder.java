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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Beater Amps", beater.getStatorCurrent());
    SmartDashboard.putNumber("Feeder Amps", feeder.getStatorCurrent());
  }

  public void driveBeater(double percent) {
    beater.set(ControlMode.PercentOutput, percent);
  }

  public void driveFeeder(double percent) {
    feeder.set(ControlMode.PercentOutput, percent);
  }

  public void stopMotors() {
    beater.set(ControlMode.PercentOutput, 0);
    feeder.set(ControlMode.PercentOutput, 0);
  }

  private void configureMotors() {
    NeutralMode mode = (Constants.FEEDER_BRAKING ? NeutralMode.Brake : NeutralMode.Coast);
    beater.setNeutralMode(mode);
    feeder.setNeutralMode(mode);

    beater.setInverted(Constants.BEATER_INVERT);
    feeder.setInverted(Constants.FEEDER_INVERT);
  }
}
