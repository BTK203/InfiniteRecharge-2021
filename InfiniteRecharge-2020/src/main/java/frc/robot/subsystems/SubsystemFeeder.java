/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;

public class SubsystemFeeder extends SubsystemBase {
  
  private TalonSRX
    mainFeeder,
    turretFeeder;

  /**
   * Creates a new SubsystemFeeder.
   */
  public SubsystemFeeder() {
    mainFeeder = new TalonSRX(Constants.MAINFEEDER_ID);
    turretFeeder = new TalonSRX(Constants.TURRETFEEDER_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startFeeder(){
    mainFeeder.set(ControlMode.PercentOutput, Util.getAndSetDouble("Main Feeder Speed", 1));
    turretFeeder.set(ControlMode.PercentOutput, Util.getAndSetDouble("Main Turret Feeder Speed", 1));
  }

  public void stopFeeder(){
    mainFeeder.set(ControlMode.PercentOutput, 0);
    turretFeeder.set(ControlMode.PercentOutput, 0);
  }
}
