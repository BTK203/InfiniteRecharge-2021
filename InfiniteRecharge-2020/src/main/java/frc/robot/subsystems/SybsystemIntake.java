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

public class SybsystemIntake extends SubsystemBase {
  
  private TalonSRX
    mainIntake,
    feederIntake;

  /**
   * Creates a new SybsystemIntake.
   */
  public SybsystemIntake() {
    mainIntake = new TalonSRX(Constants.MAININTAKE_ID);
    feederIntake = new TalonSRX(Constants.FEEDINTAKE_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startIntake(){
    mainIntake.set(ControlMode.PercentOutput, Util.getAndSetDouble("Main Intake Speed", 1));
    feederIntake.set(ControlMode.PercentOutput, Util.getAndSetDouble("Main Feeder Intake Speed", 1));
  }

  public void stopIntake(){
    mainIntake.set(ControlMode.PercentOutput, 0);
    feederIntake.set(ControlMode.PercentOutput, 0);
  }
}
