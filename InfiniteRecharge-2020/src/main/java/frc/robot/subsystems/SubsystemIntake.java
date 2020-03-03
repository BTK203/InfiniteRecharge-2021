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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveEater(double percent) {
    eater.set(ControlMode.PercentOutput, percent);
  }

  public void driveSlapper(double percent) {
    slapper.set(ControlMode.PercentOutput, percent);
  }

  public void stopMotors() {
    eater.set(ControlMode.PercentOutput, 0);
    slapper.set(ControlMode.PercentOutput, 0);
  }
  
  private void configureMotors() {
    NeutralMode mode = (Constants.INTAKE_BRAKING ? NeutralMode.Brake : NeutralMode.Coast);
    eater.setNeutralMode(mode);
    slapper.setNeutralMode(mode);

    eater.setInverted(Constants.EATER_INVERT);
    slapper.setInverted(Constants.SLAPPER_INVERT);
  }
}
