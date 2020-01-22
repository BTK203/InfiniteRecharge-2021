/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.*;
public class SubsystemSpinner extends SubsystemBase {
  /**
   * Creates a new SubsystemSpinner.
   */

  private TalonSRX spinner; 

  public SubsystemSpinner() {
    spinner = new TalonSRX(Constants.SPINNER_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void startSpinner(double speed) {
    spinner.set(ControlMode.PercentOutput, speed);
  }

  public void stopSpinner() {
    spinner.set(ControlMode.PercentOutput, 0);
  }
}
