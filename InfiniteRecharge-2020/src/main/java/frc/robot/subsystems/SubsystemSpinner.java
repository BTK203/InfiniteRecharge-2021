/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;
import frc.robot.Constants;
import frc.robot.util.Util;

public class SubsystemSpinner extends SubsystemBase {
  /**
   * Creates a new SubsystemSpinner.
   */

  private TalonSRX spinner; 
  private ColorSensorV3 sensor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  public SubsystemSpinner() {
    spinner = new TalonSRX(Constants.SPINNER_ID);
    sensor = new ColorSensorV3(i2cPort);
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
  public Color getColor() {
    Color detectedColor = sensor.getColor();
    return detectedColor;
  }

  public void spinRotations() {
    
  }
  public boolean spinColor(char colorToFind) {
    switch(colorToFind){
      case 'R':
        //code for red
        if((Constants.TARGET_RED[0] < sensor.getRed() && sensor.getRed() < Constants.TARGET_RED[3]) && (Constants.TARGET_RED[1] < sensor.getGreen() && sensor.getGreen() < Constants.TARGET_RED[4]) && (Constants.TARGET_RED[2] < sensor.getBlue() && sensor.getBlue() < Constants.TARGET_RED[5])){
        stopSpinner();
        return true;
        }
        return false;
      case 'G':
        //code for green
        if((Constants.TARGET_GREEN[0] < sensor.getRed() && sensor.getRed() < Constants.TARGET_GREEN[3]) && (Constants.TARGET_GREEN[1] < sensor.getGreen() && sensor.getGreen() < Constants.TARGET_RED[4]) && (Constants.TARGET_RED[2] < sensor.getBlue() && sensor.getBlue() < Constants.TARGET_RED[5])){
        stopSpinner();
        return true;
        }
        return false;
      case 'B':
        //code for blue
        stopSpinner();
        return true;
      case 'Y':
        //code for yellow
        stopSpinner();
        return true;
      default:
        startSpinner(Util.getAndSetDouble("Spin Inhibitor", Constants.SPINNER_SPEED));
        return false;
      }
  }
}
