/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;
import frc.robot.Constants;
import frc.robot.util.Util;

/**
 * Thing that spins the color wheel.
 */
public class SubsystemSpinner extends SubsystemBase {
  private TalonSRX spinner; 
  private ColorSensorV3 sensor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private int rotations = 0;
  private double trueRotations;
  private Color detectedColor;
  private boolean prevRed = false;

  public SubsystemSpinner() {
    spinner = new TalonSRX(Constants.SPINNER_ID);
    sensor = new ColorSensorV3(i2cPort);
    rotations = 0;
    trueRotations = 0;
  }

  /**
   * Runs with every robot frame.
   */
  @Override
  public void periodic() {
    detectedColor = sensor.getColor();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RED", detectedColor.red * 255.0);
    SmartDashboard.putNumber("BLUE", detectedColor.blue * 255.0);
    SmartDashboard.putNumber("GREEN", detectedColor.green * 255.0);

    SmartDashboard.putBoolean("Found Red", isRed(detectedColor));
    SmartDashboard.putBoolean("Found Green", isGreen(detectedColor));
    SmartDashboard.putBoolean("Found Blue", isBlue(detectedColor));
    SmartDashboard.putBoolean("Found Yellow", isYellow(detectedColor));

    SmartDashboard.putNumber("Spinner Amps", spinner.getStatorCurrent());
  }

  /**
   * Drives the spinner.
   * @param speed Percent output to set the spinner to.
   */
  public void startSpinner(double speed) {
    spinner.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stops spinner motor.
   */
  public void stopSpinner() {
    spinner.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Returns the color that the color sensor sees.
   */
  public Color getColor() {
    detectedColor = sensor.getColor();
    return detectedColor;
  }

  /**
   * Attempts to complete rotation control.
   * @param reset should always be false, unless the number of rotations needs to be reset.
   * @return true if rotation control was completed, false otherwise.
   */
  public boolean spinRotations(boolean reset) {
    if (reset){
      rotations = 0;
      trueRotations = 0;
    }

    if(isRed(detectedColor)){
      if(!prevRed){
        prevRed = true;
        rotations++;
      }
    }

    if(!isRed(detectedColor)){
      prevRed = false;
    }

    trueRotations = rotations/2;
    if (trueRotations >= Util.getAndSetDouble("Total Rotations", 3)){
      return true;
    }

    return false;
  }

  /**
   * Attempts to complete position control.
   * @param colorToFind the color to find, taken directly from FMS.
   * @return true if position control was completed, false otherwise.
   */
  public boolean spinColor(char colorToFind) {
    startSpinner(Util.getAndSetDouble("Spin Inhibitor", Constants.SPINNER_SPEED));

    switch(colorToFind){
      case 'R':
        //code for red
        if(isRed(detectedColor)){
          stopSpinner();
          return true;
        }

        return false;
      case 'G':
        //code for green
        if(isGreen(detectedColor)){
          stopSpinner();
          return true;
        }

        return false;
      case 'B':
        //code for blue
        if(isBlue(detectedColor)){
          stopSpinner();
          return true;
        }

        return false;
      case 'Y':
        //code for yellow
        if(isYellow(detectedColor)){
          stopSpinner();
          return true;
        }

        return false;
      }
    return false;
  }

  /**
   * Is the color red?
   * @param clr Color to test
   * @return true if the color is red, false otherwise.
   */
  private boolean isRed(Color clr) {
    return (
      (Constants.TARGET_RED[0] < detectedColor.red * 255.0 && detectedColor.red * 255.0 < Constants.TARGET_RED[3]) && 
      (Constants.TARGET_RED[1] < detectedColor.green * 255.0 && detectedColor.green * 255.0 < Constants.TARGET_RED[4]) && 
      (Constants.TARGET_RED[2] < detectedColor.blue * 255.0 && detectedColor.blue * 255.0 < Constants.TARGET_RED[5])
    );
  }

  /**
   * Is the color green?
   * @param clr The color to test
   * @return true if the color is green, false otherwise.
   */
  private boolean isGreen(Color clr) {
    return (
      (Constants.TARGET_GREEN[0] < detectedColor.red * 255.0 && detectedColor.red * 255.0 < Constants.TARGET_GREEN[3]) && 
      (Constants.TARGET_GREEN[1] < detectedColor.green * 255.0 && detectedColor.green * 255.0 < Constants.TARGET_GREEN[4]) && 
      (Constants.TARGET_GREEN[2] < detectedColor.blue * 255.0 && detectedColor.blue * 255.0 < Constants.TARGET_GREEN[5])
    );
  }

  /**
   * Is the color blue?
   * @param clr color to test
   * @return true if the color is blue, false otherwise
   */
  private boolean isBlue(Color clr) {
    return (
      (Constants.TARGET_BLUE[0] < detectedColor.red * 255.0 && detectedColor.red * 255.0 < Constants.TARGET_BLUE[3]) && 
      (Constants.TARGET_BLUE[1] < detectedColor.green * 255.0 && detectedColor.green * 255.0 < Constants.TARGET_BLUE[4]) && 
      (Constants.TARGET_BLUE[2] < detectedColor.blue * 255.0 && detectedColor.blue * 255.0 < Constants.TARGET_BLUE[5])
    );
  }

  /**
   * Is the color yellow?
   * @param clr color to test
   * @return true if the color is yellow, false otherwise.
   */
  private boolean isYellow(Color clr) {
    return (
      (Constants.TARGET_YELLOW[0] < detectedColor.red * 255.0 && detectedColor.red * 255.0 < Constants.TARGET_YELLOW[3]) && 
      (Constants.TARGET_YELLOW[1] < detectedColor.green * 255.0 && detectedColor.green * 255.0 < Constants.TARGET_YELLOW[4]) && 
      (Constants.TARGET_YELLOW[2] < detectedColor.blue * 255.0 && detectedColor.blue * 255.0 < Constants.TARGET_YELLOW[5])
    );
  }
}
