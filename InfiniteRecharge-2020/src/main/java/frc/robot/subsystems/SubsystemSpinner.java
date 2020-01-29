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

public class SubsystemSpinner extends SubsystemBase {
  /**
   * Creates a new SubsystemSpinner.
   */

  private TalonSRX spinner; 
  private ColorSensorV3 sensor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  int rotations;
  double trueRotations;
  Color detectedColor;
  boolean prevRed = false;

  public SubsystemSpinner() {
    spinner = new TalonSRX(Constants.SPINNER_ID);
    sensor = new ColorSensorV3(i2cPort);
    rotations = 0;
    trueRotations = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("RED", detectedColor.red*255);
    //SmartDashboard.putNumber("BLUE", detectedColor.blue*255);
    //SmartDashboard.putNumber("GREEN", detectedColor.green*255);
  }
  public void startSpinner(double speed) {
    spinner.set(ControlMode.PercentOutput, speed);
  }

  public void stopSpinner() {
    spinner.set(ControlMode.PercentOutput, 0);
  }
  public Color getColor() {
    detectedColor = sensor.getColor();
    return detectedColor;
  }
  // pass though true to reset the rotations done, otherwise false should always be passed though
  public boolean spinRotations(boolean reset) {
  


    if (reset == true){
      rotations = 0;
      trueRotations = 0;
    }
    if((Constants.TARGET_RED[0] < detectedColor.red*255 && detectedColor.red*255 < Constants.TARGET_RED[3]) && (Constants.TARGET_RED[1] < detectedColor.green*255 && detectedColor.green*255 < Constants.TARGET_RED[4]) && (Constants.TARGET_RED[2] < detectedColor.blue*255 && detectedColor.blue*255 < Constants.TARGET_RED[5])){
      if(prevRed == false){
        prevRed = true;
        rotations++;
      }
    }

    if(!((Constants.TARGET_RED[0] < detectedColor.red*255 && detectedColor.red*255 < Constants.TARGET_RED[3]) && (Constants.TARGET_RED[1] < detectedColor.green*255 && detectedColor.green*255 < Constants.TARGET_RED[4]) && (Constants.TARGET_RED[2] < detectedColor.blue*255 && detectedColor.blue*255 < Constants.TARGET_RED[5]))){
      prevRed = false;
      }

    trueRotations = rotations/2;
    if (trueRotations >= 3){
      return true;
    }
    return false;
  }

  // Pass though R,G,Y, or B to find that color, pass through anything else and it will start the spinner instead
  public boolean spinColor(char colorToFind) {
    detectedColor = sensor.getColor();
    startSpinner(Util.getAndSetDouble("Spin Inhibitor", Constants.SPINNER_SPEED));
    SmartDashboard.putNumber("RED", detectedColor.red*255);
    SmartDashboard.putNumber("BLUE", detectedColor.blue*255);
    SmartDashboard.putNumber("GREEN", detectedColor.green*255);
    switch(colorToFind){
      case 'R':
        //code for red
        if((Constants.TARGET_RED[0] < detectedColor.red*255 && detectedColor.red*255 < Constants.TARGET_RED[3]) && (Constants.TARGET_RED[1] < detectedColor.green*255 && detectedColor.green*255 < Constants.TARGET_RED[4]) && (Constants.TARGET_RED[2] < detectedColor.blue*255 && detectedColor.blue*255 < Constants.TARGET_RED[5])){
        stopSpinner();
        SmartDashboard.putBoolean("Found Red", true);
        SmartDashboard.putBoolean("Found Green", false);
        SmartDashboard.putBoolean("Found Blue", false);
        SmartDashboard.putBoolean("Found Yellow", false);
        return true;
        }
        return false;
      case 'G':
        //code for green
        if((Constants.TARGET_GREEN[0] < detectedColor.red*255 && detectedColor.red*255 < Constants.TARGET_GREEN[3]) && (Constants.TARGET_GREEN[1] < detectedColor.green*255 && detectedColor.green*255 < Constants.TARGET_GREEN[4]) && (Constants.TARGET_GREEN[2] < detectedColor.blue*255 && detectedColor.blue*255 < Constants.TARGET_GREEN[5])){
        stopSpinner();
        SmartDashboard.putBoolean("Found Red", false);
        SmartDashboard.putBoolean("Found Green", true);
        SmartDashboard.putBoolean("Found Blue", false);
        SmartDashboard.putBoolean("Found Yellow", false);
        return true;
        }
        return false;
      case 'B':
        //code for blue
        if((Constants.TARGET_BLUE[0] < detectedColor.red*255 && detectedColor.red*255 < Constants.TARGET_BLUE[3]) && (Constants.TARGET_BLUE[1] < detectedColor.green*255 && detectedColor.green*255 < Constants.TARGET_BLUE[4]) && (Constants.TARGET_BLUE[2] < detectedColor.blue*255 && detectedColor.blue*255 < Constants.TARGET_BLUE[5])){
        stopSpinner();
        SmartDashboard.putBoolean("Found Red", false);
        SmartDashboard.putBoolean("Found Green", true);
        SmartDashboard.putBoolean("Found Blue", false);
        SmartDashboard.putBoolean("Found Yellow", false);
        return true;
        }
        return false;
      case 'Y':
        //code for yellow
        if((Constants.TARGET_YELLOW[0] < detectedColor.red*255 && detectedColor.red*255 < Constants.TARGET_YELLOW[3]) && (Constants.TARGET_YELLOW[1] < detectedColor.green*255 && detectedColor.green*255 < Constants.TARGET_YELLOW[4]) && (Constants.TARGET_YELLOW[2] < detectedColor.blue*255 && detectedColor.blue*255 < Constants.TARGET_YELLOW[5])){
        stopSpinner();
        SmartDashboard.putBoolean("Found Red", false);
        SmartDashboard.putBoolean("Found Green", false);
        SmartDashboard.putBoolean("Found Blue", false);
        SmartDashboard.putBoolean("Found Yellow", true);
        return true;
        }
        return false;
      default:
        startSpinner(Util.getAndSetDouble("Spin Inhibitor", Constants.SPINNER_SPEED));
        SmartDashboard.putBoolean("Found Red", false);
        SmartDashboard.putBoolean("Found Green", false);
        SmartDashboard.putBoolean("Found Blue", false);
        SmartDashboard.putBoolean("Found Yellow", false);
        return false;
      }
  }
}
